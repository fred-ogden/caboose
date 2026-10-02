/*
 * h8_probe.c
 *
 * Minimal serial probe for Onset H08 family loggers.
 *
 * Purpose: confirm that the logger answers over a USB-serial adapter
 * (e.g., Keyspan USA-19HS on /dev/ttyUSB0) and dump the raw bytes of
 * each response in hex and ASCII, so the protocol can be mapped out.
 *
 * Protocol facts used here were first published in the 2002 program by
 * Tonu Samuel and Andrei Errapart ("H**Ovision"):
 *   - 1200 baud, 8 data bits, no parity, 1 stop bit, no flow control
 *   - single-byte ASCII commands:
 *       E  return copyright / identification string
 *       C  return current reading: 5 raw 8-bit counts, then 0xFF
 *       D  handshake / wake; logger replies 00 FF
 *   Every reply observed so far ends with a 0xFF end-of-response byte.
 *       H  hangup / reset   (effect on logged data UNKNOWN - avoid)
 *
 * Usage:
 *   h8_probe [-d device] [-i idle_ms] [-w wait_after_send_ms]
 *                 [-n] [-t] token [token ...]
 *
 *   Each token is sent in order. A token is either:
 *     - a literal string of characters, e.g.  E   or  DC
 *     - a hex byte written as 0xNN, e.g.     0x45
 *     - the word  sleepNNNN  to pause NNNN milliseconds, e.g. sleep2000
 *   After each send, the program reads until the line has been idle for
 *   idle_ms milliseconds (or a hard cap is reached), then dumps what came
 *   back.
 *
 *   -t   print arrival time (ms after send) of every received chunk
 *   -n   de-assert DTR and RTS (default is to assert both, which is what
 *        the original program implicitly did)
 *
 * Examples:
 *   ./h8_probe E
 *   ./h8_probe D sleep2000 C
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -O2 -o h8_probe h8_probe.c
 *
 * Acknowledgement: the serial settings and the D/E/C/H commands were first
 * published by Tonu Samuel and Andrei Errapart (2002). No code from their
 * program is used here; this file is an independent implementation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Fred Ogden
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define _DEFAULT_SOURCE /* for cfmakeraw() and usleep() under -std=c99 */

#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* Tunable defaults                                                    */
/* ------------------------------------------------------------------ */

/* When nonzero, use 2 stop bits (8N2) instead of 8N1. */
static int use_two_stop_bits_flag = 0;

#define DEFAULT_SERIAL_DEVICE_PATH          "/dev/ttyUSB0"
#define DEFAULT_LINE_IDLE_TIMEOUT_ms        500   /* stop reading after this much silence */
#define DEFAULT_WAIT_AFTER_SEND_ms          0     /* extra pause after each send, before reading */
#define MAXIMUM_READ_DURATION_PER_TOKEN_ms  10000 /* hard cap so a chatty line cannot hang us */
#define RESPONSE_BUFFER_CAPACITY_bytes      65536 /* big enough for a memory dump if one appears */
#define TRANSMIT_BYTE_BUFFER_CAPACITY_bytes 256

/* ------------------------------------------------------------------ */
/* Time helper: monotonic clock in milliseconds                        */
/* ------------------------------------------------------------------ */

static long long monotonic_time_now_ms(void)
{
    struct timespec current_time_spec;
    clock_gettime(CLOCK_MONOTONIC, &current_time_spec);
    return (long long)current_time_spec.tv_sec * 1000LL
         + (long long)(current_time_spec.tv_nsec / 1000000L);
}

static void sleep_for_milliseconds(long duration_ms)
{
    struct timespec requested_sleep_spec;
    requested_sleep_spec.tv_sec  = duration_ms / 1000L;
    requested_sleep_spec.tv_nsec = (duration_ms % 1000L) * 1000000L;
    while (nanosleep(&requested_sleep_spec, &requested_sleep_spec) == -1
           && errno == EINTR) {
        /* resume sleeping for the remaining time after a signal */
    }
}

/* ------------------------------------------------------------------ */
/* Serial port setup: 1200 baud, 8N1, raw, no flow control             */
/* ------------------------------------------------------------------ */

static int open_and_configure_serial_port(const char *serial_device_path,
                                          int assert_dtr_and_rts_flag)
{
    int serial_file_descriptor = open(serial_device_path, O_RDWR | O_NOCTTY);
    if (serial_file_descriptor < 0) {
        fprintf(stderr, "ERROR: cannot open %s: %s\n",
                serial_device_path, strerror(errno));
        return -1;
    }

    struct termios serial_port_attributes;
    if (tcgetattr(serial_file_descriptor, &serial_port_attributes) != 0) {
        fprintf(stderr, "ERROR: tcgetattr failed: %s\n", strerror(errno));
        close(serial_file_descriptor);
        return -1;
    }

    /* Raw mode: no echo, no canonical line editing, no CR/LF translation,
     * no signal characters. Binary bytes pass through untouched. */
    cfmakeraw(&serial_port_attributes);

    cfsetispeed(&serial_port_attributes, B1200);
    cfsetospeed(&serial_port_attributes, B1200);

    serial_port_attributes.c_cflag |=  (CLOCAL | CREAD); /* ignore modem status, enable receiver */
    serial_port_attributes.c_cflag &= ~PARENB;           /* no parity  */
    if (use_two_stop_bits_flag) {
        serial_port_attributes.c_cflag |= CSTOPB;        /* optional 2-stop-bit comparison mode */
    } else {
        serial_port_attributes.c_cflag &= ~CSTOPB;       /* 1 stop bit */
    }
    serial_port_attributes.c_cflag &= ~CSIZE;
    serial_port_attributes.c_cflag |=  CS8;              /* 8 data bits */
    serial_port_attributes.c_cflag &= ~CRTSCTS;          /* no hardware flow control */
    serial_port_attributes.c_iflag &= ~(IXON | IXOFF | IXANY); /* no software flow control */

    /* Non-blocking style reads: we use poll() for timeouts instead. */
    serial_port_attributes.c_cc[VMIN]  = 0;
    serial_port_attributes.c_cc[VTIME] = 0;

    if (tcsetattr(serial_file_descriptor, TCSANOW, &serial_port_attributes) != 0) {
        fprintf(stderr, "ERROR: tcsetattr failed: %s\n", strerror(errno));
        close(serial_file_descriptor);
        return -1;
    }

    /* Set DTR and RTS explicitly. The logger may take power or a wake
     * signal from these lines, so the default is to assert both. */
    int modem_control_line_bits = TIOCM_DTR | TIOCM_RTS;
    int ioctl_request_code = assert_dtr_and_rts_flag ? TIOCMBIS : TIOCMBIC;
    if (ioctl(serial_file_descriptor, ioctl_request_code, &modem_control_line_bits) != 0) {
        fprintf(stderr, "WARNING: could not set DTR/RTS: %s\n", strerror(errno));
    }

    /* Report the modem line state so we know what the logger sees. */
    int current_modem_line_state = 0;
    if (ioctl(serial_file_descriptor, TIOCMGET, &current_modem_line_state) == 0) {
        printf("Modem lines: DTR=%d RTS=%d CTS=%d DSR=%d DCD=%d\n",
               (current_modem_line_state & TIOCM_DTR) ? 1 : 0,
               (current_modem_line_state & TIOCM_RTS) ? 1 : 0,
               (current_modem_line_state & TIOCM_CTS) ? 1 : 0,
               (current_modem_line_state & TIOCM_DSR) ? 1 : 0,
               (current_modem_line_state & TIOCM_CD)  ? 1 : 0);
    }

    /* Discard anything already sitting in the buffers. */
    tcflush(serial_file_descriptor, TCIOFLUSH);

    /* Give the adapter and logger a moment to settle after line changes. */
    sleep_for_milliseconds(250);

    return serial_file_descriptor;
}

/* ------------------------------------------------------------------ */
/* Read until the line goes idle, returning the number of bytes read   */
/* ------------------------------------------------------------------ */

/* When nonzero, print the arrival time and bytes of every read() chunk. */
static int print_chunk_timing_flag = 0;


static size_t read_until_line_idle(int serial_file_descriptor,
                                   unsigned char *response_buffer,
                                   size_t response_buffer_capacity_bytes,
                                   long line_idle_timeout_ms,
                                   long long *first_byte_latency_ms_out)
{
    size_t total_bytes_received = 0;
    long long read_start_time_ms = monotonic_time_now_ms();
    *first_byte_latency_ms_out = -1;

    for (;;) {
        long long elapsed_since_start_ms = monotonic_time_now_ms() - read_start_time_ms;
        if (elapsed_since_start_ms >= MAXIMUM_READ_DURATION_PER_TOKEN_ms) {
            printf("  (hit %d ms read cap; line may still be sending)\n",
                   MAXIMUM_READ_DURATION_PER_TOKEN_ms);
            break;
        }

        struct pollfd serial_poll_descriptor;
        serial_poll_descriptor.fd      = serial_file_descriptor;
        serial_poll_descriptor.events  = POLLIN;
        serial_poll_descriptor.revents = 0;

        int poll_result = poll(&serial_poll_descriptor, 1, (int)line_idle_timeout_ms);
        if (poll_result < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "ERROR: poll failed: %s\n", strerror(errno));
            break;
        }
        if (poll_result == 0) {
            break; /* line idle for line_idle_timeout_ms: response is complete */
        }

        size_t remaining_capacity_bytes = response_buffer_capacity_bytes - total_bytes_received;
        if (remaining_capacity_bytes == 0) {
            printf("  (response buffer full at %zu bytes)\n", total_bytes_received);
            break;
        }

        ssize_t bytes_read_this_call = read(serial_file_descriptor,
                                            response_buffer + total_bytes_received,
                                            remaining_capacity_bytes);
        if (bytes_read_this_call < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            fprintf(stderr, "ERROR: read failed: %s\n", strerror(errno));
            break;
        }
        if (bytes_read_this_call > 0 && *first_byte_latency_ms_out < 0) {
            *first_byte_latency_ms_out = monotonic_time_now_ms() - read_start_time_ms;
        }
        if (bytes_read_this_call > 0 && print_chunk_timing_flag) {
            /* At 1200 baud one byte takes ~8.3 ms on the wire, so a gap much
             * larger than that between chunks means the logger paused. */
            printf("    t=%5lld ms  +%zd byte(s):",
                   monotonic_time_now_ms() - read_start_time_ms, bytes_read_this_call);
            for (ssize_t chunk_byte_index = 0; chunk_byte_index < bytes_read_this_call;
                 ++chunk_byte_index) {
                printf(" %02x", response_buffer[total_bytes_received + (size_t)chunk_byte_index]);
            }
            printf("\n");
        }
        total_bytes_received += (size_t)bytes_read_this_call;
    }

    return total_bytes_received;
}

/* ------------------------------------------------------------------ */
/* Hex + ASCII dump, 16 bytes per row, with byte offsets               */
/* ------------------------------------------------------------------ */

static void print_hex_and_ascii_dump(const unsigned char *data_bytes,
                                     size_t data_length_bytes)
{
    const size_t bytes_per_dump_row = 16;
    for (size_t row_start_offset = 0; row_start_offset < data_length_bytes;
         row_start_offset += bytes_per_dump_row) {
        printf("  %04zx  ", row_start_offset);
        for (size_t column_index = 0; column_index < bytes_per_dump_row; ++column_index) {
            size_t byte_offset = row_start_offset + column_index;
            if (byte_offset < data_length_bytes) printf("%02x ", data_bytes[byte_offset]);
            else                                 printf("   ");
        }
        printf(" |");
        for (size_t column_index = 0; column_index < bytes_per_dump_row; ++column_index) {
            size_t byte_offset = row_start_offset + column_index;
            if (byte_offset >= data_length_bytes) break;
            unsigned char current_byte = data_bytes[byte_offset];
            putchar((current_byte >= 0x20 && current_byte < 0x7f) ? current_byte : '.');
        }
        printf("|\n");
    }
}

/* ------------------------------------------------------------------ */
/* Decode what we know of the 'C' (current reading) packet              */
/* ------------------------------------------------------------------ */

static void print_current_reading_packet_interpretation(const unsigned char *data_bytes,
                                                        size_t data_length_bytes)
{
    /* The published protocol information attributed above describes the
     * 'C' reply as five data bytes followed by a 0xFF end-of-response marker:
     *   index 0..4 = raw 8-bit counts, index 5 = 0xFF.
     * The volts conversion below ASSUMES an 8-bit ADC spanning 0 to 2.5 V;
     * verify it against a known input voltage. */
    const double assumed_full_scale_input_V = 2.5;
    const double assumed_adc_full_scale_counts = 255.0;

    if (data_length_bytes == 6 && data_bytes[5] == 0xFF) {
        printf("  Looks like a 'C' reply (5 data bytes + 0xFF). Candidate channel counts:\n");
        for (size_t byte_index = 0; byte_index <= 4; ++byte_index) {
            unsigned int raw_adc_count = data_bytes[byte_index];
            double assumed_input_voltage_V =
                (double)raw_adc_count / assumed_adc_full_scale_counts * assumed_full_scale_input_V;
            printf("    byte[%zu] = %3u counts  (~%.3f V if 0-2.5 V / 8-bit)\n",
                   byte_index, raw_adc_count, assumed_input_voltage_V);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Parse one command-line token into bytes to transmit                  */
/* Returns number of bytes, -1 for a sleep directive, or -2 for a      */
/* serial break directive (duration in *sleep_duration_ms_out).         */
/* ------------------------------------------------------------------ */

static int parse_token_into_transmit_bytes(const char *token_string,
                                           unsigned char *transmit_bytes,
                                           size_t transmit_capacity_bytes,
                                           long *sleep_duration_ms_out)
{
    if (strncmp(token_string, "sleep", 5) == 0) {
        *sleep_duration_ms_out = strtol(token_string + 5, NULL, 10);
        return -1;
    }
    if (strncmp(token_string, "break", 5) == 0) {
        *sleep_duration_ms_out = strtol(token_string + 5, NULL, 10);
        return -2;
    }
    if ((token_string[0] == '0') && (token_string[1] == 'x' || token_string[1] == 'X')) {
        transmit_bytes[0] = (unsigned char)strtoul(token_string + 2, NULL, 16);
        return 1;
    }
    size_t token_length_bytes = strlen(token_string);
    if (token_length_bytes > transmit_capacity_bytes) {
        token_length_bytes = transmit_capacity_bytes;
    }
    memcpy(transmit_bytes, token_string, token_length_bytes);
    return (int)token_length_bytes;
}

/* ------------------------------------------------------------------ */
/* Help text                                                           */
/* ------------------------------------------------------------------ */

static void print_usage(FILE *output_stream, const char *program_name)
{
    fprintf(output_stream,
"Usage: %s [options] TOKEN [TOKEN ...]\n"
"\n"
"Low-level probe for Onset H8 loggers. Opens the serial port at 1200\n"
"baud 8N1, sends each TOKEN in order, and after each one prints every byte\n"
"received (hex and ASCII) until the line has been quiet for the idle time.\n"
"\n"
"Tokens:\n"
"  LETTERS      sent as ASCII bytes, e.g.  D   E   A   or  DE (two bytes)\n"
"  0xNN         a single byte given in hex, e.g. 0x45\n"
"  sleepNNNN    pause NNNN milliseconds before the next token, e.g. sleep2000\n"
"  breakNNNN    hold a serial BREAK for NNNN milliseconds, e.g. break3000\n"
"\n"
"Commands known to be safe (read only):\n"
"  D   wake / reset page pointer        reply: 00 FF\n"
"  A   identify                         reply: BE FF, then ID bytes, FF\n"
"  E   send next 256-byte memory page   reply: 256 bytes, FF\n"
"Do NOT send B, F, H, I, or M with this tool: they change logger settings,\n"
"start a deployment, or change the baud rate.\n"
"\n"
"Options:\n"
"  -i, --idle=MS            Stop collecting a reply after MS milliseconds of\n"
"                           silence. Default: %d\n"
"  -w, --wait=MS            Extra pause after sending each token, before\n"
"                           listening. Default: %d\n"
"  -t, --timing             Print the arrival time of every received chunk.\n"
"  -n, --no-handshake       De-assert DTR and RTS (default: both asserted).\n"
"  -2, --two-stop-bits      Use 8N2 (2 stop bits) for comparison testing.\n"
"                           Default: 8N1.\n"
"  -d, --device=PATH        Serial device. Default: %s\n"
"  -h, --help               Show this help and exit.\n"
"\n"
"Examples:\n"
"  %s D sleep500 D            is the logger awake?\n"
"  %s -t D sleep100 D A       identify, with byte timing\n"
"  %s break3000 sleep300 D    3 s break, then D (wake test)\n",
            program_name, DEFAULT_LINE_IDLE_TIMEOUT_ms, DEFAULT_WAIT_AFTER_SEND_ms,
            DEFAULT_SERIAL_DEVICE_PATH, program_name, program_name, program_name);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argument_count, char **argument_values)
{
    const char *serial_device_path   = DEFAULT_SERIAL_DEVICE_PATH;
    long line_idle_timeout_ms        = DEFAULT_LINE_IDLE_TIMEOUT_ms;
    long wait_after_send_ms          = DEFAULT_WAIT_AFTER_SEND_ms;
    int  assert_dtr_and_rts_flag     = 1;

    static const struct option long_option_table[] = {
        { "idle",         required_argument, NULL, 'i' },
        { "wait",         required_argument, NULL, 'w' },
        { "timing",       no_argument,       NULL, 't' },
        { "no-handshake", no_argument,       NULL, 'n' },
        { "two-stop-bits", no_argument,      NULL, '2' },
        { "device",       required_argument, NULL, 'd' },
        { "help",         no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };
    int option_character;
    while ((option_character = getopt_long(argument_count, argument_values, "d:i:w:nth2",
                                           long_option_table, NULL)) != -1) {
        switch (option_character) {
        case 'd': serial_device_path   = optarg;                    break;
        case 'i': line_idle_timeout_ms = strtol(optarg, NULL, 10);  break;
        case 'w': wait_after_send_ms   = strtol(optarg, NULL, 10);  break;
        case 'n': assert_dtr_and_rts_flag = 0;                      break;
        case '2': use_two_stop_bits_flag = 1;                       break;
        case 't': print_chunk_timing_flag = 1;                      break;
        case 'h': print_usage(stdout, argument_values[0]);          return 0;
        default:
            fprintf(stderr, "Try '%s --help' for a full description of the options.\n",
                    argument_values[0]);
            return 2;
        }
    }
    if (optind >= argument_count) {
        fprintf(stderr, "ERROR: no tokens given\n\n");
        print_usage(stderr, argument_values[0]);
        return 2;
    }

    printf("Device %s, 1200 %s, idle timeout %ld ms, DTR/RTS %s\n",
           serial_device_path, use_two_stop_bits_flag ? "8N2" : "8N1", line_idle_timeout_ms,
           assert_dtr_and_rts_flag ? "asserted" : "de-asserted");

    int serial_file_descriptor = open_and_configure_serial_port(serial_device_path,
                                                                assert_dtr_and_rts_flag);
    if (serial_file_descriptor < 0) return 1;

    static unsigned char response_buffer[RESPONSE_BUFFER_CAPACITY_bytes];
    unsigned char transmit_bytes[TRANSMIT_BYTE_BUFFER_CAPACITY_bytes];

    for (int token_index = optind; token_index < argument_count; ++token_index) {
        const char *token_string = argument_values[token_index];
        long sleep_duration_ms = 0;

        int transmit_length_bytes = parse_token_into_transmit_bytes(
            token_string, transmit_bytes, sizeof transmit_bytes, &sleep_duration_ms);

        if (transmit_length_bytes == -2) {
            /* Serial break: hold the transmit line in the space (logic 0)
             * state for the given time; this can wake a running logger
             * does for 3000 ms before retrying 'D'. */
            printf("\n[break %ld ms]\n", sleep_duration_ms);
            if (ioctl(serial_file_descriptor, TIOCSBRK, 0) != 0) {
                fprintf(stderr, "ERROR: could not start break: %s\n", strerror(errno));
            }
            sleep_for_milliseconds(sleep_duration_ms);
            if (ioctl(serial_file_descriptor, TIOCCBRK, 0) != 0) {
                fprintf(stderr, "ERROR: could not end break: %s\n", strerror(errno));
            }
            tcflush(serial_file_descriptor, TCIFLUSH);
            continue;
        }
        if (transmit_length_bytes < 0) {
            printf("\n[sleep %ld ms]\n", sleep_duration_ms);
            sleep_for_milliseconds(sleep_duration_ms);
            continue;
        }

        printf("\n>>> send %d byte(s):", transmit_length_bytes);
        for (int byte_index = 0; byte_index < transmit_length_bytes; ++byte_index) {
            printf(" %02x", transmit_bytes[byte_index]);
        }
        printf("  (\"%s\")\n", token_string);

        ssize_t bytes_written = write(serial_file_descriptor, transmit_bytes,
                                      (size_t)transmit_length_bytes);
        if (bytes_written != transmit_length_bytes) {
            fprintf(stderr, "ERROR: write failed: %s\n", strerror(errno));
            break;
        }
        tcdrain(serial_file_descriptor); /* wait until the bytes are physically out */

        if (wait_after_send_ms > 0) sleep_for_milliseconds(wait_after_send_ms);

        long long first_byte_latency_ms = -1;
        size_t response_length_bytes = read_until_line_idle(
            serial_file_descriptor, response_buffer, sizeof response_buffer,
            line_idle_timeout_ms, &first_byte_latency_ms);

        if (response_length_bytes == 0) {
            printf("<<< no response\n");
            continue;
        }
        printf("<<< %zu byte(s), first byte after %lld ms\n",
               response_length_bytes, first_byte_latency_ms);
        print_hex_and_ascii_dump(response_buffer, response_length_bytes);
        print_current_reading_packet_interpretation(response_buffer, response_length_bytes);
    }

    close(serial_file_descriptor);
    return 0;
}
