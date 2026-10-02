/*
 * h8_read.c
 *
 * Read out the full memory of an Onset H07 and H08 data logger (developed on
 * an H08-006-04 "H08-006-04") over a USB-serial adapter, and
 * write:
 *   <prefix>.bin   raw 32 KB memory image (header = bytes 0x00..0xF7 of page 0;
 *                  data from byte 0xF8 of page 0 through page 127)
 *   <prefix>.csv   one line per record: index, elapsed time, time stamp, and the
 *                  raw 8-bit count for each enabled channel
 *
 * Protocol (see h8_protocol_notes.txt):
 *   1200 baud, 8N1, no flow control, DTR and RTS asserted.
 *   'D' -> logger replies 0x00 0xFF; resets the page pointer to page 0.
 *   'E' -> logger sends the next 256-byte page followed by 0xFF, and advances
 *          the page pointer.
 *   The trailing 0xFF is a protocol terminator and is explicitly verified.
 *
 * H7/H8 header fields identified through interoperability analysis,
 * memory-image comparison, and testing with physical loggers:
 *   0x05        channel count
 *   0x07        family code, must be 8 for the H8 family
 *   0x08        config bits; bits 4-5 = memory size (0:8K 1:16K 2:32K 3:64K)
 *   0x32        channel enable mask, bit n = channel n+1
 *   0xB6..0xB7  checksum, big-endian 16-bit sum of bytes 0x00..0xB5
 *   0xBA..0xBD  launch time, big-endian seconds since 1980-01-01 00:00 in the
 *               LOCAL STANDARD TIME of the PC that launched the logger
 *   0xBE..0xC0  big-endian 24-bit, stored as 0xFFFFFF - value, half-second
 *               units; role not yet confirmed (0 on our unit)
 *   0xC2..0xC3  sampling interval, stored as a negated big-endian 16-bit
 *               count of half-seconds (0xFFFF -> 1 -> 0.5 s). Consistent with
 *               the H8 interval range of 0.5 s to ~9.1 h (65535 half-seconds).
 *   0xC1        launch flags; bits 0-3 appear to enable channels 1-4 (0x2F on
 *               the TEST deployment; inferred, not yet verified)
 *   0xC9..0xF1  launch description, NUL-terminated
 * Data begins at byte 0xF8 of page 0 for 8 KB and 32 KB loggers (0xF4 for
 * 16 KB and 64 KB), continuing through the following pages.
 * In the data a byte value of 0x00 never occurs as a reading: the logger
 * writes 0x00 after the last sample, so it marks the end of the deployment.
 * Real counts run 1..255. A launch does not erase memory, so bytes after
 * the marker belong to earlier deployments and are ignored.
 *   0xB8..0xB9  launch count, big-endian; the logger increments it itself
 *
 * The CSV assumes record 0 was taken at the launch time and each following
 * record one interval later. Both the half-second interval unit and the
 * first-sample timing were inferred during interoperability analysis and
 * have not yet been independently verified against a launch with known settings.
 *
 * Usage:
 *   h8_read [-d device] [-o output_prefix] [-c channel_count] [-p page_count]
 *   (-c forces channels 1..N per record instead of using header byte 0xC1)
 *   h8_read -f existing_image.bin [-o output_prefix]   (decode only, no logger)
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -O2 -o h8_read h8_read.c
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

#define _DEFAULT_SOURCE /* cfmakeraw() under -std=c99 */

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
#include <sys/stat.h>

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define DEFAULT_SERIAL_DEVICE_PATH            "/dev/ttyUSB0"   /* change for your hardware */
#define DEFAULT_OUTPUT_PREFIX                 "h8"
#define DEFAULT_CHANNEL_COUNT                 4     /* H08-006-04 */
#define DEFAULT_MEMORY_PAGE_COUNT             128   /* 32 KB / 256 bytes */
#define MEMORY_PAGE_SIZE_bytes                256
#define MAXIMUM_MEMORY_PAGE_COUNT             512   /* sanity cap for -p */

#define WAKE_ATTEMPT_COUNT_MAXIMUM            20    /* bounded wake attempts */
#define WAKE_BREAK_DURATION_ms                3000  /* wakes a logger that is logging */
#define WAKE_POST_BREAK_SETTLE_ms             500   /* let the post-break 0xFF run finish */
#define WAKE_ATTEMPTS_BEFORE_BREAK            3     /* an idle logger answers at once */
#define WAKE_REPLY_TIMEOUT_ms                 300
#define INTER_COMMAND_DELAY_ms                30    /* conservative command spacing */
#define PER_BYTE_READ_TIMEOUT_ms              1000  /* 1 byte = 8.3 ms at 1200 baud */
#define PAGE_READ_RETRY_COUNT_MAXIMUM         3

#define HEADER_CHECKSUM_OFFSET_bytes          0xB6  /* big-endian 16-bit sum of bytes 0x00..0xB5 */
#define HEADER_SERIAL_NUMBER_OFFSET_bytes     0x00  /* big-endian 32-bit */
#define HEADER_MODEL_STRING_OFFSET_bytes      0x09
#define HEADER_DESCRIPTION_OFFSET_bytes       0xC9
#define HEADER_CHANNEL_COUNT_OFFSET_bytes     0x05
#define HEADER_FAMILY_CODE_OFFSET_bytes       0x07
#define HEADER_CONFIG_BITS_OFFSET_bytes       0x08
#define HEADER_CHANNEL_ENABLE_MASK_OFFSET_bytes 0x32
#define HEADER_LAUNCH_COUNT_OFFSET_bytes      0xB8  /* BE16, logger increments on each launch */
#define HEADER_LAUNCH_TIME_OFFSET_bytes       0xBA  /* BE32 s since 1980-01-01 local std time */
#define HEADER_UNKNOWN_HALF_SECONDS_OFFSET_bytes 0xBE /* BE24, stored inverted */
#define HEADER_LAUNCH_FLAGS_OFFSET_bytes      0xC1  /* bits 0-3: channels 1-4 enabled (inferred) */
#define HEADER_INTERVAL_OFFSET_bytes          0xC2  /* BE16, stored negated, half-seconds */
#define H8_FAMILY_CODE                        8

/* ------------------------------------------------------------------ */
/* Time helpers                                                        */
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
    while (nanosleep(&requested_sleep_spec, &requested_sleep_spec) == -1 && errno == EINTR) {
        /* resume after a signal */
    }
}

/* ------------------------------------------------------------------ */
/* Serial port                                                         */
/* ------------------------------------------------------------------ */

static int open_serial_port_1200_8n1(const char *serial_device_path)
{
    int serial_file_descriptor = open(serial_device_path, O_RDWR | O_NOCTTY);
    if (serial_file_descriptor < 0) {
        fprintf(stderr, "ERROR: cannot open %s: %s\n", serial_device_path, strerror(errno));
        return -1;
    }

    struct termios serial_port_attributes;
    if (tcgetattr(serial_file_descriptor, &serial_port_attributes) != 0) {
        fprintf(stderr, "ERROR: %s is not a serial port: %s\n",
                serial_device_path, strerror(errno));
        close(serial_file_descriptor);
        return -1;
    }
    cfmakeraw(&serial_port_attributes);
    cfsetispeed(&serial_port_attributes, B1200);
    cfsetospeed(&serial_port_attributes, B1200);
    serial_port_attributes.c_cflag |=  (CLOCAL | CREAD);
    serial_port_attributes.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    serial_port_attributes.c_cflag |=  CS8;
    serial_port_attributes.c_iflag &= ~(IXON | IXOFF | IXANY);
    serial_port_attributes.c_cc[VMIN]  = 0;
    serial_port_attributes.c_cc[VTIME] = 0;
    if (tcsetattr(serial_file_descriptor, TCSANOW, &serial_port_attributes) != 0) {
        fprintf(stderr, "ERROR: tcsetattr failed: %s\n", strerror(errno));
        close(serial_file_descriptor);
        return -1;
    }

    /* Assert DTR and RTS; the logger interface may draw from them. */
    int modem_control_line_bits = TIOCM_DTR | TIOCM_RTS;
    ioctl(serial_file_descriptor, TIOCMBIS, &modem_control_line_bits);

    sleep_for_milliseconds(250);
    tcflush(serial_file_descriptor, TCIOFLUSH);
    return serial_file_descriptor;
}

static int send_single_command_byte(int serial_file_descriptor, unsigned char command_byte)
{
    if (write(serial_file_descriptor, &command_byte, 1) != 1) {
        fprintf(stderr, "ERROR: write failed: %s\n", strerror(errno));
        return -1;
    }
    tcdrain(serial_file_descriptor);
    return 0;
}

/* Read exactly byte_count bytes, allowing per_byte_timeout_ms between bytes.
 * Returns the number of bytes actually read (less than byte_count on timeout). */
static size_t read_exact_byte_count(int serial_file_descriptor, unsigned char *destination_buffer,
                                    size_t byte_count, long per_byte_timeout_ms)
{
    size_t total_bytes_received = 0;
    while (total_bytes_received < byte_count) {
        struct pollfd serial_poll_descriptor = { serial_file_descriptor, POLLIN, 0 };
        int poll_result = poll(&serial_poll_descriptor, 1, (int)per_byte_timeout_ms);
        if (poll_result < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (poll_result == 0) break; /* timeout */
        ssize_t bytes_read_this_call = read(serial_file_descriptor,
                                            destination_buffer + total_bytes_received,
                                            byte_count - total_bytes_received);
        if (bytes_read_this_call < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            break;
        }
        total_bytes_received += (size_t)bytes_read_this_call;
    }
    return total_bytes_received;
}

/* ------------------------------------------------------------------ */
/* Logger commands                                                     */
/* ------------------------------------------------------------------ */

/* Send a serial BREAK (transmit line held in the space state) for
 * break_duration_ms, then let the line settle and discard whatever the
 * logger sent in response. A running H8 ignores 'D' until it receives a
 * break; a 3000 ms break is used here to wake it. */
static void send_serial_break_and_settle(int serial_file_descriptor, long break_duration_ms)
{
    ioctl(serial_file_descriptor, TIOCSBRK, 0);
    sleep_for_milliseconds(break_duration_ms);
    ioctl(serial_file_descriptor, TIOCCBRK, 0);
    sleep_for_milliseconds(WAKE_POST_BREAK_SETTLE_ms);   /* logger answers with a run of 0xFF */
    tcflush(serial_file_descriptor, TCIFLUSH);
}

/* Send 'D' until the logger answers 00 FF. Also resets the page pointer.
 * If an idle logger does not answer within a few tries, send a 3 s serial
 * break (which wakes a logger that is logging, and a readout then stops
 * its deployment) and keep trying. */
static int wake_logger_and_reset_page_pointer(int serial_file_descriptor)
{
    for (int wake_attempt_index = 0; wake_attempt_index < WAKE_ATTEMPT_COUNT_MAXIMUM;
         ++wake_attempt_index) {
        tcflush(serial_file_descriptor, TCIFLUSH);
        if (send_single_command_byte(serial_file_descriptor, 'D') != 0) return -1;
        unsigned char wake_reply_bytes[2] = { 0xFF, 0xFF };
        size_t reply_byte_count = read_exact_byte_count(serial_file_descriptor, wake_reply_bytes,
                                                        2, WAKE_REPLY_TIMEOUT_ms);
        if (reply_byte_count == 2 &&
            wake_reply_bytes[0] == 0x00 && wake_reply_bytes[1] == 0xFF) {
            return 0;
        }
        if (reply_byte_count >= 1 && wake_reply_bytes[0] == 'D') {
            fprintf(stderr, "WARNING: 'D' echoed back; cable loopback without a logger?\n");
        }
        if (wake_attempt_index + 1 == WAKE_ATTEMPTS_BEFORE_BREAK) {
            fprintf(stderr, "No answer; sending a %d ms serial break to wake a running logger\n",
                    WAKE_BREAK_DURATION_ms);
            send_serial_break_and_settle(serial_file_descriptor, WAKE_BREAK_DURATION_ms);
        }
        sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    }
    fprintf(stderr, "ERROR: logger did not answer 'D' with 0x00 after %d attempts "
                    "(including a serial break)\n", WAKE_ATTEMPT_COUNT_MAXIMUM);
    return -1;
}

/* Read the next 256-byte page with 'E'. Returns 0 on success. */
static int read_next_memory_page(int serial_file_descriptor, unsigned char *page_buffer)
{
    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_command_byte(serial_file_descriptor, 'E') != 0) return -1;
    size_t page_bytes_received = read_exact_byte_count(serial_file_descriptor, page_buffer,
                                                       MEMORY_PAGE_SIZE_bytes,
                                                       PER_BYTE_READ_TIMEOUT_ms);
    if (page_bytes_received != MEMORY_PAGE_SIZE_bytes) return -1;

    unsigned char reply_terminator_byte = 0x00;
    size_t terminator_bytes_received = read_exact_byte_count(serial_file_descriptor,
                                                             &reply_terminator_byte, 1,
                                                             PER_BYTE_READ_TIMEOUT_ms);
    if (terminator_bytes_received != 1 || reply_terminator_byte != 0xFF) {
        fprintf(stderr, "ERROR: page reply missing 0xFF terminator");
        if (terminator_bytes_received == 1)
            fprintf(stderr, " (received 0x%02x)", reply_terminator_byte);
        fputc('\n', stderr);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Header helpers                                                      */
/* ------------------------------------------------------------------ */

static unsigned long read_big_endian_unsigned(const unsigned char *source_bytes, int byte_count)
{
    unsigned long assembled_value = 0;
    for (int byte_index = 0; byte_index < byte_count; ++byte_index) {
        assembled_value = (assembled_value << 8) | source_bytes[byte_index];
    }
    return assembled_value;
}

static void copy_printable_header_string(const unsigned char *header_bytes, int start_offset,
                                         char *destination_string, size_t destination_capacity)
{
    size_t output_index = 0;
    for (int header_offset = start_offset;
         header_offset < MEMORY_PAGE_SIZE_bytes && output_index + 1 < destination_capacity;
         ++header_offset) {
        unsigned char current_byte = header_bytes[header_offset];
        if (current_byte == 0x00) break;
        destination_string[output_index++] =
            (current_byte >= 0x20 && current_byte < 0x7F) ? (char)current_byte : '?';
    }
    destination_string[output_index] = '\0';
}

/* ------------------------------------------------------------------ */
/* Help text                                                           */
/* ------------------------------------------------------------------ */

static void print_usage(FILE *output_stream, const char *program_name)
{
    fprintf(output_stream,
"Usage: %s [options]                   read the logger\n"
"       %s -f IMAGE.bin [options]      decode a saved image, no logger\n"
"\n"
"Read out the memory of an Onset H8 logger (e.g. H08-006-04) over a\n"
"serial port at 1200 baud and write two files:\n"
"  PREFIX.bin   raw memory image, exactly as read from the logger\n"
"  PREFIX.csv   one line per record: record_index, elapsed_time_s,\n"
"               time stamp, and the raw 8-bit count for each channel\n"
"The header (serial number, model, launch time, interval, checksum) is\n"
"printed to the terminal.\n"
"\n"
"A running logger is woken with a 3 s serial break, and\n"
"reading it stops its deployment. A full 32 KB read\n"
"takes about 6 minutes at 1200 baud.\n"
"\n"
"Options:\n"
"  -o, --output=PREFIX      Output file name prefix. Default: %s\n"
"  -p, --pages=N            Number of 256-byte pages to read, 1 to %d.\n"
"                           Default: %d (the whole 32 KB memory). Use a small\n"
"                           number, e.g. 2, to read only the start of a short\n"
"                           deployment quickly.\n"
"  -f, --file=IMAGE.bin     Decode a previously saved .bin image instead of\n"
"                           reading the logger. The .bin is not rewritten.\n"
"  -c, --channels=N         Treat every record as channels 1..N. Default: take\n"
"                           the enabled channels from header byte 0xC1.\n"
"  -d, --device=PATH        Serial device. Default: %s\n"
"  -h, --help               Show this help and exit.\n"
"\n"
"Examples:\n"
"  %s -o aatest               read the whole logger\n"
"  %s -p 2 -o aatest          read only the first two pages\n"
"  %s -f aatest.bin -o again  re-decode a saved image\n",
            program_name, program_name, DEFAULT_OUTPUT_PREFIX,
            MAXIMUM_MEMORY_PAGE_COUNT, DEFAULT_MEMORY_PAGE_COUNT, DEFAULT_SERIAL_DEVICE_PATH,
            program_name, program_name, program_name);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argument_count, char **argument_values)
{
    const char *serial_device_path = DEFAULT_SERIAL_DEVICE_PATH;
    const char *output_prefix      = DEFAULT_OUTPUT_PREFIX;
    const char *input_image_path   = NULL; /* -f: decode an existing image instead of reading */
    int channel_count              = DEFAULT_CHANNEL_COUNT;
    int channel_count_given_flag   = 0; /* -c given: ignore header channel bits */
    int memory_page_count          = DEFAULT_MEMORY_PAGE_COUNT;

    static const struct option long_option_table[] = {
        { "output",   required_argument, NULL, 'o' },
        { "pages",    required_argument, NULL, 'p' },
        { "file",     required_argument, NULL, 'f' },
        { "channels", required_argument, NULL, 'c' },
        { "device",   required_argument, NULL, 'd' },
        { "help",     no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };
    int option_character;
    while ((option_character = getopt_long(argument_count, argument_values, "d:o:c:p:f:h",
                                           long_option_table, NULL)) != -1) {
        switch (option_character) {
        case 'd': serial_device_path = optarg;                        break;
        case 'o': output_prefix      = optarg;                        break;
        case 'c': channel_count      = (int)strtol(optarg, NULL, 10);
                  channel_count_given_flag = 1;                     break;
        case 'p': memory_page_count  = (int)strtol(optarg, NULL, 10); break;
        case 'f': input_image_path   = optarg;                        break;
        case 'h': print_usage(stdout, argument_values[0]);            return 0;
        default:
            fprintf(stderr, "Try '%s --help' for a full description of the options.\n",
                    argument_values[0]);
            return 2;
        }
    }
    if (channel_count < 1 || channel_count > 8 ||
        memory_page_count < 1 || memory_page_count > MAXIMUM_MEMORY_PAGE_COUNT) {
        fprintf(stderr, "ERROR: channel_count must be 1..8 and page_count 1..%d\n",
                MAXIMUM_MEMORY_PAGE_COUNT);
        return 2;
    }

    size_t memory_image_size_bytes = (size_t)memory_page_count * MEMORY_PAGE_SIZE_bytes;
    unsigned char *memory_image_bytes = NULL;

    if (input_image_path != NULL) {
        /* ---- Offline mode: load an image written by an earlier run ---- */
        FILE *image_input_file = fopen(input_image_path, "rb");
        if (image_input_file == NULL) {
            fprintf(stderr, "ERROR: cannot open %s: %s\n", input_image_path, strerror(errno));
            return 1;
        }
        struct stat image_file_status;
        if (fstat(fileno(image_input_file), &image_file_status) != 0 ||
            image_file_status.st_size < 2 * MEMORY_PAGE_SIZE_bytes ||
            image_file_status.st_size % MEMORY_PAGE_SIZE_bytes != 0) {
            fprintf(stderr, "ERROR: %s is not a whole number of 256-byte pages\n",
                    input_image_path);
            fclose(image_input_file);
            return 1;
        }
        memory_image_size_bytes = (size_t)image_file_status.st_size;
        memory_image_bytes = malloc(memory_image_size_bytes);
        if (memory_image_bytes == NULL ||
            fread(memory_image_bytes, 1, memory_image_size_bytes, image_input_file)
                != memory_image_size_bytes) {
            fprintf(stderr, "ERROR: cannot read %s\n", input_image_path);
            fclose(image_input_file);
            free(memory_image_bytes);
            return 1;
        }
        fclose(image_input_file);
    } else {
        /* ---- Live mode: read the logger ---- */
        int serial_file_descriptor = open_serial_port_1200_8n1(serial_device_path);
        if (serial_file_descriptor < 0) return 1;

        /* Reset the logger readout pointer to page 0. */
        if (wake_logger_and_reset_page_pointer(serial_file_descriptor) != 0) {
            close(serial_file_descriptor);
            return 1;
        }

        memory_image_bytes = malloc(memory_image_size_bytes);
        if (memory_image_bytes == NULL) {
            fprintf(stderr, "ERROR: out of memory\n");
            close(serial_file_descriptor);
            return 1;
        }

        /* Read all pages. On a failed page, re-sync with 'D' and skip forward
         * with 'E' to the failed page before retrying. */
        long long readout_start_time_ms = monotonic_time_now_ms();
        for (int page_index = 0; page_index < memory_page_count; ++page_index) {
            unsigned char *page_destination = memory_image_bytes
                                            + (size_t)page_index * MEMORY_PAGE_SIZE_bytes;
            int page_read_succeeded = 0;
            for (int retry_index = 0; retry_index < PAGE_READ_RETRY_COUNT_MAXIMUM; ++retry_index) {
                if (read_next_memory_page(serial_file_descriptor, page_destination) == 0) {
                    page_read_succeeded = 1;
                    break;
                }
                fprintf(stderr, "\nWARNING: page %d short read, re-syncing (retry %d)\n",
                        page_index, retry_index + 1);
                if (wake_logger_and_reset_page_pointer(serial_file_descriptor) != 0) break;
                unsigned char discarded_page_buffer[MEMORY_PAGE_SIZE_bytes];
                int skip_failed = 0;
                for (int skip_page_index = 0; skip_page_index < page_index; ++skip_page_index) {
                    if (read_next_memory_page(serial_file_descriptor, discarded_page_buffer) != 0) {
                        skip_failed = 1;
                        break;
                    }
                }
                if (skip_failed) break;
            }
            if (!page_read_succeeded) {
                fprintf(stderr, "\nERROR: could not read page %d\n", page_index);
                free(memory_image_bytes);
                close(serial_file_descriptor);
                return 1;
            }
            double elapsed_time_s =
                (double)(monotonic_time_now_ms() - readout_start_time_ms) / 1000.0;
            fprintf(stderr, "\rpage %3d/%d  %6.1f s", page_index + 1, memory_page_count,
                    elapsed_time_s);
        }
        fprintf(stderr, "\n");
        close(serial_file_descriptor);
    }

    /* ---- Header summary and checksum check ---- */
    const unsigned char *header_bytes = memory_image_bytes;
    unsigned int computed_header_checksum = 0;
    for (int header_offset = 0; header_offset < HEADER_CHECKSUM_OFFSET_bytes; ++header_offset) {
        computed_header_checksum += header_bytes[header_offset];
    }
    computed_header_checksum &= 0xFFFFu;
    unsigned int stored_header_checksum =
        (unsigned int)read_big_endian_unsigned(header_bytes + HEADER_CHECKSUM_OFFSET_bytes, 2);

    char model_string[64];
    char description_string[64];
    copy_printable_header_string(header_bytes, HEADER_MODEL_STRING_OFFSET_bytes,
                                 model_string, sizeof model_string);
    copy_printable_header_string(header_bytes, HEADER_DESCRIPTION_OFFSET_bytes,
                                 description_string, sizeof description_string);

    printf("Serial number:   %lu\n",
           read_big_endian_unsigned(header_bytes + HEADER_SERIAL_NUMBER_OFFSET_bytes, 4));
    printf("Model:           %s\n", model_string);
    printf("Description:     %s\n", description_string);
    printf("Header checksum: stored 0x%04X, computed 0x%04X  %s\n",
           stored_header_checksum, computed_header_checksum,
           (stored_header_checksum == computed_header_checksum) ? "OK" : "MISMATCH");

    unsigned int family_code = header_bytes[HEADER_FAMILY_CODE_OFFSET_bytes];
    unsigned int header_channel_count = header_bytes[HEADER_CHANNEL_COUNT_OFFSET_bytes];
    unsigned int channel_enable_mask = header_bytes[HEADER_CHANNEL_ENABLE_MASK_OFFSET_bytes];
    static const unsigned long memory_size_by_code_bytes[4] = { 8192UL, 16384UL, 32768UL, 65536UL };
    unsigned long header_memory_size_bytes =
        memory_size_by_code_bytes[(header_bytes[HEADER_CONFIG_BITS_OFFSET_bytes] >> 4) & 0x3];

    unsigned long launch_time_s_since_1980 =
        read_big_endian_unsigned(header_bytes + HEADER_LAUNCH_TIME_OFFSET_bytes, 4);
    unsigned long unknown_field_half_seconds = 0xFFFFFFUL
        - read_big_endian_unsigned(header_bytes + HEADER_UNKNOWN_HALF_SECONDS_OFFSET_bytes, 3);
    unsigned long sampling_interval_half_seconds =
        (0x10000UL - read_big_endian_unsigned(header_bytes + HEADER_INTERVAL_OFFSET_bytes, 2))
        & 0xFFFFUL;
    double sampling_interval_s = 0.5 * (double)sampling_interval_half_seconds;

    /* Convert "seconds since 1980-01-01 00:00" to a calendar date. timegm()
     * is used purely as calendar arithmetic; the result is the launching PC's
     * local standard time, NOT UTC. */
    struct tm epoch_1980_calendar;
    memset(&epoch_1980_calendar, 0, sizeof epoch_1980_calendar);
    epoch_1980_calendar.tm_year = 80;
    epoch_1980_calendar.tm_mday = 1;
    time_t epoch_1980_as_calendar_seconds = timegm(&epoch_1980_calendar);
    time_t launch_time_as_calendar_seconds =
        epoch_1980_as_calendar_seconds + (time_t)launch_time_s_since_1980;
    struct tm launch_time_calendar;
    gmtime_r(&launch_time_as_calendar_seconds, &launch_time_calendar);
    char launch_time_string[64];
    strftime(launch_time_string, sizeof launch_time_string, "%Y-%m-%d %H:%M:%S",
             &launch_time_calendar);

    printf("Family code:     %u %s\n", family_code,
           (family_code == H8_FAMILY_CODE) ? "(H8)" : "(NOT H8 - fields below may be wrong)");
    printf("Channels:        %u, enable mask 0x%02X\n", header_channel_count, channel_enable_mask);
    printf("Memory size:     %lu bytes\n", header_memory_size_bytes);
    printf("Launch time:     %s (launching PC's local standard time)\n", launch_time_string);
    printf("Interval:        %.1f s (%lu half-seconds)\n",
           sampling_interval_s, sampling_interval_half_seconds);
    printf("Launch count:    %lu (header 0xB8, incremented by the logger at each launch)\n",
           read_big_endian_unsigned(header_bytes + HEADER_LAUNCH_COUNT_OFFSET_bytes, 2));
    printf("Field 0xBE:      %lu half-seconds (probably start delay; unconfirmed)\n",
           unknown_field_half_seconds);

    /* ---- Write raw image (live mode only) ---- */
    char output_path[1024];
    snprintf(output_path, sizeof output_path, "%s.bin", output_prefix);
    if (input_image_path == NULL) {
    FILE *binary_output_file = fopen(output_path, "wb");
    if (binary_output_file == NULL ||
        fwrite(memory_image_bytes, 1, memory_image_size_bytes, binary_output_file)
            != memory_image_size_bytes) {
        fprintf(stderr, "ERROR: cannot write %s\n", output_path);
        free(memory_image_bytes);
        return 1;
    }
    fclose(binary_output_file);
    printf("Wrote %s (%zu bytes)\n", output_path, memory_image_size_bytes);
    }

    /* ---- Write CSV of records ----
     * Records run until the logger's 0x00 end-of-data marker, or to the end
     * of the pages read. */
    snprintf(output_path, sizeof output_path, "%s.csv", output_prefix);
    FILE *csv_output_file = fopen(output_path, "w");
    if (csv_output_file == NULL) {
        fprintf(stderr, "ERROR: cannot write %s\n", output_path);
        free(memory_image_bytes);
        return 1;
    }
    /* Which channels are in each record. Inferred (not yet verified with a
     * one-channel launch): bits 0-3 of header byte 0xC1 enable channels 1-4
     * for the deployment. -c overrides with channels 1..N. */
    int record_channel_numbers[8];
    int record_channel_count = 0;
    if (channel_count_given_flag) {
        for (int channel_index = 0; channel_index < channel_count; ++channel_index) {
            record_channel_numbers[record_channel_count++] = channel_index + 1;
        }
    } else {
        for (int channel_index = 0; channel_index < 4; ++channel_index) {
            if (header_bytes[HEADER_LAUNCH_FLAGS_OFFSET_bytes] & (1u << channel_index)) {
                record_channel_numbers[record_channel_count++] = channel_index + 1;
            }
        }
        if (record_channel_count == 0) {
            fprintf(stderr, "ERROR: no channels enabled in header byte 0xC1; use -c\n");
            fclose(csv_output_file);
            free(memory_image_bytes);
            return 1;
        }
    }
    channel_count = record_channel_count;

    fprintf(csv_output_file, "record_index,elapsed_time_s,local_standard_time");
    for (int channel_index = 0; channel_index < channel_count; ++channel_index) {
        fprintf(csv_output_file, ",ch%d_counts", record_channel_numbers[channel_index]);
    }
    fprintf(csv_output_file, "\n");

    /* Logged data begin inside page 0, right after the header, at
     * byte 0xF8 for 8 KB and 32 KB loggers and at 0xF4 for 16 KB and 64 KB
     * loggers. */
    size_t data_start_offset_bytes =
        (header_memory_size_bytes == 16384UL || header_memory_size_bytes == 65536UL)
        ? 0xF4 : 0xF8;
    const unsigned char *data_region_bytes = memory_image_bytes + data_start_offset_bytes;
    size_t data_region_size_bytes = memory_image_size_bytes - data_start_offset_bytes;
    size_t record_count_maximum = data_region_size_bytes / (size_t)channel_count;
    size_t records_written_count = 0;
    int end_marker_found_flag = 0;
    size_t end_marker_offset_bytes = 0;
    for (size_t record_index = 0; record_index < record_count_maximum; ++record_index) {
        const unsigned char *record_bytes = data_region_bytes + record_index * (size_t)channel_count;
        /* End of the deployment: the logger writes a 0x00 byte after the last
         * sample (0x00 never occurs as a reading). A launch does NOT erase
         * memory, so data from an earlier deployment may follow the marker. */
        int record_contains_end_marker = 0;
        for (int channel_index = 0; channel_index < channel_count; ++channel_index) {
            if (record_bytes[channel_index] == 0x00) record_contains_end_marker = 1;
        }
        if (record_contains_end_marker) {
            end_marker_found_flag = 1;
            end_marker_offset_bytes = data_start_offset_bytes
                                    + record_index * (size_t)channel_count;
            break;
        }
        /* 0xFF is NOT treated as unwritten memory: with one channel per
         * record a full-scale reading (e.g. a contact transient) is 0xFF. */
        double record_elapsed_time_s = (double)record_index * sampling_interval_s;
        time_t record_time_as_calendar_seconds =
            launch_time_as_calendar_seconds + (time_t)record_elapsed_time_s;
        struct tm record_time_calendar;
        gmtime_r(&record_time_as_calendar_seconds, &record_time_calendar);
        char record_time_string[32];
        strftime(record_time_string, sizeof record_time_string, "%Y-%m-%d %H:%M:%S",
                 &record_time_calendar);
        /* Append ".5" when the interval puts this record on a half second. */
        int record_is_on_half_second =
            ((sampling_interval_half_seconds * record_index) % 2UL) == 1UL;
        fprintf(csv_output_file, "%zu,%.1f,%s%s", record_index, record_elapsed_time_s,
                record_time_string, record_is_on_half_second ? ".5" : "");
        for (int channel_index = 0; channel_index < channel_count; ++channel_index) {
            fprintf(csv_output_file, ",%u", (unsigned int)record_bytes[channel_index]);
        }
        fprintf(csv_output_file, "\n");
        ++records_written_count;
    }
    fclose(csv_output_file);
    printf("Wrote %s (%zu records of %d channel(s), data from byte 0x%zX)\n", output_path,
           records_written_count, channel_count, data_start_offset_bytes);
    if (end_marker_found_flag) {
        printf("End of data:     0x00 marker at byte 0x%zX; deployment ran %.1f s "
               "(%.2f h) of records\n", end_marker_offset_bytes,
               (double)records_written_count * sampling_interval_s,
               (double)records_written_count * sampling_interval_s / 3600.0);
    } else {
        printf("End of data:     no 0x00 marker in the pages read (memory full, the\n"
               "                 logger was still running, or -p stopped short)\n");
    }

    free(memory_image_bytes);
    return 0;
}
