/*
 * h8_launch.c
 *
 * Launch (start a new deployment on) an Onset H8 family logger from
 * Linux. Developed for the H08-006-04 "H08-006-04".
 *
 * WHAT A LAUNCH DOES
 *   Starts a new deployment: the logger begins recording from the start of
 *   data memory, overwriting whatever was there. Read out the old data first
 *   (h8_read) if you want to keep it.
 *
 * PROTOCOL
 *
 * The launch protocol below was reconstructed for interoperability with
 * legacy H7/H8 hardware. Some field interpretations remain inferred and
 * are identified as such below.
 *   1200 baud, 8N1 (one byte is transmitted at a time
 *   and wait for each reply, so the extra stop bit is not needed).
 *   'D'                  -> 00 FF
 *   'B'                  -> acknowledged, ends with FF
 *   count byte 0x3A (58) -> FF, or echo then FF
 *   58 header bytes 0xBA..0xF3, one at a time; for each the logger replies
 *   with the same byte then FF. Any mismatch aborts the launch.
 *   'F'                  -> finishes the launch; logging starts.
 *   Header bytes 0x00..0xB9 (identity, sensor setup, checksum) are never
 *   written, so a launch cannot damage them.
 *
 * FIELDS THIS PROGRAM SETS (all other bytes are copied from the current header)
 *   0xBA..0xBD  launch time, big-endian seconds since 1980-01-01 00:00 local
 *               standard time, or UTC with -U
 *   0xC1        bits 0-3 = channels 1-4 enabled (INFERRED; bits 4-7 kept)
 *   0xC2..0xC3  interval, stored as negated big-endian 16-bit count of
 *               half-seconds (INFERRED unit)
 *   0xC9..0xF1  description, up to 40 ASCII characters, NUL padded
 * The two INFERRED items are exactly what a short test launch verifies.
 *
 * SAFETY
 *   Without -C the program only reads page 0, saves a backup of it, and
 *   prints what it would change. Nothing is written to the logger.
 *
 * Usage:
 *   h8_launch -i interval_s -c channel_list [-t description] [-U] [-C]
 *                  [-d device] [-R]
 *     -i  sampling interval in seconds, multiple of 0.5, 0.5 .. 32767.5
 *     -c  channels to enable, e.g. 1 or 1,2 or 1,2,3,4
 *     -t  description stored in the logger (default "Linux launch")
 *     -U  store launch time as UTC instead of local standard time
 *     -C  commit: actually launch the logger
 *     -R  read page 0 back after launching. OFF by default: a readout of a
 *         running 4-channel H8 reportedly stops logging, and in our tests a
 *         logging H8 did not answer at all.
 *     -d  serial device (default /dev/ttyUSB0)
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -O2 -o h8_launch h8_launch.c
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

#define _DEFAULT_SOURCE /* cfmakeraw(), timegm(), tzset globals under -std=c99 */

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
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define DEFAULT_SERIAL_DEVICE_PATH          "/dev/ttyUSB0"  /* change for your hardware */
#define DEFAULT_DESCRIPTION_TEXT            "Linux launch"
#define MEMORY_PAGE_SIZE_bytes              256

#define WAKE_BREAK_DURATION_ms                3000  /* wakes a logger that is logging */
#define WAKE_POST_BREAK_SETTLE_ms             500   /* let the post-break 0xFF run finish */
#define WAKE_ATTEMPTS_BEFORE_BREAK            3     /* an idle logger answers at once */
#define WAKE_ATTEMPT_COUNT_MAXIMUM          20
#define REPLY_BYTE_TIMEOUT_ms               1000
#define INTER_COMMAND_DELAY_ms              30
#define POST_REPLY_SETTLE_ms                60
#define FINISH_REPLY_WINDOW_ms              1500   /* window for post-launch response */
#define LAUNCH_TRANSFER_ALLOWANCE_s         6      /* measured: 58 echoed bytes take ~6 s, so
                                                      'F' goes out ~6 s after the time is stamped */

#define HEADER_CHECKSUM_OFFSET_bytes        0xB6
#define HEADER_FAMILY_CODE_OFFSET_bytes     0x07
#define H8_FAMILY_CODE                      8
#define LAUNCH_BLOCK_START_OFFSET_bytes     0xBA   /* first header byte written by 'B' */
#define LAUNCH_BLOCK_LENGTH_bytes           0x3A   /* 58 bytes: 0xBA..0xF3 */
#define HEADER_LAUNCH_TIME_OFFSET_bytes     0xBA
#define HEADER_LAUNCH_FLAGS_OFFSET_bytes    0xC1
#define HEADER_INTERVAL_OFFSET_bytes        0xC2
#define HEADER_DESCRIPTION_OFFSET_bytes     0xC9
#define HEADER_DESCRIPTION_LENGTH_bytes     41     /* 0xC9..0xF1, last byte forced NUL */

/* ------------------------------------------------------------------ */
/* Time helpers                                                        */
/* ------------------------------------------------------------------ */

static void sleep_for_milliseconds(long duration_ms)
{
    struct timespec requested_sleep_spec;
    requested_sleep_spec.tv_sec  = duration_ms / 1000L;
    requested_sleep_spec.tv_nsec = (duration_ms % 1000L) * 1000000L;
    while (nanosleep(&requested_sleep_spec, &requested_sleep_spec) == -1 && errno == EINTR) {
        /* resume after a signal */
    }
}

/* Seconds since 1980-01-01 00:00 on the chosen clock for the time_t given.
 * Local standard time ignores daylight saving. */
static unsigned long seconds_since_1980_for_time(time_t wall_clock_time_s,
                                                 int use_utc_flag)
{
    struct tm epoch_1980_calendar;
    memset(&epoch_1980_calendar, 0, sizeof epoch_1980_calendar);
    epoch_1980_calendar.tm_year = 80;
    epoch_1980_calendar.tm_mday = 1;
    time_t epoch_1980_as_utc_s = timegm(&epoch_1980_calendar);

    long offset_to_clock_s = 0;
    if (!use_utc_flag) {
        tzset();
        offset_to_clock_s = -timezone; /* 'timezone' = seconds WEST of UTC, standard time */
    }
    return (unsigned long)((wall_clock_time_s + offset_to_clock_s) - epoch_1980_as_utc_s);
}

static void format_seconds_since_1980(unsigned long seconds_since_1980,
                                      char *destination_string, size_t destination_capacity)
{
    struct tm epoch_1980_calendar;
    memset(&epoch_1980_calendar, 0, sizeof epoch_1980_calendar);
    epoch_1980_calendar.tm_year = 80;
    epoch_1980_calendar.tm_mday = 1;
    time_t calendar_seconds = timegm(&epoch_1980_calendar) + (time_t)seconds_since_1980;
    struct tm calendar_fields;
    gmtime_r(&calendar_seconds, &calendar_fields);
    strftime(destination_string, destination_capacity, "%Y-%m-%d %H:%M:%S", &calendar_fields);
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
    int modem_control_line_bits = TIOCM_DTR | TIOCM_RTS;
    ioctl(serial_file_descriptor, TIOCMBIS, &modem_control_line_bits);
    sleep_for_milliseconds(250);
    tcflush(serial_file_descriptor, TCIOFLUSH);
    return serial_file_descriptor;
}

static int send_single_byte(int serial_file_descriptor, unsigned char byte_to_send)
{
    if (write(serial_file_descriptor, &byte_to_send, 1) != 1) {
        fprintf(stderr, "ERROR: write failed: %s\n", strerror(errno));
        return -1;
    }
    tcdrain(serial_file_descriptor);
    return 0;
}

/* Read one byte within timeout_ms. Returns 0..255, or -1 on timeout. */
static int read_single_byte(int serial_file_descriptor, long timeout_ms)
{
    for (;;) {
        struct pollfd serial_poll_descriptor = { serial_file_descriptor, POLLIN, 0 };
        int poll_result = poll(&serial_poll_descriptor, 1, (int)timeout_ms);
        if (poll_result < 0 && errno == EINTR) continue;
        if (poll_result <= 0) return -1;
        unsigned char received_byte;
        ssize_t bytes_read = read(serial_file_descriptor, &received_byte, 1);
        if (bytes_read == 1) return received_byte;
        if (bytes_read < 0 && (errno == EINTR || errno == EAGAIN)) continue;
        return -1;
    }
}

static size_t read_exact_byte_count(int serial_file_descriptor, unsigned char *destination_buffer,
                                    size_t byte_count, long per_byte_timeout_ms)
{
    size_t total_bytes_received = 0;
    while (total_bytes_received < byte_count) {
        int received_value = read_single_byte(serial_file_descriptor, per_byte_timeout_ms);
        if (received_value < 0) break;
        destination_buffer[total_bytes_received++] = (unsigned char)received_value;
    }
    return total_bytes_received;
}

static void settle_and_flush_input(int serial_file_descriptor)
{
    sleep_for_milliseconds(POST_REPLY_SETTLE_ms);
    tcflush(serial_file_descriptor, TCIFLUSH);
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

/* 'D' until the logger answers 00 FF. Also resets the page pointer. A
 * running logger is woken with a 3 s serial break after a few tries. */
static int wake_logger_and_reset_page_pointer(int serial_file_descriptor)
{
    for (int wake_attempt_index = 0; wake_attempt_index < WAKE_ATTEMPT_COUNT_MAXIMUM;
         ++wake_attempt_index) {
        tcflush(serial_file_descriptor, TCIFLUSH);
        if (send_single_byte(serial_file_descriptor, 'D') != 0) return -1;
        int first_reply_value = read_single_byte(serial_file_descriptor, 300);
        settle_and_flush_input(serial_file_descriptor);
        if (first_reply_value == 0x00) return 0;
        if (first_reply_value == 'D') {
            fprintf(stderr, "WARNING: 'D' echoed back; cable loopback without a logger?\n");
        }
        if (wake_attempt_index + 1 == WAKE_ATTEMPTS_BEFORE_BREAK) {
            fprintf(stderr, "No answer; sending a %d ms serial break to wake a running logger\n",
                    WAKE_BREAK_DURATION_ms);
            send_serial_break_and_settle(serial_file_descriptor, WAKE_BREAK_DURATION_ms);
        }
        sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    }
    fprintf(stderr, "ERROR: logger did not answer 'D' (including after a serial break)\n");
    return -1;
}

/* Send a command byte and accept the observed acknowledgement forms:
 *   FF alone, (0xFF - command) FF, or 00 FF.
 * An echo of the command itself means a loopback (no logger). */
static int send_command_and_check_acknowledgement(int serial_file_descriptor,
                                                  unsigned char command_byte)
{
    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, command_byte) != 0) return -1;
    int first_reply_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (first_reply_value < 0) {
        fprintf(stderr, "ERROR: no reply to '%c'\n", command_byte);
        return -1;
    }
    if (first_reply_value == 0xFF) return 0;
    if (first_reply_value == command_byte) {
        fprintf(stderr, "ERROR: '%c' echoed back (loopback, no logger?)\n", command_byte);
        return -1;
    }
    int second_reply_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    int acknowledgement_is_valid =
        (first_reply_value == (0xFF - command_byte) || first_reply_value == 0x00)
        && second_reply_value == 0xFF;
    if (!acknowledgement_is_valid) {
        fprintf(stderr, "ERROR: unexpected reply to '%c': %02X %02X\n", command_byte,
                (unsigned int)first_reply_value, (unsigned int)(second_reply_value & 0xFF));
        return -1;
    }
    return 0;
}

/* Count byte after 'B': FF alone, or echo then FF. */
static int send_count_byte_and_check(int serial_file_descriptor, unsigned char count_byte)
{
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, count_byte) != 0) return -1;
    int first_reply_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (first_reply_value == 0xFF) return 0;
    if (first_reply_value == count_byte &&
        read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms) == 0xFF) {
        return 0;
    }
    fprintf(stderr, "ERROR: count byte 0x%02X not accepted (reply %d)\n",
            count_byte, first_reply_value);
    return -1;
}

/* Header data byte: logger must echo it, then FF. */
static int send_header_byte_and_verify_echo(int serial_file_descriptor, unsigned char data_byte,
                                            int header_offset)
{
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, data_byte) != 0) return -1;
    int echoed_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    int terminator_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (echoed_value != data_byte || terminator_value != 0xFF) {
        fprintf(stderr, "ERROR: header byte 0x%02X at offset 0x%02X: logger replied %d %d\n",
                data_byte, header_offset, echoed_value, terminator_value);
        return -1;
    }
    return 0;
}

/* Read page 0 (the header page) fresh: 'D' then 'E'. */
static int read_header_page(int serial_file_descriptor, unsigned char *page_buffer)
{
    if (wake_logger_and_reset_page_pointer(serial_file_descriptor) != 0) return -1;
    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, 'E') != 0) return -1;
    size_t page_bytes_received = read_exact_byte_count(serial_file_descriptor, page_buffer,
                                                       MEMORY_PAGE_SIZE_bytes,
                                                       REPLY_BYTE_TIMEOUT_ms);
    if (page_bytes_received != MEMORY_PAGE_SIZE_bytes) {
        fprintf(stderr, "ERROR: header page short read (%zu bytes)\n", page_bytes_received);
        return -1;
    }
    /* The page is followed by a 0xFF protocol terminator. */
    int terminator_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (terminator_value != 0xFF) {
        fprintf(stderr, "ERROR: header page not followed by 0xFF (got %d)\n", terminator_value);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Header editing                                                      */
/* ------------------------------------------------------------------ */

static unsigned long read_big_endian_unsigned(const unsigned char *source_bytes, int byte_count)
{
    unsigned long assembled_value = 0;
    for (int byte_index = 0; byte_index < byte_count; ++byte_index) {
        assembled_value = (assembled_value << 8) | source_bytes[byte_index];
    }
    return assembled_value;
}

static void write_big_endian_unsigned(unsigned char *destination_bytes, int byte_count,
                                      unsigned long value_to_store)
{
    for (int byte_index = byte_count - 1; byte_index >= 0; --byte_index) {
        destination_bytes[byte_index] = (unsigned char)(value_to_store & 0xFFu);
        value_to_store >>= 8;
    }
}

static void print_launch_block_summary(const char *label, const unsigned char *header_bytes)
{
    char launch_time_string[64];
    format_seconds_since_1980(
        read_big_endian_unsigned(header_bytes + HEADER_LAUNCH_TIME_OFFSET_bytes, 4),
        launch_time_string, sizeof launch_time_string);
    unsigned long interval_half_seconds =
        (0x10000UL - read_big_endian_unsigned(header_bytes + HEADER_INTERVAL_OFFSET_bytes, 2))
        & 0xFFFFUL;
    unsigned int launch_flags = header_bytes[HEADER_LAUNCH_FLAGS_OFFSET_bytes];
    char description_string[HEADER_DESCRIPTION_LENGTH_bytes + 1];
    memcpy(description_string, header_bytes + HEADER_DESCRIPTION_OFFSET_bytes,
           HEADER_DESCRIPTION_LENGTH_bytes);
    description_string[HEADER_DESCRIPTION_LENGTH_bytes] = '\0';

    printf("%s\n", label);
    printf("  launch time:  %s\n", launch_time_string);
    printf("  interval:     %.1f s (%lu half-seconds)\n",
           0.5 * (double)interval_half_seconds, interval_half_seconds);
    printf("  flags 0xC1:   0x%02X  (channels:", launch_flags);
    for (int channel_index = 0; channel_index < 4; ++channel_index) {
        if (launch_flags & (1u << channel_index)) printf(" %d", channel_index + 1);
    }
    printf(")\n");
    printf("  description:  \"%s\"\n", description_string);
}

/* ------------------------------------------------------------------ */
/* Help text                                                           */
/* ------------------------------------------------------------------ */

static void print_usage(FILE *output_stream, const char *program_name)
{
    fprintf(output_stream,
"Usage: %s -i INTERVAL_S -c CHANNELS [options]\n"
"\n"
"Launch (start a new deployment on) an Onset H8 logger, e.g. the\n"
"H08-006-04 4-channel external logger, over a serial port at 1200 baud.\n"
"\n"
"By default this is a DRY RUN: the program reads the logger's header page,\n"
"saves a backup copy of it (h8_header_<serial>_<unixtime>.bin in the\n"
"current directory), and prints which header bytes it WOULD change.\n"
"Nothing is written to the logger unless -C is given.\n"
"\n"
"Launching starts logging from the beginning of memory and overwrites the\n"
"previous deployment. Read the old data out first (h8_read) if needed.\n"
"\n"
"Required:\n"
"  -i, --interval=SECONDS   Sampling interval in seconds. Must be a multiple\n"
"                           of 0.5 s, from 0.5 to 32767.5 (about 9.1 hours).\n"
"  -c, --channels=LIST      Channels to record, 1 to 4, comma separated,\n"
"                           e.g. 1   or 1,2   or 1,2,3,4\n"
"\n"
"Options:\n"
"  -t, --description=TEXT   Text stored in the logger with the deployment,\n"
"                           printable ASCII, at most 40 characters.\n"
"                           Default: \"%s\"\n"
"  -U, --utc                Store the launch time as UTC. Default is local\n"
"                           STANDARD time (no daylight saving), which is the\n"
"                           local-standard-time convention, so during daylight saving\n"
"                           the stored time is one hour behind wall clock.\n"
"  -C, --commit             Actually launch the logger. Without this flag\n"
"                           the program only reports what it would do.\n"
"  -R, --readback           After launching, read the header page back and\n"
"                           compare it. OFF by default: that readout would\n"
"                           wake the new deployment with a break and may stop it.\n"
"  -d, --device=PATH        Serial device. Default: %s\n"
"  -h, --help               Show this help and exit.\n"
"\n"
"Examples:\n"
"  %s -i 5 -c 1 -t \"AA test 5s ch1\"        (dry run)\n"
"  %s -i 5 -c 1 -t \"AA test 5s ch1\" -C     (launch)\n"
"\n"
"A running logger ignores commands until it receives a serial break; both\n"
"this program and h8_read send a 3 s break automatically when needed.\n"
"A deployment ends when memory is full (unless set to wrap) or when the\n"
"logger is read out with h8_read.\n",
            program_name, DEFAULT_DESCRIPTION_TEXT, DEFAULT_SERIAL_DEVICE_PATH,
            program_name, program_name);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argument_count, char **argument_values)
{
    const char *serial_device_path = DEFAULT_SERIAL_DEVICE_PATH;
    const char *description_text   = DEFAULT_DESCRIPTION_TEXT;
    const char *channel_list_text  = NULL;
    double sampling_interval_s     = -1.0;
    int use_utc_flag               = 0;
    int commit_flag                = 0;
    int readback_check_flag        = 0; /* -R: read page 0 after F (may STOP logging) */

    static const struct option long_option_table[] = {
        { "interval",    required_argument, NULL, 'i' },
        { "channels",    required_argument, NULL, 'c' },
        { "description", required_argument, NULL, 't' },
        { "device",      required_argument, NULL, 'd' },
        { "utc",         no_argument,       NULL, 'U' },
        { "commit",      no_argument,       NULL, 'C' },
        { "readback",    no_argument,       NULL, 'R' },
        { "help",        no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };
    int option_character;
    while ((option_character = getopt_long(argument_count, argument_values, "i:c:t:d:UCRh",
                                           long_option_table, NULL)) != -1) {
        switch (option_character) {
        case 'i': sampling_interval_s = strtod(optarg, NULL); break;
        case 'c': channel_list_text   = optarg;               break;
        case 't': description_text    = optarg;               break;
        case 'd': serial_device_path  = optarg;               break;
        case 'U': use_utc_flag        = 1;                    break;
        case 'C': commit_flag         = 1;                    break;
        case 'R': readback_check_flag = 1;                    break;
        case 'h': print_usage(stdout, argument_values[0]);    return 0;
        default:
            fprintf(stderr, "Try '%s --help' for a full description of the options.\n",
                    argument_values[0]);
            return 2;
        }
    }

    /* ---- Validate arguments ---- */
    if (sampling_interval_s <= 0.0 || channel_list_text == NULL) {
        fprintf(stderr, "ERROR: -i INTERVAL_S and -c CHANNELS are required\n\n");
        print_usage(stderr, argument_values[0]);
        return 2;
    }
    double interval_half_seconds_real = sampling_interval_s * 2.0;
    unsigned long interval_half_seconds = (unsigned long)(interval_half_seconds_real + 0.5);
    if (interval_half_seconds < 1UL || interval_half_seconds > 65535UL ||
        interval_half_seconds_real - (double)interval_half_seconds > 1e-6 ||
        (double)interval_half_seconds - interval_half_seconds_real > 1e-6) {
        fprintf(stderr, "ERROR: interval must be a multiple of 0.5 s between 0.5 and 32767.5\n");
        return 2;
    }

    unsigned int channel_enable_bits = 0;
    for (const char *list_character = channel_list_text; *list_character != '\0';
         ++list_character) {
        if (*list_character >= '1' && *list_character <= '4') {
            channel_enable_bits |= 1u << (*list_character - '1');
        } else if (*list_character != ',' && *list_character != ' ') {
            fprintf(stderr, "ERROR: channel list must contain only 1-4 and commas\n");
            return 2;
        }
    }
    if (channel_enable_bits == 0) {
        fprintf(stderr, "ERROR: no channels selected\n");
        return 2;
    }

    size_t description_length_bytes = strlen(description_text);
    if (description_length_bytes > HEADER_DESCRIPTION_LENGTH_bytes - 1) {
        fprintf(stderr, "ERROR: description longer than %d characters\n",
                HEADER_DESCRIPTION_LENGTH_bytes - 1);
        return 2;
    }
    for (size_t character_index = 0; character_index < description_length_bytes;
         ++character_index) {
        unsigned char description_character = (unsigned char)description_text[character_index];
        if (description_character < 0x20 || description_character > 0x7E) {
            fprintf(stderr, "ERROR: description must be printable ASCII\n");
            return 2;
        }
    }

    /* ---- Read and back up the current header ---- */
    int serial_file_descriptor = open_serial_port_1200_8n1(serial_device_path);
    if (serial_file_descriptor < 0) return 1;

    unsigned char current_header_bytes[MEMORY_PAGE_SIZE_bytes];
    if (read_header_page(serial_file_descriptor, current_header_bytes) != 0) {
        close(serial_file_descriptor);
        return 1;
    }

    unsigned long serial_number = read_big_endian_unsigned(current_header_bytes, 4);
    char backup_path[256];
    snprintf(backup_path, sizeof backup_path, "h8_header_%lu_%ld.bin",
             serial_number, (long)time(NULL));
    FILE *backup_file = fopen(backup_path, "wb");
    if (backup_file == NULL ||
        fwrite(current_header_bytes, 1, MEMORY_PAGE_SIZE_bytes, backup_file)
            != MEMORY_PAGE_SIZE_bytes) {
        fprintf(stderr, "ERROR: cannot write header backup %s\n", backup_path);
        close(serial_file_descriptor);
        return 1;
    }
    fclose(backup_file);
    printf("Serial number %lu; header page backed up to %s\n", serial_number, backup_path);

    /* ---- Sanity checks on the current header ---- */
    unsigned int computed_header_checksum = 0;
    for (int header_offset = 0; header_offset < HEADER_CHECKSUM_OFFSET_bytes; ++header_offset) {
        computed_header_checksum += current_header_bytes[header_offset];
    }
    computed_header_checksum &= 0xFFFFu;
    unsigned int stored_header_checksum = (unsigned int)
        read_big_endian_unsigned(current_header_bytes + HEADER_CHECKSUM_OFFSET_bytes, 2);
    if (stored_header_checksum != computed_header_checksum ||
        current_header_bytes[HEADER_FAMILY_CODE_OFFSET_bytes] != H8_FAMILY_CODE) {
        fprintf(stderr, "ERROR: header checksum or family code wrong; refusing to launch\n");
        close(serial_file_descriptor);
        return 1;
    }

    /* ---- Build the new header (only the launch block changes) ---- */
    unsigned char new_header_bytes[MEMORY_PAGE_SIZE_bytes];
    memcpy(new_header_bytes, current_header_bytes, MEMORY_PAGE_SIZE_bytes);

    /* Launch time = when 'F' is expected to go out (transfer takes ~6 s). */
    time_t planned_launch_wall_clock_s = time(NULL) + LAUNCH_TRANSFER_ALLOWANCE_s;
    write_big_endian_unsigned(new_header_bytes + HEADER_LAUNCH_TIME_OFFSET_bytes, 4,
                              seconds_since_1980_for_time(planned_launch_wall_clock_s,
                                                          use_utc_flag));
    new_header_bytes[HEADER_LAUNCH_FLAGS_OFFSET_bytes] = (unsigned char)
        ((current_header_bytes[HEADER_LAUNCH_FLAGS_OFFSET_bytes] & 0xF0u) | channel_enable_bits);
    write_big_endian_unsigned(new_header_bytes + HEADER_INTERVAL_OFFSET_bytes, 2,
                              (0x10000UL - interval_half_seconds) & 0xFFFFUL);
    memset(new_header_bytes + HEADER_DESCRIPTION_OFFSET_bytes, 0,
           HEADER_DESCRIPTION_LENGTH_bytes);
    memcpy(new_header_bytes + HEADER_DESCRIPTION_OFFSET_bytes, description_text,
           description_length_bytes);

    print_launch_block_summary("Current deployment:", current_header_bytes);
    print_launch_block_summary(commit_flag ? "New deployment (launch time re-stamped at send):" :
                               "New deployment (DRY RUN, nothing written):",
                               new_header_bytes);
    printf("  launch time is %s\n", use_utc_flag ? "UTC" : "local standard time");

    printf("Launch block bytes that change (offset: old -> new):\n");
    int changed_byte_count = 0;
    for (int header_offset = LAUNCH_BLOCK_START_OFFSET_bytes;
         header_offset < LAUNCH_BLOCK_START_OFFSET_bytes + LAUNCH_BLOCK_LENGTH_bytes;
         ++header_offset) {
        if (current_header_bytes[header_offset] != new_header_bytes[header_offset]) {
            printf("  0x%02X: %02X -> %02X\n", header_offset,
                   current_header_bytes[header_offset], new_header_bytes[header_offset]);
            ++changed_byte_count;
        }
    }
    printf("  (%d bytes change)\n", changed_byte_count);

    if (!commit_flag) {
        printf("Dry run only. Add -C to launch. A launch restarts recording at the start\n"
               "of memory, overwriting the current deployment's data.\n");
        close(serial_file_descriptor);
        return 0;
    }

    /* ---- Launch ----
     * Refresh the launch time immediately before sending, then send
     * D, B, count, 58 bytes (each echo-checked), F. */
    /* Stamp the time at which 'F' is expected to go out, not the time the
     * 58-byte transfer begins. */
    time_t launch_time_written_wall_clock_s = time(NULL) + LAUNCH_TRANSFER_ALLOWANCE_s;
    write_big_endian_unsigned(new_header_bytes + HEADER_LAUNCH_TIME_OFFSET_bytes, 4,
                              seconds_since_1980_for_time(launch_time_written_wall_clock_s,
                                                          use_utc_flag));

    if (wake_logger_and_reset_page_pointer(serial_file_descriptor) != 0 ||
        send_command_and_check_acknowledgement(serial_file_descriptor, 'B') != 0 ||
        send_count_byte_and_check(serial_file_descriptor, LAUNCH_BLOCK_LENGTH_bytes) != 0) {
        fprintf(stderr, "Launch aborted before any header bytes were sent.\n");
        close(serial_file_descriptor);
        return 1;
    }
    for (int header_offset = LAUNCH_BLOCK_START_OFFSET_bytes;
         header_offset < LAUNCH_BLOCK_START_OFFSET_bytes + LAUNCH_BLOCK_LENGTH_bytes;
         ++header_offset) {
        if (send_header_byte_and_verify_echo(serial_file_descriptor,
                                             new_header_bytes[header_offset],
                                             header_offset) != 0) {
            fprintf(stderr, "Launch aborted part way. Re-run to launch again; the backup "
                            "header is in %s\n", backup_path);
            close(serial_file_descriptor);
            return 1;
        }
        fprintf(stderr, "\rsent %2d/%d header bytes",
                header_offset - LAUNCH_BLOCK_START_OFFSET_bytes + 1, LAUNCH_BLOCK_LENGTH_bytes);
    }
    fprintf(stderr, "\n");

    /* 'F' finishes the launch. Print whatever the logger sends back. */
    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, 'F') != 0) {
        close(serial_file_descriptor);
        return 1;
    }
    time_t finish_sent_wall_clock_s = time(NULL);
    printf("Reply to 'F':");
    int finish_reply_byte_count = 0;
    for (;;) {
        int finish_reply_value = read_single_byte(serial_file_descriptor, FINISH_REPLY_WINDOW_ms);
        if (finish_reply_value < 0) break;
        printf(" %02X", (unsigned int)finish_reply_value);
        if (++finish_reply_byte_count >= 16) break;
    }
    printf("%s\n", finish_reply_byte_count == 0 ? " (none)" : "");
    printf("Launched. 'F' went out %+ld s relative to the header launch time.\n",
           (long)(finish_sent_wall_clock_s - launch_time_written_wall_clock_s));
    printf("To end the deployment, read it out with h8_read: it wakes the\n"
           "running logger with a serial break, and the readout stops logging.\n");

    /* ---- Read back page 0 and compare the launch block ---- */
    if (readback_check_flag) {
        unsigned char readback_header_bytes[MEMORY_PAGE_SIZE_bytes];
        sleep_for_milliseconds(500);
        if (read_header_page(serial_file_descriptor, readback_header_bytes) != 0) {
            fprintf(stderr, "WARNING: could not read back the header after launching\n");
        } else {
            int mismatch_count = 0;
            for (int header_offset = LAUNCH_BLOCK_START_OFFSET_bytes;
                 header_offset < LAUNCH_BLOCK_START_OFFSET_bytes + LAUNCH_BLOCK_LENGTH_bytes;
                 ++header_offset) {
                if (readback_header_bytes[header_offset] != new_header_bytes[header_offset]) {
                    printf("  read-back mismatch at 0x%02X: wrote %02X, read %02X\n",
                           header_offset, new_header_bytes[header_offset],
                           readback_header_bytes[header_offset]);
                    ++mismatch_count;
                }
            }
            printf("Read-back check: %s\n", mismatch_count == 0 ? "launch block matches" :
                   "MISMATCHES above (the logger may adjust some fields itself)");
        }
    }

    close(serial_file_descriptor);
    return 0;
}
