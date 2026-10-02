/*
 * h8_protocol.c
 *
 * Shared protocol library for Onset H08 family loggers: analog
 * loggers (family 8) and the H07 H07 event logger (family 7). See
 * h8_protocol.h for the interface and h8_protocol_notes.txt for the
 * protocol itself.
 *
 * Summary of the protocol used here:
 *   1200 baud, 8N1, no flow control, DTR and RTS asserted.
 *   'D' -> 00 FF        wake / reset the page pointer to page 0
 *   'E' -> 256 bytes FF next memory page (page 0 = header)
 *   'B', count 0x3A, then header bytes 0xBA..0xF3 one at a time (each is
 *   echoed, then FF), then 'F': launch a new deployment.
 *   A running logger ignores everything until it receives a serial break of
 *   about 3 s; waking it that way ends its deployment.
 *   Event logger (H07): same commands, but a 0xA0-byte header and a launch
 *   block of 0x63 bytes (header 0x36..0x98); data are 4-byte time stamps.
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

#define _DEFAULT_SOURCE /* cfmakeraw(), timegm(), timezone under -std=c99 */

#include "h8_protocol.h"

#include <errno.h>
#include <stdint.h>
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
/* Timing and layout constants                                         */
/* ------------------------------------------------------------------ */

#define REPLY_BYTE_TIMEOUT_ms                1000  /* one byte is 8.3 ms at 1200 baud */
#define WAKE_REPLY_TIMEOUT_ms                300   /* timeout for wake-command reply */
#define INTER_COMMAND_DELAY_ms               30    /* conservative command spacing */
#define WAKE_ATTEMPTS_BEFORE_BREAK           3     /* an idle logger answers at once */
#define WAKE_ATTEMPTS_AFTER_BREAK            10
#define WAKE_BREAK_DURATION_ms               3000  /* wakes a running logger */
#define WAKE_POST_BREAK_SETTLE_ms            500   /* logger answers a break with 0xFF bytes */
#define PAGE_READ_RETRY_COUNT_MAXIMUM        3
#define FINISH_REPLY_WINDOW_ms               1500  /* window for post-launch response */
#define LAUNCH_TRANSFER_ALLOWANCE_s          6     /* 58 echoed bytes take ~6 s */
#define LAUNCH_TRANSFER_SECONDS_PER_BYTE     (6.0 / 58.0)  /* same rate, any block length */

#define HEADER_SERIAL_NUMBER_OFFSET_bytes    0x00  /* BE32 */
#define HEADER_CHANNEL_COUNT_OFFSET_bytes    0x05
#define HEADER_FAMILY_CODE_OFFSET_bytes      0x07
#define HEADER_CONFIG_BITS_OFFSET_bytes      0x08  /* bits 4-5: memory size code */
#define HEADER_MODEL_TEXT_OFFSET_bytes       0x09
#define HEADER_CHANNEL_MASK_OFFSET_bytes     0x32
#define HEADER_CHECKSUM_OFFSET_bytes         0xB6  /* BE16 sum of bytes 0x00..0xB5 */
#define HEADER_LAUNCH_COUNT_OFFSET_bytes     0xB8  /* BE16 */
#define HEADER_LAUNCH_TIME_OFFSET_bytes      0xBA  /* BE32 s since 1980 */
#define HEADER_START_DELAY_OFFSET_bytes      0xBE  /* BE24, stored inverted */
#define HEADER_LAUNCH_FLAGS_OFFSET_bytes     0xC1  /* bits 0-3: channels enabled */
#define HEADER_INTERVAL_OFFSET_bytes         0xC2  /* BE16, stored negated, half-seconds */
#define HEADER_DESCRIPTION_OFFSET_bytes      0xC9  /* 41 bytes, NUL terminated */
#define HEADER_DESCRIPTION_LENGTH_bytes      41
#define LAUNCH_BLOCK_START_OFFSET_bytes      0xBA  /* first header byte written by 'B' */
#define LAUNCH_BLOCK_LENGTH_bytes            0x3A  /* 58 bytes: 0xBA..0xF3 */

/* Event logger (family 7, H07 event logger) header layout, inferred during
 * interoperability analysis and checked against S/N 135911. Serial number, family code, config bits and model
 * text are at the same offsets as on the analog loggers. */
#define EVENT_HEADER_CHECKSUM_OFFSET_bytes      0x32  /* BE16 sum of bytes 0x00..0x31 */
#define EVENT_HEADER_LAUNCH_COUNT_OFFSET_bytes  0x34  /* BE16 */
#define EVENT_HEADER_LAUNCH_TIME_OFFSET_bytes   0x36  /* BE32 s since 1980 */
#define EVENT_HEADER_START_DELAY_OFFSET_bytes   0x3A  /* BE24, stored inverted, half-seconds */
#define EVENT_HEADER_LAUNCH_FLAGS_OFFSET_bytes  0x3D  /* bits 6, 7 appear significant; 0x01 seen */
#define EVENT_HEADER_LOCKOUT_OFFSET_bytes       0x3E  /* BE16, stored as 0xFFFF - value */
#define EVENT_HEADER_DESCRIPTION_OFFSET_bytes   0x43  /* 40 chars + NUL at 0x6B */
#define EVENT_HEADER_DESCRIPTION_LENGTH_bytes   41
#define EVENT_HEADER_UNITS_OFFSET_bytes         0x6C  /* 40 chars + NUL at 0x94 */
#define EVENT_HEADER_UNITS_LENGTH_bytes         41
#define EVENT_HEADER_VALUE_PER_EVENT_OFFSET_bytes 0x95 /* BE IEEE 754 single */
#define EVENT_DATA_START_OFFSET_bytes           0xA0  /* launch record, then events */
#define EVENT_RECORD_SIZE_bytes                 4
#define EVENT_TIME_MASK                         0x1FFFFFFFUL /* half-seconds since launch */
#define EVENT_VALUE_BIT                         0x40000000UL /* 1 = event, 0 = launch/stop record */
#define EVENT_END_MARKER_TOP_BYTE_LIMIT         0x80  /* top byte below this ends the data */
#define EVENT_LAUNCH_BLOCK_START_OFFSET_bytes   0x36
#define EVENT_LAUNCH_BLOCK_LENGTH_bytes         0x63  /* 99 bytes: 0x36..0x98 */

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
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

static int report_progress(h8_progress_callback progress_callback, void *progress_user_data,
                           double fraction_complete, const char *status_text)
{
    if (progress_callback == NULL) return 0;
    return progress_callback(fraction_complete, status_text, progress_user_data);
}

/* ------------------------------------------------------------------ */
/* Serial port                                                         */
/* ------------------------------------------------------------------ */

int h8_open_serial_port(const char *serial_device_path, char *error_text_out,
                        size_t error_text_capacity)
{
    int serial_file_descriptor = open(serial_device_path, O_RDWR | O_NOCTTY);
    if (serial_file_descriptor < 0) {
        snprintf(error_text_out, error_text_capacity, "cannot open %s: %s",
                 serial_device_path, strerror(errno));
        return -1;
    }
    struct termios serial_port_attributes;
    if (tcgetattr(serial_file_descriptor, &serial_port_attributes) != 0) {
        snprintf(error_text_out, error_text_capacity, "%s is not a serial port: %s",
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
        snprintf(error_text_out, error_text_capacity, "cannot configure %s: %s",
                 serial_device_path, strerror(errno));
        close(serial_file_descriptor);
        return -1;
    }
    int modem_control_line_bits = TIOCM_DTR | TIOCM_RTS;
    ioctl(serial_file_descriptor, TIOCMBIS, &modem_control_line_bits);
    sleep_for_milliseconds(250);
    tcflush(serial_file_descriptor, TCIOFLUSH);
    return serial_file_descriptor;
}

void h8_close_serial_port(int serial_file_descriptor)
{
    if (serial_file_descriptor >= 0) close(serial_file_descriptor);
}

static int send_single_byte(int serial_file_descriptor, unsigned char byte_to_send)
{
    if (write(serial_file_descriptor, &byte_to_send, 1) != 1) return H8_ERROR_IO;
    tcdrain(serial_file_descriptor);
    return H8_OK;
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
        struct pollfd serial_poll_descriptor = { serial_file_descriptor, POLLIN, 0 };
        int poll_result = poll(&serial_poll_descriptor, 1, (int)per_byte_timeout_ms);
        if (poll_result < 0 && errno == EINTR) continue;
        if (poll_result <= 0) break;
        ssize_t bytes_read_this_call = read(serial_file_descriptor,
                                            destination_buffer + total_bytes_received,
                                            byte_count - total_bytes_received);
        if (bytes_read_this_call < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            break;
        }
        if (bytes_read_this_call == 0) break;
        total_bytes_received += (size_t)bytes_read_this_call;
    }
    return total_bytes_received;
}

/* ------------------------------------------------------------------ */
/* Wake-up                                                             */
/* ------------------------------------------------------------------ */

int h8_try_single_wake_command(int serial_file_descriptor, long reply_timeout_ms)
{
    tcflush(serial_file_descriptor, TCIFLUSH);
    if (send_single_byte(serial_file_descriptor, 'D') != H8_OK) return H8_ERROR_IO;
    int first_reply_value = read_single_byte(serial_file_descriptor, reply_timeout_ms);
    if (first_reply_value == 'D') {
        tcflush(serial_file_descriptor, TCIFLUSH);
        return H8_ERROR_LOOPBACK;
    }
    if (first_reply_value == 0x00) {
        int terminator_value = read_single_byte(serial_file_descriptor, reply_timeout_ms);
        if (terminator_value == 0xFF) return 1;
    }
    tcflush(serial_file_descriptor, TCIFLUSH);
    return 0;
}

void h8_send_serial_break(int serial_file_descriptor, long break_duration_ms)
{
    ioctl(serial_file_descriptor, TIOCSBRK, 0);
    sleep_for_milliseconds(break_duration_ms);
    ioctl(serial_file_descriptor, TIOCCBRK, 0);
    sleep_for_milliseconds(WAKE_POST_BREAK_SETTLE_ms);
    tcflush(serial_file_descriptor, TCIFLUSH);
}

int h8_wake_logger(int serial_file_descriptor, int *logger_was_running_out)
{
    *logger_was_running_out = 0;
    for (int attempt_index = 0; attempt_index < WAKE_ATTEMPTS_BEFORE_BREAK; ++attempt_index) {
        int wake_result = h8_try_single_wake_command(serial_file_descriptor,
                                                     WAKE_REPLY_TIMEOUT_ms);
        if (wake_result == 1) return H8_OK;
        if (wake_result < 0 && wake_result != H8_ERROR_LOOPBACK) return wake_result;
        sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    }
    h8_send_serial_break(serial_file_descriptor, WAKE_BREAK_DURATION_ms);
    for (int attempt_index = 0; attempt_index < WAKE_ATTEMPTS_AFTER_BREAK; ++attempt_index) {
        int wake_result = h8_try_single_wake_command(serial_file_descriptor,
                                                     WAKE_REPLY_TIMEOUT_ms);
        if (wake_result == 1) {
            *logger_was_running_out = 1;
            return H8_OK;
        }
        if (wake_result < 0 && wake_result != H8_ERROR_LOOPBACK) return wake_result;
        sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    }
    return H8_ERROR_NO_LOGGER;
}

/* ------------------------------------------------------------------ */
/* Page reading                                                        */
/* ------------------------------------------------------------------ */

/* 'E' -> 256 bytes then a 0xFF terminator. */
static int read_next_memory_page(int serial_file_descriptor, unsigned char *page_buffer)
{
    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, 'E') != H8_OK) return H8_ERROR_IO;
    size_t page_bytes_received = read_exact_byte_count(serial_file_descriptor, page_buffer,
                                                       H8_MEMORY_PAGE_SIZE_bytes,
                                                       REPLY_BYTE_TIMEOUT_ms);
    if (page_bytes_received != H8_MEMORY_PAGE_SIZE_bytes) return H8_ERROR_IO;
    if (read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms) != 0xFF) {
        return H8_ERROR_IO;
    }
    return H8_OK;
}

/* 'D' resets the page pointer; a few tries in case of a line glitch. */
static int reset_page_pointer(int serial_file_descriptor)
{
    for (int attempt_index = 0; attempt_index < 5; ++attempt_index) {
        if (h8_try_single_wake_command(serial_file_descriptor, WAKE_REPLY_TIMEOUT_ms) == 1) {
            return H8_OK;
        }
        sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    }
    return H8_ERROR_NO_LOGGER;
}

int h8_read_header_page(int serial_file_descriptor, unsigned char *header_page_out)
{
    int reset_result = reset_page_pointer(serial_file_descriptor);
    if (reset_result != H8_OK) return reset_result;
    return read_next_memory_page(serial_file_descriptor, header_page_out);
}

/* ------------------------------------------------------------------ */
/* Header decoding                                                     */
/* ------------------------------------------------------------------ */

static void copy_printable_header_text(const unsigned char *header_page, int start_offset,
                                       int maximum_length_bytes, char *destination_text,
                                       size_t destination_capacity)
{
    size_t output_index = 0;
    for (int header_offset = start_offset;
         header_offset < start_offset + maximum_length_bytes &&
         header_offset < H8_MEMORY_PAGE_SIZE_bytes &&
         output_index + 1 < destination_capacity;
         ++header_offset) {
        unsigned char current_byte = header_page[header_offset];
        if (current_byte == 0x00) break;
        destination_text[output_index++] =
            (current_byte >= 0x20 && current_byte < 0x7F) ? (char)current_byte : '?';
    }
    destination_text[output_index] = '\0';
}

static double read_big_endian_float(const unsigned char *source_bytes)
{
    uint32_t raw_bits = (uint32_t)read_big_endian_unsigned(source_bytes, 4);
    float    decoded_value;
    memcpy(&decoded_value, &raw_bits, sizeof decoded_value);
    return (double)decoded_value;
}

static const unsigned long memory_size_by_code_bytes[4] = {
    8192UL, 16384UL, 32768UL, 65536UL
};

/* Family 7: H07 event logger. */
static int decode_event_header(const unsigned char *header_page, h8_header_info *header_info_out)
{
    header_info_out->logger_kind = H8_LOGGER_KIND_EVENT;
    copy_printable_header_text(header_page, HEADER_MODEL_TEXT_OFFSET_bytes,
                               EVENT_HEADER_CHECKSUM_OFFSET_bytes - HEADER_MODEL_TEXT_OFFSET_bytes,
                               header_info_out->model_text, sizeof header_info_out->model_text);
    copy_printable_header_text(header_page, EVENT_HEADER_DESCRIPTION_OFFSET_bytes,
                               EVENT_HEADER_DESCRIPTION_LENGTH_bytes,
                               header_info_out->description_text,
                               sizeof header_info_out->description_text);
    copy_printable_header_text(header_page, EVENT_HEADER_UNITS_OFFSET_bytes,
                               EVENT_HEADER_UNITS_LENGTH_bytes,
                               header_info_out->event_units_text,
                               sizeof header_info_out->event_units_text);
    header_info_out->event_value_per_event =
        read_big_endian_float(header_page + EVENT_HEADER_VALUE_PER_EVENT_OFFSET_bytes);
    header_info_out->event_lockout_value = 0xFFFFUL
        - read_big_endian_unsigned(header_page + EVENT_HEADER_LOCKOUT_OFFSET_bytes, 2);

    header_info_out->launch_flags = header_page[EVENT_HEADER_LAUNCH_FLAGS_OFFSET_bytes];
    header_info_out->data_start_offset_bytes = EVENT_DATA_START_OFFSET_bytes;
    header_info_out->event_capacity_count =
        (header_info_out->memory_size_bytes - EVENT_DATA_START_OFFSET_bytes)
        / EVENT_RECORD_SIZE_bytes - 1UL;
    header_info_out->launch_count =
        read_big_endian_unsigned(header_page + EVENT_HEADER_LAUNCH_COUNT_OFFSET_bytes, 2);
    header_info_out->launch_time_s_since_1980 =
        read_big_endian_unsigned(header_page + EVENT_HEADER_LAUNCH_TIME_OFFSET_bytes, 4);
    header_info_out->start_delay_half_seconds = 0xFFFFFFUL
        - read_big_endian_unsigned(header_page + EVENT_HEADER_START_DELAY_OFFSET_bytes, 3);

    unsigned int computed_checksum = 0;
    for (int header_offset = 0; header_offset < EVENT_HEADER_CHECKSUM_OFFSET_bytes;
         ++header_offset) {
        computed_checksum += header_page[header_offset];
    }
    unsigned int stored_checksum = (unsigned int)
        read_big_endian_unsigned(header_page + EVENT_HEADER_CHECKSUM_OFFSET_bytes, 2);
    header_info_out->checksum_is_valid = ((computed_checksum & 0xFFFFu) == stored_checksum);
    return header_info_out->checksum_is_valid ? H8_OK : H8_ERROR_BAD_HEADER;
}

int h8_decode_header(const unsigned char *header_page, h8_header_info *header_info_out)
{
    memset(header_info_out, 0, sizeof *header_info_out);

    /* Fields shared by every family. */
    header_info_out->family_code = header_page[HEADER_FAMILY_CODE_OFFSET_bytes];
    header_info_out->serial_number =
        read_big_endian_unsigned(header_page + HEADER_SERIAL_NUMBER_OFFSET_bytes, 4);
    header_info_out->installed_channel_count = header_page[HEADER_CHANNEL_COUNT_OFFSET_bytes];
    header_info_out->memory_size_bytes =
        memory_size_by_code_bytes[(header_page[HEADER_CONFIG_BITS_OFFSET_bytes] >> 4) & 0x3];

    if (header_info_out->family_code == H8_EVENT_FAMILY_CODE) {
        return decode_event_header(header_page, header_info_out);
    }

    header_info_out->logger_kind = H8_LOGGER_KIND_ANALOG;

    copy_printable_header_text(header_page, HEADER_MODEL_TEXT_OFFSET_bytes, 60,
                               header_info_out->model_text,
                               sizeof header_info_out->model_text);
    copy_printable_header_text(header_page, HEADER_DESCRIPTION_OFFSET_bytes,
                               HEADER_DESCRIPTION_LENGTH_bytes,
                               header_info_out->description_text,
                               sizeof header_info_out->description_text);

    header_info_out->installed_channel_mask  = header_page[HEADER_CHANNEL_MASK_OFFSET_bytes];
    header_info_out->launch_flags            = header_page[HEADER_LAUNCH_FLAGS_OFFSET_bytes];
    header_info_out->enabled_channel_mask    = header_info_out->launch_flags & 0x0Fu;
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        if (header_info_out->enabled_channel_mask & (1u << channel_index)) {
            header_info_out->enabled_channel_count++;
        }
    }

    header_info_out->data_start_offset_bytes =
        (header_info_out->memory_size_bytes == 16384UL ||
         header_info_out->memory_size_bytes == 65536UL) ? 0xF4 : 0xF8;

    header_info_out->launch_count =
        read_big_endian_unsigned(header_page + HEADER_LAUNCH_COUNT_OFFSET_bytes, 2);
    header_info_out->launch_time_s_since_1980 =
        read_big_endian_unsigned(header_page + HEADER_LAUNCH_TIME_OFFSET_bytes, 4);
    header_info_out->interval_half_seconds =
        (0x10000UL - read_big_endian_unsigned(header_page + HEADER_INTERVAL_OFFSET_bytes, 2))
        & 0xFFFFUL;
    header_info_out->start_delay_half_seconds =
        0xFFFFFFUL - read_big_endian_unsigned(header_page + HEADER_START_DELAY_OFFSET_bytes, 3);

    unsigned int computed_checksum = 0;
    for (int header_offset = 0; header_offset < HEADER_CHECKSUM_OFFSET_bytes; ++header_offset) {
        computed_checksum += header_page[header_offset];
    }
    unsigned int stored_checksum = (unsigned int)
        read_big_endian_unsigned(header_page + HEADER_CHECKSUM_OFFSET_bytes, 2);
    header_info_out->checksum_is_valid = ((computed_checksum & 0xFFFFu) == stored_checksum);

    if (header_info_out->family_code != H8_FAMILY_CODE) return H8_ERROR_UNSUPPORTED_FAMILY;
    if (!header_info_out->checksum_is_valid) return H8_ERROR_BAD_HEADER;
    return H8_OK;
}

/* ------------------------------------------------------------------ */
/* Offload                                                             */
/* ------------------------------------------------------------------ */

h8_data_extent h8_find_data_extent(const unsigned char *image_bytes, size_t image_size_bytes,
                                   const h8_header_info *header_info)
{
    h8_data_extent data_extent;
    memset(&data_extent, 0, sizeof data_extent);

    if (header_info->logger_kind == H8_LOGGER_KIND_EVENT) {
        size_t data_end_bytes = image_size_bytes;
        if (data_end_bytes > header_info->memory_size_bytes) {
            data_end_bytes = header_info->memory_size_bytes;
        }
        unsigned long previous_time_half_seconds = 0;
        for (size_t record_offset = EVENT_DATA_START_OFFSET_bytes;
             record_offset + EVENT_RECORD_SIZE_bytes <= data_end_bytes;
             record_offset += EVENT_RECORD_SIZE_bytes) {
            unsigned long record_word = read_big_endian_unsigned(image_bytes + record_offset, 4);
            unsigned long record_time_half_seconds = record_word & EVENT_TIME_MASK;
            int word_is_end_marker =
                image_bytes[record_offset] < EVENT_END_MARKER_TOP_BYTE_LIMIT;
            /* Time running backwards means leftovers from an older deployment. */
            int time_runs_backwards = (data_extent.record_count > 0 &&
                                       record_time_half_seconds < previous_time_half_seconds);
            if (word_is_end_marker || time_runs_backwards) {
                data_extent.end_marker_found_flag   = 1;
                data_extent.end_marker_offset_bytes = record_offset;
                return data_extent;
            }
            previous_time_half_seconds = record_time_half_seconds;
            if (record_word & EVENT_VALUE_BIT) {
                data_extent.event_count++;
            } else if (data_extent.record_count > 0) {
                /* A marker word after the launch record: the logger was woken. */
                data_extent.stop_record_found_flag = 1;
            }
            data_extent.record_count++;
        }
        return data_extent;
    }

    size_t record_size_bytes = header_info->enabled_channel_count;
    if (record_size_bytes == 0 || image_size_bytes <= header_info->data_start_offset_bytes) {
        return data_extent;
    }
    size_t data_end_bytes = image_size_bytes;
    if (data_end_bytes > header_info->memory_size_bytes) {
        data_end_bytes = header_info->memory_size_bytes;
    }
    for (size_t record_offset = header_info->data_start_offset_bytes;
         record_offset + record_size_bytes <= data_end_bytes;
         record_offset += record_size_bytes) {
        for (size_t byte_index = 0; byte_index < record_size_bytes; ++byte_index) {
            if (image_bytes[record_offset + byte_index] == 0x00) {
                data_extent.end_marker_found_flag   = 1;
                data_extent.end_marker_offset_bytes = record_offset;
                return data_extent;
            }
        }
        data_extent.record_count++;
    }
    return data_extent;
}

int h8_offload_memory(int serial_file_descriptor, unsigned char *image_out,
                      size_t *image_size_bytes_out, h8_progress_callback progress_callback,
                      void *progress_user_data)
{
    *image_size_bytes_out = 0;
    int reset_result = reset_page_pointer(serial_file_descriptor);
    if (reset_result != H8_OK) return reset_result;

    h8_header_info header_info;
    int memory_page_count = H8_MAXIMUM_MEMORY_PAGE_COUNT;
    char status_text[96];

    for (int page_index = 0; page_index < memory_page_count; ++page_index) {
        unsigned char *page_destination = image_out
                                        + (size_t)page_index * H8_MEMORY_PAGE_SIZE_bytes;
        int page_read_result = H8_ERROR_IO;
        for (int retry_index = 0; retry_index < PAGE_READ_RETRY_COUNT_MAXIMUM; ++retry_index) {
            page_read_result = read_next_memory_page(serial_file_descriptor, page_destination);
            if (page_read_result == H8_OK) break;
            /* Re-sync: reset to page 0 and skip forward to this page. */
            if (reset_page_pointer(serial_file_descriptor) != H8_OK) break;
            unsigned char discarded_page_buffer[H8_MEMORY_PAGE_SIZE_bytes];
            int skip_result = H8_OK;
            for (int skip_index = 0; skip_index < page_index && skip_result == H8_OK;
                 ++skip_index) {
                skip_result = read_next_memory_page(serial_file_descriptor,
                                                    discarded_page_buffer);
            }
            if (skip_result != H8_OK) break;
        }
        if (page_read_result != H8_OK) return page_read_result;
        *image_size_bytes_out = (size_t)(page_index + 1) * H8_MEMORY_PAGE_SIZE_bytes;

        if (page_index == 0) {
            int decode_result = h8_decode_header(image_out, &header_info);
            if (decode_result != H8_OK) return decode_result;
            memory_page_count = (int)(header_info.memory_size_bytes / H8_MEMORY_PAGE_SIZE_bytes);
        }

        /* Stop as soon as the page holding the end marker has been read. */
        h8_data_extent data_extent = h8_find_data_extent(image_out, *image_size_bytes_out,
                                                         &header_info);
        snprintf(status_text, sizeof status_text, "Read page %d (%zu records so far)",
                 page_index + 1, data_extent.record_count);
        if (data_extent.end_marker_found_flag) {
            report_progress(progress_callback, progress_user_data, 1.0, status_text);
            return H8_OK;
        }
        if (report_progress(progress_callback, progress_user_data,
                            (double)(page_index + 1) / (double)memory_page_count,
                            status_text)) {
            return H8_ERROR_CANCELLED;
        }
    }
    return H8_OK;
}

/* ------------------------------------------------------------------ */
/* Launch                                                              */
/* ------------------------------------------------------------------ */

/* Command byte acknowledged as FF, (0xFF - command) FF, or 00 FF. */
static int send_command_and_check_acknowledgement(int serial_file_descriptor,
                                                  unsigned char command_byte)
{
    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, command_byte) != H8_OK) return H8_ERROR_IO;
    int first_reply_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (first_reply_value < 0) return H8_ERROR_IO;
    if (first_reply_value == 0xFF) return H8_OK;
    if (first_reply_value == command_byte) return H8_ERROR_LOOPBACK;
    int second_reply_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if ((first_reply_value == (0xFF - command_byte) || first_reply_value == 0x00) &&
        second_reply_value == 0xFF) {
        return H8_OK;
    }
    return H8_ERROR_IO;
}

/* Count byte after 'B': FF alone, or echo then FF. */
static int send_count_byte_and_check(int serial_file_descriptor, unsigned char count_byte)
{
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, count_byte) != H8_OK) return H8_ERROR_IO;
    int first_reply_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (first_reply_value == 0xFF) return H8_OK;
    if (first_reply_value == count_byte &&
        read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms) == 0xFF) {
        return H8_OK;
    }
    return H8_ERROR_ECHO_MISMATCH;
}

/* Header data byte: the logger echoes it, then FF. */
static int send_header_byte_and_verify_echo(int serial_file_descriptor, unsigned char data_byte)
{
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, data_byte) != H8_OK) return H8_ERROR_IO;
    int echoed_value     = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    int terminator_value = read_single_byte(serial_file_descriptor, REPLY_BYTE_TIMEOUT_ms);
    if (echoed_value != data_byte || terminator_value != 0xFF) return H8_ERROR_ECHO_MISMATCH;
    return H8_OK;
}

/* 'D', 'B', count, then launch_block_length_bytes header bytes starting at
 * launch_block_start_offset_bytes (each echoed, then FF), then 'F'.
 * Identical for analog and event loggers apart from the block. */
static int send_launch_block(int serial_file_descriptor, const unsigned char *new_header_page,
                             int launch_block_start_offset_bytes, int launch_block_length_bytes,
                             h8_progress_callback progress_callback, void *progress_user_data)
{
    report_progress(progress_callback, progress_user_data, 0.0, "Starting launch");
    int step_result = reset_page_pointer(serial_file_descriptor);
    if (step_result != H8_OK) return step_result;
    step_result = send_command_and_check_acknowledgement(serial_file_descriptor, 'B');
    if (step_result != H8_OK) return step_result;
    step_result = send_count_byte_and_check(serial_file_descriptor,
                                            (unsigned char)launch_block_length_bytes);
    if (step_result != H8_OK) return step_result;

    char status_text[64];
    for (int header_offset = launch_block_start_offset_bytes;
         header_offset < launch_block_start_offset_bytes + launch_block_length_bytes;
         ++header_offset) {
        step_result = send_header_byte_and_verify_echo(serial_file_descriptor,
                                                       new_header_page[header_offset]);
        if (step_result != H8_OK) return step_result;
        int bytes_sent_count = header_offset - launch_block_start_offset_bytes + 1;
        snprintf(status_text, sizeof status_text, "Sent %d of %d header bytes",
                 bytes_sent_count, launch_block_length_bytes);
        report_progress(progress_callback, progress_user_data,
                        0.95 * (double)bytes_sent_count / (double)launch_block_length_bytes,
                        status_text);
    }

    tcflush(serial_file_descriptor, TCIFLUSH);
    sleep_for_milliseconds(INTER_COMMAND_DELAY_ms);
    if (send_single_byte(serial_file_descriptor, 'F') != H8_OK) return H8_ERROR_IO;
    /* The logger replies FF; collect and discard whatever arrives. */
    for (int reply_byte_index = 0; reply_byte_index < 16; ++reply_byte_index) {
        if (read_single_byte(serial_file_descriptor, FINISH_REPLY_WINDOW_ms) < 0) break;
    }
    report_progress(progress_callback, progress_user_data, 1.0, "Launched");
    return H8_OK;
}

static int description_text_is_valid(const char *description_text)
{
    size_t description_length_bytes = strlen(description_text);
    if (description_length_bytes > H8_DESCRIPTION_CHARACTERS_MAXIMUM) return 0;
    for (size_t character_index = 0; character_index < description_length_bytes;
         ++character_index) {
        unsigned char description_character = (unsigned char)description_text[character_index];
        if (description_character < 0x20 || description_character > 0x7E) return 0;
    }
    return 1;
}

/* Event logger launch: new launch time, no start delay, new description;
 * units label, value per event, lockout and flags are kept. */
static int launch_event_logger(int serial_file_descriptor, const unsigned char *current_header_page,
                               const char *description_text, int use_utc_flag,
                               h8_progress_callback progress_callback, void *progress_user_data)
{
    unsigned char new_header_page[H8_MEMORY_PAGE_SIZE_bytes];
    memcpy(new_header_page, current_header_page, H8_MEMORY_PAGE_SIZE_bytes);
    write_big_endian_unsigned(new_header_page + EVENT_HEADER_START_DELAY_OFFSET_bytes, 3,
                              0xFFFFFFUL); /* inverted 0 = start now */
    memset(new_header_page + EVENT_HEADER_DESCRIPTION_OFFSET_bytes, 0,
           EVENT_HEADER_DESCRIPTION_LENGTH_bytes);
    memcpy(new_header_page + EVENT_HEADER_DESCRIPTION_OFFSET_bytes, description_text,
           strlen(description_text));
    /* Launch time = when 'F' is expected to go out: the time bytes are the
     * first sent, and the whole block takes about 0.1 s per byte. */
    long transfer_allowance_s = (long)(LAUNCH_TRANSFER_SECONDS_PER_BYTE
                                       * (double)EVENT_LAUNCH_BLOCK_LENGTH_bytes + 0.5);
    write_big_endian_unsigned(new_header_page + EVENT_HEADER_LAUNCH_TIME_OFFSET_bytes, 4,
                              h8_seconds_since_1980_for_time(
                                  time(NULL) + transfer_allowance_s, use_utc_flag));
    return send_launch_block(serial_file_descriptor, new_header_page,
                             EVENT_LAUNCH_BLOCK_START_OFFSET_bytes,
                             EVENT_LAUNCH_BLOCK_LENGTH_bytes,
                             progress_callback, progress_user_data);
}

int h8_launch_logger(int serial_file_descriptor, const unsigned char *current_header_page,
                     unsigned long interval_half_seconds, unsigned int channel_mask,
                     const char *description_text, int use_utc_flag,
                     h8_progress_callback progress_callback, void *progress_user_data)
{
    /* ---- Validate ---- */
    if (!description_text_is_valid(description_text)) return H8_ERROR_ARGUMENT;
    h8_header_info current_header_info;
    int decode_result = h8_decode_header(current_header_page, &current_header_info);
    if (decode_result != H8_OK) return decode_result;
    if (current_header_info.logger_kind == H8_LOGGER_KIND_EVENT) {
        return launch_event_logger(serial_file_descriptor, current_header_page,
                                   description_text, use_utc_flag,
                                   progress_callback, progress_user_data);
    }
    if (interval_half_seconds < H8_INTERVAL_HALF_SECONDS_MINIMUM ||
        interval_half_seconds > H8_INTERVAL_HALF_SECONDS_MAXIMUM ||
        (channel_mask & 0x0Fu) == 0 || (channel_mask & ~0x0Fu) != 0) {
        return H8_ERROR_ARGUMENT;
    }
    size_t description_length_bytes = strlen(description_text);

    /* ---- Build the new launch block; everything else is copied ---- */
    unsigned char new_header_page[H8_MEMORY_PAGE_SIZE_bytes];
    memcpy(new_header_page, current_header_page, H8_MEMORY_PAGE_SIZE_bytes);
    new_header_page[HEADER_LAUNCH_FLAGS_OFFSET_bytes] = (unsigned char)
        ((current_header_page[HEADER_LAUNCH_FLAGS_OFFSET_bytes] & 0xF0u) | channel_mask);
    write_big_endian_unsigned(new_header_page + HEADER_INTERVAL_OFFSET_bytes, 2,
                              (0x10000UL - interval_half_seconds) & 0xFFFFUL);
    memset(new_header_page + HEADER_DESCRIPTION_OFFSET_bytes, 0,
           HEADER_DESCRIPTION_LENGTH_bytes);
    memcpy(new_header_page + HEADER_DESCRIPTION_OFFSET_bytes, description_text,
           description_length_bytes);
    /* Launch time = when 'F' is expected to go out. */
    write_big_endian_unsigned(new_header_page + HEADER_LAUNCH_TIME_OFFSET_bytes, 4,
                              h8_seconds_since_1980_for_time(
                                  time(NULL) + LAUNCH_TRANSFER_ALLOWANCE_s, use_utc_flag));

    /* ---- D, B, count, 58 echoed bytes, F ---- */
    return send_launch_block(serial_file_descriptor, new_header_page,
                             LAUNCH_BLOCK_START_OFFSET_bytes, LAUNCH_BLOCK_LENGTH_bytes,
                             progress_callback, progress_user_data);
}

/* ------------------------------------------------------------------ */
/* Output files                                                        */
/* ------------------------------------------------------------------ */

int h8_write_binary_image(const char *output_path, const unsigned char *image_bytes,
                          size_t image_size_bytes)
{
    FILE *binary_output_file = fopen(output_path, "wb");
    if (binary_output_file == NULL) return H8_ERROR_FILE;
    size_t bytes_written = fwrite(image_bytes, 1, image_size_bytes, binary_output_file);
    int close_result = fclose(binary_output_file);
    return (bytes_written == image_size_bytes && close_result == 0) ? H8_OK : H8_ERROR_FILE;
}

/* One line per record: the launch record (time 0) then one per event. */
static int write_event_csv(const char *output_path, const unsigned char *image_bytes,
                           size_t image_size_bytes, const h8_header_info *header_info,
                           size_t *records_written_out)
{
    FILE *csv_output_file = fopen(output_path, "w");
    if (csv_output_file == NULL) return H8_ERROR_FILE;
    fprintf(csv_output_file, "record_index,elapsed_time_s,local_standard_time,record_type,"
                             "cumulative_events,cumulative_value\n");
    h8_data_extent data_extent = h8_find_data_extent(image_bytes, image_size_bytes,
                                                     header_info);
    unsigned long cumulative_event_count = 0;
    for (size_t record_index = 0; record_index < data_extent.record_count; ++record_index) {
        unsigned long record_word = read_big_endian_unsigned(
            image_bytes + EVENT_DATA_START_OFFSET_bytes + record_index * EVENT_RECORD_SIZE_bytes,
            4);
        unsigned long elapsed_half_seconds = record_word & EVENT_TIME_MASK;
        /* record_type: 0 = launch record, 1 = event, 2 = stop record */
        int event_flag  = (record_word & EVENT_VALUE_BIT) ? 1 : 0;
        int record_type = event_flag ? 1 : (record_index == 0 ? 0 : 2);
        cumulative_event_count += (unsigned long)event_flag;
        char record_time_text[32];
        h8_format_time_since_1980(header_info->launch_time_s_since_1980
                                  + elapsed_half_seconds / 2UL,
                                  record_time_text, sizeof record_time_text);
        fprintf(csv_output_file, "%zu,%.1f,%s%s,%d,%lu,%.6g\n", record_index,
                0.5 * (double)elapsed_half_seconds, record_time_text,
                (elapsed_half_seconds % 2UL) ? ".5" : "", record_type, cumulative_event_count,
                (double)cumulative_event_count * header_info->event_value_per_event);
    }
    *records_written_out = data_extent.record_count;
    return (fclose(csv_output_file) == 0) ? H8_OK : H8_ERROR_FILE;
}

int h8_write_csv(const char *output_path, const unsigned char *image_bytes,
                 size_t image_size_bytes, const h8_header_info *header_info,
                 double volts_per_count, double volts_offset, size_t *records_written_out)
{
    *records_written_out = 0;
    if (header_info->logger_kind == H8_LOGGER_KIND_EVENT) {
        return write_event_csv(output_path, image_bytes, image_size_bytes, header_info,
                               records_written_out);
    }
    FILE *csv_output_file = fopen(output_path, "w");
    if (csv_output_file == NULL) return H8_ERROR_FILE;

    int channel_numbers[H8_CHANNEL_COUNT_MAXIMUM];
    int channel_count = 0;
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        if (header_info->enabled_channel_mask & (1u << channel_index)) {
            channel_numbers[channel_count++] = channel_index + 1;
        }
    }

    fprintf(csv_output_file, "record_index,elapsed_time_s,local_standard_time");
    for (int column_index = 0; column_index < channel_count; ++column_index) {
        fprintf(csv_output_file, ",ch%d_counts", channel_numbers[column_index]);
    }
    for (int column_index = 0; column_index < channel_count; ++column_index) {
        fprintf(csv_output_file, ",ch%d_V", channel_numbers[column_index]);
    }
    fprintf(csv_output_file, "\n");

    h8_data_extent data_extent = h8_find_data_extent(image_bytes, image_size_bytes,
                                                     header_info);
    double sampling_interval_s = 0.5 * (double)header_info->interval_half_seconds;
    for (size_t record_index = 0; record_index < data_extent.record_count; ++record_index) {
        const unsigned char *record_bytes = image_bytes + header_info->data_start_offset_bytes
                                          + record_index * (size_t)channel_count;
        unsigned long record_half_seconds = header_info->interval_half_seconds
                                          * (unsigned long)record_index;
        char record_time_text[32];
        h8_format_time_since_1980(header_info->launch_time_s_since_1980
                                  + record_half_seconds / 2UL,
                                  record_time_text, sizeof record_time_text);
        fprintf(csv_output_file, "%zu,%.1f,%s%s", record_index,
                (double)record_index * sampling_interval_s, record_time_text,
                (record_half_seconds % 2UL) ? ".5" : "");
        for (int column_index = 0; column_index < channel_count; ++column_index) {
            fprintf(csv_output_file, ",%u", (unsigned int)record_bytes[column_index]);
        }
        for (int column_index = 0; column_index < channel_count; ++column_index) {
            double channel_voltage_V = (double)record_bytes[column_index] * volts_per_count
                                     + volts_offset;
            fprintf(csv_output_file, ",%.4f", channel_voltage_V);
        }
        fprintf(csv_output_file, "\n");
    }
    *records_written_out = data_extent.record_count;
    return (fclose(csv_output_file) == 0) ? H8_OK : H8_ERROR_FILE;
}

/* ------------------------------------------------------------------ */
/* Time helpers                                                        */
/* ------------------------------------------------------------------ */

static time_t epoch_1980_as_calendar_seconds(void)
{
    struct tm epoch_1980_calendar;
    memset(&epoch_1980_calendar, 0, sizeof epoch_1980_calendar);
    epoch_1980_calendar.tm_year = 80;
    epoch_1980_calendar.tm_mday = 1;
    return timegm(&epoch_1980_calendar);
}

void h8_format_time_since_1980(unsigned long seconds_since_1980, char *text_out,
                               size_t text_capacity)
{
    /* timegm/gmtime are used purely as calendar arithmetic here. */
    time_t calendar_seconds = epoch_1980_as_calendar_seconds() + (time_t)seconds_since_1980;
    struct tm calendar_fields;
    gmtime_r(&calendar_seconds, &calendar_fields);
    strftime(text_out, text_capacity, "%Y-%m-%d %H:%M:%S", &calendar_fields);
}

unsigned long h8_seconds_since_1980_for_time(time_t wall_clock_time_s, int use_utc_flag)
{
    long offset_to_clock_s = 0;
    if (!use_utc_flag) {
        tzset();
        offset_to_clock_s = -timezone; /* 'timezone' = seconds WEST of UTC, standard time */
    }
    return (unsigned long)((wall_clock_time_s + offset_to_clock_s)
                           - epoch_1980_as_calendar_seconds());
}

const char *h8_error_text(int error_code)
{
    switch (error_code) {
    case H8_OK:                  return "OK";
    case H8_ERROR_IO:            return "serial communication error or timeout";
    case H8_ERROR_NO_LOGGER:     return "no logger answered, even after a serial break";
    case H8_ERROR_LOOPBACK:      return "the cable echoed our commands: no logger plugged in";
    case H8_ERROR_BAD_HEADER:    return "logger header checksum is wrong";
    case H8_ERROR_ECHO_MISMATCH: return "logger did not echo a launch byte correctly";
    case H8_ERROR_CANCELLED:     return "cancelled";
    case H8_ERROR_ARGUMENT:      return "invalid launch settings";
    case H8_ERROR_FILE:          return "could not write the output file";
    case H8_ERROR_UNSUPPORTED_FAMILY:
        return "this logger family is not supported (only family 8 analog and "
               "family 7 event loggers)";
    default:                     return "unknown error";
    }
}
