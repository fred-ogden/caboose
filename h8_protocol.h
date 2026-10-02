/*
 * h8_protocol.h
 *
 * Shared protocol library for Onset H07 and H08 data loggers: the analog
 * loggers (family code 8, developed on an H08-006-04) and the H07 event logger (family code 7, H07-001-02), which
 * stores a time stamp at every contact closure. Used by the GTK3 GUI; the
 * protocol is documented in h8_protocol_notes.txt.
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

#ifndef H8_PROTOCOL_H
#define H8_PROTOCOL_H

#include <stddef.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define H8_MEMORY_PAGE_SIZE_bytes             256
#define H8_MAXIMUM_MEMORY_PAGE_COUNT          256    /* 64 KB, largest H8 memory */
#define H8_FAMILY_CODE                        8      /* analog loggers, 1-4 channels */
#define H8_EVENT_FAMILY_CODE                  7      /* H07 event logger, H07 */
#define H8_CHANNEL_COUNT_MAXIMUM              4
#define H8_DESCRIPTION_CHARACTERS_MAXIMUM     40
#define H8_INTERVAL_HALF_SECONDS_MINIMUM      1UL      /* 0.5 s */
#define H8_INTERVAL_HALF_SECONDS_MAXIMUM      65535UL  /* 32767.5 s, about 9.1 h */

/* Calibration constants (measured uing hard ground -> 1 count, 1.65 V -> 166 counts). */
#define H8_DEFAULT_VOLTS_PER_COUNT            0.00994
#define H8_DEFAULT_VOLTS_OFFSET               0.0

/* Return codes */
#define H8_OK                                 0
#define H8_ERROR_IO                           (-1)  /* serial read/write failure or timeout */
#define H8_ERROR_NO_LOGGER                    (-2)  /* nothing answered, even after a break */
#define H8_ERROR_LOOPBACK                     (-3)  /* our own bytes came back: cable without logger */
#define H8_ERROR_BAD_HEADER                   (-4)  /* checksum or family code wrong */
#define H8_ERROR_ECHO_MISMATCH                (-5)  /* launch byte not echoed correctly */
#define H8_ERROR_CANCELLED                    (-6)
#define H8_ERROR_ARGUMENT                     (-7)
#define H8_ERROR_FILE                         (-8)
#define H8_ERROR_UNSUPPORTED_FAMILY           (-9)  /* header valid, but not family 7 or 8 */

/* Which kind of logger a header describes. */
typedef enum {
    H8_LOGGER_KIND_ANALOG = 0,   /* family 8: one byte per channel per sample */
    H8_LOGGER_KIND_EVENT  = 1    /* family 7: one 4-byte time stamp per event */
} h8_logger_kind;

/* Event logger capacity with 8 KB memory: (8192 - 0xA0) / 4 = 2008 words,
 * the first of which is the launch record. */
#define H8_EVENT_UNITS_CHARACTERS_MAXIMUM     40

/* ------------------------------------------------------------------ */
/* Decoded header                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    h8_logger_kind logger_kind;                   /* from the family code */
    unsigned long serial_number;
    char          model_text[64];
    char          description_text[H8_DESCRIPTION_CHARACTERS_MAXIMUM + 1];
    unsigned int  family_code;                    /* 8 for H8 */
    unsigned int  installed_channel_count;        /* header 0x05 */
    unsigned int  installed_channel_mask;         /* header 0x32 */
    unsigned int  launch_flags;                   /* header 0xC1 */
    unsigned int  enabled_channel_mask;           /* bits 0-3 of 0xC1 */
    unsigned int  enabled_channel_count;          /* bytes per record */
    unsigned long memory_size_bytes;
    size_t        data_start_offset_bytes;        /* 0xF8 (8/32 KB) or 0xF4 (16/64 KB) */
    unsigned long launch_count;                   /* header 0xB8 */
    unsigned long launch_time_s_since_1980;       /* header 0xBA, launching PC's clock */
    unsigned long interval_half_seconds;          /* header 0xC2 */
    unsigned long start_delay_half_seconds;       /* header 0xBE (unconfirmed) */
    int           checksum_is_valid;

    /* Event logger only (logger_kind == H8_LOGGER_KIND_EVENT) */
    char          event_units_text[H8_EVENT_UNITS_CHARACTERS_MAXIMUM + 1]; /* header 0x6C */
    double        event_value_per_event;          /* header 0x95, BE IEEE float */
    unsigned long event_lockout_value;            /* header 0x3E, 0xFFFF - BE16; units unconfirmed */
    unsigned long event_capacity_count;           /* event words that fit after the launch record */
} h8_header_info;

/* Result of scanning an offloaded memory image for records. For an event
 * logger a "record" is one 4-byte word, and the count includes the launch
 * record (the word at 0xA0, time 0) and the stop record, if any. */
typedef struct {
    size_t record_count;             /* records before the end marker */
    int    end_marker_found_flag;    /* 1 if the 0x00 end marker was found */
    size_t end_marker_offset_bytes;  /* valid when end_marker_found_flag is 1 */
    size_t event_count;              /* event logger: event records (bit 30 set) */
    int    stop_record_found_flag;   /* event logger: deployment ended with a stop record */
} h8_data_extent;

/* Progress callback: fraction 0..1 and a short status text. Called from
 * whatever thread runs the operation. Return nonzero to cancel. */
typedef int (*h8_progress_callback)(double fraction_complete, const char *status_text,
                                    void *user_data);

/* ------------------------------------------------------------------ */
/* Serial port and wake-up                                             */
/* ------------------------------------------------------------------ */

/* Open and configure the port at 1200 baud 8N1, DTR and RTS asserted.
 * Returns a file descriptor, or -1 (error text in error_text_out). */
int  h8_open_serial_port(const char *serial_device_path, char *error_text_out,
                         size_t error_text_capacity);
void h8_close_serial_port(int serial_file_descriptor);

/* Send 'D' once and wait up to reply_timeout_ms for 00 FF.
 * Returns 1 if the logger answered, 0 if not, H8_ERROR_LOOPBACK if the 'D'
 * came straight back, H8_ERROR_IO on a write failure. */
int  h8_try_single_wake_command(int serial_file_descriptor, long reply_timeout_ms);

/* Hold a serial break for break_duration_ms, wait for the logger's run of
 * 0xFF bytes to finish, and discard it. */
void h8_send_serial_break(int serial_file_descriptor, long break_duration_ms);

/* Full wake: a few plain 'D' tries (an idle logger answers at once); if
 * none answers, a 3000 ms break and more tries. A running logger only
 * answers after the break, and answering ENDS ITS DEPLOYMENT.
 * *logger_was_running_out is set to 1 if the break was needed.
 * Returns H8_OK or a negative error code. */
int  h8_wake_logger(int serial_file_descriptor, int *logger_was_running_out);

/* ------------------------------------------------------------------ */
/* Reading                                                             */
/* ------------------------------------------------------------------ */

/* Read page 0 (the header page): resets the page pointer with 'D', then 'E'. */
int  h8_read_header_page(int serial_file_descriptor, unsigned char *header_page_out);

/* Decode a header page of an analog (family 8) or event (family 7) logger.
 * Returns H8_OK, H8_ERROR_BAD_HEADER if the checksum is wrong, or
 * H8_ERROR_UNSUPPORTED_FAMILY for any other family (the struct is still
 * filled in as far as possible). */
int  h8_decode_header(const unsigned char *header_page, h8_header_info *header_info_out);

/* Offload the memory: pages from page 0, stopping after the page that holds
 * the 0x00 end marker (or at the end of memory). image_out must hold
 * H8_MAXIMUM_MEMORY_PAGE_COUNT pages. *image_size_bytes_out is the number
 * of bytes read. */
int  h8_offload_memory(int serial_file_descriptor, unsigned char *image_out,
                       size_t *image_size_bytes_out, h8_progress_callback progress_callback,
                       void *progress_user_data);

/* Find the records in an image. Analog: from the data start to the
 * logger's 0x00 end marker. Event: 4-byte words from 0xA0 to the first word
 * whose top byte is below 0x80 (the end marker) or whose time runs
 * backwards. Either way, at most to the end of the image. */
h8_data_extent h8_find_data_extent(const unsigned char *image_bytes, size_t image_size_bytes,
                                   const h8_header_info *header_info);

/* ------------------------------------------------------------------ */
/* Launching                                                           */
/* ------------------------------------------------------------------ */

/* Launch a new deployment. current_header_page is the page just read from
 * the logger; everything except launch time, channel bits, interval and
 * description is copied from it unchanged. The launch time is stamped for
 * the moment 'F' is sent. Returns H8_OK or a negative error code.
 * For an event logger (family 7) interval_half_seconds and channel_mask are
 * ignored: the launch rewrites the launch time, sets the start delay to 0
 * and sets the description; the units label, value per event and lockout
 * are copied from the current header. */
int  h8_launch_logger(int serial_file_descriptor, const unsigned char *current_header_page,
                      unsigned long interval_half_seconds, unsigned int channel_mask,
                      const char *description_text, int use_utc_flag,
                      h8_progress_callback progress_callback, void *progress_user_data);

/* ------------------------------------------------------------------ */
/* Output files                                                        */
/* ------------------------------------------------------------------ */

int  h8_write_binary_image(const char *output_path, const unsigned char *image_bytes,
                           size_t image_size_bytes);

/* Analog CSV columns: record_index, elapsed_time_s, time stamp, then the
 * raw counts for each enabled channel, then volts for each enabled channel
 * (volts = counts * volts_per_count + volts_offset).
 * Event CSV columns: record_index, elapsed_time_s, time stamp,
 * record_type (0 = launch record, 1 = event, 2 = stop record written when the
 * logger was woken), cumulative_events,
 * cumulative_value (cumulative_events x the value per event stored in the
 * header, in the header's units). volts_per_count and volts_offset are
 * ignored. */
int  h8_write_csv(const char *output_path, const unsigned char *image_bytes,
                  size_t image_size_bytes, const h8_header_info *header_info,
                  double volts_per_count, double volts_offset, size_t *records_written_out);

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Format "seconds since 1980-01-01 00:00" as YYYY-MM-DD HH:MM:SS. */
void h8_format_time_since_1980(unsigned long seconds_since_1980, char *text_out,
                               size_t text_capacity);

/* Seconds since 1980-01-01 00:00 for a wall-clock time, on local STANDARD
 * time or UTC. */
unsigned long h8_seconds_since_1980_for_time(time_t wall_clock_time_s, int use_utc_flag);

/* Human-readable text for an H8_ERROR_* code. */
const char *h8_error_text(int error_code);

#endif /* H8_PROTOCOL_H */
