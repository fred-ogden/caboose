/*
 * caboose.c
 *
 * GTK3 front end for Onset H07 and H08 loggers (developed on an H08-006-04
 * "H08-006-04"). Protocol work is done by h8_protocol.c in a
 * worker thread so the window stays responsive.
 *
 * Flow:
 *   1. The GUI opens the serial port and waits for a logger to be plugged in.
 *   2a. An idle logger answers 'D' at once: its details are shown with launch
 *       settings (interval, channels 1-4, description). Launch, then unplug.
 *       An H07 event logger (family 7) has only a description to set; it
 *       stores a time stamp at every contact closure.
 *   2b. A logging H8 only answers after a 3 s serial break, which ends its
 *       deployment. The GUI then offers to save the data (.bin and/or .csv),
 *       offloads it, and asks the user to unplug the logger. "View data" runs
 *       tsviewer on the CSV.
 *
 * Settings (device, output folder, calibration, last interval) are kept in
 * ~/.config/caboose.conf.
 *
 * Build:
 *   gcc -std=c99 -Wall -Wextra -O2 $(pkg-config --cflags gtk+-3.0) \
 *       -o caboose caboose.c h8_protocol.c $(pkg-config --libs gtk+-3.0)
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

#define _DEFAULT_SOURCE

#include <gtk/gtk.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "h8_protocol.h"

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define APPLICATION_ID                        "org.ogden.caboose"
#define WINDOW_TITLE                          "Caboose - H7/H8 Data Logger Utility"
#define DEFAULT_SERIAL_DEVICE_PATH            "/dev/ttyUSB0"
#define DATA_VIEWER_PROGRAM                   "tsviewer"
#define SETTINGS_FILE_NAME                    "caboose.conf"
#define SETTINGS_GROUP_NAME                   "caboose"

#define DETECT_POLL_INTERVAL_ms               500   /* 'D' tries while waiting */
#define DETECT_TRIES_BEFORE_BREAK             6     /* ~3 s of plain 'D' before a break */
#define DETECT_TRIES_AFTER_BREAK              4
#define DETECT_BREAK_DURATION_ms              3000  /* wakes (and stops) a logging H8 */
#define DETECT_PORT_RETRY_ms                  2000  /* adapter not plugged in yet */

#define STACK_PAGE_CONNECT                    "connect"
#define STACK_PAGE_IDLE                       "idle"
#define STACK_PAGE_OFFLOAD                    "offload"
#define STACK_PAGE_BUSY                       "busy"
#define STACK_PAGE_DONE                       "done"

/* ------------------------------------------------------------------ */
/* Application state                                                   */
/* ------------------------------------------------------------------ */

typedef enum {
    JOB_NONE = 0,
    JOB_DETECT,
    JOB_LAUNCH,
    JOB_OFFLOAD
} job_kind;

typedef struct {
    /* Widgets */
    GtkWidget *main_window;
    GtkWidget *page_stack;
    GtkWidget *connect_status_label;
    GtkWidget *connect_spinner;
    GtkWidget *device_path_entry;
    GtkWidget *idle_details_label;
    GtkWidget *interval_value_spin;
    GtkWidget *interval_unit_combo;
    GtkWidget *channel_check_buttons[H8_CHANNEL_COUNT_MAXIMUM];
    GtkWidget *description_entry;
    GtkWidget *utc_check_button;
    GtkWidget *launch_estimate_label;
    GtkWidget *analog_launch_widgets[4];    /* interval and channel rows: hidden for H07 */
    GtkWidget *event_launch_note_label;     /* shown only for an event logger */
    GtkWidget *launch_button;
    GtkWidget *offload_note_label;
    GtkWidget *offload_details_label;
    GtkWidget *output_folder_chooser;
    GtkWidget *output_basename_entry;
    GtkWidget *save_binary_check_button;
    GtkWidget *save_csv_check_button;
    GtkWidget *volts_per_count_spin;
    GtkWidget *volts_offset_spin;
    GtkWidget *volts_widgets[5];            /* calibration rows: hidden for H07 */
    GtkWidget *offload_button;
    GtkWidget *busy_title_label;
    GtkWidget *busy_status_label;
    GtkWidget *busy_progress_bar;
    GtkWidget *done_title_label;
    GtkWidget *done_details_label;
    GtkWidget *view_data_button;
    GtkWidget *done_next_logger_button;

    /* Serial and logger state (owned by the worker while a job runs) */
    char            serial_device_path[512];
    int             serial_file_descriptor;
    unsigned char   header_page[H8_MEMORY_PAGE_SIZE_bytes];
    h8_header_info  header_info;
    int             logger_was_running_flag;
    unsigned char  *memory_image_bytes;
    size_t          memory_image_size_bytes;

    /* Launch request (copied before the worker starts) */
    unsigned long   launch_interval_half_seconds;
    unsigned int    launch_channel_mask;
    char            launch_description_text[H8_DESCRIPTION_CHARACTERS_MAXIMUM + 1];
    int             launch_use_utc_flag;

    /* Offload request */
    char            output_binary_path[1024];
    char            output_csv_path[1024];
    int             save_binary_flag;
    int             save_csv_flag;
    double          volts_per_count;
    double          volts_offset;

    /* Worker */
    GThread        *worker_thread;
    job_kind        running_job;
    gint            cancel_requested;         /* atomic */
    int             job_result;
    char            job_message[4096];
    char            last_csv_path[1024];

    /* Settings */
    char            output_folder_path[1024];
} application_state;

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */

static const double interval_unit_seconds[3] = { 1.0, 60.0, 3600.0 };

static void show_error_dialog(application_state *state, const char *message_text)
{
    GtkWidget *error_dialog = gtk_message_dialog_new(GTK_WINDOW(state->main_window),
                                                     GTK_DIALOG_MODAL, GTK_MESSAGE_ERROR,
                                                     GTK_BUTTONS_CLOSE, "%s", message_text);
    gtk_dialog_run(GTK_DIALOG(error_dialog));
    gtk_widget_destroy(error_dialog);
}

static void format_duration_text(double duration_s, char *text_out, size_t text_capacity)
{
    if (duration_s < 120.0) {
        snprintf(text_out, text_capacity, "%.0f s", duration_s);
    } else if (duration_s < 7200.0) {
        snprintf(text_out, text_capacity, "%.1f min", duration_s / 60.0);
    } else if (duration_s < 172800.0) {
        snprintf(text_out, text_capacity, "%.1f h", duration_s / 3600.0);
    } else {
        snprintf(text_out, text_capacity, "%.1f days", duration_s / 86400.0);
    }
}

static void format_channel_list(unsigned int channel_mask, char *text_out, size_t text_capacity)
{
    text_out[0] = '\0';
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        if (channel_mask & (1u << channel_index)) {
            char channel_text[8];
            snprintf(channel_text, sizeof channel_text, "%s%d", text_out[0] ? ", " : "",
                     channel_index + 1);
            strncat(text_out, channel_text, text_capacity - strlen(text_out) - 1);
        }
    }
    if (text_out[0] == '\0') snprintf(text_out, text_capacity, "none");
}

/* Deployment summary shared by the idle and offload pages (Pango markup). */
static void format_logger_details_markup(const h8_header_info *header_info, char *markup_out,
                                         size_t markup_capacity)
{
    char launch_time_text[32];
    char channel_list_text[32];
    char *escaped_model_text       = g_markup_escape_text(header_info->model_text, -1);
    char *escaped_description_text = g_markup_escape_text(header_info->description_text, -1);
    h8_format_time_since_1980(header_info->launch_time_s_since_1980, launch_time_text,
                              sizeof launch_time_text);
    if (header_info->logger_kind == H8_LOGGER_KIND_EVENT) {
        char *escaped_units_text = g_markup_escape_text(header_info->event_units_text, -1);
        snprintf(markup_out, markup_capacity,
                 "<b>Serial number:</b>  %lu\n"
                 "<b>Model:</b>  %s\n"
                 "<b>Event logger:</b> stores a time stamp (0.5 s resolution) at each "
                 "contact closure\n"
                 "<b>Memory:</b>  %lu bytes, %lu events    <b>Launches so far:</b>  %lu\n"
                 "<b>Units:</b>  %s, %g per event\n"
                 "\n"
                 "<b>Last deployment</b>\n"
                 "  Description:  %s\n"
                 "  Launched:  %s (launching PC's clock)",
                 header_info->serial_number, escaped_model_text,
                 header_info->memory_size_bytes, header_info->event_capacity_count,
                 header_info->launch_count,
                 escaped_units_text[0] ? escaped_units_text : "(none)",
                 header_info->event_value_per_event,
                 escaped_description_text[0] ? escaped_description_text : "(none)",
                 launch_time_text);
        g_free(escaped_units_text);
        g_free(escaped_model_text);
        g_free(escaped_description_text);
        return;
    }
    format_channel_list(header_info->enabled_channel_mask, channel_list_text,
                        sizeof channel_list_text);
    snprintf(markup_out, markup_capacity,
             "<b>Serial number:</b>  %lu\n"
             "<b>Model:</b>  %s\n"
             "<b>Memory:</b>  %lu bytes    <b>Launches so far:</b>  %lu\n"
             "\n"
             "<b>Last deployment</b>\n"
             "  Description:  %s\n"
             "  Launched:  %s (launching PC's clock)\n"
             "  Interval:  %.1f s    Channels:  %s",
             header_info->serial_number, escaped_model_text,
             header_info->memory_size_bytes, header_info->launch_count,
             escaped_description_text[0] ? escaped_description_text : "(none)",
             launch_time_text, 0.5 * (double)header_info->interval_half_seconds,
             channel_list_text);
    g_free(escaped_model_text);
    g_free(escaped_description_text);
}

/* ------------------------------------------------------------------ */
/* Settings file                                                       */
/* ------------------------------------------------------------------ */

static char *settings_file_path(void)
{
    return g_build_filename(g_get_user_config_dir(), SETTINGS_FILE_NAME, NULL);
}

static void load_settings(application_state *state)
{
    g_strlcpy(state->serial_device_path, DEFAULT_SERIAL_DEVICE_PATH,
              sizeof state->serial_device_path);
    g_strlcpy(state->output_folder_path, g_get_home_dir(), sizeof state->output_folder_path);
    state->volts_per_count = H8_DEFAULT_VOLTS_PER_COUNT;
    state->volts_offset    = H8_DEFAULT_VOLTS_OFFSET;

    GKeyFile *settings_key_file = g_key_file_new();
    char *settings_path = settings_file_path();
    if (g_key_file_load_from_file(settings_key_file, settings_path, G_KEY_FILE_NONE, NULL)) {
        char *saved_text = g_key_file_get_string(settings_key_file, SETTINGS_GROUP_NAME,
                                                 "device", NULL);
        if (saved_text) {
            g_strlcpy(state->serial_device_path, saved_text, sizeof state->serial_device_path);
            g_free(saved_text);
        }
        saved_text = g_key_file_get_string(settings_key_file, SETTINGS_GROUP_NAME,
                                           "output_folder", NULL);
        if (saved_text) {
            g_strlcpy(state->output_folder_path, saved_text, sizeof state->output_folder_path);
            g_free(saved_text);
        }
        GError *read_error = NULL;
        double saved_value = g_key_file_get_double(settings_key_file, SETTINGS_GROUP_NAME,
                                                   "volts_per_count", &read_error);
        if (read_error == NULL) state->volts_per_count = saved_value;
        g_clear_error(&read_error);
        saved_value = g_key_file_get_double(settings_key_file, SETTINGS_GROUP_NAME,
                                            "volts_offset", &read_error);
        if (read_error == NULL) state->volts_offset = saved_value;
        g_clear_error(&read_error);
    }
    g_free(settings_path);
    g_key_file_free(settings_key_file);
}

static void save_settings(application_state *state)
{
    GKeyFile *settings_key_file = g_key_file_new();
    g_key_file_set_string(settings_key_file, SETTINGS_GROUP_NAME, "device",
                          state->serial_device_path);
    g_key_file_set_string(settings_key_file, SETTINGS_GROUP_NAME, "output_folder",
                          state->output_folder_path);
    char number_text[64];
    snprintf(number_text, sizeof number_text, "%.6g", state->volts_per_count);
    g_key_file_set_string(settings_key_file, SETTINGS_GROUP_NAME, "volts_per_count", number_text);
    snprintf(number_text, sizeof number_text, "%.6g", state->volts_offset);
    g_key_file_set_string(settings_key_file, SETTINGS_GROUP_NAME, "volts_offset", number_text);
    if (state->interval_value_spin != NULL) {
        snprintf(number_text, sizeof number_text, "%.6g",
                 gtk_spin_button_get_value(GTK_SPIN_BUTTON(state->interval_value_spin)));
        g_key_file_set_string(settings_key_file, SETTINGS_GROUP_NAME, "interval_value",
                              number_text);
        g_key_file_set_integer(settings_key_file, SETTINGS_GROUP_NAME, "interval_unit",
                               gtk_combo_box_get_active(GTK_COMBO_BOX(state->interval_unit_combo)));
    }
    char *settings_path = settings_file_path();
    char *settings_folder = g_path_get_dirname(settings_path);
    g_mkdir_with_parents(settings_folder, 0700);
    g_key_file_save_to_file(settings_key_file, settings_path, NULL);
    g_free(settings_folder);
    g_free(settings_path);
    g_key_file_free(settings_key_file);
}

/* ------------------------------------------------------------------ */
/* Worker thread plumbing                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    application_state *state;
    double             fraction_complete;
    char               status_text[1024];
} progress_update;

/* Runs on the GTK main loop. */
static gboolean apply_progress_update(gpointer user_data)
{
    progress_update *update = user_data;
    application_state *state = update->state;
    if (update->fraction_complete >= 0.0) {
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(state->busy_progress_bar),
                                      update->fraction_complete);
    }
    if (gtk_stack_get_visible_child_name(GTK_STACK(state->page_stack)) != NULL &&
        strcmp(gtk_stack_get_visible_child_name(GTK_STACK(state->page_stack)),
               STACK_PAGE_CONNECT) == 0) {
        gtk_label_set_text(GTK_LABEL(state->connect_status_label), update->status_text);
    } else {
        gtk_label_set_text(GTK_LABEL(state->busy_status_label), update->status_text);
    }
    g_free(update);
    return G_SOURCE_REMOVE;
}

/* Callable from the worker thread. fraction < 0 leaves the bar alone. */
static void post_status(application_state *state, double fraction_complete,
                        const char *status_text)
{
    progress_update *update = g_new0(progress_update, 1);
    update->state = state;
    update->fraction_complete = fraction_complete;
    g_strlcpy(update->status_text, status_text, sizeof update->status_text);
    g_idle_add(apply_progress_update, update);
}

/* Progress callback handed to h8_protocol. */
static int protocol_progress_callback(double fraction_complete, const char *status_text,
                                      void *user_data)
{
    application_state *state = user_data;
    post_status(state, fraction_complete, status_text);
    return g_atomic_int_get(&state->cancel_requested);
}

static gboolean job_finished_on_main_loop(gpointer user_data);

static void sleep_unless_cancelled(application_state *state, long duration_ms)
{
    for (long slept_ms = 0; slept_ms < duration_ms &&
         !g_atomic_int_get(&state->cancel_requested); slept_ms += 50) {
        g_usleep(50 * 1000);
    }
}

/* ---- Detect: open the port and wait for a logger ---- */
static void run_detect_job(application_state *state)
{
    char status_text[1024];
    char open_error_text[512];

    while (!g_atomic_int_get(&state->cancel_requested)) {
        /* Open the port, retrying while the adapter is missing. */
        if (state->serial_file_descriptor < 0) {
            state->serial_file_descriptor = h8_open_serial_port(state->serial_device_path,
                                                                open_error_text,
                                                                sizeof open_error_text);
            if (state->serial_file_descriptor < 0) {
                snprintf(status_text, sizeof status_text,
                         "Waiting for the serial adapter: %s", open_error_text);
                post_status(state, -1.0, status_text);
                sleep_unless_cancelled(state, DETECT_PORT_RETRY_ms);
                continue;
            }
        }

        /* An idle logger answers a plain 'D'. */
        snprintf(status_text, sizeof status_text,
                 "Waiting for a logger on %s. Plug it into the serial cable.",
                 state->serial_device_path);
        post_status(state, -1.0, status_text);
        for (int try_index = 0; try_index < DETECT_TRIES_BEFORE_BREAK &&
             !g_atomic_int_get(&state->cancel_requested); ++try_index) {
            int wake_result = h8_try_single_wake_command(state->serial_file_descriptor, 300);
            if (wake_result == 1) {
                state->logger_was_running_flag = 0;
                goto logger_found;
            }
            if (wake_result == H8_ERROR_LOOPBACK) {
                post_status(state, -1.0, "The cable is echoing: plug the logger in fully "
                                         "(push the plug until it clicks).");
            } else if (wake_result == H8_ERROR_IO) {
                /* Adapter unplugged: reopen next time round. */
                h8_close_serial_port(state->serial_file_descriptor);
                state->serial_file_descriptor = -1;
                break;
            }
            sleep_unless_cancelled(state, DETECT_POLL_INTERVAL_ms - 300);
        }
        if (state->serial_file_descriptor < 0 || g_atomic_int_get(&state->cancel_requested)) {
            continue;
        }

        /* A logging H8 only answers after a serial break. */
        post_status(state, -1.0, "No answer yet. Sending a serial break in case the logger "
                                 "is logging (this would end its deployment)...");
        h8_send_serial_break(state->serial_file_descriptor, DETECT_BREAK_DURATION_ms);
        for (int try_index = 0; try_index < DETECT_TRIES_AFTER_BREAK &&
             !g_atomic_int_get(&state->cancel_requested); ++try_index) {
            if (h8_try_single_wake_command(state->serial_file_descriptor, 300) == 1) {
                state->logger_was_running_flag = 1;
                goto logger_found;
            }
        }
    }
    state->job_result = H8_ERROR_CANCELLED;
    return;

logger_found:
    post_status(state, -1.0, "Logger found. Reading its header...");
    state->job_result = h8_read_header_page(state->serial_file_descriptor, state->header_page);
    if (state->job_result == H8_OK) {
        state->job_result = h8_decode_header(state->header_page, &state->header_info);
    }
    if (state->job_result != H8_OK) {
        snprintf(state->job_message, sizeof state->job_message,
                 "A logger answered, but its header could not be read: %s",
                 h8_error_text(state->job_result));
    }
}

/* ---- Launch ---- */
static void run_launch_job(application_state *state)
{
    state->job_result = h8_launch_logger(state->serial_file_descriptor, state->header_page,
                                         state->launch_interval_half_seconds,
                                         state->launch_channel_mask,
                                         state->launch_description_text,
                                         state->launch_use_utc_flag,
                                         protocol_progress_callback, state);
    if (state->job_result != H8_OK) {
        snprintf(state->job_message, sizeof state->job_message, "Launch failed: %s",
                 h8_error_text(state->job_result));
    }
}

/* ---- Offload ---- */
static void run_offload_job(application_state *state)
{
    state->job_result = h8_offload_memory(state->serial_file_descriptor,
                                          state->memory_image_bytes,
                                          &state->memory_image_size_bytes,
                                          protocol_progress_callback, state);
    if (state->job_result != H8_OK) {
        snprintf(state->job_message, sizeof state->job_message, "Offload failed: %s",
                 h8_error_text(state->job_result));
        return;
    }
    h8_decode_header(state->memory_image_bytes, &state->header_info);
    h8_data_extent data_extent = h8_find_data_extent(state->memory_image_bytes,
                                                     state->memory_image_size_bytes,
                                                     &state->header_info);
    char files_written_text[2200] = "";
    if (state->save_binary_flag) {
        if (h8_write_binary_image(state->output_binary_path, state->memory_image_bytes,
                                  state->memory_image_size_bytes) != H8_OK) {
            state->job_result = H8_ERROR_FILE;
            snprintf(state->job_message, sizeof state->job_message, "Could not write %s",
                     state->output_binary_path);
            return;
        }
        snprintf(files_written_text + strlen(files_written_text),
                 sizeof files_written_text - strlen(files_written_text),
                 "\n  %s", state->output_binary_path);
    }
    state->last_csv_path[0] = '\0';
    if (state->save_csv_flag) {
        size_t records_written_count = 0;
        if (h8_write_csv(state->output_csv_path, state->memory_image_bytes,
                         state->memory_image_size_bytes, &state->header_info,
                         state->volts_per_count, state->volts_offset,
                         &records_written_count) != H8_OK) {
            state->job_result = H8_ERROR_FILE;
            snprintf(state->job_message, sizeof state->job_message, "Could not write %s",
                     state->output_csv_path);
            return;
        }
        g_strlcpy(state->last_csv_path, state->output_csv_path, sizeof state->last_csv_path);
        snprintf(files_written_text + strlen(files_written_text),
                 sizeof files_written_text - strlen(files_written_text),
                 "\n  %s", state->output_csv_path);
    }
    if (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT) {
        snprintf(state->job_message, sizeof state->job_message,
                 "Deployment \"%s\": %zu events%s.\n\nFiles written:%s",
                 state->header_info.description_text[0] ? state->header_info.description_text
                                                        : "(no description)",
                 data_extent.event_count,
                 data_extent.end_marker_found_flag ? "" : ", no end marker: memory was full",
                 files_written_text[0] ? files_written_text : "\n  (none selected)");
        return;
    }
    char duration_text[32];
    format_duration_text((double)data_extent.record_count * 0.5
                         * (double)state->header_info.interval_half_seconds,
                         duration_text, sizeof duration_text);
    snprintf(state->job_message, sizeof state->job_message,
             "Deployment \"%s\": %zu records (%s of data)%s.\n\nFiles written:%s",
             state->header_info.description_text[0] ? state->header_info.description_text
                                                    : "(no description)",
             data_extent.record_count, duration_text,
             data_extent.end_marker_found_flag ? "" : ", no end marker: memory was full",
             files_written_text[0] ? files_written_text : "\n  (none selected)");
}

static gpointer worker_thread_main(gpointer user_data)
{
    application_state *state = user_data;
    state->job_message[0] = '\0';
    switch (state->running_job) {
    case JOB_DETECT:  run_detect_job(state);  break;
    case JOB_LAUNCH:  run_launch_job(state);  break;
    case JOB_OFFLOAD: run_offload_job(state); break;
    default: break;
    }
    g_idle_add(job_finished_on_main_loop, state);
    return NULL;
}

static void start_job(application_state *state, job_kind new_job)
{
    g_atomic_int_set(&state->cancel_requested, 0);
    state->running_job = new_job;
    state->worker_thread = g_thread_new("h8-worker", worker_thread_main, state);
}

/* ------------------------------------------------------------------ */
/* Page switching                                                      */
/* ------------------------------------------------------------------ */

static gboolean grab_focus_when_idle(gpointer user_data)
{
    gtk_widget_grab_focus(GTK_WIDGET(user_data));
    return G_SOURCE_REMOVE;
}

static void show_page(application_state *state, const char *page_name)
{
    gtk_stack_set_visible_child_name(GTK_STACK(state->page_stack), page_name);
    if (strcmp(page_name, STACK_PAGE_CONNECT) == 0) {
        gtk_spinner_start(GTK_SPINNER(state->connect_spinner));
    } else {
        gtk_spinner_stop(GTK_SPINNER(state->connect_spinner));
    }
}

static void show_busy_page(application_state *state, const char *title_text)
{
    gtk_label_set_text(GTK_LABEL(state->busy_title_label), title_text);
    gtk_label_set_text(GTK_LABEL(state->busy_status_label), "");
    gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(state->busy_progress_bar), 0.0);
    show_page(state, STACK_PAGE_BUSY);
}

static void start_detection(application_state *state)
{
    gtk_label_set_text(GTK_LABEL(state->connect_status_label), "Opening the serial port...");
    show_page(state, STACK_PAGE_CONNECT);
    start_job(state, JOB_DETECT);
}

static void update_launch_estimate(application_state *state);

static void populate_idle_page(application_state *state)
{
    char details_markup[2048];
    format_logger_details_markup(&state->header_info, details_markup, sizeof details_markup);
    gtk_label_set_markup(GTK_LABEL(state->idle_details_label), details_markup);
    /* An event logger has no interval or channels to set. */
    int logger_is_event = (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT);
    for (int widget_index = 0; widget_index < 4; ++widget_index) {
        gtk_widget_set_visible(state->analog_launch_widgets[widget_index], !logger_is_event);
    }
    gtk_widget_set_visible(state->event_launch_note_label, logger_is_event);
    /* Default the channel boxes to the channels installed on this logger. */
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        int channel_installed = (state->header_info.installed_channel_mask
                                 & (1u << channel_index)) != 0;
        gtk_widget_set_sensitive(state->channel_check_buttons[channel_index],
                                 channel_installed);
        if (!channel_installed) {
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(state->channel_check_buttons[channel_index]),
                                         FALSE);
        }
    }
    update_launch_estimate(state);
}

static void populate_offload_page(application_state *state)
{
    char details_markup[2048];
    format_logger_details_markup(&state->header_info, details_markup, sizeof details_markup);
    gtk_label_set_markup(GTK_LABEL(state->offload_details_label), details_markup);
    int logger_is_event = (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT);
    for (int widget_index = 0; widget_index < 5; ++widget_index) {
        gtk_widget_set_visible(state->volts_widgets[widget_index], !logger_is_event);
    }
    gtk_button_set_label(GTK_BUTTON(state->save_csv_check_button),
                         logger_is_event ? ".c_sv (event times, cumulative total)"
                                         : ".c_sv (counts and volts)");
    gtk_label_set_markup(GTK_LABEL(state->offload_note_label),
                         state->logger_was_running_flag
                         ? "<b>This logger was logging.</b> Waking it has ended its "
                           "deployment. Save its data now."
                         : "This logger is not logging. Its memory holds the deployment "
                           "shown below.");

    /* Default file base name: description plus launch date. */
    char launch_time_text[32];
    h8_format_time_since_1980(state->header_info.launch_time_s_since_1980, launch_time_text,
                              sizeof launch_time_text);
    /* Trim spaces around the description so the name does not start with '_'. */
    char trimmed_description_text[H8_DESCRIPTION_CHARACTERS_MAXIMUM + 1];
    g_strlcpy(trimmed_description_text, state->header_info.description_text,
              sizeof trimmed_description_text);
    g_strstrip(trimmed_description_text);
    const char *default_base_text =
        (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT) ? "h7_event" : "h8";
    char base_name_text[128];
    snprintf(base_name_text, sizeof base_name_text, "%s_%s",
             trimmed_description_text[0] ? trimmed_description_text : default_base_text,
             launch_time_text);
    for (char *name_character = base_name_text; *name_character; ++name_character) {
        char current_character = *name_character;
        int character_is_safe = (current_character >= 'A' && current_character <= 'Z') ||
                                (current_character >= 'a' && current_character <= 'z') ||
                                (current_character >= '0' && current_character <= '9') ||
                                current_character == '-' || current_character == '.';
        if (!character_is_safe) *name_character = '_';
    }
    gtk_entry_set_text(GTK_ENTRY(state->output_basename_entry), base_name_text);
    gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(state->output_folder_chooser),
                                        state->output_folder_path);
}

static void show_done_page(application_state *state, const char *title_markup,
                           const char *details_text, int offer_view_data_flag)
{
    gtk_label_set_markup(GTK_LABEL(state->done_title_label), title_markup);
    gtk_label_set_text(GTK_LABEL(state->done_details_label), details_text);
    gtk_label_select_region(GTK_LABEL(state->done_details_label), 0, 0);
    gtk_widget_set_visible(state->view_data_button, offer_view_data_flag);
    show_page(state, STACK_PAGE_DONE);
    g_idle_add(grab_focus_when_idle, state->done_next_logger_button);
}

/* Runs on the GTK main loop when the worker finishes. */
static gboolean job_finished_on_main_loop(gpointer user_data)
{
    application_state *state = user_data;
    g_thread_join(state->worker_thread);
    state->worker_thread = NULL;
    job_kind finished_job = state->running_job;
    state->running_job = JOB_NONE;

    if (finished_job == JOB_DETECT) {
        if (state->job_result == H8_ERROR_CANCELLED) {
            /* Cancelled to change device, or window closing. */
            if (state->main_window != NULL && gtk_widget_get_visible(state->main_window)) {
                h8_close_serial_port(state->serial_file_descriptor);
                state->serial_file_descriptor = -1;
                start_detection(state);
            }
            return G_SOURCE_REMOVE;
        }
        if (state->job_result != H8_OK) {
            show_error_dialog(state, state->job_message);
            start_detection(state);
            return G_SOURCE_REMOVE;
        }
        if (state->logger_was_running_flag) {
            populate_offload_page(state);
            show_page(state, STACK_PAGE_OFFLOAD);
            g_idle_add(grab_focus_when_idle, state->offload_button);
        } else {
            populate_idle_page(state);
            show_page(state, STACK_PAGE_IDLE);
            g_idle_add(grab_focus_when_idle, state->launch_button);
        }
    } else if (finished_job == JOB_LAUNCH) {
        if (state->job_result != H8_OK) {
            show_error_dialog(state, state->job_message);
            populate_idle_page(state);
            show_page(state, STACK_PAGE_IDLE);
            return G_SOURCE_REMOVE;
        }
        if (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT) {
            char event_details_text[1024];
            snprintf(event_details_text, sizeof event_details_text,
                     "Event logger %lu launched: \"%s\".\n"
                     "It records a time stamp at each contact closure, up to %lu events.\n\n"
                     "To stop it and save the data later, start this program and plug the "
                     "logger in.",
                     state->header_info.serial_number,
                     state->launch_description_text[0] ? state->launch_description_text
                                                       : "(no description)",
                     state->header_info.event_capacity_count);
            show_done_page(state, "<big><b>Launched.</b></big>", event_details_text, 0);
            return G_SOURCE_REMOVE;
        }
        char channel_list_text[32];
        char duration_text[32];
        format_channel_list(state->launch_channel_mask, channel_list_text,
                            sizeof channel_list_text);
        int enabled_channel_count = 0;
        for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
            if (state->launch_channel_mask & (1u << channel_index)) ++enabled_channel_count;
        }
        double records_capacity = (double)(state->header_info.memory_size_bytes
                                           - state->header_info.data_start_offset_bytes)
                                / (double)enabled_channel_count;
        format_duration_text(records_capacity * 0.5
                             * (double)state->launch_interval_half_seconds,
                             duration_text, sizeof duration_text);
        char details_text[1024];
        snprintf(details_text, sizeof details_text,
                 "Logger %lu launched: \"%s\", every %.1f s on channel(s) %s.\n"
                 "Memory will be full after about %s.\n\n"
                 "To stop it and save the data later, start this program and plug the "
                 "logger in.",
                 state->header_info.serial_number,
                 state->launch_description_text[0] ? state->launch_description_text
                                                   : "(no description)",
                 0.5 * (double)state->launch_interval_half_seconds, channel_list_text,
                 duration_text);
        show_done_page(state, "<big><b>Launched.</b></big>", details_text, 0);
    } else if (finished_job == JOB_OFFLOAD) {
        if (state->job_result != H8_OK) {
            show_error_dialog(state, state->job_message);
            populate_offload_page(state);
            show_page(state, STACK_PAGE_OFFLOAD);
            return G_SOURCE_REMOVE;
        }
        show_done_page(state, "<big><b>Data saved.</b></big>", state->job_message,
                       state->last_csv_path[0] != '\0');
    }
    return G_SOURCE_REMOVE;
}

/* ------------------------------------------------------------------ */
/* Button handlers                                                     */
/* ------------------------------------------------------------------ */

static int read_description_setting(application_state *state, char *error_text_out,
                                    size_t error_text_capacity);

static int read_launch_settings(application_state *state, char *error_text_out,
                                size_t error_text_capacity)
{
    if (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT) {
        /* Interval and channels do not apply; the library ignores them. */
        state->launch_interval_half_seconds = 0;
        state->launch_channel_mask = 0;
        return read_description_setting(state, error_text_out, error_text_capacity);
    }
    double interval_value = gtk_spin_button_get_value(GTK_SPIN_BUTTON(state->interval_value_spin));
    int unit_index = gtk_combo_box_get_active(GTK_COMBO_BOX(state->interval_unit_combo));
    if (unit_index < 0 || unit_index > 2) unit_index = 0;
    double interval_half_seconds_real = interval_value * interval_unit_seconds[unit_index] * 2.0;
    double rounded_half_seconds = floor(interval_half_seconds_real + 0.5);
    if (fabs(interval_half_seconds_real - rounded_half_seconds) > 1e-6 ||
        rounded_half_seconds < (double)H8_INTERVAL_HALF_SECONDS_MINIMUM ||
        rounded_half_seconds > (double)H8_INTERVAL_HALF_SECONDS_MAXIMUM) {
        snprintf(error_text_out, error_text_capacity,
                 "Interval must be a multiple of 0.5 s, from 0.5 s to 32767.5 s (9.1 h).");
        return 0;
    }
    state->launch_interval_half_seconds = (unsigned long)rounded_half_seconds;

    state->launch_channel_mask = 0;
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->channel_check_buttons[channel_index]))) {
            state->launch_channel_mask |= 1u << channel_index;
        }
    }
    if (state->launch_channel_mask == 0) {
        snprintf(error_text_out, error_text_capacity, "Select at least one channel.");
        return 0;
    }
    return read_description_setting(state, error_text_out, error_text_capacity);
}

/* Description and UTC choice, common to both logger kinds. */
static int read_description_setting(application_state *state, char *error_text_out,
                                    size_t error_text_capacity)
{
    const char *description_text = gtk_entry_get_text(GTK_ENTRY(state->description_entry));
    if (strlen(description_text) > H8_DESCRIPTION_CHARACTERS_MAXIMUM) {
        snprintf(error_text_out, error_text_capacity,
                 "Description is limited to %d characters.", H8_DESCRIPTION_CHARACTERS_MAXIMUM);
        return 0;
    }
    for (const char *description_character = description_text; *description_character;
         ++description_character) {
        if ((unsigned char)*description_character < 0x20 ||
            (unsigned char)*description_character > 0x7E) {
            snprintf(error_text_out, error_text_capacity,
                     "Description must be plain ASCII text.");
            return 0;
        }
    }
    g_strlcpy(state->launch_description_text, description_text,
              sizeof state->launch_description_text);
    state->launch_use_utc_flag =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->utc_check_button));
    return 1;
}

static void update_launch_estimate(application_state *state)
{
    char error_text[256];
    if (!read_launch_settings(state, error_text, sizeof error_text)) {
        char *escaped_error_text = g_markup_escape_text(error_text, -1);
        char markup_text[512];
        snprintf(markup_text, sizeof markup_text, "<span foreground=\"#c01c28\">%s</span>",
                 escaped_error_text);
        gtk_label_set_markup(GTK_LABEL(state->launch_estimate_label), markup_text);
        g_free(escaped_error_text);
        gtk_widget_set_sensitive(state->launch_button, FALSE);
        return;
    }
    if (state->header_info.logger_kind == H8_LOGGER_KIND_EVENT) {
        char event_estimate_text[256];
        snprintf(event_estimate_text, sizeof event_estimate_text,
                 "Memory holds %lu events. Logging stops when it is full.",
                 state->header_info.event_capacity_count);
        gtk_label_set_text(GTK_LABEL(state->launch_estimate_label), event_estimate_text);
        gtk_widget_set_sensitive(state->launch_button, TRUE);
        return;
    }
    int enabled_channel_count = 0;
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        if (state->launch_channel_mask & (1u << channel_index)) ++enabled_channel_count;
    }
    double records_capacity = (double)(state->header_info.memory_size_bytes
                                       - state->header_info.data_start_offset_bytes)
                            / (double)enabled_channel_count;
    double run_time_s = records_capacity * 0.5 * (double)state->launch_interval_half_seconds;
    char duration_text[32];
    format_duration_text(run_time_s, duration_text, sizeof duration_text);
    time_t memory_full_time = time(NULL) + (time_t)run_time_s;
    struct tm memory_full_calendar;
    localtime_r(&memory_full_time, &memory_full_calendar);
    char memory_full_text[64];
    strftime(memory_full_text, sizeof memory_full_text, "%Y-%m-%d %H:%M",
             &memory_full_calendar);
    char estimate_text[256];
    snprintf(estimate_text, sizeof estimate_text,
             "Memory holds %.0f records: full after %s (about %s).",
             records_capacity, duration_text, memory_full_text);
    gtk_label_set_text(GTK_LABEL(state->launch_estimate_label), estimate_text);
    gtk_widget_set_sensitive(state->launch_button, TRUE);
}

static void on_launch_setting_changed(GtkWidget *changed_widget, gpointer user_data)
{
    (void)changed_widget;
    update_launch_estimate(user_data);
}

static void on_launch_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    char error_text[256];
    if (!read_launch_settings(state, error_text, sizeof error_text)) {
        show_error_dialog(state, error_text);
        return;
    }
    GtkWidget *confirm_dialog = gtk_message_dialog_new(
        GTK_WINDOW(state->main_window), GTK_DIALOG_MODAL, GTK_MESSAGE_QUESTION,
        GTK_BUTTONS_OK_CANCEL,
        "Launch a new deployment?\n\nLogging restarts at the beginning of memory and "
        "overwrites the data now in the logger. Use \"Offload data\" first if you need it.");
    gtk_dialog_set_default_response(GTK_DIALOG(confirm_dialog), GTK_RESPONSE_OK);
    int dialog_response = gtk_dialog_run(GTK_DIALOG(confirm_dialog));
    gtk_widget_destroy(confirm_dialog);
    if (dialog_response != GTK_RESPONSE_OK) return;
    save_settings(state);
    show_busy_page(state, "Launching the logger...");
    start_job(state, JOB_LAUNCH);
}

static void on_idle_offload_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    populate_offload_page(state);
    show_page(state, STACK_PAGE_OFFLOAD);
    g_idle_add(grab_focus_when_idle, state->offload_button);
}

static void on_offload_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    state->save_binary_flag =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->save_binary_check_button));
    state->save_csv_flag =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(state->save_csv_check_button));
    if (!state->save_binary_flag && !state->save_csv_flag) {
        show_error_dialog(state, "Select .bin, .csv, or both.");
        return;
    }
    char *output_folder = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(state->output_folder_chooser));
    if (output_folder == NULL) {
        show_error_dialog(state, "Choose a folder for the data files.");
        return;
    }
    const char *base_name_text = gtk_entry_get_text(GTK_ENTRY(state->output_basename_entry));
    if (base_name_text[0] == '\0' || strchr(base_name_text, '/') != NULL) {
        g_free(output_folder);
        show_error_dialog(state, "Enter a file name (without a folder or extension).");
        return;
    }
    g_strlcpy(state->output_folder_path, output_folder, sizeof state->output_folder_path);
    snprintf(state->output_binary_path, sizeof state->output_binary_path, "%s/%s.bin",
             output_folder, base_name_text);
    snprintf(state->output_csv_path, sizeof state->output_csv_path, "%s/%s.csv",
             output_folder, base_name_text);
    g_free(output_folder);

    /* Warn before overwriting existing files. */
    int binary_exists = state->save_binary_flag
                        && g_file_test(state->output_binary_path, G_FILE_TEST_EXISTS);
    int csv_exists = state->save_csv_flag
                     && g_file_test(state->output_csv_path, G_FILE_TEST_EXISTS);
    if (binary_exists || csv_exists) {
        GtkWidget *overwrite_dialog = gtk_message_dialog_new(
            GTK_WINDOW(state->main_window), GTK_DIALOG_MODAL, GTK_MESSAGE_WARNING,
            GTK_BUTTONS_OK_CANCEL, "A file named \"%s\" already exists. Overwrite it?",
            base_name_text);
        int dialog_response = gtk_dialog_run(GTK_DIALOG(overwrite_dialog));
        gtk_widget_destroy(overwrite_dialog);
        if (dialog_response != GTK_RESPONSE_OK) return;
    }

    state->volts_per_count = gtk_spin_button_get_value(GTK_SPIN_BUTTON(state->volts_per_count_spin));
    state->volts_offset    = gtk_spin_button_get_value(GTK_SPIN_BUTTON(state->volts_offset_spin));
    save_settings(state);
    show_busy_page(state, "Offloading data...");
    start_job(state, JOB_OFFLOAD);
}

static void on_view_data_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    if (state->last_csv_path[0] == '\0') return;
    char *viewer_arguments[] = { DATA_VIEWER_PROGRAM, state->last_csv_path, NULL };
    GError *spawn_error = NULL;
    if (!g_spawn_async(NULL, viewer_arguments, NULL, G_SPAWN_SEARCH_PATH, NULL, NULL, NULL,
                       &spawn_error)) {
        char message_text[1024];
        snprintf(message_text, sizeof message_text, "Could not start %s: %s",
                 DATA_VIEWER_PROGRAM, spawn_error->message);
        g_error_free(spawn_error);
        show_error_dialog(state, message_text);
    }
}

/* "Next logger": the logger has been unplugged, so close the port and
 * start waiting again. */
static void on_next_logger_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    h8_close_serial_port(state->serial_file_descriptor);
    state->serial_file_descriptor = -1;
    start_detection(state);
}

static void on_apply_device_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    g_strlcpy(state->serial_device_path,
              gtk_entry_get_text(GTK_ENTRY(state->device_path_entry)),
              sizeof state->serial_device_path);
    save_settings(state);
    if (state->running_job == JOB_DETECT) {
        /* The detect job ends, and its completion handler reopens the port. */
        g_atomic_int_set(&state->cancel_requested, 1);
        gtk_label_set_text(GTK_LABEL(state->connect_status_label), "Switching device...");
    }
}

static gboolean on_main_window_delete(GtkWidget *window_widget, GdkEvent *delete_event,
                                      gpointer user_data)
{
    (void)window_widget;
    (void)delete_event;
    application_state *state = user_data;
    if (state->running_job == JOB_LAUNCH || state->running_job == JOB_OFFLOAD) {
        show_error_dialog(state, "Please wait until the logger operation finishes.");
        return TRUE;
    }
    save_settings(state);
    if (state->worker_thread != NULL) {
        g_atomic_int_set(&state->cancel_requested, 1);
        g_thread_join(state->worker_thread);
        state->worker_thread = NULL;
    }
    h8_close_serial_port(state->serial_file_descriptor);
    state->serial_file_descriptor = -1;
    state->main_window = NULL;
    return FALSE;
}

/* ------------------------------------------------------------------ */
/* Building the window                                                 */
/* ------------------------------------------------------------------ */

static GtkWidget *new_left_label(const char *label_text)
{
    GtkWidget *label_widget = gtk_label_new(label_text);
    gtk_label_set_xalign(GTK_LABEL(label_widget), 0.0f);
    gtk_label_set_line_wrap(GTK_LABEL(label_widget), TRUE);
    return label_widget;
}

static GtkWidget *new_page_box(void)
{
    GtkWidget *page_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 12);
    gtk_container_set_border_width(GTK_CONTAINER(page_box), 18);
    return page_box;
}

static GtkWidget *build_connect_page(application_state *state)
{
    GtkWidget *page_box = new_page_box();
    GtkWidget *title_label = new_left_label(NULL);
    gtk_label_set_markup(GTK_LABEL(title_label), "<big><b>Connect a logger</b></big>");
    gtk_box_pack_start(GTK_BOX(page_box), title_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(page_box), new_left_label(
        "1. Plug the serial adapter into this computer.\n"
        "2. Plug the logger into the serial cable, pushing the plug in until it clicks.\n\n"
        "An idle logger is found within a second. A logger that is logging only answers "
        "after a 3 s serial break, and that break ends its deployment. "
        "Offload did."), FALSE, FALSE, 0);

    GtkWidget *status_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    state->connect_spinner = gtk_spinner_new();
    gtk_box_pack_start(GTK_BOX(status_row), state->connect_spinner, FALSE, FALSE, 0);
    state->connect_status_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(status_row), state->connect_status_label, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(page_box), status_row, FALSE, FALSE, 12);

    GtkWidget *device_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_box_pack_start(GTK_BOX(device_row), gtk_label_new("Serial device:"), FALSE, FALSE, 0);
    state->device_path_entry = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(state->device_path_entry), state->serial_device_path);
    gtk_box_pack_start(GTK_BOX(device_row), state->device_path_entry, TRUE, TRUE, 0);
    GtkWidget *apply_device_button = gtk_button_new_with_mnemonic("_Use this device");
    g_signal_connect(apply_device_button, "clicked", G_CALLBACK(on_apply_device_clicked), state);
    gtk_box_pack_start(GTK_BOX(device_row), apply_device_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(page_box), device_row, FALSE, FALSE, 0);
    return page_box;
}

static GtkWidget *build_idle_page(application_state *state)
{
    GtkWidget *page_box = new_page_box();
    GtkWidget *title_label = new_left_label(NULL);
    gtk_label_set_markup(GTK_LABEL(title_label), "<big><b>Logger connected (not logging)</b></big>");
    gtk_box_pack_start(GTK_BOX(page_box), title_label, FALSE, FALSE, 0);
    state->idle_details_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(page_box), state->idle_details_label, FALSE, FALSE, 0);

    GtkWidget *launch_frame = gtk_frame_new("New deployment");
    GtkWidget *launch_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(launch_grid), 8);
    gtk_grid_set_column_spacing(GTK_GRID(launch_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(launch_grid), 10);

    state->analog_launch_widgets[0] = new_left_label("Interval:");
    gtk_grid_attach(GTK_GRID(launch_grid), state->analog_launch_widgets[0], 0, 0, 1, 1);
    state->interval_value_spin = gtk_spin_button_new_with_range(0.5, 32767.5, 0.5);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(state->interval_value_spin), 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(state->interval_value_spin), 60.0);
    state->interval_unit_combo = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(state->interval_unit_combo), "seconds");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(state->interval_unit_combo), "minutes");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(state->interval_unit_combo), "hours");
    gtk_combo_box_set_active(GTK_COMBO_BOX(state->interval_unit_combo), 0);
    GtkWidget *interval_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(interval_row), state->interval_value_spin, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(interval_row), state->interval_unit_combo, FALSE, FALSE, 0);
    gtk_grid_attach(GTK_GRID(launch_grid), interval_row, 1, 0, 1, 1);
    state->analog_launch_widgets[1] = interval_row;

    state->analog_launch_widgets[2] = new_left_label("Channels:");
    gtk_grid_attach(GTK_GRID(launch_grid), state->analog_launch_widgets[2], 0, 1, 1, 1);
    GtkWidget *channel_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        char channel_label_text[8];
        snprintf(channel_label_text, sizeof channel_label_text, "_%d", channel_index + 1);
        state->channel_check_buttons[channel_index] =
            gtk_check_button_new_with_mnemonic(channel_label_text);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(state->channel_check_buttons[channel_index]),
                                     channel_index == 0);
        gtk_box_pack_start(GTK_BOX(channel_row), state->channel_check_buttons[channel_index],
                           FALSE, FALSE, 0);
    }
    gtk_grid_attach(GTK_GRID(launch_grid), channel_row, 1, 1, 1, 1);
    state->analog_launch_widgets[3] = channel_row;

    /* Event loggers take no interval or channels; this note replaces them. */
    state->event_launch_note_label = new_left_label(
        "Event logger: no interval or channels to set. Each contact closure is "
        "stored as a time stamp.");
    gtk_widget_set_no_show_all(state->event_launch_note_label, TRUE);
    gtk_grid_attach(GTK_GRID(launch_grid), state->event_launch_note_label, 0, 5, 2, 1);

    gtk_grid_attach(GTK_GRID(launch_grid), new_left_label("Description:"), 0, 2, 1, 1);
    state->description_entry = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(state->description_entry),
                             H8_DESCRIPTION_CHARACTERS_MAXIMUM);
    gtk_entry_set_width_chars(GTK_ENTRY(state->description_entry), 40);
    gtk_grid_attach(GTK_GRID(launch_grid), state->description_entry, 1, 2, 1, 1);

    state->utc_check_button = gtk_check_button_new_with_mnemonic(
        "Store launch time as _UTC (default: local standard time)");
    gtk_grid_attach(GTK_GRID(launch_grid), state->utc_check_button, 0, 3, 2, 1);

    state->launch_estimate_label = new_left_label("");
    gtk_grid_attach(GTK_GRID(launch_grid), state->launch_estimate_label, 0, 4, 2, 1);
    gtk_container_add(GTK_CONTAINER(launch_frame), launch_grid);
    gtk_box_pack_start(GTK_BOX(page_box), launch_frame, FALSE, FALSE, 0);

    g_signal_connect(state->interval_value_spin, "value-changed",
                     G_CALLBACK(on_launch_setting_changed), state);
    g_signal_connect(state->interval_unit_combo, "changed",
                     G_CALLBACK(on_launch_setting_changed), state);
    g_signal_connect(state->description_entry, "changed",
                     G_CALLBACK(on_launch_setting_changed), state);
    for (int channel_index = 0; channel_index < H8_CHANNEL_COUNT_MAXIMUM; ++channel_index) {
        g_signal_connect(state->channel_check_buttons[channel_index], "toggled",
                         G_CALLBACK(on_launch_setting_changed), state);
    }

    GtkWidget *button_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    state->launch_button = gtk_button_new_with_mnemonic("_Launch");
    gtk_style_context_add_class(gtk_widget_get_style_context(state->launch_button),
                                "suggested-action");
    g_signal_connect(state->launch_button, "clicked", G_CALLBACK(on_launch_clicked), state);
    GtkWidget *offload_instead_button = gtk_button_new_with_mnemonic("_Offload data");
    g_signal_connect(offload_instead_button, "clicked", G_CALLBACK(on_idle_offload_clicked), state);
    GtkWidget *disconnect_button = gtk_button_new_with_mnemonic("_Disconnect");
    g_signal_connect(disconnect_button, "clicked", G_CALLBACK(on_next_logger_clicked), state);
    gtk_box_pack_end(GTK_BOX(button_row), state->launch_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(button_row), offload_instead_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(button_row), disconnect_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(page_box), button_row, FALSE, FALSE, 0);
    return page_box;
}

static GtkWidget *build_offload_page(application_state *state)
{
    GtkWidget *page_box = new_page_box();
    GtkWidget *title_label = new_left_label(NULL);
    gtk_label_set_markup(GTK_LABEL(title_label), "<big><b>Save logger data</b></big>");
    gtk_box_pack_start(GTK_BOX(page_box), title_label, FALSE, FALSE, 0);
    state->offload_note_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(page_box), state->offload_note_label, FALSE, FALSE, 0);
    state->offload_details_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(page_box), state->offload_details_label, FALSE, FALSE, 0);

    GtkWidget *save_frame = gtk_frame_new("Output");
    GtkWidget *save_grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(save_grid), 8);
    gtk_grid_set_column_spacing(GTK_GRID(save_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(save_grid), 10);

    gtk_grid_attach(GTK_GRID(save_grid), new_left_label("Folder:"), 0, 0, 1, 1);
    state->output_folder_chooser = gtk_file_chooser_button_new(
        "Choose a folder for the data files", GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER);
    gtk_widget_set_hexpand(state->output_folder_chooser, TRUE);
    gtk_grid_attach(GTK_GRID(save_grid), state->output_folder_chooser, 1, 0, 2, 1);

    gtk_grid_attach(GTK_GRID(save_grid), new_left_label("File name:"), 0, 1, 1, 1);
    state->output_basename_entry = gtk_entry_new();
    gtk_widget_set_hexpand(state->output_basename_entry, TRUE);
    gtk_grid_attach(GTK_GRID(save_grid), state->output_basename_entry, 1, 1, 2, 1);

    gtk_grid_attach(GTK_GRID(save_grid), new_left_label("Formats:"), 0, 2, 1, 1);
    GtkWidget *format_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 12);
    state->save_binary_check_button = gtk_check_button_new_with_mnemonic(
        "._bin (raw memory image)");
    state->save_csv_check_button = gtk_check_button_new_with_mnemonic(
        ".c_sv (counts and volts)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(state->save_binary_check_button), TRUE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(state->save_csv_check_button), TRUE);
    gtk_box_pack_start(GTK_BOX(format_row), state->save_binary_check_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(format_row), state->save_csv_check_button, FALSE, FALSE, 0);
    gtk_grid_attach(GTK_GRID(save_grid), format_row, 1, 2, 2, 1);

    state->volts_widgets[0] = new_left_label("Volts per count:");
    gtk_grid_attach(GTK_GRID(save_grid), state->volts_widgets[0], 0, 3, 1, 1);
    state->volts_per_count_spin = gtk_spin_button_new_with_range(0.0, 1.0, 0.00001);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(state->volts_per_count_spin), 5);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(state->volts_per_count_spin),
                              state->volts_per_count);
    gtk_grid_attach(GTK_GRID(save_grid), state->volts_per_count_spin, 1, 3, 1, 1);
    state->volts_widgets[1] = new_left_label("Offset (V):");
    gtk_grid_attach(GTK_GRID(save_grid), state->volts_widgets[1], 0, 4, 1, 1);
    state->volts_offset_spin = gtk_spin_button_new_with_range(-10.0, 10.0, 0.001);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(state->volts_offset_spin), 4);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(state->volts_offset_spin), state->volts_offset);
    gtk_grid_attach(GTK_GRID(save_grid), state->volts_offset_spin, 1, 4, 1, 1);
    state->volts_widgets[2] = new_left_label("volts = counts x volts per count + offset");
    gtk_grid_attach(GTK_GRID(save_grid), state->volts_widgets[2], 2, 3, 1, 2);
    state->volts_widgets[3] = state->volts_per_count_spin;
    state->volts_widgets[4] = state->volts_offset_spin;
    gtk_container_add(GTK_CONTAINER(save_frame), save_grid);
    gtk_box_pack_start(GTK_BOX(page_box), save_frame, FALSE, FALSE, 0);

    GtkWidget *button_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    state->offload_button = gtk_button_new_with_mnemonic("_Offload and save");
    gtk_style_context_add_class(gtk_widget_get_style_context(state->offload_button),
                                "suggested-action");
    g_signal_connect(state->offload_button, "clicked", G_CALLBACK(on_offload_clicked), state);
    GtkWidget *disconnect_button = gtk_button_new_with_mnemonic("_Disconnect");
    g_signal_connect(disconnect_button, "clicked", G_CALLBACK(on_next_logger_clicked), state);
    gtk_box_pack_end(GTK_BOX(button_row), state->offload_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(button_row), disconnect_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(page_box), button_row, FALSE, FALSE, 0);
    return page_box;
}

static GtkWidget *build_busy_page(application_state *state)
{
    GtkWidget *page_box = new_page_box();
    state->busy_title_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(page_box), state->busy_title_label, FALSE, FALSE, 0);
    state->busy_progress_bar = gtk_progress_bar_new();
    gtk_box_pack_start(GTK_BOX(page_box), state->busy_progress_bar, FALSE, FALSE, 0);
    state->busy_status_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(page_box), state->busy_status_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(page_box), new_left_label(
        "Do not unplug the logger until this finishes."), FALSE, FALSE, 0);
    return page_box;
}

/* "Exit": same clean-up as closing the window, then quit. */
static void on_exit_clicked(GtkButton *clicked_button, gpointer user_data)
{
    (void)clicked_button;
    application_state *state = user_data;
    GtkWidget *window_to_close = state->main_window;
    if (!on_main_window_delete(window_to_close, NULL, state)) {
        gtk_widget_destroy(window_to_close);
    }
}

static GtkWidget *build_done_page(application_state *state)
{
    GtkWidget *page_box = new_page_box();

    /* Top: what happened ("Data saved." / "Launched.") and the details. */
    state->done_title_label = new_left_label("");
    gtk_box_pack_start(GTK_BOX(page_box), state->done_title_label, FALSE, FALSE, 0);
    state->done_details_label = new_left_label("");
    gtk_label_set_selectable(GTK_LABEL(state->done_details_label), TRUE);
    gtk_box_pack_start(GTK_BOX(page_box), state->done_details_label, FALSE, FALSE, 0);

    /* Centre: the instruction that matters, in large bold letters. */
    GtkWidget *unplug_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(unplug_label),
                         "<span size=\"xx-large\" weight=\"bold\" foreground=\"#c01c28\">"
                         "UNPLUG THE LOGGER</span>");
    gtk_label_set_justify(GTK_LABEL(unplug_label), GTK_JUSTIFY_CENTER);
    gtk_widget_set_halign(unplug_label, GTK_ALIGN_CENTER);
    gtk_widget_set_valign(unplug_label, GTK_ALIGN_CENTER);
    gtk_box_pack_start(GTK_BOX(page_box), unplug_label, TRUE, TRUE, 0);

    /* Bottom: the three ways to continue. */
    GtkWidget *button_row = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    state->view_data_button = gtk_button_new_with_mnemonic("_View data");
    g_signal_connect(state->view_data_button, "clicked", G_CALLBACK(on_view_data_clicked), state);
    GtkWidget *next_logger_button = gtk_button_new_with_mnemonic("_Launch another logger");
    state->done_next_logger_button = next_logger_button;
    gtk_style_context_add_class(gtk_widget_get_style_context(next_logger_button),
                                "suggested-action");
    g_signal_connect(next_logger_button, "clicked", G_CALLBACK(on_next_logger_clicked), state);
    GtkWidget *exit_button = gtk_button_new_with_mnemonic("E_xit");
    g_signal_connect(exit_button, "clicked", G_CALLBACK(on_exit_clicked), state);
    gtk_box_pack_start(GTK_BOX(button_row), state->view_data_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(button_row), exit_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(button_row), next_logger_button, FALSE, FALSE, 0);
    gtk_box_pack_end(GTK_BOX(page_box), button_row, FALSE, FALSE, 0);
    return page_box;
}

static void on_application_activate(GtkApplication *gtk_application, gpointer user_data)
{
    application_state *state = user_data;
    state->main_window = gtk_application_window_new(gtk_application);
    gtk_window_set_title(GTK_WINDOW(state->main_window), WINDOW_TITLE);
    gtk_window_set_default_size(GTK_WINDOW(state->main_window), 680, 560);
    g_signal_connect(state->main_window, "delete-event", G_CALLBACK(on_main_window_delete), state);

    state->page_stack = gtk_stack_new();
    gtk_stack_set_transition_type(GTK_STACK(state->page_stack),
                                  GTK_STACK_TRANSITION_TYPE_CROSSFADE);
    gtk_stack_add_named(GTK_STACK(state->page_stack), build_connect_page(state), STACK_PAGE_CONNECT);
    gtk_stack_add_named(GTK_STACK(state->page_stack), build_idle_page(state), STACK_PAGE_IDLE);
    gtk_stack_add_named(GTK_STACK(state->page_stack), build_offload_page(state), STACK_PAGE_OFFLOAD);
    gtk_stack_add_named(GTK_STACK(state->page_stack), build_busy_page(state), STACK_PAGE_BUSY);
    gtk_stack_add_named(GTK_STACK(state->page_stack), build_done_page(state), STACK_PAGE_DONE);
    gtk_container_add(GTK_CONTAINER(state->main_window), state->page_stack);

    /* Restore the last interval used. */
    GKeyFile *settings_key_file = g_key_file_new();
    char *settings_path = settings_file_path();
    if (g_key_file_load_from_file(settings_key_file, settings_path, G_KEY_FILE_NONE, NULL)) {
        GError *read_error = NULL;
        double saved_interval_value = g_key_file_get_double(settings_key_file,
                                                            SETTINGS_GROUP_NAME,
                                                            "interval_value", &read_error);
        if (read_error == NULL) {
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(state->interval_value_spin),
                                      saved_interval_value);
        }
        g_clear_error(&read_error);
        int saved_unit_index = g_key_file_get_integer(settings_key_file, SETTINGS_GROUP_NAME,
                                                      "interval_unit", &read_error);
        if (read_error == NULL && saved_unit_index >= 0 && saved_unit_index <= 2) {
            gtk_combo_box_set_active(GTK_COMBO_BOX(state->interval_unit_combo),
                                     saved_unit_index);
        }
        g_clear_error(&read_error);
    }
    g_free(settings_path);
    g_key_file_free(settings_key_file);

    gtk_widget_show_all(state->main_window);
    start_detection(state);
}

int main(int argument_count, char **argument_values)
{
    static application_state state;
    memset(&state, 0, sizeof state);
    state.serial_file_descriptor = -1;
    state.memory_image_bytes = g_malloc((size_t)H8_MAXIMUM_MEMORY_PAGE_COUNT
                                        * H8_MEMORY_PAGE_SIZE_bytes);
    load_settings(&state);

    /* Optional: first argument overrides the serial device. */
    if (argument_count > 1 && argument_values[1][0] == '/') {
        g_strlcpy(state.serial_device_path, argument_values[1], sizeof state.serial_device_path);
        argument_count = 1;
    }

#if GLIB_CHECK_VERSION(2, 74, 0)
    GtkApplication *gtk_application = gtk_application_new(APPLICATION_ID,
                                                          G_APPLICATION_DEFAULT_FLAGS);
#else
    GtkApplication *gtk_application = gtk_application_new(APPLICATION_ID,
                                                          G_APPLICATION_FLAGS_NONE);
#endif
    g_signal_connect(gtk_application, "activate", G_CALLBACK(on_application_activate), &state);
    int exit_status = g_application_run(G_APPLICATION(gtk_application), argument_count,
                                        argument_values);
    g_object_unref(gtk_application);
    g_free(state.memory_image_bytes);
    return exit_status;
}
