/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "audio_hw_primary"
/*#define LOG_NDEBUG 0*/

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include <log/log.h>
#include <cutils/properties.h>
#include <cutils/str_parms.h>

#include <hardware/audio.h>
#include <hardware/hardware.h>

#include <system/audio.h>

#include <linux/ioctl.h>
#include <sound/asound.h>
#include <tinyalsa/asoundlib.h>

#include <audio_utils/channels.h>
#include <audio_utils/format.h>
#include <audio_utils/resampler.h>
#include <audio_route/audio_route.h>

#define PCM_CARD 0
#define PCM_CARD_DEFAULT 0
#define PCM_DEVICE 0

#define OUT_PERIOD_SIZE 1024
#define OUT_SHORT_PERIOD_COUNT 2
#define OUT_LONG_PERIOD_COUNT 8
#define OUT_PERIOD_COUNT OUT_SHORT_PERIOD_COUNT
#define OUT_SAMPLING_RATE 48000
#define OUT_SAMPLING_RATE_FALLBACK 44100

#define IN_PERIOD_SIZE 1024 //default period size
#define IN_PERIOD_MS 10
#define IN_PERIOD_COUNT 4
#define IN_SAMPLING_RATE 48000
#define IN_SAMPLING_RATE_FALLBACK 44100

#define AUDIO_PARAMETER_HFP_ENABLE   "hfp_enable"
#define AUDIO_PARAMETER_BT_SCO       "BT_SCO"
#define AUDIO_BT_DRIVER_NAME         "btaudiosource"
#define SAMPLE_SIZE_IN_BYTES          2
#define SAMPLE_SIZE_IN_BYTES_STEREO   4

//#define DEBUG_PCM_DUMP

#ifdef DEBUG_PCM_DUMP
// To enable dumps, explicitly create "/vendor/dump/" folder and reboot device
FILE *sco_call_write = NULL;
FILE *sco_call_write_remapped = NULL;
FILE *sco_call_write_bt = NULL;
FILE *sco_call_read = NULL;
FILE *sco_call_read_remapped = NULL;
FILE *sco_call_read_bt = NULL;
FILE *out_write_dump = NULL;
FILE *in_read_dump = NULL;
#endif

struct pcm_config pcm_config_out = {
    .channels = 2,
    .rate = OUT_SAMPLING_RATE,
    .period_size = OUT_PERIOD_SIZE,
    .period_count = OUT_PERIOD_COUNT,
    .format = PCM_FORMAT_S16_LE,
    .start_threshold = OUT_PERIOD_SIZE * OUT_PERIOD_COUNT,
};

struct pcm_config pcm_config_in = {
    .channels = 2,
    .rate = IN_SAMPLING_RATE,
    .period_size = IN_PERIOD_SIZE,
    .period_count = IN_PERIOD_COUNT,
    .format = PCM_FORMAT_S16_LE,
    .start_threshold = 1,
    .stop_threshold = (IN_PERIOD_SIZE * IN_PERIOD_COUNT),
};

//[ BT ALSA Card config
struct pcm_config bt_out_config = {
    .channels = 1,
    .rate = 8000,
    .period_size = 240,
    .period_count = 5,
    .start_threshold = 0,
    .stop_threshold = 0,
    .silence_threshold = 0,
    .silence_size = 0,
    .avail_min = 0
};

struct pcm_config bt_in_config = {
    .channels = 1,
    .rate = 8000,
    .period_size = 240,
    .period_count = 5,
    .start_threshold = 0,
    .stop_threshold = 0,
    .silence_threshold = 0,
    .silence_size = 0,
    .avail_min = 0
};

struct audio_device {
    struct audio_hw_device hw_device;

    pthread_mutex_t lock; /* see note below on mutex acquisition order */
    unsigned int out_device;
    unsigned int in_device;
    bool standby;
    bool mic_mute;
    struct audio_route *ar;
    bool screen_off;

    int card;
    int cardc;
    struct stream_out *active_out;
    struct stream_in *active_in;

//[BT-HFP Voice Call
    bool is_hfp_call_active;
//BT-HFP Voice Call]

    bool in_needs_standby;
    bool out_needs_standby;

//[BT SCO VoIP Call
    bool in_sco_voip_call;
    int bt_card;
    struct resampler_itfe *voip_in_resampler;
    struct resampler_itfe *voip_out_resampler;
//BT SCO VoIP Call]
};

struct stream_out {
    struct audio_stream_out stream;

    pthread_mutex_t lock; /* see note below on mutex acquisition order */
    struct pcm *pcm;
    struct pcm_config pcm_config;       /* per-stream copy */
    struct audio_config req_config;
    bool unavailable;
    bool standby;
    uint64_t written;
    struct audio_device *dev;

    /* output resampler (push mode) */
    struct resampler_itfe *resampler;
    int16_t *buffer;
    size_t buffer_frames;
};

struct stream_in {
    struct audio_stream_in stream;

    pthread_mutex_t lock; /* see note below on mutex acquisition order */
    struct pcm *pcm;
    struct pcm_config pcm_config;       /* per-stream copy */
    struct audio_config req_config;
    bool unavailable;
    bool standby;

    struct audio_device *dev;

    /* input resampler (pull mode) */
    struct resampler_itfe *resampler;
    struct resampler_buffer_provider buf_provider;
    int16_t *buffer;
    size_t buffer_size;
    size_t frames_in;
    int read_status;
    unsigned int requested_rate;
};

static uint32_t out_get_sample_rate(const struct audio_stream *stream);
static size_t out_get_buffer_size(const struct audio_stream *stream);
static audio_format_t out_get_format(const struct audio_stream *stream);
static uint32_t in_get_sample_rate(const struct audio_stream *stream);
static size_t in_get_buffer_size(const struct audio_stream *stream);
static audio_format_t in_get_format(const struct audio_stream *stream);

static void select_devices(struct audio_device *adev)
{
    int headphone_on;
    int speaker_on;
    int main_mic_on;
    int headset_mic_on;

    headphone_on = adev->out_device & (AUDIO_DEVICE_OUT_WIRED_HEADSET |
                                    AUDIO_DEVICE_OUT_WIRED_HEADPHONE);
    speaker_on = adev->out_device & AUDIO_DEVICE_OUT_SPEAKER;
    main_mic_on = adev->in_device & AUDIO_DEVICE_IN_BUILTIN_MIC;
    headset_mic_on = adev->in_device & AUDIO_DEVICE_IN_WIRED_HEADSET;

    audio_route_reset(adev->ar);
    
    if (speaker_on)
        audio_route_apply_path(adev->ar, "speaker");
    if (headphone_on)
        audio_route_apply_path(adev->ar, "headphone");
    if (main_mic_on)
        audio_route_apply_path(adev->ar, "main-mic");
    if (headset_mic_on)
        audio_route_apply_path(adev->ar, "headset-mic");

    audio_route_update_mixer(adev->ar);
    
    ALOGV("%s : hp=%c speaker=%c main-mic=%c headset-mic=%c",__func__,
      headphone_on ? 'y' : 'n', speaker_on ? 'y' : 'n',
      main_mic_on ? 'y' : 'n', headset_mic_on ? 'y' : 'n' );
}

/* must be called with hw device and output stream mutexes locked */
static void do_out_standby(struct stream_out *out)
{
    struct audio_device *adev = out->dev;
    if (!out->standby) {
        pcm_close(out->pcm);
        out->pcm = NULL;
        adev->active_out = NULL;
        if (out->resampler) {
            release_resampler(out->resampler);
            out->resampler = NULL;
        }
        if (out->buffer) {
            free(out->buffer);
            out->buffer = NULL;
        }
        out->standby = true;
    }
}

/* must be called with hw device and input stream mutexes locked */
static void do_in_standby(struct stream_in *in)
{
    struct audio_device *adev = in->dev;
    if (!in->standby) {
        pcm_close(in->pcm);
        in->pcm = NULL;
        adev->active_in = NULL;
        if (in->resampler) {
            release_resampler(in->resampler);
            in->resampler = NULL;
        }
        if (in->buffer) {
            free(in->buffer);
            in->buffer = NULL;
        }
        in->standby = true;
    }
}

static int get_pcm_card(const char* name)
{
        char id_filepath[PATH_MAX] = {0};
        char number_filepath[PATH_MAX] = {0};
        ssize_t written;

        snprintf(id_filepath, sizeof(id_filepath), "/proc/asound/%s", name);

        written = readlink(id_filepath, number_filepath, sizeof(number_filepath));
        if (written < 0) {
            ALOGE("Sound card %s does not exist\n", name);
            return -1;
        } else if (written >= (ssize_t)sizeof(id_filepath)) {
            ALOGE("Sound card %s name is too long - setting default \n", name);
            return -1;
        }
        ALOGI("Sound card %s exists\n", name);
        return atoi(number_filepath + 4);
}

/*
 * parse_pcm_node() - Parse a /dev/snd/ PCM node name like "pcmC0D0p".
 *
 * Format: pcmC<card>D<device><p|c>
 *   - 'p' = playback (PCM_OUT)
 *   - 'c' = capture  (PCM_IN)
 *
 * @node_name: The node name to parse (e.g., "pcmC0D0p" or "pcmC1D3c").
 * @out_card: Output pointer for the parsed card number.
 * @out_device: Output pointer for the parsed device number.
 * @direction: Expected direction (PCM_OUT or PCM_IN). Returns -1 if mismatched.
 *
 * Returns 0 on success, -1 on parse failure or direction mismatch.
 */
static int parse_pcm_node(const char *node_name, int *out_card, int *out_device,
                           unsigned int direction)
{
    int card_num, device_num;
    char dir_char;

    if (sscanf(node_name, "pcmC%dD%d%c", &card_num, &device_num, &dir_char) != 3)
        return -1;

    /* Validate direction character */
    if (dir_char == 'p' && direction != PCM_OUT)
        return -1;
    if (dir_char == 'c' && direction != PCM_IN)
        return -1;
    if (dir_char != 'p' && dir_char != 'c')
        return -1;

    /* Validate the node actually exists in /dev/snd/ */
    char dev_path[PATH_MAX];
    snprintf(dev_path, sizeof(dev_path), "/dev/snd/%s", node_name);
    if (access(dev_path, F_OK) != 0) {
        ALOGW("%s: node '%s' does not exist", __func__, dev_path);
        return -1;
    }

    *out_card = card_num;
    *out_device = device_num;
    return 0;
}

/*
 * try_resolve_card() - Resolve a property value to a card number.
 *
 * Supports two formats:
 *   - /dev/snd node name: e.g. "pcmC0D0p", "pcmC1D0c"
 *   - /proc/asound name:  e.g. "PCH", "Intel", "sofhdadsp"
 *
 * Returns card number >= 0 on success, -1 on failure.
 */
static int try_resolve_card(const char *value, unsigned int direction)
{
    int card, dev;

    /* Try as /dev/snd/ node name first (e.g. "pcmC0D0p") */
    if (parse_pcm_node(value, &card, &dev, direction) == 0)
        return card;

    /* Try as /proc/asound/ card name (e.g. "PCH") */
    card = get_pcm_card(value);
    return card;
}

/*
 * scan_devices() - Scan /dev/snd/ for the first suitable PCM card.
 *
 * Iterates over /dev/snd/ entries, filtering by direction (playback/capture).
 * Skips cards whose id contains "IntelHDMI" (those belong to the HDMI HAL module).
 *
 * @direction: PCM_OUT or PCM_IN
 * Returns card number >= 0 on success, -1 if nothing found.
 */
static int scan_devices(unsigned int direction)
{
    struct dirent **namelist;
    int n;
    int found_card = -1;
    char dir_suffix = (direction == PCM_OUT) ? 'p' : 'c';
    int is_input = (direction == PCM_IN) ? 1 : 0;

    n = scandir("/dev/snd/", &namelist, NULL, alphasort);
    if (n < 0) {
        ALOGW("%s: failed to scan /dev/snd/: %s", __func__, strerror(errno));
        return -1;
    }

    for (int i = 0; i < n; i++) {
        const char *name = namelist[i]->d_name;
        int len = strlen(name);

        /* Filter: must start with "pcmC" and end with direction suffix */
        if (strncmp(name, "pcmC", 4) != 0 || len < 6 || name[len - 1] != dir_suffix) {
            free(namelist[i]);
            continue;
        }

        if (found_card >= 0) {
            /* Already found one, skip the rest */
            free(namelist[i]);
            continue;
        }

        char dev_path[PATH_MAX];
        snprintf(dev_path, sizeof(dev_path), "/dev/snd/%s", name);

        int fd = open(dev_path, O_RDWR);
        if (fd < 0) {
            ALOGV("%s: cannot open %s: %s", __func__, dev_path, strerror(errno));
            free(namelist[i]);
            continue;
        }

        struct snd_pcm_info info;
        if (ioctl(fd, SNDRV_PCM_IOCTL_INFO, &info) != 0) {
            close(fd);
            free(namelist[i]);
            continue;
        }
        close(fd);

        /* Verify direction matches */
        if ((int)info.stream != is_input) {
            free(namelist[i]);
            continue;
        }

        /* Skip IntelHDMI cards — those belong to the HDMI HAL */
        if (strstr((const char *)info.id, "IntelHDMI")) {
            ALOGV("%s: skipping IntelHDMI card '%s'", __func__, name);
            free(namelist[i]);
            continue;
        }

        found_card = info.card;
        ALOGI("%s: found card %d ('%s') via /dev/snd/ scan", __func__,
              found_card, info.id);
        free(namelist[i]);
    }

    free(namelist);
    return found_card;
}

/*
 * get_format_from_prop() - Read PCM format from a vendor property.
 *
 * Validates only S16_LE, S32_LE, and S8. Falls back to default_format if
 * property is unset or value is invalid.
 */
static enum pcm_format get_format_from_prop(const char *prop_name,
                                             enum pcm_format default_format)
{
    char value[PROPERTY_VALUE_MAX];
    if (property_get(prop_name, value, NULL) <= 0)
        return default_format;

    int fmt = atoi(value);
    switch (fmt) {
    case PCM_FORMAT_S16_LE:
    case PCM_FORMAT_S32_LE:
    case PCM_FORMAT_S8:
        return (enum pcm_format)fmt;
    default:
        ALOGW("%s: invalid format %d from property '%s', using default",
              __func__, fmt, prop_name);
        return default_format;
    }
}

/*
 * find_card() - Unified card discovery with property override support.
 *
 * The property value supports two formats:
 *   - /proc/asound name:  e.g. "PCH", "Intel", "sofhdadsp"
 *   - /dev/snd node name: e.g. "pcmC0D0p", "pcmC1D0c"
 *
 * Fallback chain:
 *   1. Check per-device vendor.hal.* property (based on routing)
 *   2. Check generic vendor.hal.audio.out / vendor.hal.audio.in property
 *   3. Try Celadon's standard chain: PCH -> Intel -> sofhdadsp -> Dummy
 *   4. Scan /dev/snd/ for first suitable card
 *   5. Return -1 if nothing found
 *
 * @prop_key: Generic property name (e.g., "vendor.hal.audio.out"), or NULL to skip.
 * @direction: PCM_OUT or PCM_IN, used to validate the card supports the direction.
 * @routing: Audio device routing mask for per-device property lookup. Pass 0 to skip.
 */
static int find_card(const char *prop_key, unsigned int direction, unsigned int routing)
{
    char prop_name[PROPERTY_VALUE_MAX];
    char prop_value[PROPERTY_VALUE_MAX];
    int card;

    /* Step 1: Check per-device property override based on routing */
    if (routing != 0) {
        const char *suffix = NULL;
        if (direction == PCM_OUT) {
            if (routing & (AUDIO_DEVICE_OUT_WIRED_HEADSET | AUDIO_DEVICE_OUT_WIRED_HEADPHONE))
                suffix = "headphone";
            else if (routing & AUDIO_DEVICE_OUT_SPEAKER)
                suffix = "speaker";
            else if (routing & AUDIO_DEVICE_OUT_AUX_DIGITAL)
                suffix = "hdmi";
        } else {
            if (routing & AUDIO_DEVICE_IN_WIRED_HEADSET)
                suffix = "headset";
            else if (routing & AUDIO_DEVICE_IN_BUILTIN_MIC)
                suffix = "mic";
        }

        if (suffix) {
            snprintf(prop_name, sizeof(prop_name), "vendor.hal.audio.%s.%s",
                     (direction == PCM_OUT) ? "out" : "in", suffix);
            if (property_get(prop_name, prop_value, NULL) > 0) {
                card = try_resolve_card(prop_value, direction);
                if (card >= 0) {
                    ALOGI("%s: using card '%s' (card %d) from per-device property '%s'",
                          __func__, prop_value, card, prop_name);
                    return card;
                }
                ALOGW("%s: per-device property '%s' set to '%s' but card not found",
                      __func__, prop_name, prop_value);
            }
        }
    }

    /* Step 2: Check generic property override */
    if (prop_key && property_get(prop_key, prop_value, NULL) > 0) {
        card = try_resolve_card(prop_value, direction);
        if (card >= 0) {
            ALOGI("%s: using card '%s' (card %d) from property '%s'",
                  __func__, prop_value, card, prop_key);
            return card;
        }

        ALOGW("%s: property '%s' set to '%s' but card not found, trying defaults",
              __func__, prop_key, prop_value);
    }

    /* Step 3: Celadon's standard card name chain */
    static const char *card_names[] = { "PCH", "Intel", "sofhdadsp", "Dummy", NULL };
    for (int i = 0; card_names[i] != NULL; i++) {
        card = get_pcm_card(card_names[i]);
        if (card >= 0) {
            struct pcm_params *params = pcm_params_get(card, PCM_DEVICE, direction);
            if (params) {
                free(params);
                ALOGI("%s: found card '%s' (card %d)", __func__, card_names[i], card);
                return card;
            }
            /* Card exists but doesn't support this direction, keep trying */
            ALOGW("%s: card '%s' (card %d) found but no %s support",
                  __func__, card_names[i], card,
                  (direction == PCM_OUT) ? "playback" : "capture");
        }
    }

    /* Step 4: Scan /dev/snd/ for first suitable card */
    card = scan_devices(direction);
    if (card >= 0)
        return card;

    ALOGE("%s: no suitable card found", __func__);
    return -1;
}

void update_bt_card(struct audio_device *adev){
    adev->bt_card = get_pcm_card(AUDIO_BT_DRIVER_NAME); //update driver name if changed from BT side.
}

static unsigned int round_to_16_mult(unsigned int size)
{
    return (size + 15) & ~15;   /* 0xFFFFFFF0; */
}

/* must be called with hw device and output stream mutexes locked */
static int start_output_stream(struct stream_out *out)
{
    struct audio_device *adev = out->dev;

    ALOGV("%s : config : [rate %d format %d channels %d]",__func__,
            out->pcm_config.rate, out->pcm_config.format, out->pcm_config.channels);

    if (out->unavailable) {
        ALOGV("start_output_stream: output not available");
        return -ENODEV;
    }

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        ALOGD("%s : sco voip call active", __func__);

        ALOGV("%s : opening pcm [%d : %d] for config : [rate %d format %d channels %d]", __func__, adev->bt_card, PCM_DEVICE,
                bt_out_config.rate, bt_out_config.format, bt_out_config.channels);

        out->pcm = pcm_open(adev->bt_card, PCM_DEVICE /*0*/, PCM_OUT, &bt_out_config);
//BT SCO VoIP Call]
    } else {
        /* Check format override property */
        out->pcm_config.format = get_format_from_prop("vendor.hal.audio.out.format",
                                                       out->pcm_config.format);

        ALOGI("PCM playback card selected = %d, \n", adev->card);
        out->pcm = pcm_open(adev->card, PCM_DEVICE, PCM_OUT | PCM_NORESTART | PCM_MONOTONIC, &out->pcm_config);

        /* 44100 Hz fallback: retry if pcm_open failed at the default rate */
        if ((!out->pcm || !pcm_is_ready(out->pcm)) && out->pcm_config.rate != OUT_SAMPLING_RATE_FALLBACK) {
            ALOGW("%s: pcm_open failed at %dHz, retrying at %dHz",
                  __func__, out->pcm_config.rate, OUT_SAMPLING_RATE_FALLBACK);
            if (out->pcm) {
                pcm_close(out->pcm);
                out->pcm = NULL;
            }
            out->pcm_config.rate = OUT_SAMPLING_RATE_FALLBACK;
            out->pcm = pcm_open(adev->card, PCM_DEVICE, PCM_OUT | PCM_NORESTART | PCM_MONOTONIC, &out->pcm_config);
        }
    }

    if (!out->pcm) {
        ALOGE("pcm_open(out) failed: device not found");
        return -ENODEV;
    } else if (!pcm_is_ready(out->pcm)) {
        ALOGE("pcm_open(out) failed: %s", pcm_get_error(out->pcm));
        pcm_close(out->pcm);
        out->unavailable = true;
        return -ENOMEM;
    }

    /* Create output resampler if framework rate differs from HW rate */
    if (out_get_sample_rate(&out->stream.common) != out->pcm_config.rate) {
        int ret = create_resampler(out_get_sample_rate(&out->stream.common),  /* src: framework rate */
                               out->pcm_config.rate,                           /* dst: HW rate */
                               out->pcm_config.channels,
                               RESAMPLER_QUALITY_DEFAULT,
                               NULL,                                           /* push mode */
                               &out->resampler);
        if (ret != 0) {
            ALOGE("%s: failed to create output resampler (%d -> %d): %d",
                  __func__, out_get_sample_rate(&out->stream.common),
                  out->pcm_config.rate, ret);
            pcm_close(out->pcm);
            out->pcm = NULL;
            return ret;
        }
        out->buffer_frames = (pcm_config_out.period_size * out->pcm_config.rate) /
                out_get_sample_rate(&out->stream.common) + 1;
        out->buffer = malloc(pcm_frames_to_bytes(out->pcm, out->buffer_frames));
        if (!out->buffer) {
            release_resampler(out->resampler);
            out->resampler = NULL;
            pcm_close(out->pcm);
            out->pcm = NULL;
            return -ENOMEM;
        }
        ALOGI("%s: output resampler created: %d -> %d Hz", __func__,
              out_get_sample_rate(&out->stream.common), out->pcm_config.rate);
    }

    adev->active_out = out;

    /* force mixer updates */
    select_devices(adev);

    return 0;
}

/*
 * Input resampler buffer provider callbacks.
 * These are used by the resampler in pull mode to fetch PCM data from hardware.
 */

static int get_next_buffer(struct resampler_buffer_provider *buffer_provider,
                           struct resampler_buffer *buffer)
{
    struct stream_in *in;

    if (buffer_provider == NULL || buffer == NULL)
        return -EINVAL;

    in = (struct stream_in *)((char *)buffer_provider -
                              offsetof(struct stream_in, buf_provider));

    if (in->pcm == NULL) {
        buffer->raw = NULL;
        buffer->frame_count = 0;
        in->read_status = -ENODEV;
        return -ENODEV;
    }

    if (in->frames_in == 0) {
        size_t read_size = in->buffer_size;

        /* For S8 format, only read half (data will expand during conversion) */
        if (in->pcm_config.format == PCM_FORMAT_S8)
            read_size /= 2;

        in->read_status = pcm_read(in->pcm, in->buffer, read_size);
        if (in->read_status != 0) {
            ALOGE("%s: pcm_read error %d", __func__, in->read_status);
            buffer->raw = NULL;
            buffer->frame_count = 0;
            return in->read_status;
        }

        /* In-place format conversion to S16 */
        if (in->pcm_config.format == PCM_FORMAT_S32_LE) {
            memcpy_by_audio_format(in->buffer, AUDIO_FORMAT_PCM_16_BIT,
                                   in->buffer, AUDIO_FORMAT_PCM_32_BIT,
                                   in->pcm_config.period_size * in->pcm_config.channels);
        } else if (in->pcm_config.format == PCM_FORMAT_S8) {
            memcpy_by_audio_format(in->buffer, AUDIO_FORMAT_PCM_16_BIT,
                                   in->buffer, AUDIO_FORMAT_PCM_8_BIT,
                                   in->pcm_config.period_size * in->pcm_config.channels);
        }

        in->frames_in = in->pcm_config.period_size;
    }

    buffer->frame_count = (buffer->frame_count > in->frames_in) ?
                          in->frames_in : buffer->frame_count;
    buffer->i16 = in->buffer + (in->pcm_config.period_size - in->frames_in) *
                  in->pcm_config.channels;

    return in->read_status;
}

static void release_buffer(struct resampler_buffer_provider *buffer_provider,
                           struct resampler_buffer *buffer)
{
    struct stream_in *in;

    if (buffer_provider == NULL || buffer == NULL)
        return;

    in = (struct stream_in *)((char *)buffer_provider -
                              offsetof(struct stream_in, buf_provider));

    in->frames_in -= buffer->frame_count;
}

/*
 * read_frames() - Read audio frames, applying resampling if needed.
 *
 * If resampler exists, pulls through the buffer provider chain.
 * Otherwise reads directly from PCM via get_next_buffer/release_buffer.
 */
static ssize_t read_frames(struct stream_in *in, void *buffer, ssize_t frames)
{
    ssize_t frames_wr = 0;

    while (frames_wr < frames) {
        size_t frames_rd = frames - frames_wr;

        if (in->resampler != NULL) {
            in->resampler->resample_from_provider(in->resampler,
                    (int16_t *)((char *)buffer +
                        frames_wr * in->pcm_config.channels * sizeof(int16_t)),
                    &frames_rd);
        } else {
            struct resampler_buffer buf = {
                .raw = NULL,
                .frame_count = frames_rd,
            };
            get_next_buffer(&in->buf_provider, &buf);
            if (buf.raw != NULL) {
                memcpy((char *)buffer +
                           frames_wr * in->pcm_config.channels * sizeof(int16_t),
                       buf.raw,
                       buf.frame_count * in->pcm_config.channels * sizeof(int16_t));
                frames_rd = buf.frame_count;
            }
            release_buffer(&in->buf_provider, &buf);
        }

        /* read_status is set by get_next_buffer() */
        if (in->read_status != 0)
            return in->read_status;

        frames_wr += frames_rd;
    }
    return frames_wr;
}

/* must be called with hw device and input stream mutexes locked */
static int start_input_stream(struct stream_in *in)
{
    struct audio_device *adev = in->dev;
    int ret;

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        ALOGD("%s : sco voip call active", __func__);

        ALOGV("%s : opening pcm [%d : %d] for config : [rate %d format %d channels %d]",__func__, adev->bt_card, PCM_DEVICE,
                bt_in_config.rate, bt_in_config.format, bt_in_config.channels);

        in->pcm = pcm_open(adev->bt_card, PCM_DEVICE, PCM_IN, &bt_in_config);
//BT SCO VoIP Call]
    } else {
        /* Check format override property */
        in->pcm_config.format = get_format_from_prop("vendor.hal.audio.in.format",
                                                      in->pcm_config.format);

        ALOGI("PCM record card selected = %d, \n", adev->cardc);

        ALOGV("%s : config : [rate %d format %d channels %d]",__func__,
            in->pcm_config.rate, in->pcm_config.format, in->pcm_config.channels);

        in->pcm = pcm_open(adev->cardc, PCM_DEVICE, PCM_IN, &in->pcm_config);

        /* 44100 Hz fallback: retry if pcm_open failed at the default rate */
        if ((!in->pcm || !pcm_is_ready(in->pcm)) && in->pcm_config.rate != IN_SAMPLING_RATE_FALLBACK) {
            ALOGW("%s: pcm_open failed at %dHz, retrying at %dHz",
                  __func__, in->pcm_config.rate, IN_SAMPLING_RATE_FALLBACK);
            if (in->pcm) {
                pcm_close(in->pcm);
                in->pcm = NULL;
            }
            in->pcm_config.rate = IN_SAMPLING_RATE_FALLBACK;
            in->pcm = pcm_open(adev->cardc, PCM_DEVICE, PCM_IN, &in->pcm_config);
        }
    }

    if (!in->pcm) {
        return -ENODEV;
    } else if (!pcm_is_ready(in->pcm)) {
        ALOGE("pcm_open(in) failed: %s", pcm_get_error(in->pcm));
        pcm_close(in->pcm);
        return -ENOMEM;
    }

    /* Allocate read buffer for input resampler / format conversion */
    in->buffer_size = pcm_frames_to_bytes(in->pcm, in->pcm_config.period_size);
    /* For S8 format, allocate double size for S8→S16 in-place expansion */
    if (in->pcm_config.format == PCM_FORMAT_S8)
        in->buffer_size *= 2;
    in->buffer = malloc(in->buffer_size);
    if (!in->buffer) {
        pcm_close(in->pcm);
        in->pcm = NULL;
        return -ENOMEM;
    }
    in->frames_in = 0;
    in->read_status = 0;

    /* Create input resampler if HW rate differs from requested rate */
    if (in->requested_rate != in->pcm_config.rate) {
        in->buf_provider.get_next_buffer = get_next_buffer;
        in->buf_provider.release_buffer = release_buffer;
        ret = create_resampler(in->pcm_config.rate,         /* src: HW rate */
                               in->requested_rate,            /* dst: framework rate */
                               in->pcm_config.channels,
                               RESAMPLER_QUALITY_DEFAULT,
                               &in->buf_provider,             /* pull mode */
                               &in->resampler);
        if (ret != 0) {
            ALOGE("%s: failed to create input resampler (%d -> %d): %d",
                  __func__, in->pcm_config.rate, in->requested_rate, ret);
            free(in->buffer);
            in->buffer = NULL;
            pcm_close(in->pcm);
            in->pcm = NULL;
            return ret;
        }
        ALOGI("%s: input resampler created: %d -> %d Hz", __func__,
              in->pcm_config.rate, in->requested_rate);
    }

    adev->active_in = in;

    /* force mixer updates */
    select_devices(adev);

    return 0;
}

/* API functions */

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    ALOGV("%s : rate %d",__func__, out->req_config.sample_rate);
    return out->req_config.sample_rate;
}

static int out_set_sample_rate(struct audio_stream *stream __unused, uint32_t rate __unused)
{
    ALOGV("out_set_sample_rate: %d", rate);
    return -ENOSYS;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    ALOGV("out_get_buffer_size");
    return pcm_config_out.period_size *
               audio_stream_out_frame_size((struct audio_stream_out *)stream);
}

static uint32_t out_get_channels(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    ALOGV("%s : channels %d",__func__,  popcount(out->req_config.channel_mask));
    return out->req_config.channel_mask;
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    ALOGV("%s",__func__);
    struct stream_out *out = (struct stream_out *)stream;
    return out->req_config.format;
}

static int out_set_format(struct audio_stream *stream __unused, audio_format_t format __unused)
{
    return -ENOSYS;
}

static int out_standby(struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;

    ALOGV("out_standby");
    pthread_mutex_lock(&out->dev->lock);
    pthread_mutex_lock(&out->lock);
    do_out_standby(out);
    pthread_mutex_unlock(&out->lock);
    pthread_mutex_unlock(&out->dev->lock);

    return 0;
}

static int out_dump(const struct audio_stream *stream __unused, int fd __unused)
{
    ALOGV("out_dump");
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    ALOGV("%s : kvpairs : %s",__func__, kvpairs);
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    int status = 0;
    unsigned int val;

    parms = str_parms_create_str(kvpairs);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));

    pthread_mutex_lock(&adev->lock);
    if (ret >= 0) {
        val = atoi(value);
        if ((adev->out_device != val) && (val != 0)) {
            adev->out_device = val;
            select_devices(adev);
        }

#if 0 // FIXME, does this need to replace above code ?
	pthread_mutex_lock(&out->lock);
        if (((adev->devices & AUDIO_DEVICE_OUT_ALL) != val) && (val != 0)) {
            adev->devices &= ~AUDIO_DEVICE_OUT_ALL;
            adev->devices |= val;
        }
        pthread_mutex_unlock(&out->lock);
#endif

    }
    pthread_mutex_unlock(&adev->lock);
    
    str_parms_destroy(parms);
    return status;
}

static char *out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    ALOGV("%s : keys : %s",__func__,keys);
    struct stream_out *out = (struct stream_out *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str_parm = NULL;
    char value[256];
    struct str_parms *reply = str_parms_create();
    int ret;

    if(reply == NULL || query == NULL) {
        if(reply != NULL) str_parms_destroy(reply);
        if(query != NULL) str_parms_destroy(query);
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_FORMATS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_FORMATS, "AUDIO_FORMAT_PCM_16_BIT");
        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_int(reply, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, out->req_config.sample_rate);

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_CHANNELS,
            (out->req_config.channel_mask == AUDIO_CHANNEL_OUT_MONO ? "AUDIO_CHANNEL_OUT_MONO" : "AUDIO_CHANNEL_OUT_STEREO"));

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    str_parms_destroy(query);
    str_parms_destroy(reply);

    ALOGV("%s : returning keyValuePair %s",__func__, str_parm);
    return str_parm;
}

static uint32_t out_get_latency(const struct audio_stream_out *stream __unused)
{
    ALOGV("out_get_latency");
    return (pcm_config_out.period_size * OUT_PERIOD_COUNT * 1000) / pcm_config_out.rate;
}

static int out_set_volume(struct audio_stream_out *stream __unused, float left __unused,
                          float right __unused)
{
     ALOGV("out_set_volume: Left:%f Right:%f", left, right);
     return -ENOSYS;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    int ret = 0;
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    size_t frame_size = audio_stream_out_frame_size(stream);
    int16_t *out_buffer = (int16_t *)buffer;
    unsigned int out_frames = bytes / frame_size;

    ALOGV("out_write: bytes: %zu", bytes);

    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the output stream mutex - e.g.
     * executing out_set_parameters() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&out->lock);

    if(adev->out_needs_standby) {
        do_out_standby(out);
        adev->out_needs_standby = false;
    }

    if (out->standby) {
        if(!adev->is_hfp_call_active) {
            ret = start_output_stream(out);
        } else {
            ret = -1;
        }
        if (ret != 0) {
            pthread_mutex_unlock(&adev->lock);
            goto exit;
        }
        out->standby = false;
    }
    pthread_mutex_unlock(&adev->lock);

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        /* VoIP pcm write in celadon devices goes to bt alsa card */
        size_t frames_in = round_to_16_mult(out->pcm_config.period_size);
        size_t frames_out = round_to_16_mult(bt_out_config.period_size);
        size_t buf_size_out = bt_out_config.channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_in = out->pcm_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_remapped = bt_out_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        int16_t *buf_out = (int16_t *) malloc (buf_size_out);
        int16_t *buf_in = (int16_t *) malloc (buf_size_in);
        int16_t *buf_remapped = (int16_t *) malloc (buf_size_remapped);

        if(adev->voip_out_resampler == NULL) {
            int ret = create_resampler(out->pcm_config.rate /*src rate*/, bt_out_config.rate /*dst rate*/, bt_out_config.channels/*dst channels*/,
                            RESAMPLER_QUALITY_DEFAULT, NULL, &(adev->voip_out_resampler));
            ALOGD("%s : frames_in %zu frames_out %zu",__func__, frames_in, frames_out);
            ALOGD("%s : to write bytes : %zu", __func__, bytes);
            ALOGD("%s : size_in %zu size_out %zu size_remapped %zu", __func__, buf_size_in, buf_size_out, buf_size_remapped);

            if (ret != 0) {
                adev->voip_out_resampler = NULL;
                ALOGE("%s : Failure to create resampler %d", __func__, ret);

                free(buf_in);
                free(buf_out);
                free(buf_remapped);
                goto exit;
            } else {
                ALOGD("%s : voip_out_resampler created rate : [%d -> %d]", __func__, out->pcm_config.rate, bt_out_config.rate);
            }
        }

        memset(buf_in, 0, buf_size_in);
        memset(buf_remapped, 0, buf_size_remapped);
        memset(buf_out, 0, buf_size_out);

        memcpy(buf_in, buffer, buf_size_in);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_write != NULL) {
            fwrite(buf_in, 1, buf_size_in, sco_call_write);
        } else {
            ALOGD("%s : sco_call_write was NULL, no dump", __func__);
        }
#endif

        adjust_channels(buf_in, out->pcm_config.channels, buf_remapped, bt_out_config.channels, 
                                        SAMPLE_SIZE_IN_BYTES, buf_size_in);

        //ALOGV("remapping : [%d -> %d]", out->pcm_config.channels, bt_out_config.channels);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_write_remapped != NULL) {
            fwrite(buf_remapped, 1, buf_size_remapped, sco_call_write_remapped);
        } else {
            ALOGD("%s : sco_call_write_remapped was NULL, no dump", __func__);
        }
#endif

        if(adev->voip_out_resampler != NULL) {
            adev->voip_out_resampler->resample_from_input(adev->voip_out_resampler, (int16_t *)buf_remapped, (size_t *)&frames_in, (int16_t *) buf_out, (size_t *)&frames_out);
            //ALOGV("%s : upsampling [%d -> %d]",__func__, out->pcm_config.rate, bt_out_config.rate);
        }

        ALOGV("%s : modified frames_in %zu frames_out %zu",__func__, frames_in, frames_out);

        buf_size_out = bt_out_config.channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        bytes = out->pcm_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;

#ifdef DEBUG_PCM_DUMP
        if(sco_call_write_bt != NULL) {
            fwrite(buf_out, 1, buf_size_out, sco_call_write_bt);
        } else {
            ALOGD("%s : sco_call_write was NULL, no dump", __func__);
        }
#endif

        ret = pcm_write(out->pcm, buf_out, buf_size_out);

        free(buf_in);
        free(buf_out);
        free(buf_remapped);
//BT SCO VoIP Call]
    } else {
        /* Adaptive buffering: adjust period count based on screen state */
        if (adev->screen_off && adev->active_in == NULL) {
            out->pcm_config.period_count = OUT_LONG_PERIOD_COUNT;
        } else {
            out->pcm_config.period_count = OUT_SHORT_PERIOD_COUNT;
        }

        /* Apply output resampling if active */
        if (out->resampler) {
            size_t in_frames = out_frames;
            out_frames = out->buffer_frames;
            out->resampler->resample_from_input(out->resampler,
                    (int16_t *)buffer, &in_frames,
                    out->buffer, (size_t *)&out_frames);
            out_buffer = out->buffer;
        }

        /* Format conversion: HAL always works in S16_LE internally */
        if (out->pcm_config.format == PCM_FORMAT_S32_LE) {
            unsigned int new_buffer_size = out_frames * frame_size * 2;
            uint8_t resize_buffer[new_buffer_size];
            memcpy_by_audio_format((void*)resize_buffer, AUDIO_FORMAT_PCM_32_BIT,
                (void*)out_buffer, AUDIO_FORMAT_PCM_16_BIT,
                out_frames * out->pcm_config.channels);
            ret = pcm_write(out->pcm, resize_buffer, new_buffer_size);
        } else if (out->pcm_config.format == PCM_FORMAT_S8) {
            unsigned int new_buffer_size = out_frames * frame_size / 2;
            uint8_t resize_buffer[new_buffer_size];
            memcpy_by_audio_format((void*)resize_buffer, AUDIO_FORMAT_PCM_8_BIT,
                (void*)out_buffer, AUDIO_FORMAT_PCM_16_BIT,
                out_frames * out->pcm_config.channels);
            ret = pcm_write(out->pcm, resize_buffer, new_buffer_size);
        } else {
            /* Normal S16_LE path */
            ret = pcm_write(out->pcm, out_buffer, out_frames * frame_size);
        }

#ifdef DEBUG_PCM_DUMP
        if(out_write_dump != NULL) {
            fwrite(out_buffer, 1, out_frames * frame_size, out_write_dump);
        } else {
            ALOGD("%s : out_write_dump was NULL, no dump", __func__);
        }
#endif

        if (ret == -EPIPE) {
            /* In case of underrun, don't sleep since we want to catch up asap */
            pthread_mutex_unlock(&out->lock);
            return ret;
        }
    }

    if (ret == 0) {
        out->written += out_frames;
    }

exit:
    pthread_mutex_unlock(&out->lock);

    if (ret != 0) {
        ALOGW("out_write error: %d, sleeping...", ret);
        usleep(bytes * 1000000 / audio_stream_out_frame_size(stream) /
               out_get_sample_rate(&stream->common));
    }

    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    struct stream_out *out = (struct stream_out *)stream;
    *dsp_frames = out->written;
    ALOGV("%s : dsp_frames: %d",__func__, *dsp_frames);
    return 0;
}

static int out_get_presentation_position(const struct audio_stream_out *stream,
                                   uint64_t *frames, struct timespec *timestamp)
{
    struct stream_out *out = (struct stream_out *)stream;
    int ret = -1;

    if (out->pcm) {
        unsigned int avail;
        if (pcm_get_htimestamp(out->pcm, &avail, timestamp) == 0) {
            unsigned int kernel_buffer_size = out->pcm_config.period_size * out->pcm_config.period_count;
            int64_t signed_frames = out->written - kernel_buffer_size + avail;
            if (signed_frames >= 0) {
            *frames = signed_frames;
            ret = 0;
            }
        }
    }

    return ret;
}

static int out_add_audio_effect(const struct audio_stream *stream __unused, effect_handle_t effect __unused)
{
    ALOGV("out_add_audio_effect: %p", effect);
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream __unused, effect_handle_t effect __unused)
{
    ALOGV("out_remove_audio_effect: %p", effect);
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream __unused,
                                        int64_t *timestamp __unused)
{
    ALOGV("%s",__func__);
    return -ENOSYS;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    ALOGV("%s : req_config %d",__func__,in->req_config.sample_rate);
    return in->req_config.sample_rate;
}

static int in_set_sample_rate(struct audio_stream *stream __unused, uint32_t rate __unused)
{
    ALOGV("in_set_sample_rate: %d", rate);
    return -ENOSYS;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (in->pcm_config.period_size * in_get_sample_rate(stream)) /
            in->pcm_config.rate;
    size = ((size + 15) / 16) * 16;

    size *= audio_stream_in_frame_size(&in->stream);
    ALOGV("%s : buffer_size : %zu",__func__, size);
    return size;
}

static uint32_t in_get_channels(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    ALOGV("%s : channels %d",__func__, popcount(in->req_config.channel_mask));
    return in->req_config.channel_mask;
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    ALOGV("%s : req_config format %d",__func__, in->req_config.format);
    return in->req_config.format;
}

static int in_set_format(struct audio_stream *stream __unused, audio_format_t format __unused)
{
    return -ENOSYS;
}

static int in_standby(struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    pthread_mutex_lock(&in->dev->lock);
    pthread_mutex_lock(&in->lock);
    do_in_standby(in);
    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&in->dev->lock);

    return 0;
}

static int in_dump(const struct audio_stream *stream __unused, int fd __unused)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    int status = 0;
    unsigned int val;

    parms = str_parms_create_str(kvpairs);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    pthread_mutex_lock(&adev->lock);
    if (ret >= 0) {
        val = atoi(value) & ~AUDIO_DEVICE_BIT_IN;
        if ((adev->in_device != val) && (val != 0)) {
            adev->in_device = val;
            select_devices(adev);
        }
    }
    pthread_mutex_unlock(&adev->lock);

    str_parms_destroy(parms);
    return status;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    ALOGV("%s : keys : %s",__func__,keys);
    struct stream_in *in = (struct stream_in *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str_parm = NULL;
    char value[256];
    struct str_parms *reply = str_parms_create();
    int ret;

    if(reply == NULL || query == NULL) {
        if(reply != NULL) str_parms_destroy(reply);
        if(query != NULL) str_parms_destroy(query);
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_FORMATS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_FORMATS, "AUDIO_FORMAT_PCM_16_BIT");
        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_int(reply, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, in->req_config.sample_rate);

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_CHANNELS,
            (in->req_config.channel_mask == AUDIO_CHANNEL_IN_MONO ? "AUDIO_CHANNEL_IN_MONO" : "AUDIO_CHANNEL_IN_STEREO"));

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    str_parms_destroy(query);
    str_parms_destroy(reply);

    ALOGV("%s : returning keyValuePair %s",__func__, str_parm);
    return str_parm;
}

static int in_set_gain(struct audio_stream_in *stream __unused, float gain __unused)
{
    return 0;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    int ret = 0;
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;

    ALOGV("%s : bytes_requested : %zu", __func__, bytes);

    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the input stream mutex - e.g.
     * executing in_set_parameters() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&in->lock);

    if(adev->in_needs_standby) {
        do_in_standby(in);
        adev->in_needs_standby = false;
    }

    if (in->standby) {
        if(!adev->is_hfp_call_active) {
            ret = start_input_stream(in);
        } else {
            ret = -1;
        }
        if (ret == 0)
            in->standby = 0;
    }
    pthread_mutex_unlock(&adev->lock);

    if (ret < 0)
        goto exit;

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        /* VoIP pcm read from bt alsa card */
        size_t frames_out = round_to_16_mult(in->pcm_config.period_size);
        size_t frames_in = round_to_16_mult(bt_in_config.period_size);
        size_t buf_size_out = in->pcm_config.channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_in = bt_in_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_remapped = in->pcm_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        int16_t *buf_out = (int16_t *) malloc (buf_size_out);
        int16_t *buf_in = (int16_t *) malloc (buf_size_in);
        int16_t *buf_remapped = (int16_t *) malloc (buf_size_remapped);

        if(adev->voip_in_resampler == NULL) {
            int ret = create_resampler(bt_in_config.rate /*src rate*/, in->pcm_config.rate /*dst rate*/, in->pcm_config.channels/*dst channels*/,
                        RESAMPLER_QUALITY_DEFAULT, NULL, &(adev->voip_in_resampler));
            ALOGV("%s : bytes_requested : %zu", __func__, bytes);
            ALOGV("%s : frames_in %zu frames_out %zu",__func__, frames_in, frames_out);
            ALOGD("%s : size_in %zu size_out %zu size_remapped %zu", __func__, buf_size_in, buf_size_out, buf_size_remapped);
            if (ret != 0) {
                adev->voip_in_resampler = NULL;
                ALOGE("%s : Failure to create resampler %d", __func__, ret);

                free(buf_in);
                free(buf_out);
                free(buf_remapped);
                goto exit;
            } else {
                ALOGD("%s : voip_in_resampler created rate : [%d -> %d]", __func__, bt_in_config.rate, in->pcm_config.rate);
            }
        }

        memset(buf_in, 0, buf_size_in);
        memset(buf_remapped, 0, buf_size_remapped);
        memset(buf_out, 0, buf_size_out);

        ret = pcm_read(in->pcm, buf_in, buf_size_in);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_read != NULL) {
            fwrite(buf_in, 1, buf_size_in, sco_call_read);
        } else {
            ALOGD("%s : sco_call_read was NULL, no dump", __func__);
        }
#endif
        adjust_channels(buf_in, bt_in_config.channels, buf_remapped, in->pcm_config.channels, 
                                        SAMPLE_SIZE_IN_BYTES, buf_size_in);

        //ALOGV("%s : remapping : [%d -> %d]", __func__, bt_in_config.channels, in->pcm_config.channels);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_read_remapped != NULL) {
            fwrite(buf_remapped, 1, buf_size_remapped, sco_call_read_remapped);
        } else {
            ALOGD("%s : sco_call_read_remapped was NULL, no dump", __func__);
        }
#endif

        if(adev->voip_in_resampler != NULL) {
            adev->voip_in_resampler->resample_from_input(adev->voip_in_resampler, (int16_t *)buf_remapped, (size_t *)&frames_in, (int16_t *) buf_out, (size_t *)&frames_out);
            //ALOGV("%s : upsampling [%d -> %d]",__func__, bt_in_config.rate, in->pcm_config.rate);
        }

        ALOGV("%s : modified frames_in %zu frames_out %zu",__func__, frames_in, frames_out);

        buf_size_out = in->pcm_config.channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        bytes = buf_size_out;

#ifdef DEBUG_PCM_DUMP
        if(sco_call_read_bt != NULL) {
            fwrite(buf_out, 1, buf_size_out, sco_call_read_bt);
        } else {
            ALOGD("%s : sco_call_read_bt was NULL, no dump", __func__);
        }
#endif

        memcpy(buffer, buf_out, buf_size_out);

        free(buf_in);
        free(buf_out);
        free(buf_remapped);
//BT SCO VoIP Call]
    } else {
        /* pcm read for primary card — use read_frames for resampling + format conversion */
        size_t in_frames = bytes / (in->pcm_config.channels * sizeof(int16_t));
        ret = read_frames(in, buffer, in_frames);
        if (ret > 0)
            ret = 0;

#ifdef DEBUG_PCM_DUMP
        if(in_read_dump != NULL) {
            fwrite(buffer, 1, bytes, in_read_dump);
        } else {
            ALOGD("%s : in_read_dump was NULL, no dump", __func__);
        }
#endif
    }
    if (ret > 0)
        ret = 0;

    /*
     * Instead of writing zeroes here, we could trust the hardware
     * to always provide zeroes when muted.
     */
    if (ret == 0 && adev->mic_mute)
        memset(buffer, 0, bytes);

exit:
    pthread_mutex_unlock(&in->lock);
    if (ret < 0)
        usleep(bytes * 1000000 / audio_stream_in_frame_size(stream) /
               in_get_sample_rate(&stream->common));

    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream __unused)
{
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream __unused,
                               effect_handle_t effect __unused)
{
    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream __unused,
                                  effect_handle_t effect __unused)
{
    return 0;
}


static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle __unused,
                                   audio_devices_t devices __unused,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out,
                                   const char *address __unused)
{
    ALOGD("%s : requested config : [rate %d format %d channels %d flags %#x]",__func__,
        config->sample_rate, config->format, popcount(config->channel_mask), flags);

    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_out *out;
    struct pcm_params *params;

    int ret;

    adev->card = find_card("vendor.hal.audio.out", PCM_OUT, 0);
    if (adev->card >= 0)
        params = pcm_params_get(adev->card, PCM_DEVICE, PCM_OUT);
    else
        params = NULL;

    if (!params)
        return -ENOSYS;

    ALOGI("PCM playback card selected = %d, \n", adev->card);
    out = (struct stream_out *)calloc(1, sizeof(struct stream_out));
    if (!out) {
        free(params);
        return -ENOMEM;
    }

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;
    out->stream.get_presentation_position = out_get_presentation_position;

    memcpy(&out->pcm_config, &pcm_config_out, sizeof(struct pcm_config));

    out->written = 0;

// VTS : Device doesn't support mono channel or sample_rate other than 48000
//       make a copy of requested config to feed it back if requested.
    memcpy(&out->req_config, config, sizeof(struct audio_config));

    out->dev = adev;
    out->standby = true;
    out->unavailable = false;
    out->resampler = NULL;
    out->buffer = NULL;

    config->format = out_get_format(&out->stream.common);
    config->channel_mask = out_get_channels(&out->stream.common);
    config->sample_rate = out_get_sample_rate(&out->stream.common);

    *stream_out = &out->stream;

    free(params);

    return 0;
}

static void adev_close_output_stream(struct audio_hw_device *dev __unused,
                                     struct audio_stream_out *stream)
{
    out_standby(&stream->common);
    free(stream);
}

//called with adev lock
static void stop_existing_output_input(struct audio_device *adev){
    ALOGD("%s during call scenario", __func__);
    adev->in_needs_standby = true;
    adev->out_needs_standby = true;
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    ALOGD("%s : kvpairs: %s", __func__, kvpairs);

    struct audio_device * adev = (struct audio_device *)dev;
    char value[32];
    int ret;
    struct str_parms *parms;

    parms = str_parms_create_str(kvpairs);

    if(parms == NULL) {
        return 0;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_HFP_ENABLE, value, sizeof(value));
    if (ret >= 0) {
        pthread_mutex_lock(&adev->lock);
        if (strcmp(value, "true") == 0){
            stop_existing_output_input(adev);
            adev->is_hfp_call_active = true;
        } else {
            adev->is_hfp_call_active = false;
        }
        pthread_mutex_unlock(&adev->lock);
    }

//[BT SCO VoIP Call
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_BT_SCO, value, sizeof(value));
    if (ret >= 0) {
        pthread_mutex_lock(&adev->lock);
        if (strcmp(value, "on") == 0){
            adev->in_sco_voip_call = true;
            stop_existing_output_input(adev);
        } else {
            adev->in_sco_voip_call = false;
            stop_existing_output_input(adev);

            release_resampler(adev->voip_in_resampler);
            adev->voip_in_resampler = NULL;
            release_resampler(adev->voip_out_resampler);
            adev->voip_out_resampler = NULL;
        }
        pthread_mutex_unlock(&adev->lock);
    }
//BT SCO VoIP Call]

    /* Screen state tracking for adaptive buffering */
    ret = str_parms_get_str(parms, "screen_state", value, sizeof(value));
    if (ret >= 0) {
        pthread_mutex_lock(&adev->lock);
        if (strcmp(value, "off") == 0) {
            adev->screen_off = true;
            ALOGV("%s: screen off, switching to long buffer", __func__);
        } else {
            adev->screen_off = false;
            ALOGV("%s: screen on, switching to short buffer", __func__);
        }
        pthread_mutex_unlock(&adev->lock);
    }

    str_parms_destroy(parms);
    return 0;
}

static char * adev_get_parameters(const struct audio_hw_device *dev __unused,
                                  const char *keys)
{
    ALOGV("%s : keys : %s",__func__,keys);
    struct str_parms *query = str_parms_create_str(keys);
    char value[256];
    int ret;

    if(query == NULL) {
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_HW_AV_SYNC, value, sizeof(value));
    if (ret >= 0) {
        str_parms_destroy(query);
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_KEY_TTY_MODE, value, sizeof(value));
    if(ret >= 0) {
        ALOGE("%s : no support of TTY",__func__);
        str_parms_destroy(query);
        return NULL;
    }

    str_parms_destroy(query);
    return strdup(keys);
}

static int adev_init_check(const struct audio_hw_device *dev __unused)
{
     ALOGV("adev_init_check");
     return 0;
}

//Supported vol range [0:1], return OK for inrange volume request
static int adev_set_voice_volume(struct audio_hw_device *dev __unused, float volume)
{
    ALOGV("adev_set_voice_volume: %f : This platform provides no such handling", volume);

    int32_t ret = 0;

    if(volume < 0.0f){
        ret = -EINVAL;
    }

    return ret;
}

static int adev_set_master_volume(struct audio_hw_device *dev __unused, float volume __unused)
{
    ALOGV("adev_set_master_volume: %f", volume);
    return -ENOSYS;
}

static int adev_get_master_volume(struct audio_hw_device *dev __unused, float *volume __unused)
{
    ALOGV("adev_get_master_volume:");
    return -ENOSYS;
}

static int adev_set_master_mute(struct audio_hw_device *dev __unused, bool muted __unused)
{
    ALOGV("adev_set_master_mute: %d", muted);
    return -ENOSYS;
}

static int adev_get_master_mute(struct audio_hw_device *dev __unused, bool *muted __unused)
{
    ALOGV("adev_get_master_mute: %d", *muted);
    return -ENOSYS;
}
static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    ALOGD("%s : mode : %d", __func__, mode);
    struct audio_device *adev = (struct audio_device *)dev;

    pthread_mutex_lock(&adev->lock);
    stop_existing_output_input(adev);
    pthread_mutex_unlock(&adev->lock);

    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    ALOGV("adev_set_mic_mute: %d",state);
    struct audio_device *adev = (struct audio_device *)dev;
    adev->mic_mute = state;
    return 0;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    ALOGV("adev_get_mic_mute");
    struct audio_device *adev = (struct audio_device *)dev;
    *state = adev->mic_mute;
    return 0;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev __unused,
                                         const struct audio_config *config)
{
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (pcm_config_in.period_size * config->sample_rate) / pcm_config_in.rate;
    size = ((size + 15) / 16) * 16;

    return (size * popcount(config->channel_mask) *
                audio_bytes_per_sample(config->format));
}

static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle __unused,
                                  audio_devices_t devices __unused,
                                  struct audio_config *config,
                                  struct audio_stream_in **stream_in,
                                  audio_input_flags_t flags __unused,
                                  const char *address __unused,
                                  audio_source_t source __unused)

{
    ALOGD("%s : requested config : [rate %d format %d channels %d flags %#x]",__func__,
        config->sample_rate, config->format, popcount(config->channel_mask), flags);

    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_in *in;
    struct pcm_params *params;

    *stream_in = NULL;

    adev->cardc = find_card("vendor.hal.audio.in", PCM_IN, 0);
    if (adev->cardc >= 0)
        params = pcm_params_get(adev->cardc, PCM_DEVICE, PCM_IN);
    else
        params = NULL;

    if (!params)
        return -ENOSYS;

    ALOGI("PCM capture card selected = %d, \n", adev->cardc);

    in = (struct stream_in *)calloc(1, sizeof(struct stream_in));
    if (!in) {
        free(params);
        return -ENOMEM;
    }

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

    in->dev = adev;
    in->standby = true;

    memcpy(&in->pcm_config, &pcm_config_in, sizeof(struct pcm_config));

    in->requested_rate = config->sample_rate;
    in->resampler = NULL;
    in->buffer = NULL;
    in->frames_in = 0;

// VTS : Device doesn't support mono channel or sample_rate other than 48000
//       make a copy of requested config to feed it back if requested.
    memcpy(&in->req_config, config, sizeof(struct audio_config));

    *stream_in = &in->stream;

    free(params);
    return 0;
}

static void adev_close_input_stream(struct audio_hw_device *dev __unused,
                                   struct audio_stream_in *stream)
{
    ALOGV("adev_close_input_stream...");

    in_standby(&stream->common);
    free(stream);
}

static int adev_dump(const audio_hw_device_t *device __unused, int fd __unused)
{
    ALOGV("adev_dump");
    return 0;
}

static int adev_get_microphones(const audio_hw_device_t *device __unused, struct audio_microphone_characteristic_t *mic_array, size_t *actual_mics)
{
    ALOGV("%s",__func__);
    int32_t ret = 0;
    *actual_mics = 1;
    memset(&mic_array[0], 0, sizeof(mic_array[0]));

    return ret;
}

static int adev_close(hw_device_t *device)
{
    ALOGV("adev_close");

    struct audio_device *adev = (struct audio_device *)device;

    audio_route_free(adev->ar);

#ifdef DEBUG_PCM_DUMP
    if(sco_call_write != NULL) {
        fclose(sco_call_write);
    }
    if(sco_call_write_remapped != NULL) {
        fclose(sco_call_write_remapped);
    }
    if(sco_call_write_bt != NULL) {
        fclose(sco_call_write_bt);
    }
    if(sco_call_read != NULL) {
        fclose(sco_call_read);
    }
    if(sco_call_read_remapped != NULL) {
        fclose(sco_call_read_remapped);
    }
    if(sco_call_read_bt != NULL) {
        fclose(sco_call_read_bt);
    }
    if(out_write_dump != NULL) {
        fclose(out_write_dump);
    }
    if(in_read_dump != NULL) {
        fclose(in_read_dump);
    }
#endif

    free(device);
    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    ALOGV("adev_open: %s", name);

    struct audio_device *adev;
    int card = 0;
    char mixer_path[PATH_MAX];

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct audio_device));
    if (!adev)
        return -ENOMEM;

    adev->hw_device.common.tag = HARDWARE_DEVICE_TAG;
    adev->hw_device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->hw_device.common.module = (struct hw_module_t *) module;
    adev->hw_device.common.close = adev_close;
    adev->hw_device.init_check = adev_init_check;
    adev->hw_device.set_voice_volume = adev_set_voice_volume;
    adev->hw_device.set_master_volume = adev_set_master_volume;
    adev->hw_device.get_master_volume = adev_get_master_volume;
    adev->hw_device.set_master_mute = adev_set_master_mute;
    adev->hw_device.get_master_mute = adev_get_master_mute;
    adev->hw_device.set_mode = adev_set_mode;
    adev->hw_device.set_mic_mute = adev_set_mic_mute;
    adev->hw_device.get_mic_mute = adev_get_mic_mute;
    adev->hw_device.set_parameters = adev_set_parameters;
    adev->hw_device.get_parameters = adev_get_parameters;
    adev->hw_device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->hw_device.open_output_stream = adev_open_output_stream;
    adev->hw_device.close_output_stream = adev_close_output_stream;
    adev->hw_device.open_input_stream = adev_open_input_stream;
    adev->hw_device.close_input_stream = adev_close_input_stream;
    adev->hw_device.dump = adev_dump;
    adev->hw_device.get_microphones = adev_get_microphones;

    card = find_card("vendor.hal.audio.out", PCM_OUT, 0);
    if (card == -1) {
       ALOGW("%s: no playback card found, trying any card for mixer", __func__);
       card = PCM_CARD_DEFAULT;
    }

    snprintf(mixer_path,PATH_MAX,"/vendor/etc/mixer_paths_0.xml");
    adev->ar = audio_route_init(card, mixer_path);
    if (!adev->ar) {
        ALOGE("%s: Failed to init audio route controls for card %d, aborting.",
            __func__, card);
        goto error;
    }
    
    adev->out_device = AUDIO_DEVICE_OUT_SPEAKER;
    adev->in_device = AUDIO_DEVICE_IN_BUILTIN_MIC & ~AUDIO_DEVICE_BIT_IN;

    *device = &adev->hw_device.common;

// CLK target codec only works with sample_rate 48000, identify the target and update default pcm_config.rate if needed.
    char product[PROPERTY_VALUE_MAX] = "cel_kbl";
    if(property_get("ro.hardware", product, NULL) <= 0) {
        ALOGE("%s : failed to read ro.hardware", __func__);
    } else {
        if(strcmp(product, "clk") == 0) {
            pcm_config_in.rate = 48000;
        }
    }

//Update period_size based on sample rate and period_ms
    size_t size = (pcm_config_in.rate * IN_PERIOD_MS * SAMPLE_SIZE_IN_BYTES_STEREO) / 1000;
    pcm_config_in.period_size = size;

    ALOGI("%s : will use input [rate : period] as [%d : %u] for %s variants", __func__, pcm_config_in.rate, pcm_config_in.period_size, product);

//[BT SCO VoIP Call
    update_bt_card(adev);

    adev->in_sco_voip_call = false;
    adev->is_hfp_call_active = false;
    adev->voip_in_resampler = NULL;
    adev->voip_out_resampler = NULL;
//BT SCO VoIP Call]

    adev->in_needs_standby = false;
    adev->out_needs_standby = false;

#ifdef DEBUG_PCM_DUMP
    sco_call_write = fopen("/vendor/dump/sco_call_write.pcm", "a");
    sco_call_write_remapped = fopen("/vendor/dump/sco_call_write_remapped.pcm", "a");
    sco_call_write_bt = fopen("/vendor/dump/sco_call_write_bt.pcm", "a");
    sco_call_read = fopen("/vendor/dump/sco_call_read.pcm", "a");
    sco_call_read_remapped = fopen("/vendor/dump/sco_call_read_remapped.pcm", "a");
    sco_call_read_bt = fopen("/vendor/dump/sco_call_read_bt.pcm", "a");
    out_write_dump = fopen("/vendor/dump/out_write_dump.pcm", "a");
    in_read_dump = fopen("/vendor/dump/in_read_dump.pcm", "a");

    if(sco_call_write == NULL || sco_call_write_remapped == NULL || sco_call_write_bt == NULL || sco_call_read == NULL || 
            sco_call_read_bt == NULL || sco_call_read_remapped == NULL || out_write_dump == NULL || in_read_dump == NULL)
        ALOGD("%s : failed to open dump files",__func__);
    else
        ALOGD("%s : success in opening dump files",__func__);
#endif

    return 0;

 error:
    free(adev);
    return -ENODEV;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "Android IA minimal HW HAL",
        .author = "The Android Open Source Project",
        .methods = &hal_module_methods,
    },
};
