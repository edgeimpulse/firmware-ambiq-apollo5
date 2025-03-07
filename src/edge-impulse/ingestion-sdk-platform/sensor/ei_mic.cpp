/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ei_mic.h"
#include "ns_audio.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "ingestion-sdk-platform/apollo5/ei_device_apollo5.h"
#include "model-parameters/model_metadata.h"
#include "ingestion-sdk-c/sensor_aq_mbedtls_hs256.h"
#include "hal/am_hal_global.h"
#include "edge-impulse/inference/ei_run_impulse.h"

/* Edge Impulse */
static bool create_header(sensor_aq_payload_info *payload);
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*);
static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin);
static bool ei_microphone_sample_start(void);
static void ingestion_callback(void *buffer, uint32_t n_bytes);
static void write_value_to_cbor_buffer(uint8_t *buf, int16_t value);

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static uint8_t n_audio_channels = 1;
static uint32_t cbor_current_sample;

static inference_t inference;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;

AM_SHARED_RW static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Ambiq */
// Audio Configuration
// High level audio parameters
#define NUM_CHANNELS        (1)
#define SAMPLE_RATE         (16000)
#define SAMPLES_IN_FRAME    (NS_AUDIO_DMA_BUFFER_SIZE * 4)
#define DEFAULT_GAIN        (AM_HAL_PDM_GAIN_P90DB)

bool volatile static g_audioRecording = false;
bool volatile static g_audioReady = false;

#ifndef NS_AMBIQSUITE_VERSION_R4_1_0
am_hal_offset_cal_coeffs_array_t sOffsetCalib;
#endif

void audio_frame_callback(ns_audio_config_t *config, uint16_t bytesCollected);

// Audio IPC and config
uint8_t static g_bufsel = 0; // for pingponging
alignas(16) int16_t static audioDataBuffer[2][SAMPLES_IN_FRAME];
alignas(16) uint32_t static dmaBuffer[SAMPLES_IN_FRAME * NUM_CHANNELS * 2];

ns_audio_config_t audio_config = {
    .api = &ns_audio_V2_1_0,
    .eAudioApiMode = NS_AUDIO_API_CALLBACK,
    .callback = audio_frame_callback,
    .audioBuffer = (void *)&audioDataBuffer,
    .eAudioSource = NS_AUDIO_SOURCE_PDM,
    .sampleBuffer = dmaBuffer,
    .workingBuffer = NULL,  // NULL for PDM
    .numChannels = NUM_CHANNELS,
    .numSamples = SAMPLES_IN_FRAME,
    .sampleRate = SAMPLE_RATE,
    .audioSystemHandle = NULL, // filled in by audio_init()
    .bufferHandle = NULL,   // NULL as not using ring buffer
#if !defined(NS_AMBIQSUITE_VERSION_R4_1_0) && defined(NS_AUDADC_PRESENT)
    .sOffsetCalib = &sOffsetCalib,
#endif
};

#define MIC_BUFFER_READY    (1 << 0)

/**
 * @brief 
 * 
 * @param offset 
 * @param length 
 * @param out_ptr 
 * @return int 
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    inference.buf_ready = 0;

    return 0;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_init(void)
{
    g_audioRecording = false;
    g_audioReady = false;

    // init audio system
    NS_TRY(ns_audio_init(&audio_config), "Audio initialization Failed.\n");

    // set gain
    //ns_audio_set_gain(DEFAULT_GAIN, DEFAULT_GAIN);

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_pdm_microphone_sample_start_mono(void)
{
    n_audio_channels = 1;
    audio_config.eAudioSource = NS_AUDIO_SOURCE_PDM;

    return ei_microphone_sample_start();
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_microphone_sample_start(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    if (n_audio_channels > 1) {
        payload.sensors[1] = { "audio2", "wav"};
    }

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: /fs/%s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if (samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;
    cbor_current_sample = 0;

    ei_printf("Starting in 2000 ms... (or until all flash was erased)\n");
    ei_sleep(2000); // no need to erase

    if (create_header(&payload) == false) {
        return false;
    }

    g_audioReady = false;
    g_audioRecording = true;

    if (ns_start_audio(&audio_config)) {
        ei_printf("Failed to start audio\r\n");
    }

    for(int i = 0; i < 10; i++) {
        do{
            __WFI();
        } while (g_audioReady == false);  // skip the first
        g_audioReady = false;
    }

    ei_printf("Sampling...\r\n");
    dev->set_state(eiStateSampling);

    g_audioReady = false;
    g_audioRecording = true;

    while (current_sample < samples_required) {
        __WFI();
        ei_mic_thread(&ingestion_callback);
    }
    g_audioRecording = false;

    // stop audio
    if (ns_end_audio(&audio_config)) {
        ei_printf("Failed to end audio\r\n");
    }

    dev->set_state(eiStateIdle);

    /* Write end of cbor + dummy */
    const uint8_t end_of_cbor[] = {0xff, 0xff, 0xff, 0xff};
    mem->write_sample_data((uint8_t*)end_of_cbor, headerOffset + cbor_current_sample, 4);

    int ctx_err =
        ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    ei_printf("Done sampling, total bytes collected: %lu\n", (current_sample * 2));
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", (cbor_current_sample + 1 + headerOffset));
    ei_printf("OK\n");

    return true;
}

/**
 * @brief 
 * 
 * @param config 
 * @param bytesCollected 
 */
void audio_frame_callback(ns_audio_config_t *config, uint16_t bytesCollected) 
{
    if (g_audioRecording) {
        ns_audio_getPCM_v2(config, &(audioDataBuffer[g_bufsel][0]));
        g_bufsel ^= 1;
        g_audioReady = true;
    }
}

/**
 * @brief 
 * 
 * @param cb 
 */
void ei_mic_thread(void (*callback)(void *buffer, uint32_t n_bytes))
{
    if ((callback != NULL) && g_audioRecording && g_audioReady) {
        callback(&(audioDataBuffer[g_bufsel ^= 1][0]), SAMPLES_IN_FRAME * 2);
        g_audioReady = false;
    }
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @param interval_ms 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_start(uint32_t n_samples, uint8_t n_channels_inference, uint32_t freq)
{
    if (n_channels_inference > 2) {
        ei_printf("Wrong number of channels\r\n");
        return false;
    }

    n_audio_channels = n_channels_inference;

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t)  * n_audio_channels);
    if (inference.buffers[0] == NULL) {
        ei_printf("Can't allocate first audio buffer\r\n");
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t)  * n_audio_channels);
    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        ei_printf("Can't allocate second audio buffer\r\n");
        return false;
    }

    inference.buf_select = 0;
    inference.n_samples = n_samples;    // this represents the number of samples per channel
    
    ei_microphone_inference_reset_buffers();

    g_audioRecording = false;
    audio_config.eAudioSource = NS_AUDIO_SOURCE_PDM;

    if (ns_start_audio(&audio_config)) {
        ei_printf("Failed to start audio\r\n");
    }

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_is_recording(void)
{
    return (inference.buf_ready == 0);
}

/**
 * @brief 
 * 
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;

    memset(inference.buffers[0], 0, inference.n_samples * sizeof(microphone_sample_t) * n_audio_channels);
    memset(inference.buffers[1], 0, inference.n_samples * sizeof(microphone_sample_t) * n_audio_channels);

}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_end(void)
{
    g_audioReady = false;
    g_audioRecording = false;

    if (ns_end_audio(&audio_config)) {
        ei_printf("Failed to end audio\r\n");
    }

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);

    return true; 
}

/**
 * @brief 
 * 
 */
void ei_mic_run_inference(void)
{
    g_audioReady = false;
    g_audioRecording = true;
    
    do {
        __WFI();
        ei_mic_thread(&ei_mic_inference_samples_callback);
    }while ((ei_microphone_inference_is_recording() == true) && (is_inference_running() == true));

    g_audioRecording = false;
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param sample_count 
 */
void ei_mic_inference_samples_callback(void *buffer, uint32_t sample_count)
{
    int16_t *samples = (int16_t *)buffer;

    for (uint32_t i = 0; i < (sample_count >> 1); i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = samples[i];

        if (inference.buf_count >= (inference.n_samples * n_audio_channels)) {  // n_samples is per channel

            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }   
}

/**
 * @brief Create a header object
 * 
 * @param payload 
 * @return true 
 * @return false 
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();

    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param size 
 * @param count 
 * @param stream 
 * @return size_t 
 */
static size_t ei_write(const void* buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM* stream)
{
    (void)buffer;
    (void)size;
    (void)stream;

    return count;
}

/**
 * @brief 
 * 
 * @param stream 
 * @param offset 
 * @param origin 
 * @return int 
 */
static int ei_seek(EI_SENSOR_AQ_STREAM* stream, long int offset, int origin)
{
    (void)stream;
    (void)offset;
    (void)origin;

    return 0;
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param n_bytes 
 */
static void ingestion_callback(void *buffer, uint32_t n_bytes)
{
    EiDeviceMemory *mem = EiDeviceInfo::get_device()->get_memory();
    int16_t *sbuffer = (int16_t *)buffer;
    uint32_t sbuf_ptr = 0;
    uint32_t n_samples = (n_bytes >> 1);

    /* Calculate the cbor buffer length: header + 3 bytes per audio channel */
    uint32_t cbor_length = n_samples + ((n_samples + n_bytes) * n_audio_channels);    
    uint8_t *cbor_buf = (uint8_t *)ei_malloc(cbor_length);

    if (cbor_buf == NULL) {
        ei_printf("ERR: memory allocation error\r\n");
        return;
    }

    for (uint32_t i = 0; i < n_samples; i++) {
        uint32_t cval_ptr = i * ((n_audio_channels * 3) + 1);

        cbor_buf[cval_ptr] = 0x80 + n_audio_channels;
        for (uint8_t y = 0; y < n_audio_channels; y++) {
            write_value_to_cbor_buffer(&cbor_buf[cval_ptr + 1 + (3 * y)], sbuffer[sbuf_ptr + y]);
        }
        sbuf_ptr += n_audio_channels;
    }

    mem->write_sample_data((uint8_t*)cbor_buf, headerOffset + cbor_current_sample, cbor_length);

    ei_free(cbor_buf);

    cbor_current_sample += cbor_length;
    current_sample += n_samples;

}

/**
 * @brief Convert to CBOR int16 format and write to buf
 */
static void write_value_to_cbor_buffer(uint8_t *buf, int16_t value)
{
    uint8_t datatype;
    uint16_t sample;

    if (value < 0) {
        datatype = 0x39;
        /* Convert from 2's complement */
        sample = (uint16_t)~value + 1;
    }
    else {
        datatype = 0x19;
        sample = value;
    }
    buf[0] = datatype;
    buf[1] = sample >> 8;
    buf[2] = sample & 0xFF;    
}
