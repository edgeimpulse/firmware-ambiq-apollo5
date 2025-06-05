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

/* Include ----------------------------------------------------------------- */
#include "ei_sampler.h"
#include "ingestion-sdk-platform/apollo5/ei_device_apollo5.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "common_events.h"

/* Private function prototypes --------------------------------------------- */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static time_t ei_time(time_t *t);
static bool create_header(sensor_aq_payload_info *payload);
static void ei_write_last_data(void);

/* Private variables ------------------------------------------------------- */
static uint32_t total_sample_length_ms;
static uint32_t sample_interval_ms;
static uint32_t current_sample;
static uint32_t sample_buffer_size;
static uint32_t headerOffset = 0;
EI_SENSOR_AQ_STREAM stream;

static uint8_t write_word_buf[4];
static int write_addr = 0;
static unsigned char ei_sensor_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_sensor_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_sensor_hs_ctx;
static sensor_aq_ctx ei_sensor_ctx = {
    { ei_sensor_ctx_buffer, 1024 },
    &ei_sensor_signing_ctx,
    &ei_write,
    &ei_seek,
    &ei_time,
};

/**
 * @brief      Setup and start sampling, write CBOR header to flash
 *
 * @param      v_ptr_payload  sensor_aq_payload_info pointer hidden as void
 * @param[in]  sample_size    Number of bytes for 1 sample (include all axis)
 *
 * @return     true if successful
 */
bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    EiAmbiqApollo5* dev = static_cast<EiAmbiqApollo5*>(EiAmbiqApollo5::get_device());
    EiDeviceMemory* mem = static_cast<EiDeviceMemory*>(dev->get_memory());

    sensor_aq_payload_info *payload = (sensor_aq_payload_info *)v_ptr_payload;

    ei_printf("Sampling settings:\n");    
    ei_printf("\tInterval: ");
    ei_printf_float(dev->get_sample_interval_ms());
    ei_printf("ms.\n");
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    char filename[256];
    int fn_r = snprintf(filename, 256, "/fs/%s", dev->get_sample_label().c_str());
    if (fn_r <= 0) {
        ei_printf("ERR: Failed to allocate file name\n");
        return false;
    }

    total_sample_length_ms = (uint32_t)((float)dev->get_sample_length_ms());
    sample_interval_ms = (uint32_t)dev->get_sample_interval_ms();

    sample_buffer_size = (total_sample_length_ms * sample_size) * 2;  // TODO 4 ?
    current_sample = 0;

    // Minimum delay of 2000 ms for daemon
    if (((sample_buffer_size / mem->block_size)+1) * mem->block_erase_time < 2000) {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", (uint32_t)2000);
        ei_sleep(2000);
    }
    else {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n",
        ((sample_buffer_size / mem->block_size)+1) * mem->block_erase_time);
    }

	if (mem->erase_sample_data(0, sample_buffer_size + mem->block_size) == 0) {
        ei_printf("Error in erase_sample_data\n");
        return false;
    }
	
    if (create_header(payload) == false) {
        ei_printf("Error in create_header\n");
        return false;
    }
        
    if (ei_sample_start(&sample_data_callback, dev->get_sample_interval_ms()) == false) {
        ei_printf("Error in ei_sample_start\n");
        return false;
    }

	ei_printf("Sampling...\n");

    xEventGroupWaitBits(common_event_group, 
                                        EVENT_INGESTION_DONE,    //  uxBitsToWaitFor 
                                        pdTRUE,                 //  xClearOnExit
                                        pdFALSE,                //  xWaitForAllBits
                                        portMAX_DELAY);
    
    ei_write_last_data();
    write_addr++;

    uint8_t final_byte[] = { 0xff };
    int ctx_err = ei_sensor_ctx.signature_ctx->update(ei_sensor_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        return ctx_err;
    }

    // finish the signing
    ctx_err = ei_sensor_ctx.signature_ctx->finish(ei_sensor_ctx.signature_ctx, ei_sensor_ctx.hash_buffer.buffer);

    ei_printf("Done sampling, total bytes collected: %lu\n", total_sample_length_ms);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", write_addr + headerOffset);
    ei_printf("OK\n");

    return true;
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
    EiAmbiqApollo5* dev = static_cast<EiAmbiqApollo5*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = static_cast<EiDeviceMemory*>(dev->get_memory());

    sensor_aq_init_mbedtls_hs256_context(&ei_sensor_signing_ctx, &ei_sensor_hs_ctx, dev->get_sample_hmac_key().c_str());

    int tr = sensor_aq_init(&ei_sensor_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_sensor_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_sensor_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    tr = mem->write_sample_data((uint8_t*)ei_sensor_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    ei_printf("Try to write %d bytes\r\n", end_of_header_ix);
    if ((size_t)tr != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    ei_sensor_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

/**
 * @brief      Write samples to FLASH in CBOR format
 *
 * @param[in]  sample_buf  The sample buffer
 * @param[in]  byteLenght  The byte lenght
 *
 * @return     true if all required samples are received. Caller should stop sampling,
 */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    static uint32_t start_t, stop_t;
    sensor_aq_add_data(&ei_sensor_ctx, (float *)sample_buf, byteLenght / sizeof(float));
    current_sample += sample_interval_ms;
    //ei_printf("Sampled %lu ms, current sample: %lu ms\r\n", current_sample, total_sample_length_ms);
    stop_t = ei_read_timer_ms();
    //ei_printf("Current time: %lu ms\r\n", stop_t - start_t);
    start_t = stop_t;

    if(current_sample > total_sample_length_ms) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param size 
 * @param count 
 * @return size_t 
 */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    EiAmbiqApollo5* dev = static_cast<EiAmbiqApollo5*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = static_cast<EiDeviceMemory*>(dev->get_memory());

    for (size_t i = 0; i < count; i++) {

        write_word_buf[write_addr&0x3] = *((char *)buffer+i);

       if ((++write_addr & 0x03) == 0x00) {
           mem->write_sample_data(write_word_buf, (write_addr - 4) + headerOffset, 4);
       }

    }

    return count;
}

/**
 * @brief 
 * 
 * @param offset 
 * @param origin 
 * @return int 
 */
static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

/**
 * @brief      File handle time function. Not used
 */
static time_t ei_time(time_t* t)
{
    time_t cur_time = 4564867;
    if (t)
        *(t) = cur_time;
    return cur_time;
}

/**
 * @brief 
 * 
 */
static void ei_write_last_data(void)
{
    EiAmbiqApollo5* dev = static_cast<EiAmbiqApollo5*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = static_cast<EiDeviceMemory*>(dev->get_memory());
    uint8_t fill = ((uint8_t)write_addr & 0x03);
    uint8_t insert_end_address = 0;

    if (fill != 0x00) {
        for (uint8_t i=fill; i<4; i++) {
            write_word_buf[i] = 0xFF;
        }

        mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
        insert_end_address = 4;
    }

    /* Write appending word for end character */
    for (uint8_t i=0; i < 4; i++) {
        write_word_buf[i] = 0xFF;
    }
    mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset + insert_end_address, 4);
}
