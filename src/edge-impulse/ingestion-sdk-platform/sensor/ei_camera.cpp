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

#include "ei_camera.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/third_party/flatbuffers/include/flatbuffers/flexbuffers.h"
#include "ns_core.h"
#include "ns_camera.h"
#include "ArducamCamera.h"

extern ArducamCamera camera; // Arducam driver assumes this is a global, so :shrug:

void picture_dma_complete(ns_camera_config_t *cfg);
void picture_taken_complete(ns_camera_config_t *cfg);
static bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len);

ei_device_snapshot_resolutions_t EiAmbiqCamera::resolutions[] = {
        {96, 96},
        {128, 128},
        {160, 160},
    };

#define BLOCKING_MODE   1

AM_SHARED_RW static uint8_t jpgBuffer[JPG_BUFF_SIZE] __attribute__((aligned(16)));
static uint32_t start_jpg_dma(void);
static void press_jpg_shutter_button(uint8_t img_modex_index);

AM_SHARED_RW static uint8_t rgbBuffer[RGB_BUFF_SIZE] __attribute__((aligned(16)));
AM_SHARED_RW static uint8_t snapshot_buffer[RGB888_BUFF_SIZE] __attribute__((aligned(16)));

static uint32_t bufferOffset = 0;

volatile bool dmaComplete = false; // ISR-land
volatile bool pictureTaken = false;
volatile uint32_t buffer_length = 0;

// Camera needs to handle interrupt from the corrent ISR
#define iom_isr am_iom_isrx(CAM_SPI_IOM)
#define am_iom_isrx(n) am_iom_isr(n)
#define am_iom_isr(n) am_iomaster##n##_isr
extern "C" void iom_isr(void) {
    ns_spi_handle_iom_isr();
}

ns_camera_config_t camera_config = {
    .api = &ns_camera_V1_0_0,
    .spiSpeed = CAM_SPI_SPEED,
    .cameraHw = NS_ARDUCAM,
    .imageMode = NS_CAM_IMAGE_MODE_320X320,
    .imagePixFmt = NS_CAM_IMAGE_PIX_FMT_JPEG,
    .spiConfig = {.iom = CAM_SPI_IOM}, // Only IOM1 is currently supported
    .dmaCompleteCb = picture_dma_complete,
    .pictureTakenCb = picture_taken_complete,
};

EiAmbiqCamera::EiAmbiqCamera() :
    camera_found(false),
    tried_init(false),
    width(96),
    height(96)
{
}

/**
 * @brief 
 * 
 * @param width 
 * @param height 
 * @return true 
 * @return false 
 */
bool EiAmbiqCamera::init(uint16_t width, uint16_t height)
{
    if (tried_init == true && camera_found == true) {
        this->width = width;
        this->height = height;

        if (this->width == 96 && this->height == 96) {
            this->img_mode_index = 0;
        } else if (this->width == 128 && this->height == 128) {
            this->img_mode_index = 1;
        } else if (this->width == 160 && this->height == 160) {
            this->img_mode_index = 2;
        } else {
            ei_printf("Unsupported resolution %dx%d\n", this->width, this->height);
            return false;
        }

        return camera_found;
    }
    tried_init = true;

    if (ns_camera_init(&camera_config) != 0) {
        ei_printf("Failed to init camera\r\n");
        return false;
    }

    camera_found = true;
    
    ei_printf("Camera Init Success\n");
    ns_stop_camera(&camera_config);

    camera_found = true;

    this->width = width;
    this->height = height;

    ns_start_camera(&camera_config);
    setBrightness(&camera, CAM_BRIGHTNESS_LEVEL_DEFAULT);
    setEV(&camera, CAM_EV_LEVEL_DEFAULT);
    setContrast(&camera, CAM_CONTRAST_LEVEL_DEFAULT);
    setAutoExposure(&camera, true); 
    ns_delay_us(10000);

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiAmbiqCamera::deinit(void)
{
    return true;
}

/**
 * @brief 
 * 
 * @param res 
 * @param res_num 
 */
void EiAmbiqCamera::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num)
{
    *res = &EiAmbiqCamera::resolutions[0];
    *res_num = sizeof(EiAmbiqCamera::resolutions) / sizeof(ei_device_snapshot_resolutions_t);
}

/**
 * @brief 
 * 
 * @param res 
 * @return true 
 * @return false 
 */
bool EiAmbiqCamera::set_resolution(const ei_device_snapshot_resolutions_t res)
{
    this->width = res.width;
    this->height = res.height;

    return true;
}

bool EiAmbiqCamera::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image,
    uint32_t image_size)
{
    pictureTaken = false;

    memset(image, 0, image_size);
    memset(rgbBuffer, 0, RGB_BUFF_SIZE);
    memset(jpgBuffer, 0, JPG_BUFF_SIZE);

#ifdef BLOCKING_MODE && (BLOCKING_MODE == 1)
    uint32_t retval = 0;

    // blocking mode
    retval = ns_take_picture(&camera_config);
    buffer_length = ns_transfer_picture(&camera_config, jpgBuffer, &bufferOffset, JPG_BUFF_SIZE);

#else
    // shutter + dmn
    press_jpg_shutter_button(this->img_mode_index);
    ns_delay_us(1000);
    
    do {
        __WFE();
    } while (!pictureTaken);

    buffer_length = start_jpg_dma();

    if (buffer_length > JPG_BUFF_SIZE) {
        ei_printf("Buffer overflow\n");
        return false;
    }
    ns_delay_us(1000);
    
    do {
        __WFE();
    } while (!dmaComplete);
#endif
    buffer_length = ns_chop_off_trailing_zeros(jpgBuffer, buffer_length);

    uint32_t scaling_factor[] = {3, 2, 2};

    camera_decode_image(jpgBuffer, buffer_length, rgbBuffer, this->width, this->height, scaling_factor[this->img_mode_index]);

    RBG565ToRGB888(rgbBuffer, image, (this->width * this->height * 2));

    SCB_InvalidateICache();
    SCB_CleanInvalidateDCache();

    return true;
}

/**
 * @brief
 */
bool EiAmbiqCamera::get_fb_ptr(uint8_t** fb_ptr)
{
    *fb_ptr = snapshot_buffer;
    return true;
}

/**
 * @brief 
 * 
 * @return ei_device_snapshot_resolutions_t 
 */
ei_device_snapshot_resolutions_t EiAmbiqCamera::get_min_resolution(void)
{
    return EiAmbiqCamera::resolutions[0];
}

/**
 * @brief 
 * 
 * @param required_width 
 * @param required_height 
 * @return ei_device_snapshot_resolutions_t 
 */
ei_device_snapshot_resolutions_t EiAmbiqCamera::search_resolution(uint32_t required_width, uint32_t required_height)
{
    ei_device_snapshot_resolutions_t res;
    uint16_t max_width;
    uint16_t max_height;

    // camera_get_max_res(&max_width, &max_height);
    max_width = 160;
    max_height = 160;

    if ((required_width <= max_width) && (required_height <= max_height)) {
        res.height = required_height;
        res.width = required_width;
    }
    else {
        res.height = max_height;
        res.width = max_width;
    }

    return res;
}

/**
 *
 * @return
 */
EiCamera* EiCamera::get_camera(void)
{
    static EiAmbiqCamera cam;

    return &cam;
}

static void press_jpg_shutter_button(uint8_t img_modex_index) 
{
    (void)img_modex_index; // unused
    camera_config.imageMode = NS_CAM_IMAGE_MODE_320X320; // with jpg we take picture at 320x320 then resize when decoding
    camera_config.imagePixFmt = NS_CAM_IMAGE_PIX_FMT_JPEG; 
    ns_press_shutter_button(&camera_config);
}
/**
 * @brief 
 * 
 * @param bufferId 
 * @return uint32_t 
 */
static uint32_t start_jpg_dma(void)
{
    uint32_t camLength = 0;

    dmaComplete = false;
    camLength = ns_start_dma_read(&camera_config, jpgBuffer, &bufferOffset, JPG_BUFF_SIZE);

    bufferOffset = 1;

    return camLength;
}


/**
 * @brief 
 * 
 * @param cfg 
 */
void picture_dma_complete(ns_camera_config_t *cfg) 
{    
    dmaComplete = true;    
}

/**
 * @brief 
 * 
 * @param cfg 
 */
void picture_taken_complete(ns_camera_config_t *cfg) 
{
    pictureTaken = true;
}

/**
 *
 * @param src_buf
 * @param dst_buf
 * @param src_len
 * @return
 */
static bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len)
{
    uint32_t pix_count = src_len / 2;
    uint8_t r5, g6, b5;
    uint16_t* psrc = (uint16_t*)src_buf;

    for (uint32_t i = 0; i < pix_count; i ++, psrc++) {
        r5 = (*psrc & 0xF800) >> 11;
        g6 = (*psrc & 0x07E0) >> 5;
        b5 = (*psrc & 0x001F);

        *dst_buf++ = (r5 * 527 + 23) >> 6;
        *dst_buf++ = (g6 * 259 + 33) >> 6;
        *dst_buf++ = (b5 * 527 + 23) >> 6;
    }

    return true;
}
