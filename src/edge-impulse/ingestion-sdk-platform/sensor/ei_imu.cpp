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

#include "ei_imu.h"
#include "ns_imu.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "ns_spi.h"
#include "imu/inv_imu_driver.h"


static void imu_frame_available_cb(void *arg);

static float inertial_data[INERTIAL_AXIS_SAMPLED];
static ns_imu_sensor_data_t imu_frame[1];

static ns_imu_config_t imu_cfg = {
    .api = &ns_imu_V1_0_0,
    .sensor = NS_IMU_SENSOR_ICM45605,
    .iom    = 0,
    .accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G,
    .gyro_fsr  = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS,
    .accel_odr = ACCEL_CONFIG0_ACCEL_ODR_100_HZ,
    .gyro_odr  = GYRO_CONFIG0_GYRO_ODR_100_HZ,
    .accel_ln_bw = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4,
    .gyro_ln_bw  = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4,
    .calibrate = true,
    //.frame_available_cb = imu_frame_available_cb,
    //.frame_size = sizeof(imu_frame) / sizeof(ns_imu_sensor_data_t),
    //.frame_buffer = imu_frame
};

/**
 * @brief
 */
bool ei_inertial_init(void)
{
    if (ns_imu_configure(&imu_cfg) != NS_STATUS_SUCCESS) {
        ei_printf("Failed to configure IMU\r\n");
        return false;
    }

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        ei_printf("ERR: failed to register Inertial sensor!\r\n");
        return false;
    }

    return true;
}

/**
 * 
 */
float *ei_fusion_inertial_read_data(int n_samples)
{
    memset(inertial_data, 0, sizeof(float) * 6);

    if (n_samples >= 3)  {
        ns_imu_get_data(&imu_cfg, imu_frame);
        inertial_data[0] = imu_frame[0].accel_g[0];
        inertial_data[1] = imu_frame[0].accel_g[1];
        inertial_data[2] = imu_frame[0].accel_g[2];
    }

    if (n_samples == 6) {
        inertial_data[3] = imu_frame[0].gyro_dps[0];
        inertial_data[4] = imu_frame[0].gyro_dps[1];
        inertial_data[5] = imu_frame[0].gyro_dps[2];
    }

    return inertial_data;
}

/**
 * 
 */
static void imu_frame_available_cb(void *arg)
{
    ns_imu_config_t *cfg = (ns_imu_config_t *)arg;
}
