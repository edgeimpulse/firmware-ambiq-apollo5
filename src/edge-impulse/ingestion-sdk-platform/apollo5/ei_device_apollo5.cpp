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
/* Includes ---------------------------------------------------------------- */
#include "ei_device_apollo5.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"
#include "ingestion-sdk-platform/sensor/ei_mic.h"
#include "hal/am_hal_global.h"
#include "am_util_id.h"

EiAmbiqApollo5::EiAmbiqApollo5(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    init_device_id();
    load_config();

    device_type = "AMBIQ_APOLLO5";

    sensors[PDM_MICROPHONE].name = "PDM Microphone";
    sensors[PDM_MICROPHONE].frequencies[0] = 16000;
    sensors[PDM_MICROPHONE].max_sample_length_s = 5;
    sensors[PDM_MICROPHONE].start_sampling_cb = ei_pdm_microphone_sample_start_mono;
}

/**
 * @brief 
 * 
 */
void EiAmbiqApollo5::init_device_id(void)
{
    char temp[20];
    am_util_id_t chip_id;

    am_util_id_device(&chip_id);

    snprintf(temp, sizeof(temp), "%02x:%02x:%02x:%02x:%02x",
        (uint8_t)(chip_id.sMcuCtrlDevice.ui32ChipID1 & 0xFF),
        (uint8_t)((chip_id.sMcuCtrlDevice.ui32ChipID1  >> 8) & 0xFF),
        (uint8_t)((chip_id.sMcuCtrlDevice.ui32ChipID1  >> 16) & 0xFF),
        (uint8_t)(chip_id.sMcuCtrlDevice.ui32ChipID0 & 0xFF),
        (uint8_t)(chip_id.sMcuCtrlDevice.ui32ChipID0 >> 8) & 0xFF);

    device_id = std::string(temp);

    /* Init camera instance */
    camera = static_cast<EiAmbiqCamera*>(EiCamera::get_camera());
}

bool EiAmbiqApollo5::get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size)
{
    *p_sensor_list = sensors;
    *sensor_list_size = ARRAY_LENGTH(sensors);
    return true;
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
EiSnapshotProperties EiAmbiqApollo5::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res = NULL;

    uint8_t res_num = 0;
    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "",
        .resolutions_num = res_num,
        .resolutions = res
    };

    if (camera->is_camera_present() == true) {
        this->camera->get_resolutions(&res, &res_num);
        props.has_snapshot = true;
        props.support_stream = true;        
        props.color_depth = "RGB";
        props.resolutions_num = res_num;
        props.resolutions = res;

        return props; 
    }

    camera->get_resolutions(&res, &res_num);

    return props;
}

/**
 * @brief get_device is a static method of EiDeviceInfo class
 * It is used to implement singleton paradigm, so we are returning
 * here pointer always to the same object (dev)
 * 
 * @return EiDeviceInfo* 
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    AM_SHARED_RW static EiDeviceRAM<131072, 4> memory(sizeof(EiConfig));
    static EiAmbiqApollo5 dev(&memory);

    return &dev;
}
