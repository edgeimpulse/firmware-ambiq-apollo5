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
 
#include "FreeRTOS.h"
#include "task.h"

#include "peripheral/peripheral.h"
#include "peripheral/usb/ei_usb.h"
#include "common_events.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse/ingestion-sdk-platform/apollo5/ei_device_apollo5.h"
#include "edge-impulse/ingestion-sdk-platform/apollo5/ei_at_handlers.h"
#include "inference/ei_run_impulse.h"
#include "edge-impulse/ingestion-sdk-platform/sensor/ei_mic.h"

#if (configAPPLICATION_ALLOCATED_HEAP == 1)
size_t ucHeapSize = (NS_MALLOC_HEAP_SIZE_IN_K + 4) * 1024;
uint8_t ucHeap[(NS_MALLOC_HEAP_SIZE_IN_K + 4) * 1024] __attribute__((aligned(4)));
#endif

// main task parameters
#define EI_MAIN_TASK_STACK_SIZE_BYTE        (4096u)
#define EI_MAIN_TASK_PRIORITY               (configMAX_PRIORITIES - 6)
static TaskHandle_t ei_main_task_handle;
static void ei_main_task(void *pvParameters);

EventGroupHandle_t common_event_group;

/**
 * @brief Main
 *
 * @return int
 */
int main(void)
{
    peripheral_init();  // init basic peripheral, core etc
    ei_usb_init();         // init usb
    ei_microphone_init();
        
    /* create a task to send data via usb */
    if (xTaskCreate(ei_main_task,
        (const char*) "EI Main Thread",
        EI_MAIN_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        EI_MAIN_TASK_PRIORITY, //uxPriority
        &ei_main_task_handle) != pdPASS) {
        ei_printf("Failed to create EI Main Thread\r\n");
        
        while(1);
    }

    common_event_group = xEventGroupCreate();

    vTaskStartScheduler();

    return 0;
}

/**
 * @brief 
 * 
 * @param pvParameters 
 */
static void ei_main_task(void *pvParameters)
{
    (void)pvParameters;
    EventBits_t event_bit;
    bool in_rx_loop = false;

    EiAmbiqApollo5 *dev = static_cast<EiAmbiqApollo5*>(EiDeviceInfo::get_device());
    ATServer *at;
    
    ei_printf("Edge Impulse SDK - Ambiq Apollo 5");
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at = ei_at_init(dev);
    at->print_prompt();
    dev->get_camera()->init(160, 160);

    // Event loop
    while (1) {

        event_bit = xEventGroupWaitBits(common_event_group, 
                                        EVENT_RX_READY,    //  uxBitsToWaitFor 
                                        pdTRUE,                 //  xClearOnExit
                                        pdFALSE,                //  xWaitForAllBits
                                        portMAX_DELAY);

        if (event_bit & EVENT_RX_READY) {
            char data = ei_get_serial_byte(is_inference_running());

            in_rx_loop = false;

            while ((uint8_t)data != 0xFF) {
                if ((is_inference_running() == true) && (data == 'b') && (in_rx_loop == false)) {
                    xEventGroupSetBits(common_event_group, EVENT_WAIT_LAST_INFERENCE);

                    // wait till inference is done
                    while (is_inference_running() == true) {
                        vTaskDelay(1);
                    }
                    at->print_prompt();
                    continue;
                }

                in_rx_loop = true;
                at->handle(data);
                data = ei_get_serial_byte(is_inference_running());
            }
        }


    } // while(1)
}
