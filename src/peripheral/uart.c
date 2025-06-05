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

#include "uart.h"
#include "ns_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "common_events.h"

#include <string.h>

#if defined(EI_APOLLO_USE_UART) && (EI_APOLLO_USE_UART == 1)

static EventGroupHandle_t uart_event_group;
extern EventGroupHandle_t common_event_group;

volatile bool g_tx_sent = false; // Flag to indicate data availability

/* Macro definition */
#define CARRIAGE_ASCII            (13u)     /* Carriage return */

#define UART_EVENT_RX_READY  (1 << 0)
#define UART_EVENT_TX_READY  (1 << 1)

#define MY_RX_BUFSIZE 4096

AM_SHARED_RW static uint8_t my_rx_ff_buf[MY_RX_BUFSIZE] = {0x00};

/* Counter to update g_temp_buffer index */
static uint16_t g_rx_index = 0;
/* Index of data sent to at hanlder */
static uint16_t g_uart_rx_read_index = 0;

static bool _uart_is_init = false;

static void uart_rx_callback(ns_uart_transaction_t *transaction);
static void uart_tx_callback(ns_uart_transaction_t *transaction);

ns_uart_handle_t uart_handle = NULL;

ns_uart_config_t uart_config = {
    .api=&ns_uart_V0_0_1,
    .uart_config = &g_sUartConfig,
    .tx_blocking = true,
    .rx_blocking = true,
};

void uart_init(void)
{
    BaseType_t retval;

    uart_event_group = xEventGroupCreate();

    memset(my_rx_ff_buf, 0x00, sizeof(my_rx_ff_buf));   // no reset, just clear everything    

    g_rx_index = 0;
    g_uart_rx_read_index = 0; 

    // Initialize the UART peripheral
    if (ns_uart_init(&uart_config, &uart_handle) != AM_HAL_STATUS_SUCCESS) {
        return;
    }
    _uart_is_init = true;

    ns_uart_register_callbacks(uart_handle, uart_rx_callback, NULL);
}

void uart_send(uint8_t *data, int len)
{
    if (_uart_is_init == false) {
        return;
    }

    if (len == 0) {
        return;
    }
    
    ns_uart_blocking_send_data(&uart_config, data, len);
}

static void uart_rx_callback(ns_uart_transaction_t *transaction)
{
    char data = 0xFF;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t status = AM_HAL_STATUS_SUCCESS;

    status = ns_uart_nonblocking_receive_data(&uart_config, &data, 1);

    // Handle received data
    if (status == AM_HAL_STATUS_SUCCESS) {
        // Store the received data in the buffer
        my_rx_ff_buf[g_rx_index] = data;
        g_rx_index = (g_rx_index +1) % MY_RX_BUFSIZE; // Wrap around if needed

        if ((data == CARRIAGE_ASCII) || (data == 'b')) {
            xEventGroupSetBitsFromISR(
                common_event_group,         /* The event group being updated. */
                EVENT_RX_READY,             /* The bits being set. */
                &xHigherPriorityTaskWoken);
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void uart_tx_callback(ns_uart_transaction_t *transaction)
{
    // Handle transmitted data
    // transaction->tx_buffer contains the transmitted data
    // transaction->status indicates the status of the transaction
    g_tx_sent = true; // Set the flag to indicate that data has been sent
}

/**
 * @brief Returns char from uart rx buffer
 *
 * @param is_inference_running If inference is running, we need to check for a single 'b'
 * @return
 */
char ei_get_serial_byte(bool is_inference_running)
{
    char to_send = -1;

    if (g_uart_rx_read_index < g_rx_index) {
        to_send = my_rx_ff_buf[g_uart_rx_read_index];    // get one

        g_uart_rx_read_index = (g_uart_rx_read_index + 1)%MY_RX_BUFSIZE;  // increment and wrap
    }
    else if (g_uart_rx_read_index == g_rx_index) {  // when equal and different from zero
        memset(my_rx_ff_buf, 0x00, sizeof(my_rx_ff_buf));   // no reset, just clear everything        
        g_uart_rx_read_index = 0;
        g_rx_index = 0;        
    }

    return to_send;
}

void uart_set_baudrate(uint32_t baudrate)
{
    g_sUartConfig.ui32BaudRate = baudrate;
    ns_uart_change_baud_rate(uart_handle, baudrate);
}

#endif // EI_APOLLO_USE_UART
