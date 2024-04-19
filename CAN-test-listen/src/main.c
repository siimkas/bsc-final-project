/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a Listen Only node in a TWAI network. The
 * Listen Only node will not take part in any TWAI bus activity (no acknowledgments
 * and no error frames). This example will execute multiple iterations, with each
 * iteration the Listen Only node will do the following:
 * 1) Listen for ping and ping response
 * 2) Listen for start command
 * 3) Listen for data messages
 * 4) Listen for stop and stop response
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define NO_OF_ITERS                     5
#define RX_TASK_PRIO                    9
#define TX_GPIO_NUM                     19
#define RX_GPIO_NUM                     23
#define EXAMPLE_TAG                     "TWAI Listen Only"

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config_50kbit = TWAI_TIMING_CONFIG_50KBITS();
static const twai_timing_config_t t_config_125kbit = TWAI_TIMING_CONFIG_125KBITS();
static const twai_timing_config_t t_config_250kbit = TWAI_TIMING_CONFIG_250KBITS();
static const twai_timing_config_t t_config_500kbit = TWAI_TIMING_CONFIG_500KBITS();
static const twai_timing_config_t t_config_800kbit = TWAI_TIMING_CONFIG_800KBITS();
static const twai_timing_config_t t_config_1mbit = TWAI_TIMING_CONFIG_1MBITS();
//Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = {.mode = TWAI_MODE_LISTEN_ONLY,
                                               .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                               .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 0, .rx_queue_len = 5,
                                               .alerts_enabled = TWAI_ALERT_AND_LOG | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN,
                                               .clkout_divider = 0
                                              };

static SemaphoreHandle_t rx_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);
    bool start_cmd = false;
    bool stop_resp = false;
    uint32_t iterations = 0;
    uint32_t data_count = 0;
    while (iterations < NO_OF_ITERS) {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY);
        if (rx_msg.identifier == ID_MASTER_PING) {
            ESP_LOGI(EXAMPLE_TAG, "Received master ping");
        } else if (rx_msg.identifier == ID_SLAVE_PING_RESP) {
            ESP_LOGI(EXAMPLE_TAG, "Received slave ping response");
        } else if (rx_msg.identifier == ID_MASTER_START_CMD) {
            ESP_LOGI(EXAMPLE_TAG, "Received master start command");
            start_cmd = true;
        } else if (rx_msg.identifier == ID_SLAVE_DATA) {
            data_count++;
        } else if (rx_msg.identifier == ID_MASTER_STOP_CMD) {
            ESP_LOGI(EXAMPLE_TAG, "Received master stop command");
        } else if (rx_msg.identifier == ID_SLAVE_STOP_RESP) {
            ESP_LOGI(EXAMPLE_TAG, "Received slave stop response");
            stop_resp = true;
        }
        if (start_cmd && stop_resp) {
            //Each iteration is complete after a start command and stop response is received
            iterations++;
            twai_status_info_t status_info;
            twai_get_status_info(&status_info);
            ESP_LOGI(EXAMPLE_TAG, "#################################");
            ESP_LOGI(EXAMPLE_TAG, "Nr of data messages: %" PRIu32, data_count);
            ESP_LOGI(EXAMPLE_TAG, "Rx missed: %" PRIu32, status_info.rx_missed_count);
            ESP_LOGI(EXAMPLE_TAG, "Rx overrun: %" PRIu32 , status_info.rx_overrun_count);
            ESP_LOGI(EXAMPLE_TAG, "Bus errors: %" PRIu32, status_info.bus_error_count);
            ESP_LOGI(EXAMPLE_TAG, "#################################");
            start_cmd = false;
            stop_resp = false;
            data_count = 0;
        }
    }

    xSemaphoreGive(rx_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    for (int i = 0; i < 6; i++){

        rx_sem = xSemaphoreCreateBinary();
        xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

        //Install and start TWAI driver
        switch (i)
        {
        case 0:
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config_50kbit, &f_config));
            ESP_LOGI(EXAMPLE_TAG, "Results for 50kbit");            
            break;
        case 1:
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config_125kbit, &f_config));
            ESP_LOGI(EXAMPLE_TAG, "Results for 125kbit");            
            break;    
        case 2:
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config_250kbit, &f_config));
            ESP_LOGI(EXAMPLE_TAG, "Results for 250kbit");            
            break;
        case 3:
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config_500kbit, &f_config));
            ESP_LOGI(EXAMPLE_TAG, "Results for 500kbit");            
            break;
        case 4:
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config_800kbit, &f_config));
            ESP_LOGI(EXAMPLE_TAG, "Results for 800kbit");            
            break;
        case 5:
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config_1mbit, &f_config));
            ESP_LOGI(EXAMPLE_TAG, "Results for 1mbit");            
            break;
        }
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");

        xSemaphoreGive(rx_sem);                     //Start RX task
        vTaskDelay(pdMS_TO_TICKS(100));
        xSemaphoreTake(rx_sem, portMAX_DELAY);      //Wait for RX task to complete

        //Stop and uninstall TWAI driver
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        ESP_ERROR_CHECK(twai_driver_uninstall());
        ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

        //Cleanup
        vSemaphoreDelete(rx_sem);
    }
}