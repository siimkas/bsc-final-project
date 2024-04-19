//  This is a modified version of the example provided by Espressif: 
//  https://github.com/espressif/esp-idf/tree/4fb2310/examples/peripherals/twai/twai_network

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "esp_timer.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define DATA_PERIOD_MS                  1
#define PING_PERIOD_MS                  250
#define NO_OF_ITERS                     5
#define ITER_DELAY_MS                   1000
#define RX_TASK_PRIO                    8       //Receiving task priority
#define TX_TASK_PRIO                    9       //Sending task priority
#define CTRL_TSK_PRIO                   10      //Control task priority
#define TX_GPIO_NUM                     19
#define RX_GPIO_NUM                     18
#define EXAMPLE_TAG                     "TWAI Slave"

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

typedef enum {
    TX_SEND_PING_RESP,
    TX_SEND_DATA,
    TX_SEND_STOP_RESP,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING,
    RX_RECEIVE_START_CMD,
    RX_RECEIVE_STOP_CMD,
    RX_TASK_EXIT,
} rx_task_action_t;

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config_50kbit = TWAI_TIMING_CONFIG_50KBITS();
static const twai_timing_config_t t_config_125kbit = TWAI_TIMING_CONFIG_125KBITS();
static const twai_timing_config_t t_config_250kbit = TWAI_TIMING_CONFIG_250KBITS();
static const twai_timing_config_t t_config_500kbit = TWAI_TIMING_CONFIG_500KBITS();
static const twai_timing_config_t t_config_800kbit = TWAI_TIMING_CONFIG_800KBITS();
static const twai_timing_config_t t_config_1mbit = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_message_t ping_resp = {.identifier = ID_SLAVE_PING_RESP, .data_length_code = 0,
                                         .data = {0, 0, 0, 0, 0, 0, 0, 0}
                                        };
static const twai_message_t stop_resp = {.identifier = ID_SLAVE_STOP_RESP, .data_length_code = 0,
                                         .data = {0, 0, 0, 0, 0, 0, 0, 0}
                                        };
//Data bytes of data message will be initialized in the transmit task
static twai_message_t data_message = {.identifier = ID_SLAVE_DATA, .data_length_code = 4,
                                      .data = {0, 0, 0, 0, 0, 0, 0, 0}
                                     };

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t stop_data_sem;
static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t stop_ping_res_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1) {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);
        if (action == RX_RECEIVE_PING) {
            //Listen for pings from master
            ESP_LOGI(EXAMPLE_TAG, "Listening for ping");
            twai_message_t rx_msg;
            while (1) {
                if(twai_receive(&rx_msg, portMAX_DELAY)==ESP_OK){
                    ESP_LOGI(EXAMPLE_TAG, "Received message with id %"PRIu32, rx_msg.identifier);
                }else{
                    ESP_LOGI(EXAMPLE_TAG, "Receiveing failed");
                };
                if (rx_msg.identifier == ID_MASTER_PING) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_START_CMD) {
            //Listen for start command from master
            twai_message_t rx_msg;
            ESP_LOGI(EXAMPLE_TAG, "Listening for start command");
            while (1) {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_START_CMD) {
                    xSemaphoreGive(stop_ping_res_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_STOP_CMD) {
            //Listen for stop command from master
            twai_message_t rx_msg;
            while (1) {
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_MASTER_STOP_CMD) {
                    xSemaphoreGive(stop_data_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    while (1) {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PING_RESP) {
            //Transmit ping response to master
            while (xSemaphoreTake(stop_ping_res_sem, 0) != pdTRUE) {
                twai_transmit(&ping_resp, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(PING_PERIOD_MS));
            }
            ESP_LOGI(EXAMPLE_TAG, "Transmitted ping response successfully");
            // xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_SEND_DATA) {
            //Transmit data messages until stop command is received
            ESP_LOGI(EXAMPLE_TAG, "Start transmitting data");
            u_int32_t data_count = 0;
            while (1) {
                //FreeRTOS tick count used to simulate sensor data
                uint32_t sensor_data = xTaskGetTickCount();
                for (int i = 0; i < 4; i++) {
                    data_message.data[i] = (sensor_data >> (i * 8)) & 0xFF;
                }
                if(twai_transmit(&data_message, portMAX_DELAY)==ESP_OK){
                    data_count++;
                }
                // ESP_LOGI(EXAMPLE_TAG, "Transmitted data value %"PRIu32, sensor_data);
                // vTaskDelay(pdMS_TO_TICKS(DATA_PERIOD_MS));
                if (xSemaphoreTake(stop_data_sem, 0) == pdTRUE) {
                    break;
                }
            }
            ESP_LOGI(EXAMPLE_TAG, "Nr of data transmissions: %" PRIu32,data_count);
        } else if (action == TX_SEND_STOP_RESP) {
            //Transmit stop response to master
            twai_transmit(&stop_resp, portMAX_DELAY);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted stop response");
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;
    int64_t time_us;
    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(EXAMPLE_TAG, "Driver started");
        time_us = esp_timer_get_time();
        //Listen of pings from master
        rx_action = RX_RECEIVE_PING;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Send ping response
        tx_action = TX_SEND_PING_RESP;
        rx_action = RX_RECEIVE_START_CMD;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        // xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Listen for start command
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Start sending data messages and listen for stop command
        tx_action = TX_SEND_DATA;
        rx_action = RX_RECEIVE_STOP_CMD;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Send stop response
        tx_action = TX_SEND_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Wait for bus to become free
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        while (status_info.msgs_to_tx > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            twai_get_status_info(&status_info);
        }
        time_us = esp_timer_get_time() - time_us;

        ESP_LOGI(EXAMPLE_TAG, "#################################");
        ESP_LOGI(EXAMPLE_TAG, "Time elapsed: %" PRId64 " us", time_us);
        ESP_LOGI(EXAMPLE_TAG, "Rx missed: %" PRIu32, status_info.rx_missed_count);
        ESP_LOGI(EXAMPLE_TAG, "Rx overrun: %" PRIu32 , status_info.rx_overrun_count);
        ESP_LOGI(EXAMPLE_TAG, "Tx failed: %" PRIu32 , status_info.tx_failed_count);
        ESP_LOGI(EXAMPLE_TAG, "Bus errors: %" PRIu32, status_info.bus_error_count);
        ESP_LOGI(EXAMPLE_TAG, "Arbitration lost: %" PRIu32, status_info.arb_lost_count);
        ESP_LOGI(EXAMPLE_TAG, "#################################");
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
    }
    
    //Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    //Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    
    for (int i = 0; i < 6; i++){
        
        //Add short delay to allow master it to initialize first
        for (int i = 3; i > 0; i--) {
            printf("Slave starting in %d\n", i);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        //Create semaphores and tasks
        tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
        rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
        ctrl_task_sem = xSemaphoreCreateBinary();
        stop_data_sem  = xSemaphoreCreateBinary();
        stop_ping_res_sem = xSemaphoreCreateBinary();
        done_sem  = xSemaphoreCreateBinary();
        xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);
        
        //Install TWAI driver
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
        if(twai_reconfigure_alerts(TWAI_ALERT_AND_LOG | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BELOW_ERR_WARN, NULL)==ESP_OK){
            ESP_LOGI(EXAMPLE_TAG, "Alerts configured");
        } else {
            ESP_LOGI(EXAMPLE_TAG, "Could not enable alerts.");
        };
        
        xSemaphoreGive(ctrl_task_sem);              //Start Control task
        xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for tasks to complete

        //Uninstall TWAI driver
        ESP_ERROR_CHECK(twai_driver_uninstall());
        ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

        //Cleanup
        vSemaphoreDelete(ctrl_task_sem);
        vSemaphoreDelete(stop_data_sem);
        vSemaphoreDelete(done_sem);
        vSemaphoreDelete(stop_ping_res_sem);
        vQueueDelete(tx_task_queue);
        vQueueDelete(rx_task_queue);

    }
}