/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;


#define UART_MHZ14A_NUM         UART_NUM_1
// #define UART_MHZ14A_TXD_PIN     GPIO_NUM_17
// #define UART_MHZ14A_RXD_PIN     GPIO_NUM_16

#define UART_MHZ14A_TXD_PIN     GPIO_NUM_4
#define UART_MHZ14A_RXD_PIN     GPIO_NUM_5

#define UART_MHZ14A_BAUDRATE    9600


void uart_init(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate) {
    const uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void app_uart_init(void)
{
    uart_init(UART_MHZ14A_NUM, UART_MHZ14A_TXD_PIN, UART_MHZ14A_RXD_PIN, UART_MHZ14A_BAUDRATE);
    uart_init(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, 115200);
}

int uart_transmit(const char* logName, uart_port_t uart_num, const char* data, const int len)
{
    const int txBytes = uart_write_bytes(uart_num, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_mhz14a_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_MHZ14A_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    const char mhz14a_query_co2_cmd[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
    const int cmd_len = sizeof(mhz14a_query_co2_cmd);
    while (1) {
        uart_transmit(TX_TASK_TAG, UART_MHZ14A_NUM, mhz14a_query_co2_cmd, cmd_len);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_mhz14a_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_MHZ14A_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_MHZ14A_NUM, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            uint16_t co2_concentration = data[2] * 0x100 + data[3];
            ESP_LOGI(RX_TASK_TAG, "\n\n\tCO2 Concentration: %u\n\n", co2_concentration);
        }
    }
    free(data);
}

void app_assign_task(void) {
    xTaskCreate(rx_mhz14a_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_mhz14a_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}

void app_main(void)
{
    app_uart_init();
    app_assign_task();
}
