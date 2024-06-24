// // 

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_log.h"
// #include "driver/uart.h"
// #include "string.h"
// #include "driver/gpio.h"
// #include "../include/bmx280/bmx280.h"
// #include <inttypes.h>

// // Định nghĩa cho MH-Z14A
// static const int RX_BUF_SIZE = 1024;
// #define UART_MHZ14A_NUM         UART_NUM_1
// #define UART_MHZ14A_TXD_PIN     GPIO_NUM_4
// #define UART_MHZ14A_RXD_PIN     GPIO_NUM_5
// #define UART_MHZ14A_BAUDRATE    9600
 
// // Định nghĩa cho PMS7003
// #define RX_BUFFER_SIZE 128
// #define UART_PMS7003_NUM        UART_NUM_2
// #define UART_PMS7003_TXD_PIN    GPIO_NUM_17
// #define UART_PMS7003_RXD_PIN    GPIO_NUM_16
// #define UART_PMS7003_BAUDRATE   9600

// // Calibration functions for PMS7003
// float calibrate_pm1_0(float raw_pm1_0);
// float calibrate_pm2_5(float raw_pm2_5);
// float calibrate_pm10(float raw_pm10);

// // Calibration function for MH-Z14A
// float calibrate_co2(float raw_co2);

// // Calibration functions for BME280
// float calibrate_temperature(float raw_temp) {
//     return raw_temp + 0.5;  // Adjust based on your calibration data
// }

// float calibrate_pressure(float raw_press) {
//     return raw_press + 1.0;  // Adjust based on your calibration data
// }

// float calibrate_humidity(float raw_hum) {
//     return raw_hum + 2.0;  // Adjust based on your calibration data
// }
// static const char *TAG_PMS7003 = "PMS7003";

// // Khai báo hàm trước khi sử dụng
// esp_err_t pms7003_readData(uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10);

// void uart_init(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate) {
//     const uart_config_t uart_config = {
//         .baud_rate = baudrate,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };
//     uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(uart_num, &uart_config);
//     uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// void app_uart_init(void)
// {
//     uart_init(UART_MHZ14A_NUM, UART_MHZ14A_TXD_PIN, UART_MHZ14A_RXD_PIN, UART_MHZ14A_BAUDRATE);
//     uart_init(UART_PMS7003_NUM, UART_PMS7003_TXD_PIN, UART_PMS7003_RXD_PIN, UART_PMS7003_BAUDRATE);
// }

// int uart_transmit(const char* logName, uart_port_t uart_num, const char* data, const int len)
// {
//     const int txBytes = uart_write_bytes(uart_num, data, len);
//     ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//     return txBytes;
// }

// static void tx_mhz14a_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_MHZ14A_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

//     const char mhz14a_query_co2_cmd[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
//     const int cmd_len = sizeof(mhz14a_query_co2_cmd);
//     while (1) {
//         uart_transmit(TX_TASK_TAG, UART_MHZ14A_NUM, mhz14a_query_co2_cmd, cmd_len);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// static void rx_mhz14a_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_MHZ14A_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART_MHZ14A_NUM, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

//             uint16_t co2_concentration = data[2] * 0x100 + data[3];
//             ESP_LOGI(RX_TASK_TAG, "\n\n\tCO2 Concentration: %u\n\n", co2_concentration);
//         }
//     }
//     free(data);
// }

// static void rx_pms7003_task(void *arg)
// {
//     uint32_t pm1_0, pm2_5, pm10;
//     while (1) {
//         esp_err_t err = pms7003_readData(&pm1_0, &pm2_5, &pm10);
//         if (err == ESP_OK) {
//             ESP_LOGI(TAG_PMS7003, "PM1.0: %" PRIu32 " ug/m3, PM2.5: %" PRIu32 " ug/m3, PM10: %" PRIu32 " ug/m3", pm1_0, pm2_5, pm10);
//         } else {
//             ESP_LOGE(TAG_PMS7003, "Failed to read data from PMS7003");
//         }
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// esp_err_t pms7003_readData(uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10)
// {
//     vTaskDelay(2000 / portTICK_PERIOD_MS);  // Đợi 2 giây để cảm biến ổn định
//     uint8_t rawData[RX_BUFFER_SIZE];
//     int lengthSensorDataArray = uart_read_bytes(UART_PMS7003_NUM, rawData, RX_BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
//     ESP_LOGI(TAG_PMS7003, "Bytes read: %d", lengthSensorDataArray);

//     bool check = false;
//     for (int i = 0; i < lengthSensorDataArray - 6; i++)
//     {
//         if (rawData[i] == 0x42 && rawData[i + 1] == 0x4d) // Kiểm tra header của packet
//         {
//             int startByte = i + 4; // Bỏ qua 4 byte đầu của header và frame length

//             *pm1_0 = ((uint32_t)rawData[startByte] << 8) + rawData[startByte + 1];
//             *pm2_5 = ((uint32_t)rawData[startByte + 2] << 8) + rawData[startByte + 3];
//             *pm10  = ((uint32_t)rawData[startByte + 4] << 8) + rawData[startByte + 5];

//             ESP_LOGI(TAG_PMS7003, "PMS7003 sensor read data successful.");
//             ESP_LOGI(TAG_PMS7003, "PM1.0: %" PRIu32 " ug/m3, PM2.5: %" PRIu32 " ug/m3, PM10: %" PRIu32 " ug/m3", *pm1_0, *pm2_5, *pm10);
//             check = true;
//             break;
//         }
//     }

//     if (!check) {
//         ESP_LOGE(TAG_PMS7003, "PMS7003 sensor read data failed.");
//         return ESP_FAIL;
//     }
//     return ESP_OK;
// }

// static void i2c_bme280_task(void *arg)
// {
//     // BMX280 sensor setup
//     i2c_config_t i2c_cfg = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = GPIO_NUM_21,
//         .scl_io_num = GPIO_NUM_22,
//         .sda_pullup_en = false,
//         .scl_pullup_en = false,

//         .master = {
//             .clk_speed = 100000
//         }
//     };

//     ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
//     ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

//     bmx280_t* bmx280 = bmx280_create(I2C_NUM_0);

//     if (!bmx280) { 
//         ESP_LOGE("test", "Could not create bmx280 driver.");
//         return;
//     }

//     ESP_ERROR_CHECK(bmx280_init(bmx280));

//     bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
//     ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

//     while (1)
//     {
//         ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
//         do {
//             vTaskDelay(pdMS_TO_TICKS(1));
//         } while(bmx280_isSampling(bmx280));

//         float temp = 0, pres = 0, hum = 0;
//         ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));

//         ESP_LOGI("test", "Read Values: temp = %.4f, pres = %.4f, hum = %.4f", temp, pres, hum);
//         vTaskDelay(2000 / portTICK_PERIOD_MS); // Add a delay to allow other tasks to run
//     }
// }

// void app_assign_task(void) {
//     xTaskCreate(rx_mhz14a_task, "uart_rx_mhz14a_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
//     xTaskCreate(tx_mhz14a_task, "uart_tx_mhz14a_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
//     xTaskCreate(rx_pms7003_task, "uart_rx_pms7003_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
//     xTaskCreate(i2c_bme280_task, "i2c_bme280_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
// }

// void app_main(void)
// {
//     app_uart_init();
//     app_assign_task();
// }
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "../include/bmx280/bmx280.h"
#include <inttypes.h>

// Định nghĩa cho MH-Z14A
static const int RX_BUF_SIZE = 1024;
#define UART_MHZ14A_NUM         UART_NUM_1
#define UART_MHZ14A_TXD_PIN     GPIO_NUM_4
#define UART_MHZ14A_RXD_PIN     GPIO_NUM_5
#define UART_MHZ14A_BAUDRATE    9600

// Định nghĩa cho PMS7003
#define RX_BUFFER_SIZE 128
#define UART_PMS7003_NUM        UART_NUM_2
#define UART_PMS7003_TXD_PIN    GPIO_NUM_17
#define UART_PMS7003_RXD_PIN    GPIO_NUM_16
#define UART_PMS7003_BAUDRATE   9600

// Calibration functions for all sensors
float calibrate_pm1_0(float raw_pm1_0) { return raw_pm1_0 + 5.0; }
float calibrate_pm2_5(float raw_pm2_5) { return raw_pm2_5 + 3.0; }
float calibrate_pm10(float raw_pm10) { return raw_pm10 + 2.0; }
float calibrate_co2(float raw_co2) { return raw_co2 * 1.1; }
float calibrate_temperature(float raw_temp) { return raw_temp + 0.5; }
float calibrate_pressure(float raw_press) { return raw_press + 1.0; }
float calibrate_humidity(float raw_hum) { return raw_hum + 2.0; }

static const char *TAG_PMS7003 = "PMS7003";

// Function declarations
esp_err_t pms7003_readData(uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10);

void uart_init(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate) {
    const uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void app_uart_init(void) {
    uart_init(UART_MHZ14A_NUM, UART_MHZ14A_TXD_PIN, UART_MHZ14A_RXD_PIN, UART_MHZ14A_BAUDRATE);
    uart_init(UART_PMS7003_NUM, UART_PMS7003_TXD_PIN, UART_PMS7003_RXD_PIN, UART_PMS7003_BAUDRATE);
}

int uart_transmit(const char* logName, uart_port_t uart_num, const char* data, const int len) {
    const int txBytes = uart_write_bytes(uart_num, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_mhz14a_task(void *arg) {
    static const char *TX_TASK_TAG = "TX_MHZ14A_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    const char mhz14a_query_co2_cmd[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
    const int cmd_len = sizeof(mhz14a_query_co2_cmd);
    while (1) {
        uart_transmit(TX_TASK_TAG, UART_MHZ14A_NUM, mhz14a_query_co2_cmd, cmd_len);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_mhz14a_task(void *arg) {
    static const char *RX_TASK_TAG = "RX_MHZ14A_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_MHZ14A_NUM, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            uint16_t raw_co2 = data[2] * 0x100 + data[3];
            float co2_concentration = calibrate_co2((float)raw_co2);
            ESP_LOGI(RX_TASK_TAG, "\n\n\tCalibrated CO2 Concentration: %.2f ppm\n\n", co2_concentration);
        }
    }
    free(data);
}

static void rx_pms7003_task(void *arg) {
    uint32_t pm1_0, pm2_5, pm10;
    while (1) {
        esp_err_t err = pms7003_readData(&pm1_0, &pm2_5, &pm10);
        if (err == ESP_OK) {
            // Apply calibration to sensor data
            float calibrated_pm1_0 = calibrate_pm1_0((float)pm1_0);
            float calibrated_pm2_5 = calibrate_pm2_5((float)pm2_5);
            float calibrated_pm10 = calibrate_pm10((float)pm10);
            ESP_LOGI(TAG_PMS7003, "Calibrated PM1.0: %.2f ug/m3, PM2.5: %.2f ug/m3, PM10: %.2f ug/m3", calibrated_pm1_0, calibrated_pm2_5, calibrated_pm10);
        } else {
            ESP_LOGE(TAG_PMS7003, "Failed to read data from PMS7003");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

esp_err_t pms7003_readData(uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Đợi 2 giây để cảm biến ổn định
    uint8_t rawData[RX_BUFFER_SIZE];
    int lengthSensorDataArray = uart_read_bytes(UART_PMS7003_NUM, rawData, RX_BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG_PMS7003, "Bytes read: %d", lengthSensorDataArray);

    bool check = false;
    for (int i = 0; i < lengthSensorDataArray - 6; i++) {
        if (rawData[i] == 0x42 && rawData[i + 1] == 0x4d) { // Kiểm tra header của packet
            int startByte = i + 4; // Bỏ qua 4 byte đầu của header và frame length

            *pm1_0 = ((uint32_t)rawData[startByte] << 8) + rawData[startByte + 1];
            *pm2_5 = ((uint32_t)rawData[startByte + 2] << 8) + rawData[startByte + 3];
            *pm10  = ((uint32_t)rawData[startByte + 4] << 8) + rawData[startByte + 5];

            ESP_LOGI(TAG_PMS7003, "PMS7003 sensor read data successful.");
            ESP_LOGI(TAG_PMS7003, "PM1.0: %" PRIu32 " ug/m3, PM2.5: %" PRIu32 " ug/m3, PM10: %" PRIu32 " ug/m3", *pm1_0, *pm2_5, *pm10);
            check = true;
            break;
        }
    }

    if (!check) {
        ESP_LOGE(TAG_PMS7003, "PMS7003 sensor read data failed.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// BME280 task as described earlier...
static void i2c_bme280_task(void *arg)
{
    // BMX280 sensor setup
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = false,
        .scl_pullup_en = false,

        .master = {
            .clk_speed = 100000
        }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    bmx280_t* bmx280 = bmx280_create(I2C_NUM_0);

    if (!bmx280) { 
        ESP_LOGE("test", "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));

    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    while (1)
    {
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        do {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while(bmx280_isSampling(bmx280));

        float temp = 0, pres = 0, hum = 0;
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));

        ESP_LOGI("test", "Read Values: temp = %.4f, pres = %.4f, hum = %.4f", temp, pres, hum);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Add a delay to allow other tasks to run
    }
}
void app_assign_task(void) {
    xTaskCreate(rx_mhz14a_task, "uart_rx_mhz14a_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_mhz14a_task, "uart_tx_mhz14a_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(rx_pms7003_task, "uart_rx_pms7003_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(i2c_bme280_task, "i2c_bme280_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}

void app_main(void) {
    app_uart_init();
    app_assign_task();
}
