#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

#define RX_BUFFER_SIZE 128
#define CONFIG_PMS_UART_PORT UART_NUM_1
#define CONFIG_PMS_PIN_TX 17
#define CONFIG_PMS_PIN_RX 16

static const char *TAG = "PMS7003";

// Khai báo hàm
esp_err_t pms7003_initUart(void);
esp_err_t pms7003_readData(uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10);

// Hàm khởi tạo UART
esp_err_t pms7003_initUart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t error_1 = uart_driver_install(CONFIG_PMS_UART_PORT, RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    esp_err_t error_2 = uart_param_config(CONFIG_PMS_UART_PORT, &uart_config);
    esp_err_t error_3 = uart_set_pin(CONFIG_PMS_UART_PORT, CONFIG_PMS_PIN_TX, CONFIG_PMS_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (error_1 == ESP_OK && error_2 == ESP_OK && error_3 == ESP_OK)
    {
        ESP_LOGI(TAG, "PMS7003 UART port initialized successfully.");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "PMS7003 UART port initialization failed.");
        return ESP_FAIL;
    }
}

// Hàm đọc dữ liệu từ cảm biến
esp_err_t pms7003_readData(uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Đợi 2 giây để cảm biến ổn định
    uint8_t rawData[RX_BUFFER_SIZE];
    int lengthSensorDataArray = uart_read_bytes(CONFIG_PMS_UART_PORT, rawData, RX_BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Bytes read: %d", lengthSensorDataArray);
    for (int i = 0; i < lengthSensorDataArray; i++) {
        ESP_LOGI(TAG, "rawData[%d] = %02x", i, rawData[i]);
    }

    bool check = false;
    for (int i = 0; i < lengthSensorDataArray - 6; i++)
    {
        if (rawData[i] == 0x42 && rawData[i + 1] == 0x4d) // Kiểm tra header của packet
        {
            int startByte = i + 4; // Bỏ qua 4 byte đầu của header và frame length

            *pm1_0 = ((uint32_t)rawData[startByte] << 8) + rawData[startByte + 1];
            *pm2_5 = ((uint32_t)rawData[startByte + 2] << 8) + rawData[startByte + 3];
            *pm10  = ((uint32_t)rawData[startByte + 4] << 8) + rawData[startByte + 5];

            ESP_LOGI(TAG, "PMS7003 sensor read data successful.");
            ESP_LOGI(TAG, "PM1.0: %" PRIu32 " ug/m3, PM2.5: %" PRIu32 " ug/m3, PM10: %" PRIu32 " ug/m3", *pm1_0, *pm2_5, *pm10);
            check = true;
            break;
        }
    }

    if (!check) {
        ESP_LOGE(TAG, "PMS7003 sensor read data failed.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void app_main()
{
    if (pms7003_initUart() == ESP_OK) {
        uint32_t pm1_0, pm2_5, pm10;
        if (pms7003_readData(&pm1_0, &pm2_5, &pm10) == ESP_OK) {
            // Đọc dữ liệu thành công
        }
    }
}