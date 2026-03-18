#include "../new_common.h"

#if PLATFORM_ESPIDF

#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "drv_public.h"
#include "drv_local.h"
#include "../hal/hal_pins.h"
#include "../mqtt/new_mqtt.h"

#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"

static spi_device_handle_t lora_spi;

#define LORA_NSS  7
#define LORA_RST  10
#define LORA_DIO0 3

#define REG_VERSION             0x42
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_LNA                 0x0C
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_RX_CURRENT     0x10
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_MODEM_CONFIG_3      0x26
#define REG_PAYLOAD_LENGTH      0x22
#define REG_DIO_MAPPING_1       0x40

#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define MODE_RX_CONTINUOUS      0x05

static const uint8_t ENCRYPT_KEY[16] =
{'1','2','3','4','5','6','7','8','9','0','A','B','C','D','E','F'};

// --- MQTT Discovery для Home Assistant ---
#if ENABLE_MQTT
void LoRa_SendDiscovery(int id) {
    char t[128];
    char p[512];

    // 1. Температура. Добавляем "/" в начало топика t!
    snprintf(t, sizeof(t), "/homeassistant/sensor/lora_%d_t/config", id);
    snprintf(p, sizeof(p), 
        "{"
            "\"name\":\"Temperature\","
            "\"stat_t\":\"esp32/lora/%d/s/get\"," // Полный путь из логов!
            "\"val_tpl\":\"{{value_json.t}}\","
            "\"unit_of_meas\":\"°C\","
            "\"dev_cla\":\"temperature\","
            "\"uniq_id\":\"lora_%d_t\","
            "\"dev\":{\"ids\":[\"lora_%d\"],\"name\":\"LoRa Node %d\"}"
        "}", id, id, id, id);
    MQTT_PublishMain_StringString(t, p, OBK_PUBLISH_FLAG_RETAIN);

    // 2. Батарея. Снова "/" в начале топика t!
    snprintf(t, sizeof(t), "/homeassistant/sensor/lora_%d_v/config", id);
    snprintf(p, sizeof(p), 
        "{"
            "\"name\":\"Battery\","
            "\"stat_t\":\"esp32/lora/%d/s/get\"," // Полный путь из логов!
            "\"val_tpl\":\"{{value_json.v}}\","
            "\"unit_of_meas\":\"V\","
            "\"dev_cla\":\"voltage\","
            "\"uniq_id\":\"lora_%d_v\","
            "\"dev\":{\"ids\":[\"lora_%d\"]}" // ID тот же для связки в одну карточку
        "}", id, id, id);
    MQTT_PublishMain_StringString(t, p, OBK_PUBLISH_FLAG_RETAIN);
}

#endif

// --- Низкоуровневые функции SPI ---
void LoRa_WriteReg(uint8_t addr, uint8_t val) {
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = data;
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
}

uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0 };
    uint8_t rx[2] = {0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
    return rx[1];
}

void LoRa_SetFrequency(long freq) {
    long frf = ((long long)freq << 19) / 32000000;
    LoRa_WriteReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_WriteReg(REG_FRF_LSB, (uint8_t)(frf));
}

// --- Инициализация драйвера ---
void LoRa_Init_Driver() {
    spi_bus_config_t buscfg = {
        .miso_io_num = 5, .mosi_io_num = 6, .sclk_io_num = 4,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = -1, .queue_size = 7
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);

    HAL_PIN_Setup_Output(LORA_NSS);
    HAL_PIN_Setup_Output(LORA_RST);
    HAL_PIN_Setup_Input(LORA_DIO0);

    HAL_PIN_SetOutputValue(LORA_RST, 0);
    delay_ms(10);
    HAL_PIN_SetOutputValue(LORA_RST, 1);
    delay_ms(10);

    uint8_t version = LoRa_ReadReg(REG_VERSION);
    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "SX127x version: %02X", version);

    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    LoRa_SetFrequency(433000000);
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR, 0);
    LoRa_WriteReg(REG_LNA, 0x23);
    LoRa_WriteReg(REG_MODEM_CONFIG_1, 0x72);
    LoRa_WriteReg(REG_MODEM_CONFIG_2, 0xC4);
    LoRa_WriteReg(REG_MODEM_CONFIG_3, 0x0C);
    LoRa_WriteReg(REG_PAYLOAD_LENGTH, 0xFF);
    LoRa_WriteReg(REG_DIO_MAPPING_1, 0x00);
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "LoRa RX started");
}

// --- Основной цикл обработки кадров ---
void LoRa_RunFrame() {
    if (HAL_PIN_ReadDigitalInput(LORA_DIO0)) {
        uint8_t irq = LoRa_ReadReg(REG_IRQ_FLAGS);
        LoRa_WriteReg(REG_IRQ_FLAGS, irq); // Сброс флагов прерывания

        if (irq & 0x40) { // RX Done
            uint8_t len = LoRa_ReadReg(REG_RX_NB_BYTES);
            uint8_t buffer[128];
            
            LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_RX_CURRENT));
            for(int i = 0; i < len && i < 127; i++) {
                buffer[i] = LoRa_ReadReg(0x00);
            }

            int id = buffer[0];
            int pLen = buffer[1];
            if(pLen > 64) pLen = 64;

            // Дешифровка XOR
            for (int i = 0; i < pLen; i++) {
                buffer[i+2] ^= ENCRYPT_KEY[i % 16];
            }
            buffer[2 + pLen] = '\0';

            addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "LoRa Node %d: %s", id, (char*)&buffer[2]);

            float vcc, temp;
            int ppm;
            char smoke[16];
            
            // Парсинг строки: "VCC,TEMP,SMOKE,PPM"
            if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {
                // 1. Обновление внутренних каналов OpenBeken
                //CHANNEL_Set(1, (int)(vcc * 10), 0);
                //CHANNEL_Set(2, (int)temp, 0);
                //CHANNEL_Set(3, (strcmp(smoke, "YES") == 0 ? 1 : 0), 0);
                //CHANNEL_Set(4, ppm, 0);

#if ENABLE_MQTT
    // 1. Регистрация карточки (вызывай один раз при получении новой ноды)
    LoRa_SendDiscovery(id);

    // 2. Подготовка топика данных (должен В ТОЧНОСТИ совпадать с "stat_t" из твоего конфига)
    char state_topic[64];
    snprintf(state_topic, sizeof(state_topic), "lora/%d/s", id);

    // 3. Формируем JSON с ключами "t" (температура) и "v" (батарея)
    // Эти ключи должны совпадать с тем, что ты написал в "val_tpl" (value_json.t и value_json.v)
    char state_payload[128];
    snprintf(state_payload, sizeof(state_payload), 
             "{\"t\":%.1f,\"v\":%.2f}", 
             temp, vcc); // Берем значения из твоего парсинга

    // 4. Пуляем в MQTT. Retain = 0, чтобы не забивать брокер старьем.
    MQTT_PublishMain_StringString(state_topic, state_payload, 0);
#endif
            }
        }
    }
}

#endif
