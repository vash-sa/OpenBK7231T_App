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
#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_log.h"

static spi_device_handle_t lora_spi;

#define LORA_NSS  7
#define LORA_RST  10
#define LORA_DIO0 3

static const uint8_t ENCRYPT_KEY[16] = {'1','2','3','4','5','6','7','8','9','0','A','B','C','D','E','F'};
static uint32_t g_registered_nodes = 0; 

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
    uint8_t res = 0;
    uint8_t cmd = addr & 0x7F;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.rxlength = 8;
    t.rx_buffer = &res;
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
    return res;
}

// РЕГИСТРАЦИЯ УСТРОЙСТВА В HA
void LoRa_SendDiscovery(int id) {
    char t[64];
    char p[256];

    // 1. Температура (3 штуки %d в строке -> 3 аргумента id)
    snprintf(t, sizeof(t), "homeassistant/sensor/lora_%d_t/config", id);
    snprintf(p, sizeof(p), "{\"name\":\"Temp\",\"stat_t\":\"lora/%d/s\",\"val_tpl\":\"{{value_json.t}}\",\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\",\"dev\":{\"ids\":[\"lora_%d\"],\"name\":\"LoRa Node %d\"}}", 
            id, id, id);
    MQTT_Publish(t, p, 0, 1);

    // 2. Батарея (2 штуки %d в строке -> 2 аргумента id)
    snprintf(t, sizeof(t), "homeassistant/sensor/lora_%d_v/config", id);
    snprintf(p, sizeof(p), "{\"name\":\"Batt\",\"stat_t\":\"lora/%d/s\",\"val_tpl\":\"{{value_json.v}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"dev\":{\"ids\":[\"lora_%d\"]}}", 
            id, id);
    MQTT_Publish(t, p, 0, 1);

    // 3. Газ (2 штуки %d в строке -> 2 аргумента id)
    snprintf(t, sizeof(t), "homeassistant/sensor/lora_%d_p/config", id);
    snprintf(p, sizeof(p), "{\"name\":\"Gas\",\"stat_t\":\"lora/%d/s\",\"val_tpl\":\"{{value_json.p}}\",\"unit_of_meas\":\"ppm\",\"dev\":{\"ids\":[\"lora_%d\"]}}", 
            id, id);
    MQTT_Publish(t, p, 0, 1);

    // 4. Дым (2 штуки %d в строке -> 2 аргумента id)
    snprintf(t, sizeof(t), "homeassistant/binary_sensor/lora_%d_s/config", id);
    snprintf(p, sizeof(p), "{\"name\":\"Smoke\",\"stat_t\":\"lora/%d/s\",\"val_tpl\":\"{{'ON' if value_json.s=='YES' else 'OFF'}}\",\"dev_cla\":\"smoke\",\"dev\":{\"ids\":[\"lora_%d\"]}}", 
            id, id);
    MQTT_Publish(t, p, 0, 1);
}


void LoRa_Init_Driver() {
    spi_bus_config_t buscfg = { .miso_io_num = 5, .mosi_io_num = 6, .sclk_io_num = 4, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32 };
    spi_device_interface_config_t devcfg = { .clock_speed_hz = 1*1000*1000, .mode = 0, .spics_io_num = -1, .queue_size = 7 };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);

    HAL_PIN_Setup_Output(LORA_NSS);
    HAL_PIN_Setup_Output(LORA_RST);
    HAL_PIN_Setup_Input(LORA_DIO0);

    HAL_PIN_SetOutputValue(LORA_RST, 0);
    delay_ms(10);
    HAL_PIN_SetOutputValue(LORA_RST, 1);
    delay_ms(10);

    LoRa_WriteReg(0x01, 0x80); 
    LoRa_WriteReg(0x01, 0x85); 
    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "LoRa Initialized");
}

void LoRa_RunFrame() {
    if (HAL_PIN_ReadDigitalInput(LORA_DIO0)) {
        uint8_t irq = LoRa_ReadReg(0x12);
        LoRa_WriteReg(0x12, irq);

        if (irq & 0x40) {
            uint8_t len = LoRa_ReadReg(0x13);
            uint8_t buffer[128];
            LoRa_WriteReg(0x0D, LoRa_ReadReg(0x10));
            for(int i=0; i<len && i<127; i++) buffer[i] = LoRa_ReadReg(0x00);

            uint8_t id = buffer[0];
            uint8_t pLen = buffer[1];
            if(pLen > 64) pLen = 64;

            for (int i = 0; i < pLen; i++) buffer[i+2] ^= ENCRYPT_KEY[i % 16];
            buffer[2 + pLen] = '\0';

            float vcc, temp;
            int ppm;
            char smoke[16];
            if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {
                // Регистрируем, если еще не было такого ID
                if (id < 32 && !(g_registered_nodes & (1 << id))) {
                    LoRa_SendDiscovery(id);
                    g_registered_nodes |= (1 << id);
                }

                // Шлем данные состояния в JSON
                char s_topic[64], s_payload[256];
                snprintf(s_topic, sizeof(s_topic), "lora/%d/s", id);
                snprintf(s_payload, sizeof(s_payload), "{\"v\":%.2f,\"t\":%.1f,\"s\":\"%s\",\"p\":%d}", vcc, temp, smoke, ppm);
                MQTT_Publish(s_topic, s_payload, 0, 0);

                addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "Node %d: T=%.1f V=%.2f Smoke=%s", id, temp, vcc, smoke);
            }
        }
    }
}
#endif
