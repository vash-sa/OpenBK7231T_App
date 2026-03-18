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

#if ENABLE_MQTT
static bool g_lora_discovered[256] = {0};

void LoRa_SendDiscovery(int id) {
    if (g_lora_discovered[id]) return;
    g_lora_discovered[id] = true;

    char t[128];
    char p[512];
    char avail_topic[64];

    snprintf(avail_topic, sizeof(avail_topic), "lora/%d/online", id);

    // TEMPERATURE
    snprintf(t, sizeof(t), "homeassistant/sensor/lora_%d_temp/config", id);
    snprintf(p, sizeof(p),
        "{\"name\":\"Temperature\",\"uniq_id\":\"lora_%d_temp\",\"stat_t\":\"lora/%d/state\","
        "\"val_tpl\":\"{{ value_json.temperature }}\",\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\","
        "\"stat_cla\":\"measurement\",\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
        "\"dev\":{\"ids\":[\"lora_%d\"],\"name\":\"LoRa Sensor %d\",\"mdl\":\"LoRa MultiSensor\",\"mf\":\"DIY\"}}",
        id, id, avail_topic, id, id);
    MQTT_PublishMain_StringString(t, p, OBK_PUBLISH_FLAG_RETAIN);

    // BATTERY
    snprintf(t, sizeof(t), "homeassistant/sensor/lora_%d_battery/config", id);
    snprintf(p, sizeof(p),
        "{\"name\":\"Battery\",\"uniq_id\":\"lora_%d_battery\",\"stat_t\":\"lora/%d/state\","
        "\"val_tpl\":\"{{ value_json.battery }}\",\"unit_of_meas\":\"%%\",\"dev_cla\":\"battery\","
        "\"stat_cla\":\"measurement\",\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
        "\"dev\":{\"ids\":[\"lora_%d\"]}}",
        id, id, avail_topic, id);
    MQTT_PublishMain_StringString(t, p, OBK_PUBLISH_FLAG_RETAIN);

    // GAS
    snprintf(t, sizeof(t), "homeassistant/sensor/lora_%d_gas/config", id);
    snprintf(p, sizeof(p),
        "{\"name\":\"Gas\",\"uniq_id\":\"lora_%d_gas\",\"stat_t\":\"lora/%d/state\","
        "\"val_tpl\":\"{{ value_json.gas }}\",\"unit_of_meas\":\"ppm\",\"stat_cla\":\"measurement\","
        "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
        "\"dev\":{\"ids\":[\"lora_%d\"]}}",
        id, id, avail_topic, id);
    MQTT_PublishMain_StringString(t, p, OBK_PUBLISH_FLAG_RETAIN);

    // SMOKE
    snprintf(t, sizeof(t), "homeassistant/binary_sensor/lora_%d_smoke/config", id);
    snprintf(p, sizeof(p),
        "{\"name\":\"Smoke\",\"uniq_id\":\"lora_%d_smoke\",\"stat_t\":\"lora/%d/state\","
        "\"val_tpl\":\"{{ 'ON' if value_json.smoke == 'YES' else 'OFF' }}\",\"dev_cla\":\"smoke\","
        "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
        "\"dev\":{\"ids\":[\"lora_%d\"]}}",
        id, id, avail_topic, id);
    MQTT_PublishMain_StringString(t, p, OBK_PUBLISH_FLAG_RETAIN);
}
#endif

// SPI
void LoRa_WriteReg(uint8_t addr, uint8_t val) {
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = data;
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
}

uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {0};
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

// INIT
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
}

// LOOP
void LoRa_RunFrame() {
    if (HAL_PIN_ReadDigitalInput(LORA_DIO0)) {
        uint8_t irq = LoRa_ReadReg(REG_IRQ_FLAGS);
        LoRa_WriteReg(REG_IRQ_FLAGS, irq);

        if (irq & 0x40) {
            uint8_t len = LoRa_ReadReg(REG_RX_NB_BYTES);
            uint8_t buffer[128];

            LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_RX_CURRENT));
            for(int i = 0; i < len && i < 127; i++) {
                buffer[i] = LoRa_ReadReg(0x00);
            }

            int id = buffer[0];
            int pLen = buffer[1];
            if(pLen > 64) pLen = 64;

            for (int i = 0; i < pLen; i++) {
                buffer[i+2] ^= ENCRYPT_KEY[i % 16];
            }
            buffer[2 + pLen] = '\0';

            float vcc, temp;
            int ppm;
            char smoke[16];

            if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {

#if ENABLE_MQTT
                LoRa_SendDiscovery(id);

                char state_topic[64];
                char state_payload[128];
                char avail_topic[64];

                snprintf(state_topic, sizeof(state_topic), "lora/%d/state", id);
                snprintf(avail_topic, sizeof(avail_topic), "lora/%d/online", id);

                int battery = (int)((vcc - 3.0f) * 100.0f / (4.2f - 3.0f));
                if (battery < 0) battery = 0;
                if (battery > 100) battery = 100;

                snprintf(state_payload, sizeof(state_payload),
                    "{\"temperature\":%.1f,\"battery\":%d,\"smoke\":\"%s\",\"gas\":%d}",
                    temp, battery, smoke, ppm);

                MQTT_PublishMain_StringString(avail_topic, "online", 0);
                MQTT_PublishMain_StringString(state_topic, state_payload, 0);
#endif
            }
        }
    }
}

#endif
