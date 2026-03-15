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

#define REG_FIFO                0x00
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
#define REG_VERSION             0x42

#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define MODE_RX_CONTINUOUS      0x05

#define IRQ_RX_DONE             0x40

static const uint8_t ENCRYPT_KEY[16] =
{'1','2','3','4','5','6','7','8','9','0','A','B','C','D','E','F'};

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void LoRa_WriteReg(uint8_t addr, uint8_t val) {
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val };

    spi_transaction_t t;
    memset(&t,0,sizeof(t));

    t.length = 16;
    t.tx_buffer = data;

    HAL_PIN_SetOutputValue(LORA_NSS,0);
    spi_device_polling_transmit(lora_spi,&t);
    HAL_PIN_SetOutputValue(LORA_NSS,1);
}

uint8_t LoRa_ReadReg(uint8_t addr) {

    uint8_t tx[2] = { addr & 0x7F, 0 };
    uint8_t rx[2] = {0};

    spi_transaction_t t;
    memset(&t,0,sizeof(t));

    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    HAL_PIN_SetOutputValue(LORA_NSS,0);
    spi_device_polling_transmit(lora_spi,&t);
    HAL_PIN_SetOutputValue(LORA_NSS,1);

    return rx[1];
}

void LoRa_SetFrequency(long freq) {

    long frf = ((long long)freq << 19) / 32000000;

    LoRa_WriteReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_WriteReg(REG_FRF_LSB, (uint8_t)(frf));
}

void LoRa_Init_Driver() {

    spi_bus_config_t buscfg = {
        .miso_io_num = 5,
        .mosi_io_num = 6,
        .sclk_io_num = 4,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 7
    };

    spi_bus_initialize(SPI2_HOST,&buscfg,SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST,&devcfg,&lora_spi);

    HAL_PIN_Setup_Output(LORA_NSS);
    HAL_PIN_Setup_Output(LORA_RST);
    HAL_PIN_Setup_Input(LORA_DIO0);

    HAL_PIN_SetOutputValue(LORA_RST,0);
    delay_ms(10);
    HAL_PIN_SetOutputValue(LORA_RST,1);
    delay_ms(10);

    uint8_t version = LoRa_ReadReg(REG_VERSION);

    addLogAdv(LOG_INFO,LOG_FEATURE_DRV,"SX127x version: %02X",version);

    LoRa_WriteReg(REG_OP_MODE,MODE_LONG_RANGE_MODE | MODE_SLEEP);

    LoRa_SetFrequency(433000000);

    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR,0);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR,0);

    LoRa_WriteReg(REG_LNA,0x23);

    LoRa_WriteReg(REG_MODEM_CONFIG_1,0x72);
    LoRa_WriteReg(REG_MODEM_CONFIG_2,0xC4);
    LoRa_WriteReg(REG_MODEM_CONFIG_3,0x0C);

    LoRa_WriteReg(REG_PAYLOAD_LENGTH,0xFF);

    LoRa_WriteReg(REG_DIO_MAPPING_1,0x00);

    LoRa_WriteReg(REG_OP_MODE,MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    addLogAdv(LOG_INFO,LOG_FEATURE_DRV,"LoRa RX started");
}

void LoRa_RunFrame() {

    if (!HAL_PIN_ReadDigitalInput(LORA_DIO0))
        return;

    uint8_t irq = LoRa_ReadReg(REG_IRQ_FLAGS);

    LoRa_WriteReg(REG_IRQ_FLAGS, irq);

    if (!(irq & IRQ_RX_DONE))
        return;

    uint8_t len = LoRa_ReadReg(REG_RX_NB_BYTES);

    uint8_t buffer[128];

    LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_RX_CURRENT));

    for (int i = 0; i < len && i < sizeof(buffer); i++) {
        buffer[i] = LoRa_ReadReg(REG_FIFO);
    }

    if (len < 5)
        return;

    uint16_t receivedCRC =
        (buffer[len-2] << 8) |
         buffer[len-1];

    uint16_t calcCRC =
        crc16_ccitt(buffer,len-2);

    if (receivedCRC != calcCRC) {

        addLogAdv(LOG_WARN,LOG_FEATURE_DRV,
        "CRC error %04X/%04X",receivedCRC,calcCRC);

        return;
    }

    uint8_t id = buffer[0];
    uint8_t payloadLen = buffer[1];

    if (payloadLen > 64)
        payloadLen = 64;

    for (int i = 0; i < payloadLen; i++) {
        buffer[i+2] ^= ENCRYPT_KEY[i % 16];
    }

    buffer[2+payloadLen] = '\0';

    addLogAdv(LOG_INFO,LOG_FEATURE_DRV,
    "LoRa Node %d: %s",id,(char*)&buffer[2]);

    float vcc,temp;
    int ppm;
    char smoke[8];

    if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {
        char topic[64], payload[192], dev_id[32];
        int is_smoke = (strcmp(smoke, "YES") == 0 ? 1 : 0);
        
        // Уникальный ID устройства для HA
        snprintf(dev_id, sizeof(dev_id), "lora_node_%d", id);

        // 1. ПУБЛИКАЦИЯ ДАННЫХ (одной строкой для экономии трафика)
        snprintf(topic, sizeof(topic), "lora/%s/state", dev_id);
        snprintf(payload, sizeof(payload), "{\"vcc\":%.2f,\"temp\":%.1f,\"smoke\":%d,\"ppm\":%d}", 
                 vcc, temp, is_smoke, ppm);
        MQTT_Publish(topic, payload, 0, 0);

        // 2. АВТО-РЕГИСТРАЦИЯ В HA (Discovery)
        // Чтобы не слать это каждую секунду, можно слать только если id новый, 
        // но для теста оставим так - HA сам отсеет дубликаты.
        
        // Регистрируем ТЕМПЕРАТУРУ
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_T/config", dev_id);
        snprintf(payload, sizeof(payload), 
            "{\"name\":\"Temp\",\"stat_t\":\"lora/%s/state\",\"val_tpl\":\"{{value_json.temp}}\",\"unit_of_meas\":\"°C\",\"dev\":{\"ids\":[\"%s\"],\"name\":\"LoRa Node %d\",\"mf\":\"DIY\",\"mdl\":\"SX1278\"}}", 
            dev_id, dev_id, id);
        MQTT_Publish(topic, payload, 0, 0);

        // Регистрируем ДЫМ (как binary_sensor)
        snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s_S/config", dev_id);
        snprintf(payload, sizeof(payload), 
            "{\"name\":\"Smoke\",\"stat_t\":\"lora/%s/state\",\"val_tpl\":\"{{value_json.smoke}}\",\"payload_on\":1,\"payload_off\":0,\"dev_cla\":\"smoke\",\"dev\":{\"ids\":[\"%s\"]}}", 
            dev_id, dev_id);
        MQTT_Publish(topic, payload, 0, 0);
        
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "LoRa Node %d proccessed", id);
    }
}

#endif
