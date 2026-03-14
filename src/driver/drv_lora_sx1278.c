#include "../new_common.h"

#if PLATFORM_ESPIDF

#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "drv_public.h"
#include "drv_local.h"
#include "../hal/hal_pins.h"

#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"

// Глобальная переменная для SPI
static spi_device_handle_t lora_spi;

#define LORA_NSS  7
#define LORA_RST  10
#define LORA_DIO0 3

static const uint8_t ENCRYPT_KEY[16] = {'1','2','3','4','5','6','7','8','9','0','A','B','C','D','E','F'};

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
        .clock_speed_hz = 1*1000*1000,
        .mode = 0,
        .spics_io_num = -1, 
        .queue_size = 7
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

    LoRa_WriteReg(0x01, 0x80); 
    LoRa_WriteReg(0x06, 0x6C); 
    LoRa_WriteReg(0x07, 0x80); 
    LoRa_WriteReg(0x08, 0x00); 
    LoRa_WriteReg(0x1D, 0x72); 
    LoRa_WriteReg(0x1E, 0xC4); 
    LoRa_WriteReg(0x26, 0x08); 
    LoRa_WriteReg(0x01, 0x85); 
    
    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "LoRa SX1278 Initialized");
}

void LoRa_RunFrame() {
    if (HAL_PIN_ReadDigitalInput(LORA_DIO0)) {
        uint8_t irq = LoRa_ReadReg(0x12);
        LoRa_WriteReg(0x12, irq);

        if (irq & 0x40) {
            uint8_t len = LoRa_ReadReg(0x13);
            uint8_t buffer[128];
            
            LoRa_WriteReg(0x0D, LoRa_ReadReg(0x10));
            for(int i=0; i<len && i<127; i++) {
                buffer[i] = LoRa_ReadReg(0x00);
            }

            uint8_t id = buffer[0];
            uint8_t pLen = buffer[1];
            if(pLen > 64) pLen = 64;

            for (int i = 0; i < pLen; i++) {
                buffer[i+2] ^= ENCRYPT_KEY[i % 16];
            }
            buffer[2 + pLen] = '\0';

            addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "LoRa Node %d: %s", id, (char*)&buffer[2]);

            float vcc, temp;
            int ppm;
            char smoke[8];
            if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {
                CHANNEL_Set(1, (int)(vcc * 10), 0);
                CHANNEL_Set(2, (int)temp, 0);
                CHANNEL_Set(3, (strcmp(smoke, "YES") == 0 ? 1 : 0), 0);
                CHANNEL_Set(4, ppm, 0);
            }
        }
    }
}

#endif
