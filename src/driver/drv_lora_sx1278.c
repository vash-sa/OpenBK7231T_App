#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "drv_public.h"
#include "drv_local.h"
#include "hal/hal_pins.h"
#include <string.h>

// Используем общие функции SPI OpenBK
extern void SPI_WriteByte(uint8_t b);
extern uint8_t SPI_ReadByte();

#define LORA_NSS  7
#define LORA_RST  10
#define LORA_DIO0 3

static const char *ENCRYPT_KEY = "1234567890ABCDEF";

void LoRa_WriteReg(uint8_t addr, uint8_t val) {
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    SPI_WriteByte(addr | 0x80);
    SPI_WriteByte(val);
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
}

uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t res;
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    SPI_WriteByte(addr & 0x7F);
    res = SPI_ReadByte();
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
    return res;
}

void LoRa_Init_Driver() {
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
