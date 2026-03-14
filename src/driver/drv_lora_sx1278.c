#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "drv_public.h"
#include "drv_local.h"
#include "hal/hal_pins.h"
#include <string.h>

// Пины для SuperMini (GPIO номера)
#define LORA_NSS  7
#define LORA_RST  10
#define LORA_DIO0 3

// Ключ шифрования (как на Nano)
static const char *ENCRYPT_KEY = "1234567890ABCDEF";

// Вспомогательная функция записи в регистр SX1278 через SPI OpenBK
void LoRa_WriteReg(uint8_t addr, uint8_t val) {
    HAL_PIN_SetOutputValue(LORA_NSS, 0); // CS Low
    SPI_WriteByte(addr | 0x80);          // Бит записи 0x80
    SPI_WriteByte(val);
    HAL_PIN_SetOutputValue(LORA_NSS, 1); // CS High
}

uint8_t LoRa_ReadReg(uint8_t addr) {
    uint8_t res;
    HAL_PIN_SetOutputValue(LORA_NSS, 0);
    SPI_WriteByte(addr & 0x7F);          // Бит чтения 0x00
    res = SPI_ReadByte();
    HAL_PIN_SetOutputValue(LORA_NSS, 1);
    return res;
}

// Инициализация LoRa (Аналог LoRa.begin)
void LoRa_Init_Driver() {
    // Настройка пинов управления
    HAL_PIN_Setup_Output(LORA_NSS);
    HAL_PIN_Setup_Output(LORA_RST);
    HAL_PIN_Setup_Input(LORA_DIO0);

    // Сброс модуля
    HAL_PIN_SetOutputValue(LORA_RST, 0);
    delay_ms(10);
    HAL_PIN_SetOutputValue(LORA_RST, 1);
    delay_ms(10);

    // Минимальный конфиг регистров (433MHz, SF12, BW125)
    LoRa_WriteReg(0x01, 0x80); // Sleep mode & LoRa mode
    LoRa_WriteReg(0x06, 0x6C); // Frequency 433.000 MHz (MSB)
    LoRa_WriteReg(0x07, 0x80); // (Mid)
    LoRa_WriteReg(0x08, 0x00); // (LSB)
    LoRa_WriteReg(0x1D, 0x72); // BW 125kHz, CR 4/5, Explicit Header
    LoRa_WriteReg(0x1E, 0xC4); // SF 12, CRC On
    LoRa_WriteReg(0x26, 0x08); // Low Data Rate Optimize ON
    
    LoRa_WriteReg(0x01, 0x85); // Режим постоянного приема (RXCONTINUOUS)
    
    ADDLOG_INFO(LOG_FEATURE_LORA, "LoRa SX1278 Driver Initialized on 433MHz");
}

// Цикл проверки пакетов (вызывается каждую итерацию основного цикла)
void LoRa_RunFrame() {
    // Проверяем DIO0 (пришел ли пакет)
    if (HAL_PIN_ReadDigitalInput(LORA_DIO0)) {
        uint8_t irq = LoRa_ReadReg(0x12); // Читаем IRQ Flags
        LoRa_WriteReg(0x12, irq);        // Сбрасываем флаги

        if (irq & 0x40) { // Packet Rx Done
            uint8_t len = LoRa_ReadReg(0x13); // Длина полученных байт
            uint8_t buffer[64];
            
            // Читаем данные из FIFO
            LoRa_WriteReg(0x0D, LoRa_ReadReg(0x10)); // Устанавливаем адрес FIFO на начало пакета
            for(int i=0; i<len; i++) {
                buffer[i] = LoRa_ReadReg(0x00);
            }

            // Твой XOR-дешифратор
            uint8_t id = buffer[0];
            uint8_t payloadLen = buffer[1];
            for (int i = 0; i < payloadLen; i++) {
                buffer[i+2] ^= ENCRYPT_KEY[i % 16];
            }
            buffer[2 + payloadLen] = '\0';

            // Вывод в лог OpenBK (увидишь в браузере)
            ADDLOG_INFO(LOG_FEATURE_LORA, "LoRa Node %d Data: %s", id, (char*)&buffer[2]);

            // ПАРСИНГ В КАНАЛЫ (для Home Assistant)
            float vcc, temp;
            int ppm;
            char smoke[4];
            if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {
                CHANNEL_Set(1, (int)(vcc * 10), 0); // Канал 1: Вольты * 10
                CHANNEL_Set(2, (int)temp, 0);       // Канал 2: Температура
                CHANNEL_Set(3, (strcmp(smoke, "YES") == 0 ? 1 : 0), 0); // Канал 3: Дым
            }
        }
    }
}
