#include "../obk_config.h"

#if defined(PLATFORM_LN882H)

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_bh1750.h"

// Адреса датчика со сдвигом влево на 1 бит (как AHT2X_I2C_ADDR)
#define BH1750_I2C_ADDR_23 (0x23 << 1)
#define BH1750_I2C_ADDR_5C (0x5C << 1)

// Локальная структура шины, строго как в drv_aht2x.c
static softI2C_t g_bh1750_i2c; 

static int g_bh1750_lux_channel = -1;
static int g_bh1750_secondsBetweenMeasurements = 1;
static int g_bh1750_secondsElapsed = 0;
static bool g_bh1750_init_ok = false;
static uint8_t g_bh1750_active_addr = 0;

// Чтение уровня освещенности
void BH1750_Measure(void) {
    uint8_t byte_high = 0;
    uint8_t byte_low = 0;
    uint16_t raw_lux = 0;
    float lux = 0.0f;

    if (!g_bh1750_init_ok || g_bh1750_active_addr == 0) {
        return;
    }

    // Стартуем чтение с добавлением бита чтения (| 1), строго по образцу AHT2X!
    if (Soft_I2C_Start(&g_bh1750_i2c, g_bh1750_active_addr | 1) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Sensor lost on 0x%02X", g_bh1750_active_addr >> 1);
        return;
    }
    
    // Читаем байты (false - ACK, true - NACK для завершения чтения)
    byte_high = Soft_I2C_ReadByte(&g_bh1750_i2c, false); 
    byte_low = Soft_I2C_ReadByte(&g_bh1750_i2c, true); 
    Soft_I2C_Stop(&g_bh1750_i2c);

    // Склеиваем результат
    raw_lux = (byte_high << 8) | byte_low;
    lux = (float)raw_lux / 1.2f;

    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Calculated: %d lux", (int)lux);

    // Выводим в веб-интерфейс
    CHANNEL_Set(g_bh1750_lux_channel, (int)lux, 0);
}

// Изменение интервала опроса
commandResult_t BH1750_Cycle(const void *context, const char *cmd, const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_GetArgsCount() < 1) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    g_bh1750_secondsBetweenMeasurements = Tokenizer_GetArgInteger(0);
    if (g_bh1750_secondsBetweenMeasurements < 1) {
        g_bh1750_secondsBetweenMeasurements = 1;
    }
    return CMD_RES_OK;
}

// Принудительный ручной замер
commandResult_t BH1750_ForceMeasure(const void *context, const char *cmd, const char *args, int cmdFlags) {
    BH1750_Measure();
    return CMD_RES_OK;
}

// Инициализация строго по логике твоего примера AHT2X
void BH1750_Init(void) {
    // Берем пины через Tokenizer_GetPin из аргументов startDriver
    g_bh1750_i2c.pin_clk = Tokenizer_GetPin(1, 2);  
    g_bh1750_i2c.pin_data = Tokenizer_GetPin(2, 3); 
    g_bh1750_lux_channel = Tokenizer_GetArgIntegerDefault(3, 6); 

    // Инициализация софтового I2C
    Soft_I2C_PreInit(&g_bh1750_i2c);
    rtos_delay_milliseconds(100);

    // Сканируем, на каком адресе отзовется датчик в режиме записи
    if (Soft_I2C_Start(&g_bh1750_i2c, BH1750_I2C_ADDR_23) == true) {
        g_bh1750_active_addr = BH1750_I2C_ADDR_23;
    } else {
        Soft_I2C_Stop(&g_bh1750_i2c);
        if (Soft_I2C_Start(&g_bh1750_i2c, BH1750_I2C_ADDR_5C) == true) {
            g_bh1750_active_addr = BH1750_I2C_ADDR_5C;
        } else {
            Soft_I2C_Stop(&g_bh1750_i2c);
            addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Init failed, sensor not found on 0x23 or 0x5C");
            g_bh1750_init_ok = false;
            return;
        }
    }

    // Если адрес определен, шлем команды запуска (уже внутри сессии старта)
    Soft_I2C_WriteByte(&g_bh1750_i2c, 0x01); // Power ON
    Soft_I2C_Stop(&g_bh1750_i2c);

    rtos_delay_milliseconds(10);

    // Запускаем режим Continuous High Res Mode (0x10)
    Soft_I2C_Start(&g_bh1750_i2c, g_bh1750_active_addr);
    Soft_I2C_WriteByte(&g_bh1750_i2c, 0x10);
    Soft_I2C_Stop(&g_bh1750_i2c);

    CMD_RegisterCommand("BH1750_Cycle", BH1750_Cycle, NULL);
    CMD_RegisterCommand("BH1750_Measure", BH1750_ForceMeasure, NULL);
              
    g_bh1750_init_ok = true;
    g_bh1750_secondsElapsed = 0;
    
    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Successfully initialized on 0x%02X", g_bh1750_active_addr >> 1);
}

// Ежесекундный таймер
void BH1750_OnEverySecond(void) {
    if (!g_bh1750_init_ok) {
        return;
    }

    g_bh1750_secondsElapsed++;
    if (g_bh1750_secondsElapsed >= g_bh1750_secondsBetweenMeasurements) {
        BH1750_Measure();
        g_bh1750_secondsElapsed = 0;
    }
}

#endif // PLATFORM_LN882H
