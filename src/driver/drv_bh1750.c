#include "../obk_config.h"

#if defined(PLATFORM_LN882H)

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_bh1750.h"

// Используем реальную общую шину из drv_main.c
extern softI2C_t g_i2c; 

static int g_bh1750_lux_channel = -1;
static int g_bh1750_secondsBetweenMeasurements = 1;
static int g_bh1750_secondsElapsed = 0;
static bool g_bh1750_init_ok = false;

// Чтение уровня освещенности через системную шину g_i2c
void BH1750_Measure(void) {
    uint8_t byte_high = 0;
    uint8_t byte_low = 0;
    uint16_t raw_lux = 0;
    float lux = 0.0f;
    uint8_t active_addr = 0;

    if (!g_bh1750_init_ok) {
        return;
    }

    // Запускаем чтение с адреса 0x23 через глобальную g_i2c
    if (Soft_I2C_Start(&g_i2c, 0x23) == true) {
        active_addr = 0x23;
    } else {
        Soft_I2C_Stop(&g_i2c);
        // Пробуем альтернативный адрес 0x5C
        if (Soft_I2C_Start(&g_i2c, 0x5C) == true) {
            active_addr = 0x5C;
        } else {
            Soft_I2C_Stop(&g_i2c);
            addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: System I2C NACK during measure.");
            return;
        }
    }
    
    // Читаем байты (0 - ACK, 1 - NACK)
    byte_high = Soft_I2C_ReadByte(&g_i2c, 0); 
    byte_low = Soft_I2C_ReadByte(&g_i2c, 1); 
    Soft_I2C_Stop(&g_i2c);

    // Склеиваем результат
    raw_lux = (byte_high << 8) | byte_low;
    lux = (float)raw_lux / 1.2f;

    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Success on 0x%02X! Calculated: %d lux", active_addr, (int)lux);

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

// Инициализация
void BH1750_Init(void) {
    // Извлекаем пины ядра стандартными функциями
    g_i2c.pin_clk = Tokenizer_GetPin(1, 9);
    g_i2c.pin_data = Tokenizer_GetPin(2, 14);
    g_bh1750_lux_channel = Tokenizer_GetArgIntegerDefault(3, -1);

    // Запуск общей шины I2C
    Soft_I2C_PreInit(&g_i2c);
    rtos_delay_milliseconds(100);

    // Включение датчика (Команда 0x01)
    if (Soft_I2C_Start(&g_i2c, 0x23) == true) {
        Soft_I2C_WriteByte(&g_i2c, 0x01);
        Soft_I2C_Stop(&g_i2c);
    }
    if (Soft_I2C_Start(&g_i2c, 0x5C) == true) {
        Soft_I2C_WriteByte(&g_i2c, 0x01);
        Soft_I2C_Stop(&g_i2c);
    }

    rtos_delay_milliseconds(10);

    // Запуск измерения (Команда 0x10)
    if (Soft_I2C_Start(&g_i2c, 0x23) == true) {
        Soft_I2C_WriteByte(&g_i2c, 0x10);
        Soft_I2C_Stop(&g_i2c);
    }
    if (Soft_I2C_Start(&g_i2c, 0x5C) == true) {
        Soft_I2C_WriteByte(&g_i2c, 0x10);
        Soft_I2C_Stop(&g_i2c);
    }

    CMD_RegisterCommand("BH1750_Cycle", BH1750_Cycle, NULL);
    CMD_RegisterCommand("BH1750_Measure", BH1750_ForceMeasure, NULL);
              
    g_bh1750_init_ok = true;
    g_bh1750_secondsElapsed = 0;
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
