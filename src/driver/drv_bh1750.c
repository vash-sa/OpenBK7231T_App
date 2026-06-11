#include "../obk_config.h"

#if defined(PLATFORM_LN882H)

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_bh1750.h"

static softI2C_t g_bh1750_i2c;
static int g_bh1750_lux_channel = -1;

static int g_bh1750_secondsBetweenMeasurements = 1;
static int g_bh1750_secondsElapsed = 0;
static bool g_bh1750_init_ok = false;

// Чтение уровня освещенности с логами
void BH1750_Measure(void) {
    uint8_t byte_high;
    uint8_t byte_low;
    uint16_t raw_lux;
    float lux;
    uint8_t active_addr = 0;

    if (!g_bh1750_init_ok) {
        return;
    }

    // Пробуем адрес 0x23
    if (Soft_I2C_Start(&g_bh1750_i2c, (0x23 << 1) | 1) == true) {
        active_addr = 0x23;
    } else {
        Soft_I2C_Stop(&g_bh1750_i2c);
        // Если 0x23 молчит, пробуем адрес 0x5C
        if (Soft_I2C_Start(&g_bh1750_i2c, (0x5C << 1) | 1) == true) {
            active_addr = 0x5C;
        } else {
            Soft_I2C_Stop(&g_bh1750_i2c);
            addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: I2C NACK on both 0x23 and 0x5C. Check lines SCL/SDA");
            return;
        }
    }
    
    // Читаем байты
    byte_high = Soft_I2C_ReadByte(&g_bh1750_i2c, 0); 
    byte_low = Soft_I2C_ReadByte(&g_bh1750_i2c, 1); 
    Soft_I2C_Stop(&g_bh1750_i2c);

    // Склеиваем результат
    raw_lux = (byte_high << 8) | byte_low;
    lux = (float)raw_lux / 1.2f;

    // Выводим отладочную информацию в лог ядра
    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Responded on 0x%02X. Raw high: 0x%02X, low: 0x%02X. Calculated: %d lux", 
              active_addr, byte_high, byte_low, (int)lux);

    // Пишем в веб-интерфейс
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
    if (Tokenizer_GetArgsCount() < 3) {
        g_bh1750_init_ok = false;
        return;
    }

    g_bh1750_i2c.pin_clk = Tokenizer_GetArgInteger(0);
    g_bh1750_i2c.pin_data = Tokenizer_GetArgInteger(1);
    g_bh1750_lux_channel = Tokenizer_GetArgInteger(2);

    Soft_I2C_PreInit(&g_bh1750_i2c);

    // Прогрев датчика: шлем Power On на оба возможных адреса
    Soft_I2C_Start(&g_bh1750_i2c, (0x23 << 1));
    Soft_I2C_WriteByte(&g_bh1750_i2c, 0x01);
    Soft_I2C_Stop(&g_bh1750_i2c);

    Soft_I2C_Start(&g_bh1750_i2c, (0x5C << 1));
    Soft_I2C_WriteByte(&g_bh1750_i2c, 0x01);
    Soft_I2C_Stop(&g_bh1750_i2c);

    rtos_delay_milliseconds(10);

    // Задаем рабочий режим высокого разрешения на оба адреса
    Soft_I2C_Start(&g_bh1750_i2c, (0x23 << 1));
    Soft_I2C_WriteByte(&g_bh1750_i2c, 0x10);
    Soft_I2C_Stop(&g_bh1750_i2c);

    Soft_I2C_Start(&g_bh1750_i2c, (0x5C << 1));
    Soft_I2C_WriteByte(&g_bh1750_i2c, 0x10);
    Soft_I2C_Stop(&g_bh1750_i2c);

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
