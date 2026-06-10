#if ENABLE_DRIVER_BH1750

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_bh1750.h"

static int g_bh1750_scl_pin = -1;
static int g_bh1750_sda_pin = -1;
static int g_bh1750_lux_channel = -1;

static int g_bh1750_secondsBetweenMeasurements = 1; // По умолчанию 1 сек (как в AHT2X)
static int g_bh1750_secondsElapsed = 0;
static bool g_bh1750_init_ok = false;

// Чтение уровня освещенности
void BH1750_Measure(void) {
    uint8_t buffer[2];
    uint16_t raw_lux;
    float lux;

    if (!g_bh1750_init_ok) {
        return;
    }

    // Читаем 2 байта результата через софтовый I2C, завязанный на пины из аргументов
    if (Soft_I2C_ReadBytes(g_bh1750_scl_pin, g_bh1750_sda_pin, BH1750_I2C_ADDR, buffer, 2) == false) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Read failed on SCL %d, SDA %d", g_bh1750_scl_pin, g_bh1750_sda_pin);
        return;
    }

    // Склеиваем байты (MSB << 8 | LSB)
    raw_lux = (buffer[0] << 8) | buffer[1];
    
    // Формула люксметра
    lux = (float)raw_lux / 1.2f;

    // Пишем в канал, переданный при старте драйвера
    CHANNEL_Set(g_bh1750_lux_channel, (int)lux, 0);
    addLogAdv(LOG_DEBUG, LOG_FEATURE_DRV, "BH1750: %d lux", (int)lux);
}

// Изменение интервала опроса (Эталон: AHT2X_Cycle)
commandResult_t BH1750_Cycle(const void *context, const char *cmd, const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_GetArgsCount() < 1) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750_Cycle requires 1 argument (seconds)");
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    g_bh1750_secondsBetweenMeasurements = Tokenizer_GetArgInteger(0);
    if (g_bh1750_secondsBetweenMeasurements < 1) {
        g_bh1750_secondsBetweenMeasurements = 1;
    }
    return CMD_RES_OK;
}

// Принудительный ручной замер (Эталон: AHT2X_Measure)
commandResult_t BH1750_ForceMeasure(const void *context, const char *cmd, const char *args, int cmdFlags) {
    BH1750_Measure();
    return CMD_RES_OK;
}

// Инициализация (Эталон: startDriver AHT2X 2 3 2 3)
void BH1750_Init(void) {
    uint8_t cmd;

    // Проверяем, переданы ли аргументы (нужно минимум 3: SCL, SDA, Channel)
    if (Tokenizer_GetArgsCount() < 3) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750 driver requires 3 args: [SCL] [SDA] [Lux_Channel]");
        g_bh1750_init_ok = false;
        return;
    }

    g_bh1750_scl_pin = Tokenizer_GetArgInteger(0);
    g_bh1750_sda_pin = Tokenizer_GetArgInteger(1);
    g_bh1750_lux_channel = Tokenizer_GetArgInteger(2);

    // Мягкая инициализация пинов в режим I2C
    Soft_I2C_PreInit(g_bh1750_scl_pin, g_bh1750_sda_pin);

    // 1. Команда Power ON
    cmd = BH1750_CMD_POWER_ON;
    if (Soft_I2C_WriteBytes(g_bh1750_scl_pin, g_bh1750_sda_pin, BH1750_I2C_ADDR, &cmd, 1) == false) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Power ON failed, check connections");
        g_bh1750_init_ok = false;
        return;
    }

    rtos_delay_milliseconds(10);

    // 2. Режим Continuous High Res Mode
    cmd = BH1750_CMD_CONTINUOUS;
    if (Soft_I2C_WriteBytes(g_bh1750_scl_pin, g_bh1750_sda_pin, BH1750_I2C_ADDR, &cmd, 1) == false) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Failed to set measurement mode");
        g_bh1750_init_ok = false;
        return;
    }

    // Регистрируем дополнительные рантайм-команды
    CMD_RegisterCommand("BH1750_Cycle", BH1750_Cycle, NULL);
    CMD_RegisterCommand("BH1750_Measure", BH1750_ForceMeasure, NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Started on SCL:%d SDA:%d TargetChannel:%d", 
              g_bh1750_scl_pin, g_bh1750_sda_pin, g_bh1750_lux_channel);
              
    g_bh1750_init_ok = true;
    g_bh1750_secondsElapsed = 0;
}

// Ежесекундный таймер ядра OpenBeken
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

#endif
