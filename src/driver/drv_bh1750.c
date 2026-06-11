#include "../obk_config.h"

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

// Чтение уровня освещенности
void BH1750_Measure(void) {
    uint8_t buffer[2];
    uint16_t raw_lux;
    float lux;

    if (!g_bh1750_init_ok) {
        return;
    }

    // Запускаем чтение 2 байт через структуру I2C
    Soft_I2C_Start(&g_bh1750_i2c);
    if (Soft_I2C_WriteByte(&g_bh1750_i2c, (BH1750_I2C_ADDR << 1) | 1) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Address NACK on read");
        return;
    }
    
    Soft_I2C_ReadBytes(&g_bh1750_i2c, buffer, 2);
    Soft_I2C_Stop(&g_bh1750_i2c);

    // Склеиваем байты
    raw_lux = (buffer[0] << 8) | buffer[1];
    
    // Формула люксметра
    lux = (float)raw_lux / 1.2f;

    // Пишем в канал
    CHANNEL_Set(g_bh1750_lux_channel, (int)lux, 0);
    addLogAdv(LOG_DEBUG, LOG_FEATURE_DRV, "BH1750: %d lux", (int)lux);
}

// Изменение интервала опроса
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

// Принудительный ручной замер
commandResult_t BH1750_ForceMeasure(const void *context, const char *cmd, const char *args, int cmdFlags) {
    BH1750_Measure();
    return CMD_RES_OK;
}

// Инициализация
void BH1750_Init(void) {
    uint8_t cmd;

    if (Tokenizer_GetArgsCount() < 3) {
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750 driver requires 3 args: [SCL] [SDA] [Lux_Channel]");
        g_bh1750_init_ok = false;
        return;
    }

    g_bh1750_i2c.pin_scl = Tokenizer_GetArgInteger(0);
    g_bh1750_i2c.pin_sda = Tokenizer_GetArgInteger(1);
    g_bh1750_lux_channel = Tokenizer_GetArgInteger(2);

    // Инициализация пинов I2C структуры
    Soft_I2C_PreInit(&g_bh1750_i2c);

    // 1. Команда Power ON
    Soft_I2C_Start(&g_bh1750_i2c);
    if (Soft_I2C_WriteByte(&g_bh1750_i2c, (BH1750_I2C_ADDR << 1)) == false ||
        Soft_I2C_WriteByte(&g_bh1750_i2c, BH1750_CMD_POWER_ON) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Power ON failed");
        g_bh1750_init_ok = false;
        return;
    }
    Soft_I2C_Stop(&g_bh1750_i2c);

    rtos_delay_milliseconds(10);

    // 2. Режим Continuous High Res Mode
    Soft_I2C_Start(&g_bh1750_i2c);
    if (Soft_I2C_WriteByte(&g_bh1750_i2c, (BH1750_I2C_ADDR << 1)) == false ||
        Soft_I2C_WriteByte(&g_bh1750_i2c, BH1750_CMD_CONTINUOUS) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Failed to set mode");
        g_bh1750_init_ok = false;
        return;
    }
    Soft_I2C_Stop(&g_bh1750_i2c);

    // Регистрируем команды
    CMD_RegisterCommand("BH1750_Cycle", BH1750_Cycle, NULL);
    CMD_RegisterCommand("BH1750_Measure", BH1750_ForceMeasure, NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "BH1750: Started on SCL:%d SDA:%d TargetChannel:%d", 
              g_bh1750_i2c.pin_scl, g_bh1750_i2c.pin_sda, g_bh1750_lux_channel);
              
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
