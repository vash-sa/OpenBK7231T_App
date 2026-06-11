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

// Чтение уровня освещенности (Аналог lightMeter.readLightLevel())
void BH1750_Measure(void) {
    uint8_t buffer[2]; // СТРОГО массив из 2 байт, чтобы функция заполнила его корректно
    uint16_t raw_lux;
    float lux;

    if (!g_bh1750_init_ok) {
        return;
    }

    // Старт чтения с адреса 0x23 (в сдвиге с битом чтения дает 0x47)
    if (Soft_I2C_Start(&g_bh1750_i2c, (0x23 << 1) | 1) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        return;
    }
    
    // Считываем 2 байта ответа датчика в массив
    Soft_I2C_ReadBytes(&g_bh1750_i2c, buffer, 2);
    Soft_I2C_Stop(&g_bh1750_i2c);

    // Склеиваем MSB и LSB байты результатов
    raw_lux = (buffer[0] << 8) | buffer[1];
    
    // Делим на 1.2 по даташиту BH1750
    lux = (float)raw_lux / 1.2f;

    // Отправляем результат в веб-интерфейс
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

// Инициализация драйвера (Аналог lightMeter.begin)
void BH1750_Init(void) {
    if (Tokenizer_GetArgsCount() < 3) {
        g_bh1750_init_ok = false;
        return;
    }

    // Назначаем пины I2C
    g_bh1750_i2c.pin_clk = Tokenizer_GetArgInteger(0);
    g_bh1750_i2c.pin_data = Tokenizer_GetArgInteger(1);
    g_bh1750_lux_channel = Tokenizer_GetArgInteger(2);

    Soft_I2C_PreInit(&g_bh1750_i2c);

    // Посылаем команду Power On (0x01) на адрес 0x23
    if (Soft_I2C_Start(&g_bh1750_i2c, (0x23 << 1)) == false ||
        Soft_I2C_WriteByte(&g_bh1750_i2c, 0x01) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        g_bh1750_init_ok = false;
        return;
    }
    Soft_I2C_Stop(&g_bh1750_i2c);

    rtos_delay_milliseconds(10);

    // Посылаем команду CONTINUOUS_HIGH_RES_MODE (0x10) на адрес 0x23
    if (Soft_I2C_Start(&g_bh1750_i2c, (0x23 << 1)) == false ||
        Soft_I2C_WriteByte(&g_bh1750_i2c, 0x10) == false) {
        Soft_I2C_Stop(&g_bh1750_i2c);
        g_bh1750_init_ok = false;
        return;
    }
    Soft_I2C_Stop(&g_bh1750_i2c);

    // Регистрация команд консоли OpenBeken
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
