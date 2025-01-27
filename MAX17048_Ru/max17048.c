/******************************************************************************
 * @file    MAX17048.c
 * @brief   Реализация функционала работы с топливомером MAX17048 для STM32
 *****************************************************************************/

#include "max17048.h"
#include <stdio.h>  // Для отладочных print, если нужны
#include <string.h> // Для memset, если нужно

/******************************************************************************
 * Вспомогательные структуры и переменные для асинхронного (неблокирующего) режима
 *****************************************************************************/

/**
 * @brief Перечисление возможных операций, которые могут выполняться асинхронно.
 */
typedef enum
{
    MAX17048_OP_NONE = 0,
    MAX17048_OP_GET_ALERT_STATUS,
    MAX17048_OP_GET_VOLTAGE,
    MAX17048_OP_GET_SOC,
    MAX17048_OP_GET_CRATE,
    MAX17048_OP_CLEAR_ALERT_STATUS
} MAX17048_Operation_t;

/**
 * @brief Глобальный контекст для отслеживания состояния асинхронных операций.
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;      ///< Указатель на I2C
    MAX17048_Operation_t op;      ///< Текущая операция
    MAX17048_Callback_t callback; ///< Указатель на пользовательский коллбэк
    void *pUserData;              ///< Пользовательские данные (результат и т.д.)

    uint8_t txBuf[3];      ///< Буфер для передачи (адрес + 2 байта)
    uint8_t rxBuf[2];      ///< Буфер для приёма (2 байта)
    uint16_t regValue;     ///< Промежуточное 16-битное значение
    uint8_t alertsToClear; ///< Маска флагов, которые нужно сбросить (для CLEAR_ALERT_STATUS)
} MAX17048_Context_t;

/** Глобальный (static) контекст асинхронного взаимодействия. */
static MAX17048_Context_t g_maxCtx = {0};

/*==============================================================================
 * Вспомогательные (static) функции для чтения/записи регистров
 *============================================================================*/

/**
 * @brief Считать 16-битный регистр из MAX17048 (адрес reg) в переменную data.
 *        Использует блокирующий вызов HAL_I2C_Mem_Read.
 * @param hi2c     Указатель на структуру I2C
 * @param reg      Адрес регистра (8-бит)
 * @param data     Указатель на 16-битную переменную для результата
 * @return HAL_OK при успешном чтении, иначе HAL_ERROR
 */
static HAL_StatusTypeDef MAX17048_ReadRegister(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg,
                                               uint16_t *data)
{
    uint8_t buffer[2];
    // Читаем 2 байта из устройства (MSB первым)
    if (HAL_I2C_Mem_Read(hi2c,
                         (MAX17048_I2C_ADDRESS << 1),
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         2,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Склеиваем 2 байта в одно 16-битное число
    *data = (buffer[0] << 8) | buffer[1];
    return HAL_OK;
}

/**
 * @brief Записать 16-битное значение data в регистр reg устройства MAX17048.
 * @param hi2c     Указатель на структуру I2C
 * @param reg      Адрес регистра (8-бит)
 * @param data     16-битное значение для записи
 * @return HAL_OK при успешной записи, иначе HAL_ERROR
 */
static HAL_StatusTypeDef MAX17048_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);   // Старший байт
    buffer[1] = (uint8_t)(data & 0xFF); // Младший байт

    if (HAL_I2C_Mem_Write(hi2c,
                          (MAX17048_I2C_ADDRESS << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          buffer,
                          2,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*==============================================================================
 * Базовые функции инициализации / чтения
 *============================================================================*/

/**
 * @brief Инициализация устройства путём чтения регистра VERSION.
 * @param hi2c Указатель на структуру I2C
 * @return HAL_OK, если устройство отвечает, иначе HAL_ERROR
 */
HAL_StatusTypeDef MAX17048_Init(I2C_HandleTypeDef *hi2c)
{
    uint16_t version;
    HAL_StatusTypeDef status;

    // Пробуем прочитать регистр VERSION, чтобы проверить связь
    status = MAX17048_ReadRegister(hi2c, MAX17048_REG_VERSION, &version);
    if (status != HAL_OK)
    {
        return status; // Нет ответа или ошибка I2C
    }

    return HAL_OK; // Устройство успешно инициализировано
}

/**
 * @brief Чтение версии устройства из регистра VERSION.
 * @param hi2c    Указатель на I2C
 * @param version Переменная, в которую будет записана версия (младший байт)
 * @return HAL_OK или HAL_ERROR
 */
HAL_StatusTypeDef MAX17048_GetVersion(I2C_HandleTypeDef *hi2c, uint8_t version)
{
    uint16_t raw_version;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VERSION, &raw_version) != HAL_OK)
    {
        return HAL_ERROR;
    }
    version = (uint8_t)raw_version;
    return HAL_OK;
}

/**
 * @brief Чтение ID устройства (из VRESET/ID регистра).
 * @param hi2c Указатель на I2C
 * @param id   Указатель на переменную (младший байт)
 * @return HAL_OK или HAL_ERROR
 */
HAL_StatusTypeDef MAX17048_GetID(I2C_HandleTypeDef *hi2c, uint8_t *id)
{
    uint16_t raw_id;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &raw_id) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *id = (raw_id & 0xFF);
    return HAL_OK;
}

/*==============================================================================
 * Функции: напряжение, SOC, ток (CRATE)
 *============================================================================*/

/**
 * @brief Чтение напряжения батареи (VCELL) в вольтах.
 * @param hi2c     Указатель на I2C
 * @param voltage  Указатель на float для записи результата
 * @return HAL_OK или HAL_ERROR
 */
HAL_StatusTypeDef MAX17048_GetVoltage(I2C_HandleTypeDef *hi2c, float *voltage)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VCELL, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // 1 LSB = 78.125 мкВ => raw_value * 78.125e-6 В
    *voltage = (float)raw_value * 78.125f / 1000000.0f;
    return HAL_OK;
}

/**
 * @brief Альтернативный способ расчёта напряжения (другая формула).
 */
HAL_StatusTypeDef MAX17048_GetVoltageAlternativ(I2C_HandleTypeDef *hi2c, float *voltage)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VCELL, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *voltage = (float)raw_value / 12800.0f; // Пример альтернативы
    return HAL_OK;
}

/**
 * @brief Чтение уровня заряда (SOC) из регистра SOC (формат 8.8).
 * @param hi2c Указатель на I2C
 * @param soc  Указатель на float (0..100%)
 */
HAL_StatusTypeDef MAX17048_GetSOC(I2C_HandleTypeDef *hi2c, float *soc)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_SOC, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // SOC в формате 8.8 => делим на 256
    *soc = (float)raw_value / 256.0f;
    return HAL_OK;
}

/**
 * @brief Чтение тока зарядки/разрядки (CRATE) в %/час.
 * @param crate Указатель на float
 */
HAL_StatusTypeDef MAX17048_GetCrate(I2C_HandleTypeDef *hi2c, float *crate)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CRATE, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // 1 LSB = 0.208 %/час
    *crate = (float)raw_value * 0.208f;
    return HAL_OK;
}

/**
 * @brief Команда QuickStart для сброса алгоритма и новой оценки SOC.
 */
HAL_StatusTypeDef MAX17048_QuickStart(I2C_HandleTypeDef *hi2c)
{
    uint16_t mode;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_MODE, &mode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Устанавливаем бит 14 для QuickStart
    mode |= (1 << 14);
    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_MODE, mode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*==============================================================================
 * Функции: Sleep / Reset / Температурная компенсация
 *============================================================================*/

/**
 * @brief Включить/выключить режим сна (SLEEP).
 * @param enable true - включить, false - отключить
 */
HAL_StatusTypeDef MAX17048_SetSleepMode(I2C_HandleTypeDef *hi2c, bool enable)
{
    uint16_t config;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CONFIG, &config) != HAL_OK)
    {
        return HAL_ERROR;
    }

    bool isInSleep = (config & (1 << 7)) != 0;
    if ((enable && isInSleep) || (!enable && !isInSleep))
    {
        // Уже в нужном состоянии
        return HAL_OK;
    }

    if (enable)
        config |= (1 << 7);
    else
        config &= ~(1 << 7);

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_CONFIG, config) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief Установка динамической температурной компенсации (RCOMP) на основе температуры.
 */
HAL_StatusTypeDef MAX17048_SetTempCompens(I2C_HandleTypeDef *hi2c, float temperature)
{
    uint8_t rcomp;
    // Пример нестрогой логики:
    if (temperature > 20.0f)
        rcomp = 0x97 + (temperature - 20.0f) * -0.5f;
    else
        rcomp = 0x97 + (temperature - 20.0f) * -5.0f;

    // Пишем rcomp в старший байт регистра CONFIG
    uint16_t config;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CONFIG, &config) != HAL_OK)
    {
        return HAL_ERROR;
    }
    config = (config & 0x00FF) | (rcomp << 8);
    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_CONFIG, config) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief Установить RCOMP напрямую (8-бит).
 */
HAL_StatusTypeDef MAX17048_SetRComp(I2C_HandleTypeDef *hi2c, uint8_t rcomp)
{
    uint16_t config_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_CONFIG, &config_reg);
    if (status == HAL_OK)
    {
        config_reg = (config_reg & 0x00FF) | (rcomp << 8);
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_CONFIG, config_reg);
    }
    return status;
}

/**
 * @brief Сброс (POR) устройства путём записи 0x5400 в регистр CMD (0xFE).
 */
HAL_StatusTypeDef MAX17048_Reset(I2C_HandleTypeDef *hi2c)
{
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_CMD, 0x5400);
}

/*==============================================================================
 * Функции: VRESET, ID, компаратор
 *============================================================================*/

/**
 * @brief Прочитать 7-битный порог сброса (VRESET) из регистра VRESET/ID.
 */
HAL_StatusTypeDef MAX17048_GetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t *ResetVoltage)
{
    uint16_t raw_ResetVoltage;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &raw_ResetVoltage) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *ResetVoltage = (uint8_t)(raw_ResetVoltage >> 9); // 7 бит, 1 бит = 40 мВ
    return HAL_OK;
}

/**
 * @brief Установить 7-битный порог сброса (0..127). 1 бит = 40 мВ, по умолчанию 75 -> 3.0 В
 */
HAL_StatusTypeDef MAX17048_SetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    uint16_t vreset;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &vreset) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Очищаем старые биты и ставим новые (threshold)
    vreset &= 0x01FF;
    vreset |= ((uint16_t)threshold << 9);

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_VRESET_ID, vreset) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Установить порог сброса в виде float напряжения (0..5.08 В).
 *        Внутри преобразуем в 7 бит, 40 мВ на бит.
 */
HAL_StatusTypeDef MAX17048_SetResetVoltageFloat(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0.0f || threshold > 5.08f)
    {
        return HAL_ERROR;
    }
    uint8_t thresh = (uint8_t)(threshold / 0.04f);
    return MAX17048_SetResetVoltage(hi2c, thresh);
}

/**
 * @brief Включить/выключить компаратор для экономии ~0.5 мкА.
 */
HAL_StatusTypeDef MAX17048_SetComparator(I2C_HandleTypeDef *hi2c, bool enable)
{
    uint16_t vresetReg;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &vresetReg) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (enable)
        vresetReg &= ~(1 << 8);
    else
        vresetReg |= (1 << 8);

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_VRESET_ID, vresetReg) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*==============================================================================
 * Функции оповещения (Alert): VALRT, STATUS
 *============================================================================*/

HAL_StatusTypeDef MAX17048_GetAlertStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
    uint16_t regValue;
    HAL_StatusTypeDef result = MAX17048_ReadRegister(hi2c, MAX17048_REG_STATUS, &regValue);
    if (result != HAL_OK)
    {
        return result;
    }
    // Старший байт содержит флаги
    uint8_t statusReg = (uint8_t)(regValue >> 8);

    *status = statusReg & (MAX17048_ALERT_RI | MAX17048_ALERT_VH |
                           MAX17048_ALERT_VL | MAX17048_ALERT_VR |
                           MAX17048_ALERT_HD | MAX17048_ALERT_SC);
    return HAL_OK;
}

/**
 * @brief Сброс (очистка) заданных флагов ALERT в регистре STATUS.
 */
HAL_StatusTypeDef MAX17048_ClearAlertFlags(I2C_HandleTypeDef *hi2c, uint8_t flags)
{
    uint16_t status_reg;
    HAL_StatusTypeDef result = MAX17048_ReadRegister(hi2c, MAX17048_REG_STATUS, &status_reg);
    if (result == HAL_OK)
    {
        status_reg &= ~((flags & 0x3F) << 8);
        result = MAX17048_WriteRegister(hi2c, MAX17048_REG_STATUS, status_reg);
    }
    return result;
}

/**
 * @brief Очистка всех флагов алертов (запись 0x0000 в STATUS).
 */
HAL_StatusTypeDef MAX17048_ClearAllAlertStatus(I2C_HandleTypeDef *hi2c)
{
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_STATUS, 0x0000);
}

/**
 * @brief Очистить бит ALERT в регистре CONFIG.
 */
HAL_StatusTypeDef MAX17048_ClearAlert(I2C_HandleTypeDef *hi2c)
{
    uint16_t configReg;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CONFIG, &configReg) != HAL_OK)
    {
        return HAL_ERROR;
    }
    configReg &= ~MAX17048_ALERT_SC;
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_CONFIG, configReg);
}

HAL_StatusTypeDef MAX17048_GetAlert(I2C_HandleTypeDef *hi2c,
                                    bool clear,
                                    uint8_t *alertStatus)
{
    uint16_t configReg;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CONFIG, &configReg) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *alertStatus = (configReg & MAX17048_ALERT_SC) ? 1 : 0;

    if (*alertStatus && clear)
    {
        configReg &= ~MAX17048_ALERT_SC;
        if (MAX17048_WriteRegister(hi2c, MAX17048_REG_CONFIG, configReg) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief Включить/выключить SOC Change Alert (1% изменение).
 */
HAL_StatusTypeDef MAX17048_EnableSOCAlert(I2C_HandleTypeDef *hi2c, bool enable)
{
    uint16_t configReg;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CONFIG, &configReg) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (enable)
        configReg |= MAX17048_CONFIG_ALSC;
    else
        configReg &= ~MAX17048_CONFIG_ALSC;

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_CONFIG, configReg) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief Включить/выключить Voltage Reset Alert (EnVR) в регистре STATUS.
 */
HAL_StatusTypeDef MAX17048_EnableVoltageAlert(I2C_HandleTypeDef *hi2c, bool enable)
{
    uint16_t statusReg;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_STATUS, &statusReg) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (enable)
        statusReg |= MAX17048_STATUS_EnVR;
    else
        statusReg &= ~MAX17048_STATUS_EnVR;

    return MAX17048_WriteRegister(hi2c, MAX17048_REG_STATUS, statusReg);
}

/**
 * @brief Установить минимальный порог напряжения предупреждения (VALRT.MIN).
 */
HAL_StatusTypeDef MAX17048_SetVAlertMinThreshold(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0 || threshold > 5.1f)
        return HAL_ERROR;

    uint16_t valrt_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_VALRT, &valrt_reg);
    if (status == HAL_OK)
    {
        // 1 LSB = 20 мВ => threshold / 0.02f
        uint8_t min_th = (uint8_t)(threshold / 0.02f) & 0xFF;
        valrt_reg = (valrt_reg & 0x00FF) | (min_th << 8);
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_VALRT, valrt_reg);
    }
    return status;
}

/**
 * @brief Установить максимальный порог напряжения предупреждения (VALRT.MAX).
 */
HAL_StatusTypeDef MAX17048_SetVAlertMaxThreshold(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0 || threshold > 5.1f)
        return HAL_ERROR;

    uint16_t valrt_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_VALRT, &valrt_reg);
    if (status == HAL_OK)
    {
        uint8_t max_th = (uint8_t)(threshold / 0.02f) & 0xFF;
        valrt_reg = (valrt_reg & 0xFF00) | max_th;
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_VALRT, valrt_reg);
    }
    return status;
}

/**
 * @brief Установка максимального порога (float), аналогично предыдущим.
 */
HAL_StatusTypeDef MAX17048_SetAlertVoltageMax(I2C_HandleTypeDef *hi2c, float voltage)
{
    uint16_t valrt;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VALRT, &valrt) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (voltage > 0.0f)
    {
        if (voltage < 5.1f)
            valrt = (valrt & 0xFF00) | (uint16_t)(voltage / 0.02f);
        else
            valrt = (valrt & 0xFF00) | 0x00FF;
    }

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_VALRT, valrt) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Установка минимального порога (float).
 */
HAL_StatusTypeDef MAX17048_SetAlertVoltageMin(I2C_HandleTypeDef *hi2c, float voltage)
{
    uint16_t valrt;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VALRT, &valrt) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (voltage > 0.0f)
    {
        if (voltage < 5.1f)
            valrt = (valrt & 0x00FF) | ((uint16_t)(voltage / 0.02f) << 8);
        else
            valrt = (valrt & 0x00FF) | 0xFF00;
    }

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_VALRT, valrt) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*==============================================================================
 * Функции гибернации (Hibernate)
 *============================================================================*/

HAL_StatusTypeDef MAX17048_SetHibernation(I2C_HandleTypeDef *hi2c, bool enable)
{
    uint16_t mode_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_MODE, &mode_reg);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    bool isHibernating = (mode_reg & (1 << 12)) != 0;
    if ((enable && isHibernating) || (!enable && !isHibernating))
    {
        return HAL_OK; // Уже в нужном состоянии
    }

    if (enable)
        mode_reg |= (1 << 12);
    else
        mode_reg &= ~(1 << 12);

    status = MAX17048_WriteRegister(hi2c, MAX17048_REG_MODE, mode_reg);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief Проверить, находится ли устройство в режиме гибернации (12-й бит MODE).
 */
HAL_StatusTypeDef MAX17048_IsHibernating(I2C_HandleTypeDef *hi2c, bool *hibernate)
{
    uint16_t mode_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_MODE, &mode_reg);
    if (status == HAL_OK)
    {
        *hibernate = (mode_reg & (1 << 12)) != 0;
    }
    return status;
}

/**
 * @brief Установить порог активности (ActThr) для выхода из гибернации, float (0..0.31875).
 *        1 LSB = 1.25 мВ
 */
HAL_StatusTypeDef MAX17048_SetHibernateActTh(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0 || threshold > 0.31875f)
    {
        return HAL_ERROR;
    }

    uint16_t hib_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_HIBRT, &hib_reg);
    if (status == HAL_OK)
    {
        uint8_t act_th = (uint8_t)(threshold / 0.00125f) & 0xFF;
        hib_reg = (hib_reg & 0xFF00) | act_th;
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, hib_reg);
    }
    return status;
}

/**
 * @brief Установить порог гибернации (HibThr) (0..53.04 В, условно).
 *        1 LSB = 0.208 %/ч
 */
HAL_StatusTypeDef MAX17048_SetHibernateHibTh(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0 || threshold > 53.04f)
    {
        return HAL_ERROR;
    }

    uint16_t hib_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_HIBRT, &hib_reg);
    if (status == HAL_OK)
    {
        uint8_t hib_th = (uint8_t)(threshold / 0.208f) & 0xFF;
        hib_reg = (hib_reg & 0x00FF) | (hib_th << 8);
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, hib_reg);
    }
    return status;
}

/**
 * @brief Чтение порога активности (ActThr) в "сырых" битах.
 */
HAL_StatusTypeDef MAX17048_GetHibernateActThreshold(I2C_HandleTypeDef *hi2c, uint8_t *threshold)
{
    uint16_t hib_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_HIBRT, &hib_reg);
    if (status == HAL_OK)
    {
        *threshold = (uint8_t)(hib_reg & 0xFF);
    }
    return status;
}

/**
 * @brief Установка порога активности (ActThr) в битах (0..255).
 */
HAL_StatusTypeDef MAX17048_SetHibernateActThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    uint16_t hib_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_HIBRT, &hib_reg);
    if (status == HAL_OK)
    {
        hib_reg &= 0xFF00;
        hib_reg |= threshold;
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, hib_reg);
    }
    return status;
}

/**
 * @brief Чтение порога гибернации (HibThr) в "сырых" битах (0..255).
 */
HAL_StatusTypeDef MAX17048_GetHibernateThreshold(I2C_HandleTypeDef *hi2c, uint8_t *threshold)
{
    uint16_t hib_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_HIBRT, &hib_reg);
    if (status == HAL_OK)
    {
        *threshold = (uint8_t)(hib_reg >> 8);
    }
    return status;
}

/**
 * @brief Установка порога гибернации (HibThr) в битах.
 */
HAL_StatusTypeDef MAX17048_SetHibernateThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    uint16_t hib_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_HIBRT, &hib_reg);
    if (status == HAL_OK)
    {
        hib_reg &= 0x00FF;
        hib_reg |= ((uint16_t)threshold << 8);
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, hib_reg);
    }
    return status;
}

/**
 * @brief Полное включение режима гибернации (0xFFFF в HIBRT).
 */
HAL_StatusTypeDef MAX17048_EnableHibernate(I2C_HandleTypeDef *hi2c)
{
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, MAX17048_HIBRT_ENHIB);
}

/**
 * @brief Полное отключение гибернации (0x0000 в HIBRT).
 */
HAL_StatusTypeDef MAX17048_DisableHibernate(I2C_HandleTypeDef *hi2c)
{
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, MAX17048_HIBRT_DISHIB);
}

/*==============================================================================
 * Асинхронные (неблокирующие) функции
 *============================================================================*/

/**
 * @brief Асинхронное чтение напряжения (пример). Возвращает HAL_OK, если операция начата.
 */
HAL_StatusTypeDef MAX17048_GetVoltage_Async(I2C_HandleTypeDef *hi2c,
                                            float *pVoltage,
                                            MAX17048_Callback_t callback,
                                            void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c = hi2c;
    g_maxCtx.op = MAX17048_OP_GET_VOLTAGE;
    g_maxCtx.callback = callback;
    g_maxCtx.pUserData = pVoltage;

    uint8_t regAddr = MAX17048_REG_VCELL;
    if (HAL_I2C_Master_Transmit_IT(hi2c,
                                   (MAX17048_I2C_ADDRESS << 1),
                                   &regAddr,
                                   1) != HAL_OK)
    {
        g_maxCtx.op = MAX17048_OP_NONE;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetSOC_Async(I2C_HandleTypeDef *hi2c,
                                        float *pSoc,
                                        MAX17048_Callback_t callback,
                                        void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c = hi2c;
    g_maxCtx.op = MAX17048_OP_GET_SOC;
    g_maxCtx.callback = callback;
    g_maxCtx.pUserData = pSoc;

    uint8_t regAddr = MAX17048_REG_SOC;
    if (HAL_I2C_Master_Transmit_IT(hi2c,
                                   (MAX17048_I2C_ADDRESS << 1),
                                   &regAddr,
                                   1) != HAL_OK)
    {
        g_maxCtx.op = MAX17048_OP_NONE;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetCrate_Async(I2C_HandleTypeDef *hi2c,
                                          float *pCrate,
                                          MAX17048_Callback_t callback,
                                          void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c = hi2c;
    g_maxCtx.op = MAX17048_OP_GET_CRATE;
    g_maxCtx.callback = callback;
    g_maxCtx.pUserData = pCrate;

    uint8_t regAddr = MAX17048_REG_CRATE;
    if (HAL_I2C_Master_Transmit_IT(hi2c,
                                   (MAX17048_I2C_ADDRESS << 1),
                                   &regAddr,
                                   1) != HAL_OK)
    {
        g_maxCtx.op = MAX17048_OP_NONE;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                uint8_t *pAlerts,
                                                MAX17048_Callback_t callback,
                                                void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c = hi2c;
    g_maxCtx.op = MAX17048_OP_GET_ALERT_STATUS;
    g_maxCtx.callback = callback;
    g_maxCtx.pUserData = pAlerts;

    uint8_t regAddr = MAX17048_REG_STATUS;
    if (HAL_I2C_Master_Transmit_IT(hi2c,
                                   (MAX17048_I2C_ADDRESS << 1),
                                   &regAddr,
                                   1) != HAL_OK)
    {
        g_maxCtx.op = MAX17048_OP_NONE;
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief Асинхронная очистка указанных флагов ALERT.
 *        Сначала читаем регистр STATUS, затем по коллбеку записываем обратно.
 */
HAL_StatusTypeDef MAX17048_ClearAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                  uint8_t alertsToClear,
                                                  MAX17048_Callback_t callback,
                                                  void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c = hi2c;
    g_maxCtx.op = MAX17048_OP_CLEAR_ALERT_STATUS;
    g_maxCtx.callback = callback;
    g_maxCtx.pUserData = pUserData;
    g_maxCtx.alertsToClear = alertsToClear;

    // Шаг 1: передаём адрес регистра STATUS
    uint8_t regAddr = MAX17048_REG_STATUS;
    if (HAL_I2C_Master_Transmit_IT(hi2c,
                                   (MAX17048_I2C_ADDRESS << 1),
                                   &regAddr,
                                   1) != HAL_OK)
    {
        g_maxCtx.op = MAX17048_OP_NONE;
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*==============================================================================
 * Обработчики прерываний I2C (Master Tx, Master Rx, Error)
 *============================================================================*/

/**
 * @brief Коллбек по завершении передачи (Tx Complete).
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != g_maxCtx.hi2c)
        return; // Проверка, что это наш hi2c

    switch (g_maxCtx.op)
    {
    case MAX17048_OP_GET_ALERT_STATUS:
    case MAX17048_OP_GET_VOLTAGE:
    case MAX17048_OP_GET_SOC:
    case MAX17048_OP_GET_CRATE:
    {
        // Теперь нужно начать чтение 2 байтов
        if (HAL_I2C_Master_Receive_IT(hi2c,
                                      (MAX17048_I2C_ADDRESS << 1),
                                      g_maxCtx.rxBuf,
                                      2) != HAL_OK)
        {
            // Если ошибка, сразу вызываем пользовательский коллбек
            if (g_maxCtx.callback)
                g_maxCtx.callback(HAL_ERROR, g_maxCtx.pUserData);

            g_maxCtx.op = MAX17048_OP_NONE;
        }
        break;
    }

    case MAX17048_OP_CLEAR_ALERT_STATUS:
    {
        // Допустим, после передачи адреса STATUS, мы завершаем вторую стадию
        // (можно быть иная логика, если нужно чтение+запись).
        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_OK, g_maxCtx.pUserData);
        g_maxCtx.op = MAX17048_OP_NONE;
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Коллбек по завершении приёма (Rx Complete).
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != g_maxCtx.hi2c)
        return;

    // Читаем 2 байта, складываем в regValue
    g_maxCtx.regValue = ((uint16_t)g_maxCtx.rxBuf[0] << 8) | g_maxCtx.rxBuf[1];

    switch (g_maxCtx.op)
    {
    case MAX17048_OP_GET_ALERT_STATUS:
    {
        uint8_t statusReg = (uint8_t)(g_maxCtx.regValue >> 8);
        uint8_t *alertsOut = (uint8_t *)g_maxCtx.pUserData;
        *alertsOut = statusReg & (MAX17048_ALERT_RI | MAX17048_ALERT_VH |
                                  MAX17048_ALERT_VL | MAX17048_ALERT_VR |
                                  MAX17048_ALERT_HD | MAX17048_ALERT_SC);

        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_OK, g_maxCtx.pUserData);

        g_maxCtx.op = MAX17048_OP_NONE;
        break;
    }

    case MAX17048_OP_GET_VOLTAGE:
    {
        float *pVoltage = (float *)g_maxCtx.pUserData;
        *pVoltage = (float)g_maxCtx.regValue * 78.125f / 1e6f;

        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_OK, g_maxCtx.pUserData);

        g_maxCtx.op = MAX17048_OP_NONE;
        break;
    }

    case MAX17048_OP_GET_SOC:
    {
        float *pSoc = (float *)g_maxCtx.pUserData;
        *pSoc = (float)g_maxCtx.regValue / 256.0f;

        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_OK, g_maxCtx.pUserData);

        g_maxCtx.op = MAX17048_OP_NONE;
        break;
    }

    case MAX17048_OP_GET_CRATE:
    {
        float *pCrate = (float *)g_maxCtx.pUserData;
        *pCrate = (float)g_maxCtx.regValue * 0.208f;

        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_OK, g_maxCtx.pUserData);

        g_maxCtx.op = MAX17048_OP_NONE;
        break;
    }

    case MAX17048_OP_CLEAR_ALERT_STATUS:
    {
        // Прочитали STATUS, теперь модифицируем биты и записываем
        uint16_t updated = g_maxCtx.regValue & ~((uint16_t)g_maxCtx.alertsToClear << 8);

        g_maxCtx.txBuf[0] = MAX17048_REG_STATUS;
        g_maxCtx.txBuf[1] = (uint8_t)(updated >> 8);
        g_maxCtx.txBuf[2] = (uint8_t)(updated & 0xFF);

        // Запускаем передачу
        if (HAL_I2C_Master_Transmit_IT(hi2c,
                                       (MAX17048_I2C_ADDRESS << 1),
                                       g_maxCtx.txBuf,
                                       3) != HAL_OK)
        {
            if (g_maxCtx.callback)
                g_maxCtx.callback(HAL_ERROR, g_maxCtx.pUserData);
            g_maxCtx.op = MAX17048_OP_NONE;
        }
        // Продолжение логики будет в HAL_I2C_MasterTxCpltCallback
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Коллбек, вызываемый при ошибке I2C (ErrorCallback).
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == g_maxCtx.hi2c)
    {
        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_ERROR, g_maxCtx.pUserData);

        g_maxCtx.op = MAX17048_OP_NONE;
    }
}
