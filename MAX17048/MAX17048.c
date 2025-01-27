/******************************************************************************
 * @file    max17048.c
 * @brief   Implementation of MAX17048 fuel-gauge functions for STM32
 *****************************************************************************/

#include "max17048.h"
#include <stdio.h>   // For debug prints if needed
#include <string.h>  // For memset, optional

/******************************************************************************
 * Global context for asynchronous mode (if needed)
 *****************************************************************************/
typedef enum
{
    MAX17048_OP_NONE = 0,
    MAX17048_OP_GET_ALERT_STATUS,
    MAX17048_OP_GET_VOLTAGE,
    MAX17048_OP_GET_SOC,
    MAX17048_OP_GET_CRATE,
    MAX17048_OP_CLEAR_ALERT_STATUS
} MAX17048_Operation_t;

/** Internal context structure for asynchronous operations. */
typedef struct
{
    I2C_HandleTypeDef    *hi2c;       ///< I2C handle
    MAX17048_Operation_t  op;         ///< Current operation
    MAX17048_Callback_t   callback;   ///< Callback function
    void                 *pUserData;  ///< User-provided data pointer

    uint8_t  txBuf[3];               ///< TX buffer
    uint8_t  rxBuf[2];               ///< RX buffer
    uint16_t regValue;               ///< Temporary storage for 16-bit register
    uint8_t  alertsToClear;          ///< Used in "clear alert" operation
} MAX17048_Context_t;

/** Global static context (for demonstration). */
static MAX17048_Context_t g_maxCtx = {0};

/*==============================================================================
 * Internal Helper Functions (static)
 *============================================================================*/

/**
 * @brief Reads a 16-bit register from MAX17048 (address 'reg') into 'data'.
 *        This is a blocking call using I2C_Mem_Read.
 */
static HAL_StatusTypeDef MAX17048_ReadRegister(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg,
                                               uint16_t *data)
{
    uint8_t buffer[2];
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

    *data = (buffer[0] << 8) | buffer[1]; // Combine two bytes
    return HAL_OK;
}

/**
 * @brief Writes a 16-bit value 'data' to register 'reg'.
 */
static HAL_StatusTypeDef MAX17048_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFF);

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
 * Basic Functions
 *============================================================================*/

HAL_StatusTypeDef MAX17048_Init(I2C_HandleTypeDef *hi2c)
{
    uint16_t version;
    HAL_StatusTypeDef status;

    // Read the VERSION register to confirm communication
    status = MAX17048_ReadRegister(hi2c, MAX17048_REG_VERSION, &version);
    if (status != HAL_OK)
    {
        return status; // Device not responding or I2C error
    }

    return HAL_OK; // Device initialized successfully
}

HAL_StatusTypeDef MAX17048_GetVersion(I2C_HandleTypeDef *hi2c, uint8_t version)
{
    uint16_t raw_version;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VERSION, &raw_version) != HAL_OK)
    {
        return HAL_ERROR;
    }
    version = (uint8_t)raw_version; // Lower byte holds the version
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetID(I2C_HandleTypeDef *hi2c, uint8_t *id)
{
    uint16_t raw_id;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &raw_id) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *id = (raw_id & 0xFF); // Lower byte is the ID
    return HAL_OK;
}

/*==============================================================================
 * Voltage, SOC, CRATE
 *============================================================================*/

HAL_StatusTypeDef MAX17048_GetVoltage(I2C_HandleTypeDef *hi2c, float *voltage)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VCELL, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 1 LSB = 78.125 µV, so value * 78.125e-6
    *voltage = (float)raw_value * 78.125f / 1000000.0f;
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetVoltageAlternativ(I2C_HandleTypeDef *hi2c, float *voltage)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VCELL, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Alternative formula for demonstration
    *voltage = (float)raw_value / 12800.0f;
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetSOC(I2C_HandleTypeDef *hi2c, float *soc)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_SOC, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // The register is 8.8 format
    *soc = (float)raw_value / 256.0f;
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetCrate(I2C_HandleTypeDef *hi2c, float *crate)
{
    uint16_t raw_value;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_CRATE, &raw_value) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // 1 LSB = 0.208 %/hour
    *crate = (float)raw_value * 0.208f;
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_QuickStart(I2C_HandleTypeDef *hi2c)
{
    uint16_t mode;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_MODE, &mode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Bit 14 => Quick Start
    mode |= (1 << 14);
    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_MODE, mode) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*==============================================================================
 * Sleep / Reset / Temperature Compensation
 *============================================================================*/

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
        // No change needed
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

HAL_StatusTypeDef MAX17048_SetTempCompens(I2C_HandleTypeDef *hi2c, float temperature)
{
    // Example approach for dynamic RCOMP
    uint8_t rcomp;
    if (temperature > 20.0f)
        rcomp = 0x97 + (temperature - 20.0f) * -0.5f;
    else
        rcomp = 0x97 + (temperature - 20.0f) * -5.0f;

    // Now write this RCOMP to the upper byte of CONFIG
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

HAL_StatusTypeDef MAX17048_Reset(I2C_HandleTypeDef *hi2c)
{
    // Write 0x5400 to CMD (0xFE) for a software POR
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_CMD, 0x5400);
}

/*==============================================================================
 * VRESET, ID, Comparator
 *============================================================================*/

HAL_StatusTypeDef MAX17048_GetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t *ResetVoltage)
{
    uint16_t raw_ResetVoltage;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &raw_ResetVoltage) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *ResetVoltage = (uint8_t)(raw_ResetVoltage >> 9);
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_SetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t threshold)
{
    uint16_t vreset;
    if (MAX17048_ReadRegister(hi2c, MAX17048_REG_VRESET_ID, &vreset) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Clear old bits, set new 7 bits (threshold)
    vreset &= 0x01FF;
    vreset |= ((uint16_t)threshold << 9);

    if (MAX17048_WriteRegister(hi2c, MAX17048_REG_VRESET_ID, vreset) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_SetResetVoltageFloat(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0.0f || threshold > 5.08f)
    {
        return HAL_ERROR;
    }
    uint8_t thresh = (uint8_t)(threshold / 0.04f);
    return MAX17048_SetResetVoltage(hi2c, thresh);
}

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
 * Alert (VALRT, STATUS) Functions
 *============================================================================*/

HAL_StatusTypeDef MAX17048_GetAlertStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
    uint16_t regValue;
    HAL_StatusTypeDef result = MAX17048_ReadRegister(hi2c, MAX17048_REG_STATUS, &regValue);
    if (result != HAL_OK)
    {
        return result;
    }
    uint8_t statusReg = (uint8_t)(regValue >> 8);

    *status = statusReg & (MAX17048_ALERT_RI | MAX17048_ALERT_VH |
                           MAX17048_ALERT_VL | MAX17048_ALERT_VR |
                           MAX17048_ALERT_HD | MAX17048_ALERT_SC);
    return HAL_OK;
}

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

HAL_StatusTypeDef MAX17048_ClearAllAlertStatus(I2C_HandleTypeDef *hi2c)
{
    // Write 0x0000 to STATUS (clears all alerts)
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_STATUS, 0x0000);
}

HAL_StatusTypeDef MAX17048_ClearAlert(I2C_HandleTypeDef *hi2c)
{
    // Clears the ALERT bit in CONFIG
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

HAL_StatusTypeDef MAX17048_SetVAlertMinThreshold(I2C_HandleTypeDef *hi2c, float threshold)
{
    if (threshold < 0 || threshold > 5.1f)
        return HAL_ERROR;

    uint16_t valrt_reg;
    HAL_StatusTypeDef status = MAX17048_ReadRegister(hi2c, MAX17048_REG_VALRT, &valrt_reg);
    if (status == HAL_OK)
    {
        uint8_t min_th = (uint8_t)(threshold / 0.02f) & 0xFF;
        valrt_reg = (valrt_reg & 0x00FF) | (min_th << 8);
        status = MAX17048_WriteRegister(hi2c, MAX17048_REG_VALRT, valrt_reg);
    }
    return status;
}

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
 * Hibernation Functions
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
        return HAL_OK; // Already in requested state
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

HAL_StatusTypeDef MAX17048_EnableHibernate(I2C_HandleTypeDef *hi2c)
{
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, MAX17048_HIBRT_ENHIB);
}

HAL_StatusTypeDef MAX17048_DisableHibernate(I2C_HandleTypeDef *hi2c)
{
    return MAX17048_WriteRegister(hi2c, MAX17048_REG_HIBRT, MAX17048_HIBRT_DISHIB);
}

/*==============================================================================
 * Asynchronous (Non-blocking) Functions
 *============================================================================*/

/* The following code is just an example illustrating how one might implement
 * asynchronous reads using HAL_I2C_Master_Transmit_IT / HAL_I2C_Master_Receive_IT.
 * It references a global context (g_maxCtx).
 */

HAL_StatusTypeDef MAX17048_GetVoltage_Async(I2C_HandleTypeDef *hi2c,
                                            float *pVoltage,
                                            MAX17048_Callback_t callback,
                                            void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c      = hi2c;
    g_maxCtx.op        = MAX17048_OP_GET_VOLTAGE;
    g_maxCtx.callback  = callback;
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

/* Similarly for SOC, CRATE, ALERT status, etc. 
 * The code logic is the same pattern: set context -> send address -> receive data in callback.
 */

HAL_StatusTypeDef MAX17048_GetSOC_Async(I2C_HandleTypeDef *hi2c,
                                        float *pSoc,
                                        MAX17048_Callback_t callback,
                                        void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c      = hi2c;
    g_maxCtx.op        = MAX17048_OP_GET_SOC;
    g_maxCtx.callback  = callback;
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

    g_maxCtx.hi2c      = hi2c;
    g_maxCtx.op        = MAX17048_OP_GET_CRATE;
    g_maxCtx.callback  = callback;
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

    g_maxCtx.hi2c     = hi2c;
    g_maxCtx.op       = MAX17048_OP_GET_ALERT_STATUS;
    g_maxCtx.callback = callback;
    g_maxCtx.pUserData= pAlerts;

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

HAL_StatusTypeDef MAX17048_ClearAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                  uint8_t alertsToClear,
                                                  MAX17048_Callback_t callback,
                                                  void *pUserData)
{
    if (g_maxCtx.op != MAX17048_OP_NONE)
        return HAL_BUSY;

    g_maxCtx.hi2c        = hi2c;
    g_maxCtx.op          = MAX17048_OP_CLEAR_ALERT_STATUS;
    g_maxCtx.callback    = callback;
    g_maxCtx.pUserData   = pUserData;
    g_maxCtx.alertsToClear = alertsToClear;

    // Step 1: read STATUS
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
 * HAL I2C Callbacks for asynchronous operations
 *============================================================================*/

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != g_maxCtx.hi2c) return;

    switch (g_maxCtx.op)
    {
        case MAX17048_OP_GET_ALERT_STATUS:
        case MAX17048_OP_GET_VOLTAGE:
        case MAX17048_OP_GET_SOC:
        case MAX17048_OP_GET_CRATE:
        {
            // Now initiate a receive of 2 bytes
            if (HAL_I2C_Master_Receive_IT(hi2c,
                                         (MAX17048_I2C_ADDRESS << 1),
                                         g_maxCtx.rxBuf,
                                         2) != HAL_OK)
            {
                // If fail, call callback with error
                if (g_maxCtx.callback)
                    g_maxCtx.callback(HAL_ERROR, g_maxCtx.pUserData);

                g_maxCtx.op = MAX17048_OP_NONE;
            }
            break;
        }

        case MAX17048_OP_CLEAR_ALERT_STATUS:
        {
            // Potential second stage if we do a read+write flow
            if (g_maxCtx.callback)
                g_maxCtx.callback(HAL_OK, g_maxCtx.pUserData);
            g_maxCtx.op = MAX17048_OP_NONE;
            break;
        }

        default:
            break;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != g_maxCtx.hi2c) return;

    // We got 2 bytes in g_maxCtx.rxBuf
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
            // We read the STATUS. Now we can modify bits and write them back.
            uint16_t updated = g_maxCtx.regValue & ~((uint16_t)g_maxCtx.alertsToClear << 8);

            g_maxCtx.txBuf[0] = MAX17048_REG_STATUS;
            g_maxCtx.txBuf[1] = (uint8_t)(updated >> 8);
            g_maxCtx.txBuf[2] = (uint8_t)(updated & 0xFF);

            if (HAL_I2C_Master_Transmit_IT(hi2c,
                                           (MAX17048_I2C_ADDRESS << 1),
                                           g_maxCtx.txBuf,
                                           3) != HAL_OK)
            {
                if (g_maxCtx.callback)
                    g_maxCtx.callback(HAL_ERROR, g_maxCtx.pUserData);
                g_maxCtx.op = MAX17048_OP_NONE;
            }
            // next step is in HAL_I2C_MasterTxCpltCallback
            break;
        }

        default:
            break;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == g_maxCtx.hi2c)
    {
        if (g_maxCtx.callback)
            g_maxCtx.callback(HAL_ERROR, g_maxCtx.pUserData);

        g_maxCtx.op = MAX17048_OP_NONE;
    }
}
