/**
 *******************************************
 * @file    MAX17048.c
 * @author  Sergey Avakov / Doctormegabit
 * @link    
 * @version 1.15
 * @date	27-Jenuary-2024
 * @brief   Source file forMAX17048 Driver
 *******************************************
 *
 * @note Repo: https://github.com/Doctormegabit/STM32-MAX17048
 */



#pragma once
#ifndef MAX17048_H
#define MAX17048_H

#include "stm32u5xx_hal.h" // Adjust to your specific STM32 series header
#include <stdbool.h>

/*==============================================================================
 *  Constants, Macros, and Register Addresses for the MAX17048
 *==============================================================================
 */

/** 7-bit I2C address of the MAX17048 device */
#define MAX17048_I2C_ADDRESS          0x36U

/** Device registers (16-bit, MSB first) */
#define MAX17048_REG_VCELL            0x02  ///< Battery voltage measurement
#define MAX17048_REG_SOC              0x04  ///< State of charge (format 8.8)
#define MAX17048_REG_MODE             0x06  ///< Mode register (quick-start, sleep, etc.)
#define MAX17048_REG_VERSION          0x08  ///< Device version
#define MAX17048_REG_HIBRT            0x0A  ///< Hibernate thresholds (ActThr, HibThr)
#define MAX17048_REG_CONFIG           0x0C  ///< Configuration register (RCOMP, SLEEP, ALRT, etc.)
#define MAX17048_REG_VALRT            0x14  ///< Voltage Alert thresholds (Min/Max)
#define MAX17048_REG_CRATE            0x16  ///< Charge/Discharge rate in %/hr
#define MAX17048_REG_VRESET_ID        0x18  ///< Reset threshold voltage (VRESET) and ID
#define MAX17048_REG_STATUS           0x1A  ///< Alert status, reset indicator
#define MAX17048_REG_TABLE            0x40  ///< Battery modeling table
#define MAX17048_REG_CMD              0xFE  ///< Command register (reset)

/* Alert Flag Bits (top byte of STATUS register) */
#define MAX17048_ALERT_RI             (1 << 0) ///< Reset Indicator
#define MAX17048_ALERT_VH             (1 << 1) ///< Voltage High Alert
#define MAX17048_ALERT_VL             (1 << 2) ///< Voltage Low Alert
#define MAX17048_ALERT_VR             (1 << 3) ///< Voltage Reset Alert
#define MAX17048_ALERT_HD             (1 << 4) ///< SOC Low Alert
#define MAX17048_ALERT_SC             (1 << 5) ///< SOC Change Alert

/** Allows enabling/disabling the voltage reset alert (EnVr) in STATUS */
#define MAX17048_STATUS_EnVR          (1 << 14)

/** Configuration bits in the CONFIG register */
#define MAX17048_CONFIG_SLEEP         (1 << 7) ///< Sleep bit (used if Mode.EnSleep is set)
#define MAX17048_CONFIG_ALSC          0x0040   ///< Enable 1% SOC change alert
#define MAX17048_CONFIG_ALERT         (1 << 5) ///< Alert bit

/** Hibernate register presets (0x0A) */
#define MAX17048_HIBRT_ENHIB          0xFFFF   ///< Always use hibernate mode
#define MAX17048_HIBRT_DISHIB         0x0000   ///< Disable hibernate mode

/** Command register resets */
#define MAX17048_CMD_RESET_POR        0x5400   ///< Write to CMD (0xFE) for a full POR reset

/*==============================================================================
 *  Data Structures
 *==============================================================================
 */

/**
 * @brief Example data structure for storing battery parameters
 */
typedef struct
{
    float    voltage;      ///< Battery voltage (Volts)
    float    soc;          ///< State of charge (percentage)
    bool     fault;        ///< Fault status (true if fault is detected)
    uint8_t  version;      ///< IC version (from VERSION register)
    uint8_t  alert_flags;  ///< Current ALERT flags (top byte of STATUS)
} MAX17048_Data;

/*==============================================================================
 *  Basic Functions
 *==============================================================================
 */

/**
 * @brief Initializes the MAX17048 device by reading the VERSION register.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Reads the device version (from the VERSION register).
 * @param hi2c Pointer to the I2C handle.
 * @param version Variable to store the version (lower byte).
 * @return HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetVersion(I2C_HandleTypeDef *hi2c, uint8_t version);

/**
 * @brief Reads the device ID (from the VRESET/ID register).
 * @param hi2c Pointer to the I2C handle.
 * @param id [out] Pointer to store the ID (lower byte).
 * @return HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetID(I2C_HandleTypeDef *hi2c, uint8_t *id);

/*==============================================================================
 *  Voltage, SOC, and Current Rate (CRATE) Functions
 *==============================================================================
 */

/**
 * @brief Reads the battery voltage from VCELL register (in Volts).
 * @param hi2c Pointer to the I2C handle.
 * @param voltage [out] Pointer to a float to store the measured voltage.
 * @return HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetVoltage(I2C_HandleTypeDef *hi2c, float *voltage);

/**
 * @brief Alternative method of reading battery voltage (using a different formula).
 * @param hi2c Pointer to the I2C handle.
 * @param voltage [out] Pointer to a float to store the measured voltage.
 * @return HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetVoltageAlternativ(I2C_HandleTypeDef *hi2c, float *voltage);

/**
 * @brief Reads the battery state-of-charge (SOC) in percentage from the SOC register.
 * @param hi2c Pointer to the I2C handle.
 * @param soc [out] Pointer to a float (range 0..100).
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetSOC(I2C_HandleTypeDef *hi2c, float *soc);

/**
 * @brief Reads the charge/discharge rate in %/hour (CRATE).
 * @param hi2c Pointer to the I2C handle.
 * @param crate [out] Pointer to a float storing the %/hour rate.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetCrate(I2C_HandleTypeDef *hi2c, float *crate);

/**
 * @brief Issues the Quick Start command (MODE.QuickStart).
 * This resets internal fuel-gauge calculations and re-estimates SOC.
 * Use with caution; see datasheet for details.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_QuickStart(I2C_HandleTypeDef *hi2c);

/*==============================================================================
 *  Sleep / Reset / Temperature Compensation
 *==============================================================================
 */

/**
 * @brief Enables or disables Sleep mode.
 * @param hi2c Pointer to the I2C handle.
 * @param enable True to enter sleep mode, false to exit.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetSleepMode(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Adjusts temperature compensation (RCOMP) dynamically based on measured temperature.
 * @param hi2c Pointer to the I2C handle.
 * @param temperature Current temperature in Celsius.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetTempCompens(I2C_HandleTypeDef *hi2c, float temperature);

/**
 * @brief Directly set the RCOMP value (8-bit).
 * @param hi2c Pointer to the I2C handle.
 * @param rcomp 8-bit RCOMP value.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetRComp(I2C_HandleTypeDef *hi2c, uint8_t rcomp);

/**
 * @brief Performs a full device reset (POR) by writing to the CMD register (0xFE).
 * After reset, the device reverts to default register values.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_Reset(I2C_HandleTypeDef *hi2c);

/*==============================================================================
 *  VRESET Threshold, ID, and Comparator Settings
 *==============================================================================
 */

/**
 * @brief Reads the 7-bit reset voltage threshold (VRESET) from register VRESET/ID.
 * @param hi2c Pointer to the I2C handle.
 * @param ResetVoltage [out] 7-bit value (0..127), default around 75 (3.0V).
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t *ResetVoltage);

/**
 * @brief Sets the 7-bit reset voltage threshold, 1 bit = 40mV, default 75 (3.0V).
 * @param hi2c Pointer to the I2C handle.
 * @param threshold Integer 0..127 (0..5.08V).
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t threshold);

/**
 * @brief Sets the reset voltage threshold in float voltage units (0..5.08V).
 * Internally converts to 7 bits, 40mV each.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold Voltage in range 0..5.08V.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetResetVoltageFloat(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Enable or disable the comparator to save ~0.5µA. 
 * @param hi2c Pointer to the I2C handle.
 * @param enable True to enable, false to disable.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetComparator(I2C_HandleTypeDef *hi2c, bool enable);

/*==============================================================================
 *  ALERT Thresholds and Flags
 *==============================================================================
 */

/**
 * @brief Reads the ALERT status (upper byte of STATUS register).
 * @param hi2c Pointer to the I2C handle.
 * @param status [out] Bits indicating triggered alerts, e.g. MAX17048_ALERT_VH, etc.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetAlertStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);

/**
 * @brief Clears (resets) specific alert flags in STATUS by providing a mask of bits.
 * @param hi2c Pointer to the I2C handle.
 * @param flags Mask of alert bits (e.g., MAX17048_ALERT_VH | MAX17048_ALERT_VL).
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAlertFlags(I2C_HandleTypeDef *hi2c, uint8_t flags);

/**
 * @brief Clears all alert flags by writing 0x0000 to the STATUS register.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAllAlertStatus(I2C_HandleTypeDef *hi2c);

/**
 * @brief Clears the ALERT bit in the CONFIG register if it has been asserted.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAlert(I2C_HandleTypeDef *hi2c);

/**
 * @brief Checks if the ALERT bit is set in the CONFIG register and optionally clears it.
 * @param hi2c Pointer to the I2C handle.
 * @param clear If true, clears the ALERT bit after reading.
 * @param alertStatus [out] 1 if ALERT was set, 0 if not.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetAlert(I2C_HandleTypeDef *hi2c, bool clear, uint8_t *alertStatus);

/**
 * @brief Enables or disables the SOC-Change Alert (1%).
 * @param hi2c Pointer to the I2C handle.
 * @param enable True to enable alert, false to disable.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_EnableSOCAlert(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Enables or disables the Voltage Reset Alert (EnVR) in STATUS.
 * @param hi2c Pointer to the I2C handle.
 * @param enable True to enable, false to disable.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_EnableVoltageAlert(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Sets the minimum voltage alert threshold (in Volts).
 * If VCELL < threshold => ALERT is triggered.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold Voltage in range 0..5.1V.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetVAlertMinThreshold(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Sets the maximum voltage alert threshold (in Volts).
 * If VCELL > threshold => ALERT is triggered.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold Voltage in range 0..5.1V.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetVAlertMaxThreshold(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Sets the maximum voltage alert threshold (float).
 * Internally calls SetAlertVoltageMax or similar approach.
 * @param hi2c Pointer to the I2C handle.
 * @param voltage Voltage in range 0..5.1V.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetAlertVoltageMax(I2C_HandleTypeDef *hi2c, float voltage);

/**
 * @brief Sets the minimum voltage alert threshold (float).
 * @param hi2c Pointer to the I2C handle.
 * @param voltage Voltage in range 0..5.1V.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetAlertVoltageMin(I2C_HandleTypeDef *hi2c, float voltage);

/*==============================================================================
 *  Hibernate Mode Functions
 *==============================================================================
 */

/**
 * @brief Enables or disables hibernate mode.
 * @param hi2c Pointer to the I2C handle.
 * @param enable True to enable, false to disable hibernate.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernation(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Checks if the device is currently in hibernate mode (12th bit of MODE register).
 * @param hi2c Pointer to the I2C handle.
 * @param hibernate [out] True if in hibernate mode.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_IsHibernating(I2C_HandleTypeDef *hi2c, bool *hibernate);

/**
 * @brief Sets the Active Threshold (ActThr) for hibernation exit. 1 LSB = 1.25mV.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold 0..0.31875V recommended.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateActTh(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Sets the Hibernation Threshold (HibThr) for entering hibernate.
 * 1 LSB = 0.208%/hour.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold 0..53.04 range in %/hr (typical).
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateHibTh(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Reads the current ActThr in raw bits (0..255) from the HIBRT register (low byte).
 * @param hi2c Pointer to the I2C handle.
 * @param threshold [out] Raw ActThr bits.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetHibernateActThreshold(I2C_HandleTypeDef *hi2c, uint8_t *threshold);

/**
 * @brief Writes the raw ActThr (0..255) to the low byte of the HIBRT register.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold 0..255.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateActThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold);

/**
 * @brief Reads the raw HibThr (0..255) from the HIBRT register (high byte).
 * @param hi2c Pointer to the I2C handle.
 * @param threshold [out] Raw HibThr bits.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetHibernateThreshold(I2C_HandleTypeDef *hi2c, uint8_t *threshold);

/**
 * @brief Writes the raw HibThr to the high byte of the HIBRT register.
 * @param hi2c Pointer to the I2C handle.
 * @param threshold 0..255.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold);

/**
 * @brief Fully enables hibernate mode by writing 0xFFFF to HIBRT.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_EnableHibernate(I2C_HandleTypeDef *hi2c);

/**
 * @brief Fully disables hibernate mode by writing 0x0000 to HIBRT.
 * @param hi2c Pointer to the I2C handle.
 * @return HAL_OK or HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_DisableHibernate(I2C_HandleTypeDef *hi2c);

/*==============================================================================
 *  Asynchronous (Non-Blocking) API
 *==============================================================================
 */

/**
 * @brief Type of callback function used for asynchronous operations.
 * @param status HAL_OK if operation succeeded, otherwise HAL_ERROR or HAL_BUSY
 * @param pUserData pointer to user data provided at call time
 */
typedef void (*MAX17048_Callback_t)(HAL_StatusTypeDef status, void *pUserData);

/**
 * @brief Asynchronous reading of battery voltage. Returns immediately.
 * Result will be notified through the callback function.
 * @param hi2c Pointer to the I2C handle.
 * @param pVoltage Pointer to a float to store the result.
 * @param callback Function pointer for the callback (may be NULL).
 * @param pUserData Any user data that will be passed to the callback.
 * @return HAL_OK if operation is started, HAL_BUSY if the device is busy, or HAL_ERROR on failure.
 */
HAL_StatusTypeDef MAX17048_GetVoltage_Async(I2C_HandleTypeDef *hi2c,
                                            float *pVoltage,
                                            MAX17048_Callback_t callback,
                                            void *pUserData);

/**
 * @brief Similar asynchronous read for SOC.
 */
HAL_StatusTypeDef MAX17048_GetSOC_Async(I2C_HandleTypeDef *hi2c,
                                        float *pSoc,
                                        MAX17048_Callback_t callback,
                                        void *pUserData);

/**
 * @brief Asynchronous read of CRATE (charge/discharge rate).
 */
HAL_StatusTypeDef MAX17048_GetCrate_Async(I2C_HandleTypeDef *hi2c,
                                          float *pCrate,
                                          MAX17048_Callback_t callback,
                                          void *pUserData);

/**
 * @brief Asynchronous read of ALERT status (top byte of STATUS).
 */
HAL_StatusTypeDef MAX17048_GetAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                uint8_t *pAlerts,
                                                MAX17048_Callback_t callback,
                                                void *pUserData);

/**
 * @brief Asynchronous clearing of specified alert flags.
 */
HAL_StatusTypeDef MAX17048_ClearAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                  uint8_t alertsToClear,
                                                  MAX17048_Callback_t callback,
                                                  void *pUserData);

/*==============================================================================
 *  STM32 HAL I2C Callbacks (if you use asynchronous mode)
 *==============================================================================
 * You might implement or forward these callbacks inside max17048.c
 */

/**
 * @brief Callback triggered by HAL upon completing I2C transmission (TX).
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief Callback triggered by HAL upon completing I2C reception (RX).
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief Callback triggered by HAL upon I2C error event.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif // MAX17048_H
