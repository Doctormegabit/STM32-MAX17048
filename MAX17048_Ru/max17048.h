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

#include "stm32u5xx_hal.h" // Зависит от конкретного семейства STM32
#include <stdbool.h>

/*==============================================================================
 *  Константы, макросы и адреса регистров MAX17048
 *==============================================================================
 */

/** Адрес I2C-устройства (7-битный) */
#define MAX17048_I2C_ADDRESS 0x36U

/** Регистры устройства (адреса) */
#define MAX17048_REG_VCELL 0x02     ///< Напряжение батареи (2 байта)
#define MAX17048_REG_SOC 0x04       ///< Уровень заряда батареи (в формате 8.8)
#define MAX17048_REG_MODE 0x06      ///< Режим работы (установка Quick-Start, Sleep и т.д.)
#define MAX17048_REG_VERSION 0x08   ///< Версия устройства
#define MAX17048_REG_HIBRT 0x0A     ///< Настройки гибернации (ActThr, HibThr)
#define MAX17048_REG_CONFIG 0x0C    ///< Конфигурационный регистр (RCOMP, Sleep, ALRT, пороги)
#define MAX17048_REG_VALRT 0x14     ///< Настройки напряжения для предупреждения (Min/Max)
#define MAX17048_REG_CRATE 0x16     ///< Ток зарядки/разрядки батареи (в %/ч)
#define MAX17048_REG_VRESET_ID 0x18 ///< Настройки порога сброса (VRESET) и ID
#define MAX17048_REG_STATUS 0x1A    ///< Статус предупреждений (ALERT), Reset Indicator
#define MAX17048_REG_TABLE 0x40     ///< Таблица данных (конфигурация батареи)
#define MAX17048_REG_CMD 0xFE       ///< Регистр команд (сброс устройства)

/* Маски флагов ALERT (старший байт регистра STATUS) */
#define MAX17048_ALERT_RI (1 << 0) ///< Reset Indicator
#define MAX17048_ALERT_VH (1 << 1) ///< Voltage High Alert
#define MAX17048_ALERT_VL (1 << 2) ///< Voltage Low Alert
#define MAX17048_ALERT_VR (1 << 3) ///< Voltage Reset Alert
#define MAX17048_ALERT_HD (1 << 4) ///< SOC Low Alert
#define MAX17048_ALERT_SC (1 << 5) ///< SOC Change Alert

/** Маска статуса, позволяющая включать Voltage Reset Alert */
#define MAX17048_STATUS_EnVR (1 << 14)

/** Биты в CONFIG-регистре */
#define MAX17048_CONFIG_SLEEP (1 << 7) ///< Разрешение/запрет SLEEP (при EnSleep)
#define MAX17048_CONFIG_ALSC 0x0040    ///< Включение 1% SOC Change Alert (SC)
#define MAX17048_CONFIG_ALERT (1 << 5) ///< Общий бит ALERT (флаг)

/** HibRT - включить/выключить режим гибернации записью в регистр (0x0A) */
#define MAX17048_HIBRT_ENHIB 0xFFFF  ///< Всегда использовать hibernate
#define MAX17048_HIBRT_DISHIB 0x0000 ///< Отключить hibernate

/** CMD-регистр для полного Reset (POR) */
#define MAX17048_CMD_RESET_POR 0x5400 ///< Значение для записи в регистр CMD (0xFE)

/*==============================================================================
 *  Структуры и типы данных
 *==============================================================================
 */

/**
 * @brief Структура для хранения основных данных о батарее (пример)
 */
typedef struct
{
    float voltage;       ///< Напряжение батареи (В)
    float soc;           ///< Уровень заряда (SOC), %
    bool fault;          ///< Есть ли неисправность (например, перегрузка)
    uint8_t version;     ///< Версия устройства (из регистра VERSION)
    uint8_t alert_flags; ///< Текущие флаги ALERT (старший байт STATUS)
} MAX17048_Data;

/*==============================================================================
 *  Функции инициализации и базового чтения
 *==============================================================================
 */

/**
 * @brief Инициализация устройства MAX17048.
 * Проверяет связь с чипом путём чтения регистра VERSION.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK при успешной инициализации, иначе ошибка.
 */
HAL_StatusTypeDef MAX17048_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Чтение версии устройства (из регистра VERSION).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param version Переменная, куда будет сохранена версия (младший байт).
 * @return HAL_OK или HAL_ERROR при сбое.
 */
HAL_StatusTypeDef MAX17048_GetVersion(I2C_HandleTypeDef *hi2c, uint8_t version);

/**
 * @brief Получить ID устройства (из регистра VRESET/ID).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param id [out] Указатель, куда сохранить ID (младший байт).
 * @return HAL_OK или HAL_ERROR при сбое.
 */
HAL_StatusTypeDef MAX17048_GetID(I2C_HandleTypeDef *hi2c, uint8_t *id);

/*==============================================================================
 *  Функции для чтения напряжения, SOC, тока (CRATE)
 *==============================================================================
 */

/**
 * @brief Считывание напряжения батареи (в Вольтах) из регистра VCELL.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param voltage [out] Указатель на float для сохранения напряжения.
 * @return HAL_OK или HAL_ERROR при сбое.
 */
HAL_StatusTypeDef MAX17048_GetVoltage(I2C_HandleTypeDef *hi2c, float *voltage);

/**
 * @brief Альтернативный метод чтения напряжения батареи (другая формула).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param voltage [out] Указатель на float для сохранения напряжения.
 * @return HAL_OK или HAL_ERROR при сбое.
 */
HAL_StatusTypeDef MAX17048_GetVoltageAlternativ(I2C_HandleTypeDef *hi2c, float *voltage);

/**
 * @brief Считывание уровня заряда батареи (в процентах), регистр SOC (формат 8.8).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param soc [out] Указатель на float (0..100%).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetSOC(I2C_HandleTypeDef *hi2c, float *soc);

/**
 * @brief Считывание тока зарядки/разрядки батареи в %/ч (CRATE).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param crate [out] Указатель на float (ток в %/ч).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetCrate(I2C_HandleTypeDef *hi2c, float *crate);

/**
 * @brief Выполнение команды Quick Start (MODE.QuickStart).
 * Сбрасывает внутренний алгоритм и заново оценивает SOC.
 * Рекомендуется использовать осторожно, см. документацию.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_QuickStart(I2C_HandleTypeDef *hi2c);

/*==============================================================================
 *  Функции Sleep/Reset/температурная компенсация
 *==============================================================================
 */

/**
 * @brief Установка режима сна (SLEEP).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param enable true, чтобы включить SLEEP, false - выключить.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetSleepMode(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Установка значения RCOMP для температурной компенсации.
 * Рекомендуется вызывать периодически, учитывая текущую температуру.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param temperature Текущая температура в градусах Цельсия.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetTempCompens(I2C_HandleTypeDef *hi2c, float temperature);

/**
 * @brief Установка RCOMP напрямую (8 бит).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param rcomp Значение (8 бит).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetRComp(I2C_HandleTypeDef *hi2c, uint8_t rcomp);

/**
 * @brief Выполнение полного сброса устройства (POR), запись в регистр CMD (0xFE).
 * Все регистры возвращаются в значения по умолчанию.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_Reset(I2C_HandleTypeDef *hi2c);

/*==============================================================================
 *  Порог сброса напряжения (VRESET), ID, компаратор
 *==============================================================================
 */

/**
 * @brief Прочитать 7-битное значение порога сброса (VRESET) из регистра VRESET/ID.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param ResetVoltage [out] Указатель на uint8_t (0..127), по умолчанию 75 (3.0В).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t *ResetVoltage);

/**
 * @brief Установить 7-битный порог сброса (0..127), 1 бит = 40мВ, по умолчанию 75 (3.0В).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold Целое значение 0..127.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetResetVoltage(I2C_HandleTypeDef *hi2c, uint8_t threshold);

/**
 * @brief То же, но принимать напряжение float (0..5.08В), внутри преобразовать в 7 бит (40мВ/бит).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold Значение напряжения, 0..5.08 В.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetResetVoltageFloat(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Включить или отключить компаратор (экономит ~0.5 мкА).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param enable true, чтобы включить, false - отключить.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetComparator(I2C_HandleTypeDef *hi2c, bool enable);

/*==============================================================================
 *  Настройка предупредительных порогов (ALERT)
 *==============================================================================
 */

/**
 * @brief Чтение текущего статуса (старший байт регистра STATUS), какие ALERT-события произошли.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param status [out] Указатель на переменную для флагов ALERT_xxx.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetAlertStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);

/**
 * @brief Очистка (сброс) определённых флагов предупреждения по маске (старший байт STATUS).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param flags Маска флагов (например, MAX17048_ALERT_VH | MAX17048_ALERT_VL).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAlertFlags(I2C_HandleTypeDef *hi2c, uint8_t flags);

/**
 * @brief Очистка всех флагов предупреждений (запись 0x0000 в регистр STATUS).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAllAlertStatus(I2C_HandleTypeDef *hi2c);

/**
 * @brief Очистка бита ALERT в CONFIG-регистре (при срабатывании).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAlert(I2C_HandleTypeDef *hi2c);

/**
 * @brief Прочитать бит ALERT из CONFIG-регистра и опционально очистить его.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param clear true, чтобы очистить бит ALERT после чтения.
 * @param alertStatus [out] 1, если ALERT был установлен, 0 - нет.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetAlert(I2C_HandleTypeDef *hi2c, bool clear, uint8_t *alertStatus);

/**
 * @brief Включить или отключить SOC Change Alert (1%-ное изменение SOC вызывает ALRT).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param enable true, чтобы включить, false - отключить.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_EnableSOCAlert(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Включить или отключить Voltage Reset Alert (EnVR).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param enable true или false.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_EnableVoltageAlert(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Установить минимальное напряжение предупреждения (в Вольтах).
 * При выходе за порог -> ALERT.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold (0..5.1 В).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetVAlertMinThreshold(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Установить максимальное напряжение предупреждения (в Вольтах).
 * При выходе за порог -> ALERT.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold (0..5.1 В).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetVAlertMaxThreshold(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Установка максимального порога напряжения (float) через SetAlertVoltageMax().
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param voltage Вольты (0..5.1).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetAlertVoltageMax(I2C_HandleTypeDef *hi2c, float voltage);

/**
 * @brief Установка минимального порога напряжения (float) через SetAlertVoltageMin().
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param voltage Вольты (0..5.1).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetAlertVoltageMin(I2C_HandleTypeDef *hi2c, float voltage);

/*==============================================================================
 *  Функции гибернации (Hibernate mode)
 *==============================================================================
 */

/**
 * @brief Включить или отключить режим гибернации (HIB).
 * Если hibernate включен, устройство переходит в низкопотребляющий режим
 * при малых токах, но продолжает отслеживать SOC с пониженной частотой.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param enable true, чтобы включить, false - выключить.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernation(I2C_HandleTypeDef *hi2c, bool enable);

/**
 * @brief Проверить, находится ли устройство в режиме гибернации (12-й бит рег. MODE).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param hibernate [out] true, если в режиме гибернации.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_IsHibernating(I2C_HandleTypeDef *hi2c, bool *hibernate);

/**
 * @brief Установить порог входа в режим активности при гибернации (ActThr).
 * 1 LSB = 1.25 мВ.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold Порог (0..0.31875 В).
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateActTh(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Установить порог гибернации (HibThr).
 * 1 LSB = 0.208 %/ч. Устройство войдёт в HIB, если |CRATE| < HibThr в течение 6 мин.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold (0..53.04 В) - Условно, поскольку регистровое значение пересчитывается
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateHibTh(I2C_HandleTypeDef *hi2c, float threshold);

/**
 * @brief Получить текущий порог активности (ActThr) в "сырых" битах (0..255).
 * Младший байт HIBRT.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold [out] Указатель, куда сохранить.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetHibernateActThreshold(I2C_HandleTypeDef *hi2c, uint8_t *threshold);

/**
 * @brief Установить порог активности (ActThr) в битах (0..255).
 * Младший байт HIBRT.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold Целое значение 0..255.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateActThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold);

/**
 * @brief Получить текущий порог Hibernation (HibThr) в "сырых" битах (0..255).
 * Старший байт HIBRT.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold [out] Указатель, куда сохранить.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_GetHibernateThreshold(I2C_HandleTypeDef *hi2c, uint8_t *threshold);

/**
 * @brief Установить порог Hibernation (HibThr) в битах (0..255).
 * Старший байт HIBRT.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param threshold Целое 0..255.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_SetHibernateThreshold(I2C_HandleTypeDef *hi2c, uint8_t threshold);

/**
 * @brief Полностью включить hibernate (запись 0xFFFF в рег HIBRT).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_EnableHibernate(I2C_HandleTypeDef *hi2c);

/**
 * @brief Полностью отключить hibernate (запись 0x0000 в рег HIBRT).
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @return HAL_OK или HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_DisableHibernate(I2C_HandleTypeDef *hi2c);

/*==============================================================================
 *  Асинхронные (неблокирующие) функции и коллбеки
 *==============================================================================
 */

/**
 * @brief Тип коллбэка, вызываемого после завершения асинхронной операции.
 * @param status Результат (HAL_OK / HAL_ERROR).
 * @param pUserData Указатель на пользовательские данные, переданные при вызове.
 */
typedef void (*MAX17048_Callback_t)(HAL_StatusTypeDef status, void *pUserData);

/**
 * @brief Асинхронное чтение напряжения батареи. После старта возвращает управление,
 * а результат придёт в коллбэке, заданном аргументом `callback`.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param pVoltage Указатель на float, где будет сохранён результат.
 * @param callback Указатель на функцию-коллбек (может быть NULL).
 * @param pUserData Любые пользовательские данные, передаваемые в коллбэк.
 * @return HAL_OK, если операция запущена, или ошибка (например, HAL_BUSY).
 */
HAL_StatusTypeDef MAX17048_GetVoltage_Async(I2C_HandleTypeDef *hi2c,
                                            float *pVoltage,
                                            MAX17048_Callback_t callback,
                                            void *pUserData);

/**
 * @brief Асинхронное чтение SOC. Аналогично `MAX17048_GetVoltage_Async`.
 */
HAL_StatusTypeDef MAX17048_GetSOC_Async(I2C_HandleTypeDef *hi2c,
                                        float *pSoc,
                                        MAX17048_Callback_t callback,
                                        void *pUserData);

/**
 * @brief Асинхронное чтение CRATE. Аналогично `MAX17048_GetVoltage_Async`.
 */
HAL_StatusTypeDef MAX17048_GetCrate_Async(I2C_HandleTypeDef *hi2c,
                                          float *pCrate,
                                          MAX17048_Callback_t callback,
                                          void *pUserData);

/**
 * @brief Асинхронное чтение статуса ALERT (старший байт STATUS).
 */
HAL_StatusTypeDef MAX17048_GetAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                uint8_t *pAlerts,
                                                MAX17048_Callback_t callback,
                                                void *pUserData);

/**
 * @brief Асинхронное очищение определённых флагов ALERT. Сначала читаем STATUS,
 * затем меняем нужные биты и записываем обратно. Результат сообщается в коллбэке.
 * @param hi2c Указатель на I2C_HandleTypeDef.
 * @param alertsToClear Маска флагов.
 * @param callback Указатель на функцию-коллбек.
 * @param pUserData Пользовательские данные.
 * @return HAL_OK или HAL_BUSY/HAL_ERROR.
 */
HAL_StatusTypeDef MAX17048_ClearAlertStatus_Async(I2C_HandleTypeDef *hi2c,
                                                  uint8_t alertsToClear,
                                                  MAX17048_Callback_t callback,
                                                  void *pUserData);

/*==============================================================================
 *  Коллбеки STM32 HAL (опционально, если вы используете асинхронный режим)
 *==============================================================================
 * Эти функции обычно переопределяются или вызываются в Вашем проекте.
 * Если вы используете полностью асинхронный код, то внутри них
 * обрабатываются прерывания I2C, обновляется состояние g_maxCtx
 * и вызывается пользовательский коллбек (см. max17048.c).
 */

/**
 * @brief Коллбек, вызываемый HAL по завершении передачи (TX) по I2C.
 * Вы можете перенаправить его вызов в max17048.c (HAL_I2C_MasterTxCpltCallback).
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief Коллбек, вызываемый HAL по завершении приёма (RX) по I2C.
 * Вы можете перенаправить его вызов в max17048.c (HAL_I2C_MasterRxCpltCallback).
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief Коллбек, вызываемый HAL при ошибке на I2C.
 * Вы можете перенаправить его вызов в max17048.c (HAL_I2C_ErrorCallback).
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif // MAX17048_H
