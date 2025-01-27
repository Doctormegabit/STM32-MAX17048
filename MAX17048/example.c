/******************************************************************************
 * @file    example.c
 * @brief   Usage example of the MAX17048 library
 *****************************************************************************/

#include "max17048.h"
#include <stdio.h>
#include <string.h>

// Assume hi2c1 is declared somewhere else in the project
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Example async callback function
 */
static void MyAsyncCallback(HAL_StatusTypeDef status, void *pUserData)
{
    if (status == HAL_OK)
    {
        // We assume we read the voltage
        float *pVoltage = (float *)pUserData;
        printf("[ASYNC] Voltage read successfully: %.3f V\n", *pVoltage);
    }
    else
    {
        printf("[ASYNC] Voltage read error!\n");
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();  // Configure system clock if needed
    MX_GPIO_Init();        // GPIO initialization
    MX_I2C1_Init();        // I2C initialization

    // 1. Initialize MAX17048
    if (MAX17048_Init(&hi2c1) == HAL_OK)
    {
        printf("MAX17048 initialization successful.\n");
    }
    else
    {
        printf("Error: MAX17048 initialization failed.\n");
        // Possibly halt or handle error
    }

    // 2. Read device version
    {
        uint8_t versionVal = 0;
        if (MAX17048_GetVersion(&hi2c1, versionVal) == HAL_OK)
        {
            // NOTE: The function signature might need adjusting to return the 'versionVal'.
            printf("MAX17048 version: 0x%02X\n", versionVal);
        }
        else
        {
            printf("Error reading version.\n");
        }
    }

    // 3. Synchronous read of battery voltage
    {
        float voltage = 0.0f;
        if (MAX17048_GetVoltage(&hi2c1, &voltage) == HAL_OK)
        {
            printf("Battery voltage: %.3f V\n", voltage);
        }
        else
        {
            printf("Error reading battery voltage.\n");
        }
    }

    // 4. Example: set reset threshold to 3.0 V (75 * 40 mV = 3.0 V)
    {
        uint8_t threshold = 75;
        if (MAX17048_SetResetVoltage(&hi2c1, threshold) == HAL_OK)
        {
            printf("Reset voltage threshold set to 3.0 V.\n");
        }
        else
        {
            printf("Error setting reset voltage.\n");
        }
    }

    // 5. Read SOC
    {
        float soc = 0.0f;
        if (MAX17048_GetSOC(&hi2c1, &soc) == HAL_OK)
        {
            printf("SOC: %.2f%%\n", soc);
        }
        else
        {
            printf("Error reading SOC.\n");
        }
    }

    // 6. Example of asynchronous read
    {
        static float asyncVoltage = 0.0f; // Must remain valid until callback
        HAL_StatusTypeDef ret = MAX17048_GetVoltage_Async(&hi2c1,
                                                          &asyncVoltage,
                                                          MyAsyncCallback,
                                                          &asyncVoltage);
        if (ret == HAL_OK)
        {
            printf("Asynchronous voltage read started...\n");
        }
        else
        {
            printf("Could not start async operation (busy or error).\n");
        }
    }

    // 7. Main loop example
    while (1)
    {
        // Periodically read ALERT flags
        HAL_Delay(1000);
        uint8_t alerts = 0;
        if (MAX17048_GetAlertStatus(&hi2c1, &alerts) == HAL_OK)
        {
            if (alerts != 0)
            {
                printf("ALERT triggered: 0x%02X\n", alerts);
                // Clear those alerts
                MAX17048_ClearAlertStatus(&hi2c1, alerts);
            }
        }

        // Also read voltage
        float voltage = 0.0f;
        if (MAX17048_GetVoltage(&hi2c1, &voltage) == HAL_OK)
        {
            printf("Periodic check: Voltage = %.3f V\n", voltage);
        }
        else
        {
            printf("Error reading voltage in loop.\n");
        }

        // Add any other housekeeping or log outputs
    }

    // main() in STM32 typically never returns
    // return 0;
}
