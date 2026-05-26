/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*app_entry_t)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADDR 0x08020000

#define OTA_MAGIC_VALUE 0xA5A55A5A

#define APP_START_SECTOR FLASH_SECTOR_1
#define APP_SECTOR_COUNT 7U   // Sector 1~7

#define OTA_PACKET_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t ota_rx_buf[OTA_PACKET_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

static void ota_wait_and_receive_test(void)
{
    HAL_StatusTypeDef ret;

    HAL_UART_Transmit(&huart8, (uint8_t*)"READY\n", 6, 100);

    SEGGER_RTT_printf(0, "UART send READY\r\n");

    ret = HAL_UART_Receive(
        &huart8,
        ota_rx_buf,
        1,
        HAL_MAX_DELAY
    );

    if (ret == HAL_OK) {
        SEGGER_RTT_printf(0, "Receive 1024 bytes OK\r\n");

        SEGGER_RTT_printf(
            0,
            "first bytes: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            ota_rx_buf[0], ota_rx_buf[1], ota_rx_buf[2], ota_rx_buf[3],
            ota_rx_buf[4], ota_rx_buf[5], ota_rx_buf[6], ota_rx_buf[7]
        );
    } else {
        SEGGER_RTT_printf(0, "UART receive failed, ret=%d\r\n", ret);
    }
}

static HAL_StatusTypeDef erase_app_flash(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t sector_error = 0;

    SEGGER_RTT_printf(0, "Erase app flash...\r\n");

    HAL_FLASH_Unlock();

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Sector = APP_START_SECTOR;
    erase_init.NbSectors = APP_SECTOR_COUNT;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "Erase failed, status=%d, sector_error=0x%08X\r\n",
                          status, sector_error);
        return status;
    }

    SEGGER_RTT_printf(0, "Erase app flash OK\r\n");

    return HAL_OK;
}

static uint8_t need_update(void)
{
    HAL_PWR_EnableBkUpAccess();

    uint32_t flag = RTC->BKP0R;

    SEGGER_RTT_printf(0, "rtc flag = 0x%08lX\r\n", flag);

    return flag == OTA_MAGIC_VALUE;
}

static void clear_update_flag(void)
{
    HAL_PWR_EnableBkUpAccess();

    RTC->BKP0R = 0;
}

static uint8_t app_is_valid(void)
{
    uint32_t app_stack = *(volatile uint32_t*)APP_ADDR;
    uint32_t app_reset = *(volatile uint32_t*)(APP_ADDR + 4);

    SEGGER_RTT_printf(0, "app_stack = 0x%08X\r\n", app_stack);
    SEGGER_RTT_printf(0, "app_reset = 0x%08X\r\n", app_reset);

    // DTCM RAM: 0x20000000 ~ 0x2001FFFF
    if (app_stack >= 0x20000000 && app_stack <= 0x20020000)
    {
        return 1;
    }

    // AXI SRAM: 0x24000000 ~ 0x2407FFFF
    if (app_stack >= 0x24000000 && app_stack <= 0x24080000)
    {
        return 1;
    }
    SEGGER_RTT_printf(0, "No useable app!", app_reset);

    return 0;
}

static void jump_to_app(void)
{
    uint32_t app_stack = *(volatile uint32_t*)APP_ADDR;
    uint32_t app_reset = *(volatile uint32_t*)(APP_ADDR + 4);

    app_entry_t app_entry = (app_entry_t)app_reset;

    __disable_irq();

    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (int i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    SCB->VTOR = APP_ADDR;
    __set_MSP(app_stack);

    __enable_irq(); // 关键：跳 App 前重新开中断

    app_entry();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "Bootloader start\r\n");
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_UART8_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */

    if (need_update())
    {
        SEGGER_RTT_printf(0, "OTA magic detected\r\n");

        clear_update_flag();

        if (erase_app_flash() != HAL_OK)
        {
            SEGGER_RTT_printf(0, "erase app failed\r\n");
            while (1)
            {
                HAL_Delay(1000);
            }
        }

        //是否真正擦除
        uint32_t v0 = *(volatile uint32_t*)APP_ADDR;
        uint32_t v1 = *(volatile uint32_t*)(APP_ADDR + 4);
        SEGGER_RTT_printf(0, "app[0]=0x%08X\r\n", v0);
        SEGGER_RTT_printf(0, "app[1]=0x%08X\r\n", v1);

        // uint32_t test_data[8] = {
        //     0x24050000,
        //     0x08035309,
        //     0x11111111,
        //     0x22222222,
        //     0x33333333,
        //     0x44444444,
        //     0x55555555,
        //     0x66666666
        // };
        //
        // HAL_FLASH_Unlock();
        //
        // HAL_FLASH_Program(
        //     FLASH_TYPEPROGRAM_FLASHWORD,
        //     APP_ADDR,
        //     (uint32_t)test_data
        // );
        // HAL_FLASH_Lock();

        ota_wait_and_receive_test();

        while (1)
        {
            SEGGER_RTT_printf(0, "Stay in bootloader, Test Check!\r\n");

            for (int i = 0; i < 8; i++)
            {
                SEGGER_RTT_printf(0, "flash[%d]=0x%08X\r\n",
                                  i,
                                  *(volatile uint32_t*)(APP_ADDR + i * 4));
            }


            HAL_Delay(1000);
        }
    }

    if (app_is_valid())
    {
        jump_to_app();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
        | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
