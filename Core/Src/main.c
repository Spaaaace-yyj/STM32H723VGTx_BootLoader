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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*app_entry_t)(void);

#define OTA_PACKET_MAGIC 0xA55A1234U
#define OTA_MAX_RETRY   5U
#define UART_TIMEOUT_MS 3000U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t packet_id;
    uint32_t offset;
    uint16_t data_len;
    uint16_t header_crc;
    uint16_t data_crc;
    uint16_t reserved;
} ota_packet_header_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADDR 0x08020000

#define OTA_MAGIC_VALUE 0xA5A55A5A

#define APP_START_SECTOR FLASH_SECTOR_1
#define APP_SECTOR_COUNT 7U   // Sector 1~7

#define APP_MAX_SIZE      (896U * 1024U)
#define OTA_PACKET_SIZE 1024U
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

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

static uint16_t ota_header_crc16(ota_packet_header_t *header)
{
    uint16_t old_crc = header->header_crc;
    header->header_crc = 0;

    uint16_t crc = crc16_ccitt((uint8_t *)header, sizeof(ota_packet_header_t));

    header->header_crc = old_crc;
    return crc;
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

static HAL_StatusTypeDef write_app_flash(uint32_t flash_addr, const uint8_t *data, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    if ((flash_addr % 32U) != 0U) {
        return HAL_ERROR;
    }

    if ((len % 32U) != 0U) {
        return HAL_ERROR;
    }

    HAL_FLASH_Unlock();

    for (uint32_t offset = 0; offset < len; offset += 32U) {
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            flash_addr + offset,
            (uint32_t)(data + offset)
        );

        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
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

static HAL_StatusTypeDef ota_receive_and_write_app(void)
{
    HAL_StatusTypeDef ret;
    uint32_t fw_size = 0;
    uint32_t expected_fw_crc32 = 0;
    uint32_t calc_fw_crc32 = 0;
    uint32_t received = 0;
    uint32_t expected_packet_id = 0;

    HAL_UART_Transmit(&huart8, (uint8_t *)"READY\n", 6, 100);
    SEGGER_RTT_printf(0, "UART send READY\r\n");

    ret = HAL_UART_Receive(&huart8, (uint8_t *)&fw_size, 4, 5000);
    if (ret != HAL_OK) {
        SEGGER_RTT_printf(0, "Receive fw_size failed, ret=%d\r\n", ret);
        return ret;
    }

    ret = HAL_UART_Receive(&huart8, (uint8_t *)&expected_fw_crc32, 4, 5000);
    if (ret != HAL_OK) {
        SEGGER_RTT_printf(0, "Receive fw_crc32 failed, ret=%d\r\n", ret);
        return ret;
    }

    SEGGER_RTT_printf(0, "fw_size=%lu, fw_crc32=0x%08lX\r\n",
                      fw_size, expected_fw_crc32);

    if (fw_size == 0 || fw_size > APP_MAX_SIZE) {
        HAL_UART_Transmit(&huart8, (uint8_t *)"SIZE_ERR\n", 9, 100);
        return HAL_ERROR;
    }

    if (erase_app_flash() != HAL_OK) {
        HAL_UART_Transmit(&huart8, (uint8_t *)"ERASE_ERR\n", 10, 100);
        return HAL_ERROR;
    }

    HAL_UART_Transmit(&huart8, (uint8_t *)"SIZE_OK\n", 8, 100);

    while (received < fw_size) {
        ota_packet_header_t header;
        uint8_t packet_ok = 0;

        for (uint32_t retry = 0; retry < OTA_MAX_RETRY; retry++) {
            memset(&header, 0, sizeof(header));
            memset(ota_rx_buf, 0xFF, sizeof(ota_rx_buf));

            ret = HAL_UART_Receive(
                &huart8,
                (uint8_t *)&header,
                sizeof(header),
                UART_TIMEOUT_MS
            );

            if (ret != HAL_OK) {
                SEGGER_RTT_printf(0, "Header timeout, retry=%lu\r\n", retry);
                HAL_UART_Transmit(&huart8, (uint8_t *)"NACK\n", 5, 100);
                continue;
            }

            uint16_t calc_header_crc = ota_header_crc16(&header);

            if (header.magic != OTA_PACKET_MAGIC ||
                header.header_crc != calc_header_crc ||
                header.packet_id != expected_packet_id ||
                header.offset != received ||
                header.data_len == 0 ||
                header.data_len > OTA_PACKET_SIZE ||
                header.data_len > (fw_size - received)) {

                SEGGER_RTT_printf(
                    0,
                    "Header invalid: magic=0x%08lX id=%lu offset=%lu len=%u hcrc=0x%04X calc=0x%04X\r\n",
                    header.magic,
                    header.packet_id,
                    header.offset,
                    header.data_len,
                    header.header_crc,
                    calc_header_crc
                );

                HAL_UART_Transmit(&huart8, (uint8_t *)"NACK\n", 5, 100);
                continue;
            }

            ret = HAL_UART_Receive(
                &huart8,
                ota_rx_buf,
                header.data_len,
                UART_TIMEOUT_MS
            );

            if (ret != HAL_OK) {
                SEGGER_RTT_printf(0, "Data timeout, retry=%lu\r\n", retry);
                HAL_UART_Transmit(&huart8, (uint8_t *)"NACK\n", 5, 100);
                continue;
            }

            uint16_t calc_data_crc = crc16_ccitt(ota_rx_buf, header.data_len);

            if (calc_data_crc != header.data_crc) {
                SEGGER_RTT_printf(
                    0,
                    "Data CRC error: recv=0x%04X calc=0x%04X\r\n",
                    header.data_crc,
                    calc_data_crc
                );

                HAL_UART_Transmit(&huart8, (uint8_t *)"NACK\n", 5, 100);
                continue;
            }

            packet_ok = 1;
            break;
        }

        if (!packet_ok) {
            SEGGER_RTT_printf(0, "Packet failed too many times\r\n");
            return HAL_ERROR;
        }

        uint32_t write_len = (header.data_len + 31U) & ~31U;

        if (write_app_flash(APP_ADDR + header.offset, ota_rx_buf, write_len) != HAL_OK) {
            SEGGER_RTT_printf(0, "Write flash failed at 0x%08lX\r\n",
                              APP_ADDR + header.offset);
            HAL_UART_Transmit(&huart8, (uint8_t *)"WRITE_ERR\n", 10, 100);
            return HAL_ERROR;
        }

        calc_fw_crc32 = crc32_update(calc_fw_crc32, ota_rx_buf, header.data_len);

        received += header.data_len;
        expected_packet_id++;

        SEGGER_RTT_printf(0, "packet OK, received=%lu/%lu\r\n", received, fw_size);

        HAL_UART_Transmit(&huart8, (uint8_t *)"ACK\n", 4, 100);
    }

    SEGGER_RTT_printf(0, "calc_crc32=0x%08lX, expected=0x%08lX\r\n",
                      calc_fw_crc32,
                      expected_fw_crc32);

    if (calc_fw_crc32 != expected_fw_crc32) {
        HAL_UART_Transmit(&huart8, (uint8_t *)"CRC_ERR\n", 8, 100);
        return HAL_ERROR;
    }

    HAL_UART_Transmit(&huart8, (uint8_t *)"DONE\n", 5, 100);
    SEGGER_RTT_printf(0, "OTA receive done\r\n");

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

        if (ota_receive_and_write_app() == HAL_OK) {
            SEGGER_RTT_printf(0, "OTA write success\r\n");

            if (app_is_valid()) {
                SEGGER_RTT_printf(0, "Jump to new app\r\n");
                HAL_Delay(100);
                jump_to_app();
            } else {
                SEGGER_RTT_printf(0, "App invalid after OTA\r\n");
            }
        } else {
            SEGGER_RTT_printf(0, "OTA failed\r\n");
        }

        while (1) {
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
