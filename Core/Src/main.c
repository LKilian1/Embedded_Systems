/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stm32f2xx_hal.h"
#include "lis2mdl.h"
#include "lis2mdl_reg.h"
#include "lis2dw12.h"
#include "lis2dw12_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

#define SENSOR_BUS hi2c1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */

// Interface functions

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);

// Configuration function for  magnetometer lis2mdl

int32_t configureRegister_A(stmdev_ctx_t *ctx, lis2mdl_cfg_reg_a_t *cfg_reg_a, uint8_t *bitMask);
int32_t configureRegister_B(stmdev_ctx_t *ctx, lis2mdl_cfg_reg_b_t *cfg_reg_b, uint8_t *bitMask);
int32_t configureRegister_C(stmdev_ctx_t *ctx, lis2mdl_cfg_reg_c_t *cfg_reg_c, uint8_t *bitMask);
int32_t configureRegister_INT(stmdev_ctx_t *ctx, lis2mdl_int_crtl_reg_t *int_crtl_reg, uint8_t *bitMask);



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

  /* var: bitMask_X
   * Argument for read and write functions
   *
   * bitMask is a variable that is used to check written bits in the configuration registers
   */
  uint8_t bitMask_A, bitMask_B, bitMask_C, bitMask_INT;

  uint8_t whoamI_mag = 0;
  uint8_t whoamI_acc = 0;
  uint8_t rst = 0;

  int16_t data_raw_magnetic[3];
  // int16_t data_raw_temperature;
  float magnetic_mG[3];
  // float temperature_degC;

  int16_t data_raw_acceleration[3];
  float acceleration_mg[3];

  uint8_t tx_buffer_mag[1000];
  uint8_t tx_buffer_acc[1000];


  /* Initialize MEMS-Driver interface for magnetometer */
  stmdev_ctx_t dev_ctx_mag;
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = &SENSOR_BUS;

  LIS2MDL_Object_t sensor_mag;
  sensor_mag.Ctx = dev_ctx_mag;
  sensor_mag.is_initialized = 0;
  sensor_mag.mag_is_enabled = 0;

  /* Initialize overall Register to check config func */
  lis2mdl_cfg_reg_a_t            cfg_reg_a = {0};
  lis2mdl_cfg_reg_b_t            cfg_reg_b = {0};
  lis2mdl_cfg_reg_c_t            cfg_reg_c = {0};
  lis2mdl_int_crtl_reg_t         int_crtl_reg = {0};

  /* Initialize MEMS-Driver interface for accelerometer */
  stmdev_ctx_t dev_ctx_acc;
  dev_ctx_acc.read_reg = platform_read;
  dev_ctx_acc.write_reg = platform_write;
  dev_ctx_acc.handle = &SENSOR_BUS;

  LIS2DW12_Object_t sensor_acc;
  sensor_acc.Ctx = dev_ctx_acc;
  sensor_acc.acc_is_enabled = 0;
  sensor_acc.is_initialized = 0;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();



  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();



  /* USER CODE BEGIN 2 */
  printf("!!!INITIALISATION STARTED!!!\n");



  /* Initialize Hardware */
  HAL_I2C_Init(&hi2c1);

  LIS2MDL_Init(&sensor_mag);
  LIS2MDL_MAG_Enable(&sensor_mag);


  /* Check device ID Magnetometer*/
  if(LIS2MDL_ReadID(&sensor_mag, &whoamI_mag) != LIS2MDL_OK)
  {
	  Error_Handler();
  }
  if (whoamI_mag != LIS2MDL_ID)
  {
	  Error_Handler();
  }

  /* Check device ID Accelereometer*/
  lis2dw12_device_id_get(&dev_ctx_acc, &whoamI_acc);
  if (whoamI_acc != LIS2DW12_ID)
    while (1) {
  	  return HAL_ERROR;
    }


  /* Adjust Configuration Register for magnetometer lis2mdl */

  configureRegister_A(&dev_ctx_mag, &cfg_reg_a, &bitMask_A);
  configureRegister_B(&dev_ctx_mag, &cfg_reg_b, &bitMask_B);
  configureRegister_C(&dev_ctx_mag, &cfg_reg_c, &bitMask_C);
  configureRegister_INT(&dev_ctx_mag, &int_crtl_reg, &bitMask_INT);

  /* Adjust Configuration Register for accelerometer lis2dw12 */

  /* Enable Block Data Update */
  lis2dw12_block_data_update_set(&dev_ctx_acc, PROPERTY_ENABLE);
  /* Set full scale */
  lis2dw12_full_scale_set(&dev_ctx_acc, LIS2DW12_2g);
  /* Configure filtering chain
   * Accelerometer - filter path / bandwidth
   */
  lis2dw12_filter_path_set(&dev_ctx_acc, LIS2DW12_LPF_ON_OUT);
  lis2dw12_filter_bandwidth_set(&dev_ctx_acc, LIS2DW12_ODR_DIV_4);
  /* Configure power mode */
  lis2dw12_power_mode_set(&dev_ctx_acc, LIS2DW12_HIGH_PERFORMANCE);
  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx_acc, LIS2DW12_XL_ODR_25Hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printf("New Samples\n");

	  /* Read magnetic field data */
	  memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
	  lis2mdl_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic);
	  magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[0]);
	  magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[1]);
	  magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[2]);
	  sprintf((char *)tx_buffer_mag,
			  "Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
			  magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
	  tx_com(tx_buffer_mag, strlen((char const *)tx_buffer_mag));
//	   		    /* Read temperature data */
//	  	        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//	  	        lis2mdl_temperature_raw_get(&dev_ctx_mag, &data_raw_temperature);
//	  	        temperature_degC = lis2mdl_from_lsb_to_celsius(data_raw_temperature);
//	  	        sprintf((char *)tx_buffer_mag, "Temperature [degC]:%6.2f\r\n",
//	  	                temperature_degC);
//	  	        tx_com(tx_buffer_mag, strlen((char const *)tx_buffer_mag));

	  HAL_Delay(500);

	  /* Read acceleration data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  lis2dw12_acceleration_raw_get(&dev_ctx_acc, data_raw_acceleration);
	  acceleration_mg[0] = lis2dw12_from_fs2_to_mg(
			  data_raw_acceleration[0]);
	  acceleration_mg[1] = lis2dw12_from_fs2_to_mg(
			  data_raw_acceleration[1]);
	  acceleration_mg[2] = lis2dw12_from_fs2_to_mg(
			  data_raw_acceleration[2]);
	  sprintf((char *)tx_buffer_acc,
			  "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
			  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	  tx_com(tx_buffer_acc, strlen((char const *)tx_buffer_acc));


  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	printf("platform_write: %d %02X\n", reg, reg);
	  reg |= 0x80U;
	  printf("platform_write: %d %02X\n", reg, reg);
	  return HAL_I2C_Mem_Write(handle, LIS2MDL_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		  	  	  	  	  	  	uint16_t len)
{
	printf("platform_read: %d %02X\n", reg, reg);
	  reg |= 0x80U;
	  printf("platform_read: %d %02X\n", reg, reg);
	  return HAL_I2C_Mem_Read(handle, LIS2MDL_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);


}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	  HAL_UART_Transmit(&huart3, tx_buffer, len, 1000);
}

/* Config Functions
 * Start Sequence / general-purpose sequence via datasheet:
 *
 * CFG_REG_A = 80h
 * ---> Enable temperature compensation, Mag = 10 Hz (high-resolution and continuous mode)
 *
 * CFG_REG_C = 01h
 * ---> Mag data-ready interrupt enable
 * bitMask is a variable that is used to check written bits in the configuration registers
 */
/* CONFIGURATION CONFIG_Register_C */
int32_t configureRegister_C(stmdev_ctx_t *ctx, lis2mdl_cfg_reg_c_t *cfg_reg_c, uint8_t *bitMask) {

	lis2mdl_cfg_reg_c_t reg;
    int32_t ret;
    uint8_t dataBuffer[1];
    dataBuffer[0] = 0;

    // Lesen des aktuellen Zustands des Konfigurationsregisters C
    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, dataBuffer, 1);

    if (ret == 0) {
    	// Lesen aktueller Zustand Config Reg C
        reg._4wspi = 0;
        reg.bdu = 0;
        reg.ble = 0;
        reg.drdy_on_pin = 1;
        reg.i2c_dis = 0;
        reg.not_used_02 = 0;
        reg.self_test = 0;
        reg.int_on_pin = 0;

        // Schreiben der neuen Konfiguration in das Register C
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
        memcpy(bitMask, (uint8_t *)&reg, sizeof(reg));
        *cfg_reg_c = (lis2mdl_cfg_reg_c_t)reg;
    }

    if(memcmp((uint8_t *)&reg, bitMask , 1) == 0)									// bitmask-based status check
      {
    	  // Konfiguration von Register B ist erfolgreich
    	  printf("Erfolgreich Register C konfiguriert!\n");
      }	else {
    	  // Konfiguration von Register B ist fehlgeschlagen
    	  printf("Fehlerhaft Register C konfiguriert!\n");
      }

    return ret;
}

/* CONFIGURATION CONFIG_Register_B */
int32_t configureRegister_B(stmdev_ctx_t *ctx,  lis2mdl_cfg_reg_b_t *cfg_reg_b, uint8_t *bitMask) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;
    uint8_t dataBuffer[1];
    dataBuffer[0] = 0;

    // Lesen aktueller Zustand Config Reg B
    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, dataBuffer, 1);

    if (ret == 0) {
    	// Konfiguration des Registers B
        reg.int_on_dataoff = 0;
        reg.lpf = 0;
        reg.not_used_01 = 0;
        reg.off_canc_one_shot = 0;
        reg.set_rst = 0;

        // Schreiben der neuen Konfiguration in das Register B
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);
        memcpy(bitMask, (uint8_t *)&reg, sizeof(reg));
        *cfg_reg_b = (lis2mdl_cfg_reg_b_t )reg;
    }

    if(memcmp((uint8_t *)&reg, bitMask, 1) == 0)									// bitmask-based status check
      {
    	  // Konfiguration von Register B ist erfolgreich
    	  printf("Erfolgreich Register B konfiguriert!\n");
      }	else {
    	  // Konfiguration von Register B ist fehlgeschlagen
    	  printf("Fehlerhaft Register B konfiguriert!\n");
      }

    return ret;
}

/* CONFIGURATION CONFIG_Register_A */
int32_t configureRegister_A(stmdev_ctx_t *ctx,lis2mdl_cfg_reg_a_t *cfg_reg_a, uint8_t *bitMask) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;
    uint8_t dataBuffer[1];
    dataBuffer[0] = 0;

    // Lesen des aktuellen Zustands des Konfigurationsregisters A
    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, dataBuffer, 1);

    if (ret == 0) {
    	// Konfiguration des Registers A
        reg.comp_temp_en = 1;														// enable the magnometer temperature compensation
        reg.lp = 0;
        reg.md = 0;																	// select continuous mode for device operations
        reg.odr = 0;																// select 100 Hz output data rate
        reg.reboot = 0;
        reg.soft_rst = 0;

        // Schreiben der neuen Konfiguration in das Register A
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
        memcpy(bitMask, (uint8_t *)&reg, sizeof(reg));
        *cfg_reg_a = (lis2mdl_cfg_reg_a_t )reg;
   }

    if(memcmp((uint8_t *)&reg, bitMask, 1) == 0)									// bitmask-based status check
      {
    	  // Konfiguration von Register B ist erfolgreich
    	  printf("Erfolgreich Register A konfiguriert!\n");
      }	else {
    	  // Konfiguration von Register B ist fehlgeschlagen
    	  printf("Fehlerhaft Register A konfiguriert!\n");
      }

    return ret;
}

/* CONFIGURATION CONFIG_Register_INT */
  int32_t configureRegister_INT(stmdev_ctx_t *ctx, lis2mdl_int_crtl_reg_t *int_crtl_reg, uint8_t *bitMask)
  {
    lis2mdl_int_crtl_reg_t reg;
    int32_t ret;
    uint8_t dataBuffer[1];
    dataBuffer[0] = 0;

    // Lesen des aktuellen Zustands des Interrupt - Konfigurationsregisters
    ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_CRTL_REG, dataBuffer, 1);

    if (ret == 0) {
    // Konfiguration des Interrupt Registers
        reg.xien = 0;
        reg.yien = 0;
        reg.zien = 0;
        reg.not_used_01 = 0;
        reg.iea = 0;
        reg.iel = 0;
        reg.ien = 0;

        // Schreiben der neuen Konfiguration in das Register A
        ret = lis2mdl_write_reg(ctx, LIS2MDL_INT_CRTL_REG, (uint8_t *)&reg, 1);
        bitMask = (uint8_t *)&reg;
        *int_crtl_reg = (lis2mdl_int_crtl_reg_t )reg;
    }

    if(memcmp((uint8_t *)&reg, bitMask, 1) == 0)									// bitmask-based status check
      {
    	  // Konfiguration von Register B ist erfolgreich
    	  printf("Erfolgreich Interrupt Register konfiguriert!\n");
      }	else {
    	  // Konfiguration von Register B ist fehlgeschlagen
    	  printf("Erfolgreich Interrupt Register konfiguriert!\n");
      }

    return ret;

  }

/**
 * @brief Print the characters to UART (printf)
 * @retval int
 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker-> Libs->Smal printf set to 'Yes') calls __ io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC_- */
{
HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);

return ch;
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
