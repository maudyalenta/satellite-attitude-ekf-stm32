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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "arm_math.h"

#include "icm42688.h"
#include "icm42688_stm32.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "system_init.h"
#include "mag_calib.h"
#include "icm42688_calib.h"

#include "imu_norm.h"
#include "quaternion.h"
#include "imu2quaternion.h"
#include "attitude_kalman_extended.h"
#include "quat2euler.h"
#include "webserial_print.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define DEBUG_PRINT 0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void print_imu_data(imu_norm *imu);

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
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */



  if (system_sensors_init(&hi2c2) != SYS_INIT_OK)
  {
      Error_Handler();
  }


  icm42688_data raw;
  icm42688_calib_data cal;
  imu_norm imu;
  int16_t mx_raw, my_raw, mz_raw;
  float mx, my, mz;
  float mx_cal, my_cal, mz_cal;

  quaternion qacc, qmag, qtotal;
  float32_t lx, ly;

  quaternion qekf;
  euler_angle ea;

//  bno055_vector_t bno_quat;
//  bno055_vector_t bno_euler;
// bno055_vector_t bno_vecgravity;
//
  mag_calib_t mag_calib = {0};
  mag_calib_init(&mag_calib);
//
//  //mag_logger_start();
//  //icm_logger_start();
//
//
 attitude_init_extended();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  icm42688_calib_print_gyro();
//	  icm42688_calib_print();
//	  mag_calib_print(&mag_cal);
//	  mag_logger_update();
//	  icm_logger_update();
//	  HAL_Delay(100);



  /* Read */
	  icm42688_readRawSensor(&raw);
	  bno055_getRawMagnetometer(&mx_raw, &my_raw, &mz_raw);   // <-- pakai mx_raw dst (int16_t), BUKAN mx/my/mz

	  /* Calibrate */
	  icm42688_calib_apply_accel_and_gyro(&raw, &cal);
	  mag_correct_ellipsoid(mx_raw, my_raw, mz_raw,            // <-- input dari raw yang benar
	                       &mag_calib, &mx, &my, &mz);          // <-- output ke mx,my,mz (float)

	  //printf("CAL: mx=%.4f my=%.4f mz=%.4f\r\n", mx, my, mz);

	  /* Normalize */
	  sensor2imu(cal, mx, my, mz, &imu);
	  accmag2quat(&qtotal, &qacc, &qmag, imu, &lx, &ly);

	  run_kalman_extended(imu, &qekf);
	  quat2euler(qekf, 0, &ea);   // 1 = output dalam derajat, 0 = radian
	  webserial_print_frame(ea, qekf);   // kirim 3 baris (Orientation, Quaternion, Calibration) ke serial


//#if DEBUG_PRINT   // <-- MULAI dari sini, semua printf di-skip kalau DEBUG_PRINT=0
//    printf("CAL: mx=%.4f my=%.4f mz=%.4f\r\n", mx, my, mz);
//    printf("EKFQuat: %.4f %.4f %.4f %.4f\r\n", qekf.s, qekf.x, qekf.y, qekf.z);
//    printf("BNOQuat: %.4f %.4f %.4f %.4f\r\n", bno_quat.w, bno_quat.x, bno_quat.y, bno_quat.z);
//    // ...printf-printf debug lain yang udah kamu punya, taruh semua di sini
//#endif
//
//	  float euler_send[3];
//	  euler_send[0] = ekf_euler.psi;      // yaw
//	  euler_send[1] = ekf_euler.theta;    // pitch
//	  euler_send[2] = ekf_euler.phi;      // roll
//
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)euler_send, sizeof(euler_send), HAL_MAX_DELAY);


//	  printf("az=%.4f lx=%.4f ly=%.4f\r\n", imu.az, lx, ly);
//	  printf("mx=%.4f my=%.4f mz=%.4f | lx=%.4f ly=%.4f\r\n",
//	         imu.mx, imu.my, imu.mz, lx, ly);


//	      bno055_calibration_state_t cal_state = bno055_getCalibrationState();
//	      printf("Calib: sys=%d gyro=%d accel=%d mag=%d\r\n",
//	             cal_state.sys, cal_state.gyro, cal_state.accel, cal_state.mag);
//
//	      bno_quat = bno055_getVectorQuaternion();
//	      bno_euler = bno055_getVectorEuler();
//	  bno_vecgravity = bno055_getVectorGravity();
//	  printf("Gravity  : x=%.2f y=%.2f z=%.2f\r\n", bno_vecgravity.x, bno_vecgravity.y, bno_vecgravity.z);
//
//	      printf("EKF  : roll=%.2f pitch=%.2f yaw=%.2f\r\n", ea.phi, ea.theta, ea.psi);
//	      printf("BNO Euler: roll=%.2f pitch=%.2f yaw=%.2f\r\n",
//	             bno_euler.x, bno_euler.y, bno_euler.z);   // cek urutan x,y,z sesuai mapping roll/pitch/yaw di driver kamu
//
//	      //printf("MyQuat : %.4f %.4f %.4f %.4f\r\n", qtotal.s, qtotal.x, qtotal.y, qtotal.z);
//	      printf("EKFQuat: %.4f %.4f %.4f %.4f\r\n", qekf.s, qekf.x, qekf.y, qekf.z);
//	      printf("BNOQuat: %.4f %.4f %.4f %.4f\r\n", bno_quat.w, bno_quat.x, bno_quat.y, bno_quat.z);
//
//	      float32_t dot = qtotal.s*bno_quat.w + qtotal.x*bno_quat.x +
//	                       qtotal.y*bno_quat.y + qtotal.z*bno_quat.z;
//	      if (dot < 0) dot = -dot;
//	      if (dot > 1.0f) dot = 1.0f;
//	      float32_t angle_diff_deg = 2.0f * acosf(dot) * 180.0f / 3.14159265f;
//	      printf("Angle diff: %.2f deg\r\n---\r\n", angle_diff_deg);
//
//
//	      quat_to_euler(qtotal.s, qtotal.x, qtotal.y, qtotal.z, &my_roll, &my_pitch, &my_yaw);
//	      quat_to_euler(bno_quat.w, bno_quat.x, bno_quat.y, bno_quat.z, &bno_roll, &bno_pitch, &bno_yaw);

//	      printf("MyEuler : roll=%.2f pitch=%.2f yaw=%.2f\r\n", my_roll, my_pitch, my_yaw);
//	      printf("BNOEuler: roll=%.2f pitch=%.2f yaw=%.2f\r\n", bno_roll, bno_pitch, bno_yaw);
//	      printf("---\r\n");

//	      print_imu_data(&imu);
//
//	      HAL_Delay(100);
//
//
//
//	      // Coba baca raw mag SAMBIL mode NDOF aktif
//	      bno055_getRawMagnetometer(&mx_raw, &my_raw, &mz_raw);
//	      printf("Raw MAG in NDOF mode: %d %d %d\r\n", mx_raw, my_raw, mz_raw);
//
//	      // Baca juga fusion quaternionnya
//	      bno_quat = bno055_getVectorQuaternion();
//	      printf("BNO Quat: %.4f %.4f %.4f %.4f\r\n", bno_quat.w, bno_quat.x, bno_quat.y, bno_quat.z);
//	      HAL_Delay(100);
//

//	  bno055_getRawMagnetometer(
//	          &mx_raw,
//	          &my_raw,
//	          &mz_raw);
//
//	  mag_correct(
//	          mx_raw,
//	          my_raw,
//	          mz_raw,
//	          &mag_cal,
//	          &mx_cal,
//	          &my_cal,
//	          &mz_cal);
//
//	  sensor2imu(
//	          imu_data,
//	          mx_cal,
//	          my_cal,
//	          mz_cal,
//	          &imu);

//	  printf(
//	      "ACC : %d %d %d\r\n",
//	      raw.x_accel,
//		  raw.y_accel,
//		  raw.z_accel);
//
//	  printf(
//	      "GYR : %.3f %.3f %.3f rad/s\r\n",
//	      imu.gx,
//	      imu.gy,
//	      imu.gz);
//
//	  printf(
//	      "MAG : %.3f %.3f %.3f\r\n",
//	      imu.mx,
//	      imu.my,
//	      imu.mz);
//
//	  printf("\r\n");


//	  acc2quat(&qacc, imu);
//
//	  qmag.s = 0;
//	  qmag.x = imu.mx;
//	  qmag.y = imu.my;
//	  qmag.z = imu.mz;
//
//	  vector_rotate(qacc, qmag, &qmag);
//
//	  lx = qmag.x;
//	  ly = qmag.y;
//
//	  mag2quat(&qmag, imu, lx, ly);
//
//	  quat_mult(&qtotal, qmag, qacc);
//
//	  quat_norm(&qtotal);


//	  printf("QACC : %.4f %.4f %.4f %.4f\r\n",
//	         qacc.s,
//	         qacc.x,
//	         qacc.y,
//	         qacc.z);
//
//	  printf("LX LY: %.4f %.4f\r\n",
//	         lx,
//	         ly);
//
//	  printf("QMAG : %.4f %.4f %.4f %.4f\r\n",
//	         qmag.s,
//	         qmag.x,
//	         qmag.y,
//	         qmag.z);
//
//	  printf("QTOTAL : %.4f %.4f %.4f %.4f\r\n",
//	         qtotal.s,
//	         qtotal.x,
//	         qtotal.y,
//	         qtotal.z);

//	  run_kalman_extended(imu, &q_est);
//
//	  quat2euler(q_est, 1, &ea);   // 1 = output dalam derajat, 0 = radian
//	  webserial_print_frame(ea, q_est);   // kirim 3 baris (Orientation, Quaternion, Calibration) ke serial

//	  printf(
//	      "Q = %.4f %.4f %.4f %.4f\r\n",
//	      q_est.s,
//	      q_est.x,
//	      q_est.y,
//	      q_est.z);

//	  printf("Roll=%.4f Pitch=%.4f Yaw=%.4f\r\n", ea.phi, ea.theta, ea.psi);

	 // HAL_Delay(50);

	  // === BNO DATA STREAM - START === //

//	  	  euler = bno055_getVectorEuler();
//	      quat = bno055_getVectorQuaternion();
//	      cal = bno055_getCalibrationState();

	      /*
	      printf(
	          "Yaw=%.2f Pitch=%.2f Roll=%.2f\r\n "
	          "Qw=%.4f Qx=%.4f Qy=%.4f Qz=%.4f\r\n "
	          "SYS=%d GYR=%d ACC=%d MAG=%d\r\n\r\n",

	          euler.x,
	          euler.y,
	          euler.z,

	          quat.w,
	          quat.x,
	          quat.y,
	          quat.z,

	          cal.sys,
	          cal.gyro,
	          cal.accel,
	          cal.mag
	      );
	      HAL_Delay(50); */

	  // === BNO DATA STREAM - END === //

	/*  // === BNO WEB SERIAL DATA STREAM - START === //
	      printf(
	             "Orientation: %.2f, %.2f, %.2f\r\n",
	             360.0f - euler.x,
	             euler.z,
	             euler.y);

	         printf(
	             "Quaternion: %.4f, %.4f, %.4f, %.4f\r\n",
	             quat.w,
	             quat.x,
	             quat.y,
	             quat.z);

	         printf(
	             "Calibration: %d, %d, %d, %d\r\n",
	             cal.sys,
	             cal.gyro,
	             cal.accel,
	             cal.mag);

	         HAL_Delay(50);
	  // === BNO WEB SERIAL DATA STREAM - END === // */



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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.Timing = 0x30A175AB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10820B1B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void print_imu_data(imu_norm *imu)
{
    printf("AX: %.6f  AY: %.6f  AZ: %.6f\r\n", imu->ax, imu->ay, imu->az);
    printf("GX: %.6f  GY: %.6f  GZ: %.6f\r\n", imu->gx, imu->gy, imu->gz);
    printf("MX: %.6f  MY: %.6f  MZ: %.6f\r\n", imu->mx, imu->my, imu->mz);
    printf("---\r\n");
}


void print_raw_data(icm42688_data *raw, float mx, float my, float mz)
{
    printf("[RAW] AX: %6d  AY: %6d  AZ: %6d\r\n", raw->x_accel, raw->y_accel, raw->z_accel);
    printf("[RAW] GX: %6d  GY: %6d  GZ: %6d\r\n", raw->x_gyro, raw->y_gyro, raw->z_gyro);
    printf("[RAW] MX: %.1f  MY: %.1f  MZ: %.1f\r\n", mx, my, mz);
}

// === WEB SERIAL === //
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(
        &hlpuart1,
        (uint8_t *)ptr,
        len,
        HAL_MAX_DELAY);

    return len;
}

// === SWV Debugging === //
//int _write(int file, char *ptr, int len)
//{
//    for(int i = 0; i < len; i++)
//    {
//        ITM_SendChar((*ptr++));
//    }
//    return len;
//}

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
#ifdef USE_FULL_ASSERT
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
