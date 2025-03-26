/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm_42688p.h"
#include "be252q.h"
#include "stdio.h"
#include "rc.h"
//#include "my1280app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
BE252Q_HandleTypeDef hbe252q;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    /* 通过 HAL 库函数发送数据，等待超时时间设置为 HAL_MAX_DELAY */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    // 初始化BE252Q驱动
//    hbe252q.huart_rx = &huart3;
//    hbe252q.huart_tx = &huart2;
//    BE252Q_Init(&hbe252q);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    printf("system init\r\n");
    printf("pwmctrl task run\r\n");
    printf("esc unlock\r\n");
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    printf("pwm start\r\n");
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
    HAL_Delay(4000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,2000);
    printf("set pwm MAX\r\n");
    HAL_Delay(6000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
    printf("set pwm MIN\r\n");
    HAL_Delay(8000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1100);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
    HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1100);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
    HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1100);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
    HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1100);
    HAL_Delay(1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1000);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART3) {
        // 更新接收索引
        hbe252q.rx_index++;

        // 检查缓冲区溢出
        if(hbe252q.rx_index >= RX_BUFFER_SIZE) {
            hbe252q.rx_index = 0;
        }

        // 重新使能接收中断
        HAL_UART_Receive_IT(huart,
                            &hbe252q.rx_buffer[hbe252q.rx_index],
                            1);
    }
}
// PPM信号参数
#define PPM_SYNC_PULSE_MIN  4000

// 全局变量
extern uint16_t ppmValues[16]; // 存储各通道值
extern uint8_t ppmUpdated;              // 数据更新标志
volatile uint8_t ppmChannelIndex = 0;         // 当前通道索引
volatile uint32_t lastCapture = 0;            // 上一次捕获时间戳
extern osThreadId_t rvRentTaskHandle;
volatile uint32_t ulHighFrequencyTimerTicks = 0;  // 统计时间戳计数器

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // 你的PPM捕获逻辑（与之前非FreeRTOS代码相同）
        uint32_t currentCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t pulseWidth = (currentCapture - lastCapture) & 0xFFFFFFFF;

        if (pulseWidth > PPM_SYNC_PULSE_MIN) {
            ppmChannelIndex = 0;
        } else {
            if (ppmChannelIndex < 16) {
                ppmValues[ppmChannelIndex++] = pulseWidth;
                if (ppmChannelIndex >= 16) {
                    ppmUpdated = 1;
                    vTaskNotifyGiveFromISR(rvRentTaskHandle, pdFALSE);
                }
            }
        }
        lastCapture = currentCapture;
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM6) {
        ulHighFrequencyTimerTicks++;  // 每 1us 计数一次
    }
  /* USER CODE END Callback 1 */
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
      printf("some error happened!\r\n");
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
