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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD1602.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

char lcd_buf[17]; // буфер для строки LCD

const char* keyboard[4][6] = 
{
    {"7", "8", "9", "+", "SHIFT", "MENU"},
    {"4", "5", "6", "-", "HEX" , "%"},
    {"1", "2", "3", "*", "BIN", "SQR"},
    {"0", ".", "B", "/", "DEC", "CLS"}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

const char* Scan_Keypad(void) {
    GPIO_TypeDef* rowPort = GPIOC;
    uint16_t rowPins[4] = {R0_Pin, R1_Pin, R2_Pin, R3_Pin};

    GPIO_TypeDef* colPort = GPIOD;
    uint16_t colPins[6] = {C0_Pin, C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin};

    for (int c = 0; c < 6; c++) {
        // Все колонки HIGH
        for (int i = 0; i < 6; i++) {
            HAL_GPIO_WritePin(colPort, colPins[i], GPIO_PIN_SET);
        }

        // Активируем колонку c
        HAL_GPIO_WritePin(colPort, colPins[c], GPIO_PIN_RESET);
        HAL_Delay(1);

        // Проверяем строки
        for (int r = 0; r < 4; r++) {
            if (HAL_GPIO_ReadPin(rowPort, rowPins[r]) == GPIO_PIN_RESET) {
                HAL_Delay(20); // антидребезг
                if (HAL_GPIO_ReadPin(rowPort, rowPins[r]) == GPIO_PIN_RESET) {
                    // Вернуть соответствующий символ
                    return keyboard[r][c];
                }
            }
        }
    }

    return NULL; // ничего не нажато
}

// Функция сканирования матрицы (ROW/COL)
void Scan_Keypad_Debug(void) {
    GPIO_TypeDef* rowPort = GPIOC;
    uint16_t rowPins[4] = {R0_Pin, R1_Pin, R2_Pin, R3_Pin};

    GPIO_TypeDef* colPort = GPIOD;
    uint16_t colPins[6] = {C0_Pin, C1_Pin, C2_Pin, C3_Pin, C4_Pin, C5_Pin};

    for (int c = 0; c < 6; c++) {
        // все столбцы HIGH
        for (int i = 0; i < 6; i++) {
            HAL_GPIO_WritePin(colPort, colPins[i], GPIO_PIN_SET);
        }

        // активируем текущий столбец (LOW)
        HAL_GPIO_WritePin(colPort, colPins[c], GPIO_PIN_RESET);
        HAL_Delay(1); // стабилизация

        // читаем строки
        for (int r = 0; r < 4; r++) {
            if (HAL_GPIO_ReadPin(rowPort, rowPins[r]) == GPIO_PIN_RESET) {
                HAL_Delay(20); // антидребезг
                if (HAL_GPIO_ReadPin(rowPort, rowPins[r]) == GPIO_PIN_RESET) {
                    snprintf(lcd_buf, sizeof(lcd_buf), "ROW:%d COL:%d", r, c);

                    LCD_Clear();
                    LCD_SetCursor(0,0);
                    LCD_PrintString(lcd_buf);

                    return; // сразу выходим после первой найденной кнопки
                }
            }
        }
    }
}

// Функция для определения нажатой кнопки + управление светодиодами
const char* Get_Button_Name(void) {
    // Сначала гасим все светодиоды
    HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);
    
    // Проверяем кнопки и включаем соответствующий светодиод
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_SET);  // Зелёный для UP
        return "UP UP";
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_SET);    // Красный для DOWN
        return "DOWN DOWN";
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_SET);   // Синий для LEFT
        return "LEFT LEFT";
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_SET); // Оранжевый для RIGHT
        return "RIGHT RIGHT";
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) {
        // Включаем ВСЕ светодиоды для ENTER
        HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_SET);
        return "ENTER ENTER";
    }
    return "NONE";
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    
    LCD_Init();
    
    LCD_SetCursor(0, 0);    
    LCD_PrintString("Press any button:");
    
    // Изначально все светодиоды выключены
    HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		    // Проверяем системные кнопки (UP/DOWN/LEFT/RIGHT/ENTER)
    const char* button = Get_Button_Name();
    if (strcmp(button, "NONE") != 0) {
        LCD_Clear();
        LCD_SetCursor(0,0);
        LCD_PrintString("Control Key:");
        LCD_SetCursor(0,1);
        LCD_PrintString(button);
        HAL_Delay(200);
    }
		
		    // Матрица
    const char* key = Scan_Keypad();
    if (key != NULL) {
        LCD_Clear();
        LCD_SetCursor(0,0);
        LCD_PrintString("Key:");
        LCD_SetCursor(0,1);
        LCD_PrintString(key);
        HAL_Delay(200);
    }

    // Проверяем матричную клавиатуру
    // Scan_Keypad_Debug();
    // HAL_Delay(50);
		
//    const char* button = Get_Button_Name();
//    if (strcmp(button, "NONE") != 0) {
//        LCD_SetCursor(0, 1);
//        LCD_PrintString("                "); // Очистка строки
//        LCD_SetCursor(0, 1);
//        LCD_PrintString(button); // LCD_PrintString((char*)button);
//        HAL_Delay(200); // Задержка для антидребезга
//    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, C0_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin|C5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : UP_Pin DOWN_Pin LEFT_Pin RIGHT_Pin
                           ENTER_Pin */
  GPIO_InitStruct.Pin = UP_Pin|DOWN_Pin|LEFT_Pin|RIGHT_Pin
                          |ENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin
                           C0_Pin C1_Pin C2_Pin C3_Pin
                           C4_Pin C5_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin
                          |C0_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin|C5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
