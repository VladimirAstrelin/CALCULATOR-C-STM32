/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CALC v2.0
  ******************************************************************************
  * @attention
  * Copyright (c) Vlad Astrelin
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"  // Основной заголовочный файл, генерируемый STM32CubeMX

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Пользовательские заголовочные файлы:
#include "LCD1602.h"  // Библиотека для работы с LCD дисплеем 16x2 (или 20x4)
#include <string.h>   // Стандартная библиотека для работы со строками
#include <stdio.h>    // Стандартная библиотека ввода/вывода
#include <stdlib.h>   // Стандартная библиотека общего назначения

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
I2C_HandleTypeDef hi2c1;  // Дескриптор для I2C1 периферии (для связи с LCD)

/* USER CODE BEGIN PV */

// ===== 8x8 Keyboard matrix =====
// Двумерный массив 8x8, содержащий текстовые обозначения для каждой кнопки
// Каждая кнопка идентифицируется по номеру строки (Row) и колонки (Column)
const char* keyboard[8][8] =
{
{"R0_C0", "R0_C1", "R0_C2", "R0_C3", "R0_C4", "R0_C5", "R0_C6", "R0_C7"},  // Строка 0
{"R1_C0", "R1_C1", "R1_C2", "R1_C3", "R1_C4", "R1_C5", "R1_C6", "R1_C7"},  // Строка 1
{"R2_C0", "R2_C1", "R2_C2", "R2_C3", "R2_C4", "R2_C5", "R2_C6", "R2_C7"},  // Строка 2
{"R3_C0", "R3_C1", "R3_C2", "R3_C3", "R3_C4", "R3_C5", "R3_C6", "R3_C7"},  // Строка 3
{"R4_C0", "R4_C1", "R4_C2", "R4_C3", "R4_C4", "R4_C5", "R4_C6", "R4_C7"},  // Строка 4
{"R5_C0", "R5_C1", "R5_C2", "R5_C3", "R5_C4", "R5_C5", "R5_C6", "R5_C7"},  // Строка 5
{"R6_C0", "R6_C1", "R6_C2", "R6_C3", "R6_C4", "R6_C5", "R6_C6", "R6_C7"},  // Строка 6
{"R7_C0", "R7_C1", "R7_C2", "R7_C3", "R7_C4", "R7_C5", "R7_C6", "R7_C7"}   // Строка 7
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);      // Функция настройки системных часов
static void MX_GPIO_Init(void);     // Функция инициализации GPIO
static void MX_I2C1_Init(void);     // Функция инициализации I2C1
/* USER CODE BEGIN PFP */

// ===== Keypad Scanner =====
// Функция сканирования матричной клавиатуры 8x8
// Возвращает строку с идентификатором нажатой кнопки или NULL, если ничего не нажато
const char* Scan_Keypad(void) {
    // --- Массивы для работы со строками (Rows) ---
    // Каждая строка подключена к определенному GPIO порту
    GPIO_TypeDef* rowPorts[8] = {
        GPIOC, // R0 - строка 0 на порту C
        GPIOC, // R1 - строка 1 на порту C
        GPIOC, // R2 - строка 2 на порту C
        GPIOC, // R3 - строка 3 на порту C
        GPIOA, // R4 - строка 4 на порту A
        GPIOC, // R5 - строка 5 на порту C (PC11)
        GPIOA, // R6 - строка 6 на порту A
        GPIOA  // R7 - строка 7 на порту A
    };

    // Пины, к которым подключены строки
    uint16_t rowPins[8] = {
        R0_Pin, R1_Pin, R2_Pin, R3_Pin,  // Пины на порту C
        R4_Pin,                           // Пин на порту A
        R5_Pin,                           // Пин на порту C (PC11)
        R6_Pin, R7_Pin                    // Пины на порту A
    };

    // --- Массивы для работы с колонками (Columns) ---
    // Колонки - это выходы, которые мы активируем по очереди
    GPIO_TypeDef* colPorts[8] = {
        GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,  // Колонки 0-6 на порту D
        GPIOB   // C7 = PB3 - колонка 7 на порту B
    };

    // Пины, к которым подключены колонки
    uint16_t colPins[8] = {
        C0_Pin, C1_Pin, C2_Pin, C3_Pin,  // Колонки 0-3
        C4_Pin, C5_Pin, C6_Pin, C7_Pin   // Колонки 4-7
    };

    // Основной цикл сканирования: перебираем все колонки
    for (int c = 0; c < 8; c++) {
        // Устанавливаем ВСЕ колонки в HIGH (неактивное состояние)
        // Это предотвращает "призрачные" нажатия
        for (int i = 0; i < 8; i++) {
            HAL_GPIO_WritePin(colPorts[i], colPins[i], GPIO_PIN_SET);
        }

        // Активируем текущую колонку: устанавливаем её в LOW (активное состояние)
        // Только одна колонка активна в каждый момент времени
        HAL_GPIO_WritePin(colPorts[c], colPins[c], GPIO_PIN_RESET);
        HAL_Delay(1);  // Короткая задержка для стабилизации сигнала

        // Проверяем все строки в активной колонке
        for (int r = 0; r < 8; r++) {
            // Если пин строки находится в состоянии LOW - кнопка нажата
            // (строки подключены с подтяжкой к питанию, поэтому нажатие = LOW)
            if (HAL_GPIO_ReadPin(rowPorts[r], rowPins[r]) == GPIO_PIN_RESET) {
                HAL_Delay(20); // Антидребезг: ждем 20мс и проверяем еще раз
                if (HAL_GPIO_ReadPin(rowPorts[r], rowPins[r]) == GPIO_PIN_RESET) {
                    // Кнопка действительно нажата - возвращаем колонку в HIGH
                    HAL_GPIO_WritePin(colPorts[c], colPins[c], GPIO_PIN_SET);
                    // Возвращаем идентификатор нажатой кнопки из массива keyboard
                    return keyboard[r][c];
                }
            }
        }

        // Возвращаем текущую колонку обратно в HIGH (неактивное состояние)
        HAL_GPIO_WritePin(colPorts[c], colPins[c], GPIO_PIN_SET);
    }

    return NULL; // Ни одна кнопка не была нажата
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
  HAL_Init();  // Инициализация HAL библиотеки

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // Настройка системных часов

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();    // Инициализация GPIO
  MX_I2C1_Init();    // Инициализация I2C1
  /* USER CODE BEGIN 2 */

  LCD_Init();        // Инициализация LCD дисплея
  LCD_SetCursor(0, 0);  // Установка курсора в начало первой строки
  LCD_PrintString("Keypad Test Ready");  // Вывод приветственного сообщения

  // Включаем все светодиоды на плате Discovery
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Сканируем клавиатуру
      const char* key = Scan_Keypad();

      // Если какая-то кнопка нажата (key не NULL)
      if (key != NULL) {
          LCD_Clear();           // Очищаем дисплей
          LCD_SetCursor(0,0);    // Курсор в начало первой строки
          LCD_PrintString("Key:");  // Выводим текст
          LCD_SetCursor(0,1);    // Курсор в начало второй строки
          LCD_PrintString(key);  // Выводим идентификатор нажатой кнопки
          HAL_Delay(200);        // Задержка для предотвращения многократного считывания
      }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // Используем внешний кварц
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                    // Включаем внешний кварц
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                // Включаем PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;        // Источник для PLL - HSE
  RCC_OscInitStruct.PLL.PLLM = 12;        // Делитель для PLL (HSE = 8MHz / 12 = 0.666MHz)
  RCC_OscInitStruct.PLL.PLLN = 96;        // Умножитель PLL (0.666MHz * 96 = 64MHz)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // Делитель для системных часов (64MHz / 2 = 32MHz)
  RCC_OscInitStruct.PLL.PLLQ = 4;         // Делитель для периферии
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();  // Обработчик ошибки, если настройка не удалась
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Источник системных часов - PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB без деления
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // APB1 = HCLK/2
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2 = HCLK/1

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
  hi2c1.Instance = I2C1;                  // Используем I2C1
  hi2c1.Init.ClockSpeed = 400000;         // Скорость 400 кГц (Fast Mode)
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; // Соотношение HIGH/LOW = 2:1
  hi2c1.Init.OwnAddress1 = 0;             // Адрес устройства (0 - не используется как slave)
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7-битная адресация
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Один адрес устройства
  hi2c1.Init.OwnAddress2 = 0;             // Второй адрес устройства
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Общий вызов отключен
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // Растяжение时钟 включено
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
  // Включаем тактирование всех используемых портов
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // Устанавливаем начальное состояние светодиодов - выключены (RESET)
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  // Устанавливаем колонки 0-6 в высокий уровень (неактивное состояние)
  HAL_GPIO_WritePin(GPIOD, C0_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin|C5_Pin|C6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  // Устанавливаем колонку 7 в высокий уровень (неактивное состояние)
  HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin
                           C0_Pin C1_Pin C2_Pin C3_Pin
                           C4_Pin C5_Pin C6_Pin */
  // Настройка пинов светодиодов и колонок 0-6 как выходов
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin
                          |C0_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin|C5_Pin|C6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Режим: выход Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // Без подтяжки
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Низкая скорость
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin
                           R5_Pin */
  // Настройка пинов строк 0-3 и 5 как входов с подтяжкой к питанию
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // Режим: вход
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // Подтяжка к питанию
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : R4_Pin R6_Pin R7_Pin */
  // Настройка пинов строк 4, 6, 7 как входов с подтяжкой к питанию
  GPIO_InitStruct.Pin = R4_Pin|R6_Pin|R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // Режим: вход
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // Подтяжка к питанию
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C7_Pin */
  // Настройка пина колонки 7 как выхода
  GPIO_InitStruct.Pin = C7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Режим: выход Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // Без подтяжки
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Низкая скорость
  HAL_GPIO_Init(C7_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();  // Отключаем прерывания
  while (1)
  {
    // Бесконечный цикл при ошибке - требуется перезагрузка
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
