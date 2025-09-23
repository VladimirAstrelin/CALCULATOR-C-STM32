/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CALC v2.0
  ******************************************************************************
  * @attention
  * Copyright (c) 23-09-2025 Vlad Astrelin
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"  // Основной заголовочный файл, сгенерированный STM32CubeMX

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Пользовательские заголовочные файлы:
#include "LCD1602.h"  // Библиотека для работы с LCD дисплеем 16x2 (или 20x4)
#include <string.h>   // Стандартная библиотека для работы со строками
#include <stdio.h>    // Стандартная библиотека ввода/вывода (для snprintf)
#include <stdlib.h>   // Стандартная библиотека общего назначения (для atof)
#include <ctype.h>    // Библиотека для проверки типов символов (isdigit)

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
I2C_HandleTypeDef hi2c1;  // Структура для работы с I2C1 (для связи с LCD дисплеем)

/* USER CODE BEGIN PV */

// ===== 8x8 Keyboard matrix =====
// Двумерный массив 8x8, содержащий текстовые обозначения для каждой кнопки
// Каждая кнопка идентифицируется по номеру строки (Row) и колонки (Column)
// Это виртуальная раскладка клавиатуры калькулятора
const char* keyboard[8][8] =
{
    // Row 0
    {"MENU", "7", "8", "9", "+", "DEC", "HEX", "BIN"},
    // Row 1
    {"SHIFT", "4", "5", "6", "-", "x^y", "x!", "SQR"},
    // Row 2
    {"CTRL", "1", "2", "3", "*", "AND", "OR", "XOR"},
    // Row 3
    {"ALT", "0", ".", "=", "/", "NOT", "<<", ">>"},
    // Row 4
    {"DEL", "(", ")", ",", "%", "SIN", "COS", "TAN"},
    // Row 5
    {"WIN", "EXP", "LN", "10^x", "e", "Pi", "LOG", "RAND"},
    // Row 6
    {"BSP", "MOD", "ABS", "NSQR", "DEG", "RAD", "HYP", "INV"},
    // Row 7
    {"CLS", "MC", "MR", "MS", "M+", "M-", "ANS", "OFF"}
};

// ===== Buffers =====
char inputBuffer[100] = ""; // Буфер для вводимой строки (максимум 20 символ + терминатор)
char lastAnswer[20] = "0"; // Буфер для хранения последнего результата (для кнопки ANS)

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
        // Это предотвращает "призрачные" нажатия (ghost pressing)
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

// Основная функция обработки нажатий клавиш
void processKey(const char* key) {
    static uint8_t firstInput = 1; // флаг первого ввода (для очистки приветственного сообщения)
    static uint8_t showResult = 0; // флаг отображения результата (для очистки при новом вводе)

    // Если это первый ввод после запуска - очищаем экран от "Calculator v2.0"
    if (firstInput) {
        LCD_Clear(); // очищаем при первом вводе
        firstInput = 0; // сбрасываем флаг
    }

    // Если показывается результат и пользователь начинает новый ввод - очищаем всё
    // Это предотвращает наложение нового ввода на старый результат
    if (showResult
    		&& strlen(key) == 1
			&& (isdigit((unsigned char)key[0])
			|| key[0]=='.'
			|| key[0]=='+'
			|| key[0]=='-'
			|| key[0]=='*'
			|| key[0]=='/')) {
        						strcpy(inputBuffer, ""); // очищаем буфер ввода
        						LCD_Clear(); // очищаем весь экран
        						showResult = 0; // сбрасываем флаг результата
    						  }

    // Обработка кнопки очистки (CLS)
    if (strcmp(key, "CLS") == 0) {
        strcpy(inputBuffer, ""); // очищаем буфер ввода
        LCD_Clear(); // очищаем экран
        firstInput = 1; // устанавливаем флаг первого ввода
        showResult = 0; // сбрасываем флаг результата
        return;
    }

    // Обработка кнопки последнего сохраненного результата (ANS)
    if (strcmp(key, "ANS") == 0) {
        strcpy(inputBuffer, ""); // очищаем буфер ввода
        LCD_Clear(); // очищаем экран
        LCD_SetCursor(0,0);
        LCD_PrintString(lastAnswer);
        firstInput = 1; // устанавливаем флаг первого ввода
        showResult = 0; // сбрасываем флаг результата
        return;
    }

    // Обработка кнопки меню (MENU)
    if (strcmp(key, "MENU") == 0) {
        strcpy(inputBuffer, ""); // очищаем буфер ввода
        LCD_Clear(); // очищаем экран
        LCD_SetCursor(0,0);
        LCD_PrintString("Menu button pressed");
        firstInput = 1; // устанавливаем флаг первого ввода
        showResult = 0; // сбрасываем флаг результата
        return;
    }

    // Обработка кнопки shift (SHIFT)
    if (strcmp(key, "SHIFT") == 0) {
        strcpy(inputBuffer, ""); // очищаем буфер ввода
        LCD_Clear(); // очищаем экран
        LCD_SetCursor(0,0);
        LCD_PrintString("Shift button pressed");
        firstInput = 1; // устанавливаем флаг первого ввода
        showResult = 0; // сбрасываем флаг результата
        return;
    }

    // Обработка кнопки равенства (=) - выполнение вычислений
    if (strcmp(key, "=") == 0) {
        char expr[32]; // временный буфер для выражения
        strcpy(expr, inputBuffer); // копируем ввод во временный буфер

        // Ищем оператор в выражении
        char *ptrOp = strpbrk(expr, "+-*/");
        if (ptrOp) {
            char op = *ptrOp; // извлекаем оператор
            *ptrOp = '\0'; // разделяем строку на две части

            // Преобразуем строки в числа
            double a = atof(expr); // первый операнд
            double b = atof(ptrOp + 1); // второй операнд

            double res = 0; // результат

            uint8_t error = 0; // флаг ошибки

            // Выполняем операцию в зависимости от оператора
            switch (op) {
                case '+': res = a + b; break;
                case '-': res = a - b; break;
                case '*': res = a * b; break;
                case '/':
                    if (b != 0) {
                        res = a / b; // нормальное деление
                    } else {
                        error = 1; // деление на ноль
                    }
                    break;
            }

            // 1 строка: первый операнд
            LCD_SetCursor(0,0);
            LCD_PrintString(expr);
            // Дополняем пробелами (для очистки остатков предыдущего текста)
            for (int i = strlen(expr); i < 20; i++) LCD_PrintString(" ");

            // 2 строка: оператор
            LCD_SetCursor(0,1);
            char opStr[2] = {op, '\0'}; // преобразуем символ в строку
            LCD_PrintString(opStr);
            // Дополняем пробелами
            for (int i = 1; i < 20; i++) LCD_PrintString(" ");

            // 3 строка: второй операнд
            LCD_SetCursor(0,2);
            LCD_PrintString(ptrOp + 1); // выводим всё после оператора
            // Дополняем пробелами если нужно
            for (int i = strlen(ptrOp + 1); i < 20; i++) LCD_PrintString(" ");

            // 4 строка: результат со знаком равно
            LCD_SetCursor(0,3);
            if (error) {
                LCD_PrintString("= ERROR: DIV 0  "); // ошибка деления на ноль
            } else {
                char buf[20];
                // Форматируем результат со знаком равно
                snprintf(buf, sizeof(buf), "=%.6g", res);
                LCD_PrintString(buf);
                // Дополняем пробелами если нужно
                for (int i = strlen(buf); i < 20; i++) LCD_PrintString(" ");

                // сохраняем результат для кнопки ANS
                snprintf(lastAnswer, sizeof(lastAnswer), "%.6g", res);
            }

            showResult = 1; // устанавливаем флаг, что показывается результат
        }
        return;
    }

    // Обработка Backspace (BSP)
    if (strcmp(key, "BSP") == 0) {
        int len = strlen(inputBuffer);
        if (len > 0) {
            inputBuffer[len - 1] = '\0'; // удаляем последний символ
            // Обновляем только строку ввода
            LCD_SetCursor(0,0);
            LCD_PrintString(inputBuffer); // выводим обновленный буфер
            // Очищаем хвост пробелом (стираем последний символ на дисплее)
            LCD_PrintString(" ");
            LCD_SetCursor(strlen(inputBuffer), 0); // возвращаем курсор на позицию
        }
        return;
    }

    // Ввод цифр и операторов
    if (strlen(key) == 1
    		&& (isdigit((unsigned char)key[0])
    		|| key[0]=='.'
    		|| key[0]=='+'
    		|| key[0]=='-'
    		|| key[0]=='*'
    		|| key[0]=='/'))
    	{
        // Добавляем символ в буфер
        int len = strlen(inputBuffer);
        if (len < sizeof(inputBuffer) - 1) { // проверяем, не переполнен ли буфер
            inputBuffer[len] = key[0]; // добавляем символ
            inputBuffer[len + 1] = '\0'; // добавляем терминатор строки

            // Выводим только новый символ! (оптимизация вместо полной перерисовки)
            LCD_SetCursor(len, 0); // устанавливаем курсор на позицию нового символа
            char temp[2] = {key[0], '\0'}; // создаем временную строку из одного символа
            LCD_PrintString(temp); // выводим символ
        }
        return;
    }

    // Для других функциональных кнопок (MC, MR, MS, M+, M-, ANS и т.д.)
    // можно добавить обработку здесь (в будущих версиях)
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
  MX_GPIO_Init();    // Инициализация GPIO (настройка пинов)
  MX_I2C1_Init();    // Инициализация I2C1 (для LCD дисплея)
  /* USER CODE BEGIN 2 */

  LCD_Init();        // Инициализация LCD дисплея

  // LCD_LedOnOff( 0 ); // Выключить подсветку дисплея (закомментировано)

  LCD_Clear();       // Очистка дисплея
  LCD_SetCursor(0,0); // Установка курсора в начало первой строки
  LCD_PrintString("Calculator v2.0"); // Вывод приветственного сообщения

  // Выключаем все светодиоды на плате Discovery
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Основной бесконечный цикл
      const char* key = Scan_Keypad(); // Сканируем клавиатуру
      if (key != NULL) { // Если кнопка нажата
          processKey(key); // Обрабатываем нажатую кнопку
          HAL_Delay(200); // Задержка для антидребезга и предотвращения многократного срабатывания
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// Далее идут функции, сгенерированные STM32CubeMX:

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // Настройка тактирования процессора от внешнего кварца 8MHz через PLL
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // Используем внешний кварц
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                   // Включаем внешний кварц
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Включаем PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // Источник для PLL - HSE
  RCC_OscInitStruct.PLL.PLLM = 12;        // Делитель: 8MHz / 12 = 0.666MHz
  RCC_OscInitStruct.PLL.PLLN = 96;        // Умножитель: 0.666MHz * 96 = 64MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // Делитель для системных часов: 64MHz / 2 = 32MHz
  RCC_OscInitStruct.PLL.PLLQ = 4;         // Делитель для периферии
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(); // Обработчик ошибки
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Источник системных часов - PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB без деления (32MHz)
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // APB1 = 16MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2 = 32MHz

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
  // Настройка I2C1 для связи с LCD дисплеем
  hi2c1.Instance = I2C1;                  // Используем I2C1
  hi2c1.Init.ClockSpeed = 400000;         // Скорость 400 кГц (Fast Mode)
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; // Соотношение HIGH/LOW = 2:1
  hi2c1.Init.OwnAddress1 = 0;             // Адрес устройства (0 - master mode)
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7-битная адресация
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Один адрес устройства
  hi2c1.Init.OwnAddress2 = 0;             // Второй адрес устройства
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Общий вызов отключен
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // Растяжение时钟 включено
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // Настройка всех GPIO пинов
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Включаем тактирование всех используемых портов
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Устанавливаем начальное состояние светодиодов - выключены
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  // Устанавливаем колонки 0-6 в высокий уровень (неактивное состояние)
  HAL_GPIO_WritePin(GPIOD, C0_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin|C5_Pin|C6_Pin, GPIO_PIN_SET);

  // Устанавливаем колонку 7 в высокий уровень (неактивное состояние)
  HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, GPIO_PIN_SET);

  // Настройка пинов пользовательской кнопки и строк 4,6,7 как входов с подтяжкой к питанию
  GPIO_InitStruct.Pin = USER_BTN_Pin|R4_Pin|R6_Pin|R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Настройка пинов светодиодов и колонок 0-6 как выходов
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin
                          |C0_Pin|C1_Pin|C2_Pin|C3_Pin
                          |C4_Pin|C5_Pin|C6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Режим: выход Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // Без подтяжки
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Низкая скорость
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // Настройка пинов строк 0-3 и 5 как входов с подтяжкой к питанию
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // Режим: вход
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // Подтяжка к питанию
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Настройка пина колонки 7 как выхода
  GPIO_InitStruct.Pin = C7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Режим: выход Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // Без подтяжки
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Низкая скорость
  HAL_GPIO_Init(C7_GPIO_Port, &GPIO_InitStruct);
}

// Обработчик ошибок
void Error_Handler(void)
{
  __disable_irq();  // Отключаем прерывания
  while (1) {}      // Бесконечный цикл при ошибке - требуется перезагрузка
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Функция для отладки assert'ов
}
#endif /* USE_FULL_ASSERT */
