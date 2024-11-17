#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_usart2_tx;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
#define FILTER_SIZE 5

uint16_t adc_values[FILTER_SIZE] = {0};
uint16_t adc_index = 0;
uint32_t variable = 0;
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;
uint16_t adc_buffer[1]; // Buffer para almacenar la lectura del ADC con DMA

char uart_buffer[100]; // Aumentado el tamaño para mensajes de depuración

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
uint16_t Read_ADC(uint8_t channel);
float map_float(float val, float in_0, float in_1, float out_0, float out_1);
void Error_Handler(void);
void MX_TIM2_MspPostInit(TIM_HandleTypeDef *htim);
void Transmit_ADC_Data(void);

int32_t pwm_val = 0;

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init(); /*** */

  // Iniciar PWM para el servomotor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // Iniciar TIM1 para medir ECHO
  HAL_TIM_Base_Start(&htim1);

  //Iniciar DAC para el buzzer
  HAL_DAC_Start(&hdac1 , DAC_CHANNEL_1);

  // Asegurarse de que TRIG esté bajo inicialmente
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  // Mensaje de inicio
  sprintf(uart_buffer, "Sistema Iniciado Correctamente\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer),
                    HAL_MAX_DELAY);

  while (1) {
      uint16_t adc_val = Read_ADC(ADC_CHANNEL_1);

      // Añadir el valor de ADC al array de filtro
      adc_values[adc_index++] = adc_val;
      if (adc_index >= FILTER_SIZE) {
          adc_index = 0;
      }

      // Calcular el promedio de los últimos valores de ADC
      uint32_t sum = 0;
      for (int i = 0; i < FILTER_SIZE; i++) {
        sum += adc_values[i];
      }
      uint16_t adc_avg = sum / FILTER_SIZE;

      // Mapear el promedio de ADC a PWM
      pwm_val = (int32_t)map_float((float)adc_avg, 0, 4095, 600, 2400);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val);

	 int angle = (int)map_float((float)adc_val, 0, 4095, 0, 180); // Opcional: Mapea el valor del ADC a un ángulo de 0 a 180 grados

    // Medición de distancia con sensor ultrasónico
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10)
      ;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    pMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) &&
           pMillis + 10 > HAL_GetTick())
      ;
    Value1 = __HAL_TIM_GET_COUNTER(&htim1);

    pMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) &&
           pMillis + 50 > HAL_GetTick())
      ;
    Value2 = __HAL_TIM_GET_COUNTER(&htim1);

    Distance = (Value2 - Value1) * 0.034 / 2;

    // Enviar ángulo y distancia formateados como "<ángulo>,<distancia>."
    snprintf(uart_buffer, sizeof(uart_buffer), "%d,%d.\r\n", angle, Distance);
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer),
                      HAL_MAX_DELAY);

    //Activar el buzzer en funcion de la distancia
    uint8_t dac_value = (uint8_t)map_float(Distance, 0, 200, 0, 255); // Rango de 0 a 200 cm mapeado a 8 bits
    HAL_DAC_SetValue(&hdac1 , DAC_CHANNEL_1 , DAC_ALING_8B_R , dac_value);//Aling 8 bit right

  }
}

uint16_t Read_ADC(uint8_t channel) {
  uint16_t ad_val = 0;
  ADC_ChannelConfTypeDef sConfig = {0};

  // Configurar el canal ADC
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 300) == HAL_OK) {
    ad_val = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);
  return ad_val;
}

float map_float(float val, float in_0, float in_1, float out_0, float out_1) {
  return (val - in_0) * (out_1 - out_0) / (in_1 - in_0) + out_0;
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configuración de los osciladores
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // Configuración de los relojes
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Habilitar los relojes de los GPIO
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Habilitar reloj de USART2
  __HAL_RCC_USART2_CLK_ENABLE();

  // Habilitar reloj de TIM2 (PWM) y TIM1 (Base)
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  // Habilitar reloj de ADC1
  __HAL_RCC_ADC1_CLK_ENABLE();

  //Habilitar reloj de DAC 
  __HAL_RCC_DAC_CCL_ENABLE();

  // Configurar el pin ECHO (PA8) como entrada
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  // Configurar el pin TRIG (PA9) como salida de push-pull
  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

  // Configurar el pin UART TX (PA2) como función alternativa push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configurar el pin UART RX (PA3) como entrada
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configurar el pin PWM (PA0) como función alternativa push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configurar el pin ADC (PA1) como entrada analógica
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Configurar el pin DAC(PA4) como salida analógica
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; // Modo Analógico
   GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  // Habilitar el reloj de TIM1
  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71; // Prescaler para 1 MHz (72MHz / (71+1))
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  // Configurar la fuente de reloj
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  // Configurar master
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Habilitar el reloj de TIM2
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71; // Prescaler para reducir 72MHz a 1MHz (72MHz / (71 + 1) = 1MHz)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999; // Periodo para 20 ms con un reloj de 1 MHz (1MHz / (19999 + 1) = 50Hz)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  // Configurar la fuente de reloj
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  // Inicializar PWM
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  // Configurar master
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  // Configurar el canal PWM
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // Inicialmente en 0
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  // Configuración de pines adicionales si es necesario
  MX_TIM2_MspPostInit(&htim2);
}

void MX_TIM2_MspPostInit(TIM_HandleTypeDef *htim) {
  // Esta función es generalmente utilizada por STM32CubeMX para la
  // configuración posterior de TIM2 Asegúrate de que el pin PWM (PA0) ya está
  // configurado como AF_PP en MX_GPIO_Init
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  // Habilitar el reloj de ADC1
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  // Configurar el canal ADC1
  sConfig.Channel = ADC_CHANNEL_1; // PA1 para STM32F103
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configuración de DMA para ADC */
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_DISABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;

  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  /* Configuración de DMA para UART TX */
  hdma_usart2_tx.Instance = DMA1_Channel7;
  hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;

  if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
}

void Error_Handler(void) {
  
  while (1) {
    // Blink LED de error o similar
  }
}

void Transmit_ADC_Data(void) {
    uint8_t high_byte = (adc_buffer[0] >> 8) & 0xFF; // Byte alto del ADC
    uint8_t low_byte = adc_buffer[0] & 0xFF;         // Byte bajo del ADC

    // Enviar los dos bytes en secuencia
    uint8_t data[2] = { high_byte, low_byte };
    HAL_UART_Transmit_DMA(&huart2, data, 2);
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  
}
#endif