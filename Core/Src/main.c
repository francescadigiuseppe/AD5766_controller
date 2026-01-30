/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Hybrid AC/DC Generator for AD5766 (Dual Board)
  * Clean version with Configurable AC/DC split.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float freq_hz;
    float amp_volts;
    float offset_volts;
    float phase_rad;
} WaveConfig_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===============================================================================
   ===                          USER CONFIGURATION                             ===
   =============================================================================== */

// Define how many channels are Dynamic (AC Waveforms).
// Channels [0 to N-1] will be AC. 
// Channels [N to 15]  will be Static DC.
#define NUM_AC_CHANNELS     2  

// Sampling Settings
#define SAMPLE_RATE         20000.0f // 20 kSps
#define NUM_SAMPLES         200      // Buffer depth (100Hz resolution at 20kSps)

/* =============================================================================== */

#define SPI_BYTES_PER_CH    3
#define V_MIN               -5.0f
#define V_MAX               5.0f

// Helper Macros
#define VOLT_TO_CODE(v)     ((uint16_t)(((v - V_MIN) / (V_MAX - V_MIN)) * 65535.0f))
#define AD5766_CMD_WR_INPUT_REG(x) (0x10 | ((x) & 0xF))
#define AD5766_CMD_WR_SPAN_REG     0x40
#define AD5766_CMD_SW_RESET        0x70
#define AD5766_DATA_RESET_CODE     0x1234
#define AD5766_CMD_MAIN_POWER      0x50
#define AD5766_CMD_DITHER_POWER    0x51
#define SPAN_CODE_5V               0x0006

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* --- VOLTAGE CONFIGURATION --- */

// 1. STATIC DC SETTINGS (Used for channels >= NUM_AC_CHANNELS)
float Board1_DC_Settings[16] = {
    0.0f, 0.0f,               // Ch 0-1 (Ignored if AC=2)
    1.5f, 2.0f, 3.3f, 5.0f,   // Ch 2-5 (Active DC)
    -1.0f, -2.5f, 0.0f, 0.0f, // Ch 6-9
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f // Ch 10-15
};

float Board2_DC_Settings[16] = { 0.0f }; // All Zero defaults

// 2. AC WAVE CONFIGURATION (Used for channels < NUM_AC_CHANNELS)
WaveConfig_t Board1_AC_Settings[16] = {
    { 100.0f, 4.0f, 0.0f, 0.0f },  // Ch 0: 100Hz, 4Vpp
    { 500.0f, 2.0f, 1.0f, 0.0f },  // Ch 1: 500Hz, 2Vpp, 1V Offset
    // Others ignored
};

WaveConfig_t Board2_AC_Settings[16] = {
    { 100.0f, 4.0f, 0.0f, 3.14f }, // Ch 0: Phase Shift 180 deg
    { 500.0f, 2.0f, 1.0f, 0.0f },  // Ch 1
};


/* --- RUNTIME BUFFERS --- */
// RAM allocation optimized for active AC channels only
uint8_t Board1_Pattern[NUM_SAMPLES][NUM_AC_CHANNELS * 3];
uint8_t Board2_Pattern[NUM_SAMPLES][NUM_AC_CHANNELS * 3];

volatile uint16_t wave_index = 0;
uint8_t SW_LDAC_ALL[3]     = {0x30, 0xFF, 0xFF}; // Update all DAC registers

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void AD5766_Init_Chips(void);
void Generate_Waveforms_LUT(void);
void Update_Static_DC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Construct 24-bit SPI Frame
void AD5766_BuildFrame(uint8_t* buff, uint8_t cmd, uint16_t data) {
      buff[0] = cmd;
      buff[1] = (data & 0xFF00) >> 8;
      buff[2] = (data & 0x00FF) >> 0;
}

// Optimized Blocking SPI Transmit (Direct Register Access)
static inline void SPI_Send_Fast(uint8_t *pData, uint16_t len) {
    volatile uint32_t *dr_reg = &hspi1.Instance->DR;
    volatile uint32_t *sr_reg = &hspi1.Instance->SR;
    for (uint16_t i = 0; i < len; i++) {
        *dr_reg = pData[i];
        while (!(*sr_reg & SPI_FLAG_TXE));
    }
    while (*sr_reg & SPI_FLAG_BSY);
}

// Pre-calculate Sine Waves into RAM (LUT)
void Generate_Waveforms_LUT(void) {
    #if (NUM_AC_CHANNELS > 0)
    float time_step = 1.0f / SAMPLE_RATE;

    for (int s = 0; s < NUM_SAMPLES; s++) {
        float t = (float)s * time_step;

        for (int ch = 0; ch < NUM_AC_CHANNELS; ch++) {
            // Board 1
            WaveConfig_t c1 = Board1_AC_Settings[ch];
            float v1 = c1.offset_volts + c1.amp_volts * sinf(2.0f * 3.14159f * c1.freq_hz * t + c1.phase_rad);
            if(v1 > V_MAX) v1 = V_MAX; 
            if(v1 < V_MIN) v1 = V_MIN;
            AD5766_BuildFrame(&Board1_Pattern[s][ch*3], AD5766_CMD_WR_INPUT_REG(ch), VOLT_TO_CODE(v1));

            // Board 2
            WaveConfig_t c2 = Board2_AC_Settings[ch];
            float v2 = c2.offset_volts + c2.amp_volts * sinf(2.0f * 3.14159f * c2.freq_hz * t + c2.phase_rad);
            if(v2 > V_MAX) v2 = V_MAX; 
            if(v2 < V_MIN) v2 = V_MIN;
            AD5766_BuildFrame(&Board2_Pattern[s][ch*3], AD5766_CMD_WR_INPUT_REG(ch), VOLT_TO_CODE(v2));
        }
    }
    #endif
}

// Update Static DC Channels (One-shot)
void Update_Static_DC(void) {
    uint8_t tx[3];
    uint16_t code;
    
    // Loop from first DC channel up to 15
    for(int ch = NUM_AC_CHANNELS; ch < 16; ch++) {
        
        // Board 1
        code = VOLT_TO_CODE(Board1_DC_Settings[ch]);
        AD5766_BuildFrame(tx, AD5766_CMD_WR_INPUT_REG(ch), code);
        FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
        SPI_Send_Fast(tx, 3);
        FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

        // Board 2
        code = VOLT_TO_CODE(Board2_DC_Settings[ch]);
        AD5766_BuildFrame(tx, AD5766_CMD_WR_INPUT_REG(ch), code);
        // Uncomment if CS2 is used
        // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
        // SPI_Send_Fast(tx, 3);
        // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);
    }

    // Latch all values
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
    SPI_Send_Fast(SW_LDAC_ALL, 3);
    FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
}

// Hardware Initialization sequence
void AD5766_Init_Chips(void) {
    uint8_t tx_cmd[3];
    // 1. Hardware Reset
    FAST_PIN_LOW(AD_RESET_GPIO_Port, AD_RESET_Pin); HAL_Delay(1);
    FAST_PIN_HIGH(AD_RESET_GPIO_Port, AD_RESET_Pin); HAL_Delay(5);

    // 2. Software Reset
    AD5766_BuildFrame(tx_cmd, AD5766_CMD_SW_RESET, AD5766_DATA_RESET_CODE);
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin); HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10); FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
    HAL_Delay(5);

    // 3. Power Up & Disable Dither
    AD5766_BuildFrame(tx_cmd, AD5766_CMD_MAIN_POWER, 0x0000);
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin); HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10); FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
    
    AD5766_BuildFrame(tx_cmd, AD5766_CMD_DITHER_POWER, 0xFFFF);
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin); HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10); FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

    // 4. Set Range +/-5V for all channels
    for(int ch=0; ch<16; ch++) {
        AD5766_BuildFrame(tx_cmd, AD5766_CMD_WR_SPAN_REG | ch, SPAN_CODE_5V);
        FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin); HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10); FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
    }
}
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // Initial Pin State
  HAL_GPIO_WritePin(AD_CS1_GPIO_Port, AD_CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, AD_LDAC_Pin|AD_RESET_Pin, GPIO_PIN_SET);

  // 1. Initialize DAC Chips
  AD5766_Init_Chips();

  // 2. Set Static DC Channels
  Update_Static_DC();

  // 3. Prepare and Start AC Channels (if any)
  #if (NUM_AC_CHANNELS > 0)
    Generate_Waveforms_LUT();
    HAL_TIM_Base_Start_IT(&htim2); // Start Interrupt
  #endif
  
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 957-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD_CS1_GPIO_Port, AD_CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, AD_LDAC_Pin|AD_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AD_CS1_Pin */
  GPIO_InitStruct.Pin = AD_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(AD_CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD_LDAC_Pin AD_RESET_Pin */
  GPIO_InitStruct.Pin = AD_LDAC_Pin|AD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// TIMER INTERRUPT: Handles AC Channels Only
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        #if (NUM_AC_CHANNELS > 0)
            uint8_t *buff1 = &Board1_Pattern[wave_index][0];
            uint8_t *buff2 = &Board2_Pattern[wave_index][0];
            
            // Iterate only through AC channels
            for(int i=0; i < NUM_AC_CHANNELS * 3; i+=3) {
                 // Board 1
                 FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
                 SPI_Send_Fast(&buff1[i], 3);
                 FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

                 // Board 2 (Uncomment if needed)
                 // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
                 // SPI_Send_Fast(&buff2[i], 3);
                 // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);
            }

            // Latch Outputs (Apply new AC values)
            FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
            SPI_Send_Fast(SW_LDAC_ALL, 3);
            FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

            if(++wave_index >= NUM_SAMPLES) wave_index = 0;
        #endif
    }
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
