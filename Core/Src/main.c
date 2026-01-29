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
#include <math.h>
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

#include <math.h>
#include <string.h>

/* --- SYSTEM SETTINGS --- */
#define SAMPLE_RATE       20000.0f // 20kHz kSps 
#define NUM_SAMPLES       200      // 20000 / 200 = 100 Hz resolution
#define CHANNELS_PER_BOARD 8       // Number of Dynamic (AC) channels per board
#define SPI_BYTES_PER_CH   3       // 24 bits = 3 bytes

/* --- CONFIGURATION --- */

// 1. DC CONFIGURATION (Static Channels: 8-15 of each board)
// Enter the desired VOLTAGE for each channel here.
float Config_DC_Volts_Board1[8] = { 
    -1.5f,  -2.0f,  5.0f,  0.0f,  3.3f,  -0.5f,  2.0f,  -1.0f 
    // Ch8,    Ch9,        Ch10,      Ch11,      Ch12,      Ch13,       Ch14,      Ch15
};

float Config_DC_Volts_Board2[8] = { 
    0.0f,   0.0f,  1.0f,  2.0f,  3.0f,   1.0f,  2.0f,   -1.0f 
    // Ch8,     Ch9,        Ch10,      Ch11,      Ch12,      Ch13,       Ch14,      Ch15
};

// 2. WAVEFORM CONFIGURATION (Dynamic Channels: 0-7 of each board)
// Set Frequency (Hz), Amplitude (Vpk), Offset (Vdc), and Phase (Rad).
// NOTE: Frequencies must be multiples of 100 Hz for perfect looping;
//       Max Recommended Freq = 5kHz, due to channel bandwidth limitation. 

typedef struct {
    float freq_hz;
    float amp_volts;
    float offset_volts;
    float phase_rad; // Optional: Set to 0.0f if phase shift is not needed
} WaveConfig_t;

WaveConfig_t Config_Waves_Board1[8] = {
    // Freq(Hz),  Amp(V),  Offset(V), Phase(Rad)
    {   100.0f,   1.0f,    0.0f,      0.0f }, // Ch 0
    {   200.0f,   2.5f,    1.0f,      0.0f }, // Ch 1
    {  1200.0f,   2.0f,    0.0f,      0.0f }, // Ch 2 
    {   500.0f,   1.0f,    0.0f,      0.0f }, // Ch 3
    {   700.0f,   2.0f,    0.0f,      0.0f }, // Ch 4
    {  1000.0f,   1.0f,    -1.0f,     3.14f}, // Ch 5 
    {   400.0f,   3.0f,    0.0f,      0.0f }, // Ch 6
    {  1600.0f,   2.0f,   -2.0f,      0.0f }  // Ch 7
};

WaveConfig_t Config_Waves_Board2[8] = {
    // Freq(Hz),  Amp(V),  Offset(V), Phase(Rad)
    {  1000.0f,   5.0f,    0.0f,      1.57f}, // Ch 0 (90 deg shift)
    {  1000.0f,   5.0f,    0.0f,      0.0f }, // Ch 1
    {   200.0f,   5.0f,    0.0f,      0.0f }, // Ch 2
    {   300.0f,   5.0f,    0.0f,      0.0f }, // Ch 3
    {   400.0f,   5.0f,    0.0f,      0.0f }, // Ch 4
    {   500.0f,   5.0f,    0.0f,      0.0f }, // Ch 5
    {   600.0f,   5.0f,    0.0f,      0.0f }, // Ch 6
    {   700.0f,   5.0f,    0.0f,      0.0f }  // Ch 7
};

/* --- AD5766 COMMANDS --- */

// Command binary 0101 (5) combined with Address 0000 (0) = 0x50
#define AD5766_CMD_MAIN_POWER   0x50 

// Command binary 0101 (5) combined with Address 0001 (1) = 0x51
#define AD5766_CMD_DITHER_POWER 0x51 

/* --- DATA DEFINITIONS --- */
// "0" means Power Up (Normal Operation). "1" means Power Down.
// We want 0x0000 to turn everything ON.
#define DATA_POWER_UP_ALL       0x0000 
#define DATA_PWR_DOWN_DITHER    0xFFFF // Keep Dither OFF (1=Off)
#define AD5766_CMD_DITHER_PWR   0x80 // Dither Power Control (datasheet Table 29)

#define AD5766_CMD_SW_RESET      0x70   // Command 0111 (7) | Address 0000 (0)
#define AD5766_DATA_RESET_CODE   0x1234 // Magic word - Table 30

// Span Settings Calculation (See Datasheet Table 24 & 25):
#define AD5766_CMD_WR_INPUT_REG(x)  (0x10 | ((x) & 0xF)) 
#define AD5766_CMD_WR_SPAN_REG  0x40 
// Bits D4-D3 (Power-Up): 00 = Zero Scale
// Bits D2-D0 (Span Range):
//   If you want +/- 5V  -> S2=1, S1=1, S0=0 -> Binary 110 -> 0x06
//   If you want +/- 10V -> S2=1, S1=1, S0=1 -> Binary 111 -> 0x07
// Use 0x0006 for +/- 5V Range with Zero Scale Power-up
#define SPAN_CODE_5V 0x0006 
// Use 0x0007 for +/- 10V Range with Zero Scale Power-up
#define SPAN_CODE_10V 0x0007
// Macro to map Voltage to DAC Code (16-bit = 65535 steps)
#define VOLT_TO_CODE(v)  ((uint16_t)(((v - V_MIN) / (V_MAX - V_MIN)) * 65535.0f))
#define V_MIN -5
#define V_MAX 5

// Command: 0011 (SW LDAC) | Address: 0000 | Data: FFFF (Update All Channels)
uint8_t SW_LDAC_all[3] = {0x30, 0xFF, 0xFF};
// Command 0x30 (SW LDAC) with Mask 0x00FF
// 0x00 -> High Byte (Ch 15-8): 0 (Do NOT update DC channels)
// 0xFF -> Low Byte  (Ch 7-0) : 1 (Update AC channels)
uint8_t SW_LDAC_AC_Only[3] = {0x30, 0x00, 0xFF};


/* --- SYSTEM VARIABLES --- */
#define DMA_BUFFER_SIZE (CHANNELS_PER_BOARD * SPI_BYTES_PER_CH)
uint8_t Board1_Pattern[NUM_SAMPLES][DMA_BUFFER_SIZE];
uint8_t Board2_Pattern[NUM_SAMPLES][DMA_BUFFER_SIZE];

volatile uint16_t wave_index = 0; // uint16_t needed because NUM_SAMPLES > 255
volatile uint8_t spi_state = 0;   // 0=Idle, 1=Board1, 2=Board2

#define UART_RX_SIZE 64
uint8_t UART_RxBuffer[UART_RX_SIZE]; // Dove accumuliamo la frase
uint8_t UART_OneByte;                // Dove riceviamo il singolo carattere
volatile uint8_t UART_RxIndex = 0;   // Indice corrente
volatile uint8_t UART_NewCmdFlag = 0;// Bandierina: 1 = Comando pronto

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Constructs a 24-bit SPI frame for the AD5766.
 */
void AD5766_BuildFrame(uint8_t* buff, uint8_t cmd, uint16_t data) {
    // Byte 2: Command (4 bit) + Address (4 bit)
    // buff[0] = cmd | (addr & 0x0F);
    // // Byte 1: Data High (8 bit)
    // buff[1] = (data >> 8) & 0xFF;
    // // Byte 0: Data Low (8 bit)
    // buff[2] = (data & 0xFF);
    	buff[0] = cmd;
   	  buff[1] = (data & 0xFF00) >> 8;
	    buff[2] = (data & 0x00FF) >> 0;
}

static inline void SPI_Send_3Bytes_Fast(uint8_t *pData) {
    // Spedisce 3 byte bypassando i controlli HAL
    hspi1.Instance->DR = pData[0];
    while (!(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE))); // Aspetta buffer vuoto
    
    hspi1.Instance->DR = pData[1];
    while (!(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)));
    
    hspi1.Instance->DR = pData[2];
    while (!(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE)));
    
    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)); // Aspetta fine trasmissione
}

// Funzione universale per inviare N bytes alla massima velocità possibile
// Blocca la CPU, ma evita l'overhead degli interrupt
static inline void SPI_Send_Fast(uint8_t *pData, uint16_t len) {
// Accesso diretto al registro dati per massima velocità
    volatile uint32_t *dr_reg = &hspi1.Instance->DR;
    volatile uint32_t *sr_reg = &hspi1.Instance->SR;

    for (uint16_t i = 0; i < len; i++) {
        // Scrivi il dato
        *dr_reg = pData[i];
        
        // Attendi che il buffer di trasmissione sia vuoto (TXE)
        while (!(*sr_reg & SPI_FLAG_TXE));
    }
    
    while (*sr_reg & SPI_FLAG_BSY);
}

/**
 * @brief Initialization function.
 * 1. Resets Chips.
 * 2. Sets Output Range (Span).
 * 3. Applies DC voltages to static channels.
 */
void AD5766_Init_Chips(void) {
    uint8_t tx_cmd[3];
    
    // 1. RESET
    // --- HW ---
    FAST_PIN_LOW(AD_RESET_GPIO_Port, AD_RESET_Pin);
    HAL_Delay(1);
    FAST_PIN_HIGH(AD_RESET_GPIO_Port, AD_RESET_Pin);
    HAL_Delay(5); // Allow boot time
    // --- SW ---
    AD5766_BuildFrame(tx_cmd, AD5766_CMD_SW_RESET, AD5766_DATA_RESET_CODE);
    // Board 1
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
    HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
    FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
    // Board 2
    // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
    // HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
    // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);
    HAL_Delay(5); // Allow boot time

    // 2. MAIN POWER UP
    AD5766_BuildFrame(tx_cmd, DATA_POWER_UP_ALL, DATA_POWER_UP_ALL);
    // Board 1
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
    HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
    FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
    // Board 2
    // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
    // HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
    // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);

    HAL_Delay(1); // Wait for internal bias to stabilize

    // 3. DISABLE DITHER
    // Use Command 0x51 with Data 0xFFFF (All Ones = All Dither Blocks OFF)
    AD5766_BuildFrame(tx_cmd, AD5766_CMD_DITHER_POWER, DATA_PWR_DOWN_DITHER);
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
    HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
    FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
    // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
    // HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
    // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);

    // 4. SET RANGE (All channels)
    for(uint8_t ch = 0; ch < 16; ch++) {
        AD5766_BuildFrame(tx_cmd, AD5766_CMD_WR_SPAN_REG | ch, SPAN_CODE_5V);
        // Send to Board 1
        FAST_PIN_LOW(AD_CS1_GPIO_Port,AD_CS1_Pin);
        HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
        FAST_PIN_HIGH(AD_CS1_GPIO_Port,AD_CS1_Pin);
        // Send to Board 2
        // FAST_PIN_LOW(AD_CS2_GPIO_Port,AD_CS2_Pin);
        // HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
        // FAST_PIN_HIGH(AD_CS2_GPIO_Port,AD_CS2_Pin);
    }
    
    // --- 3. SET CUSTOM DC VOLTAGES (Channels 8-15) ---
    // Read from the configuration arrays defined in USER PV
    for(uint8_t i = 0; i < 8; i++) {
        uint8_t dac_ch = i + 8; // Map index 0-7 to Channels 8-15
        
        // --- BOARD 1 DC ---
        uint16_t code_b1 = VOLT_TO_CODE(Config_DC_Volts_Board1[i]);
        AD5766_BuildFrame(tx_cmd, AD5766_CMD_WR_INPUT_REG(dac_ch), code_b1);

        FAST_PIN_LOW(AD_CS1_GPIO_Port,AD_CS1_Pin);
        HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
        FAST_PIN_HIGH(AD_CS1_GPIO_Port,AD_CS1_Pin);

        // --- BOARD 2 DC ---
        uint16_t code_b2 = VOLT_TO_CODE(Config_DC_Volts_Board2[i]);
        AD5766_BuildFrame(tx_cmd, AD5766_CMD_WR_INPUT_REG(dac_ch), code_b2);
        
        // FAST_PIN_LOW(AD_CS2_GPIO_Port,AD_CS2_Pin);
        // HAL_SPI_Transmit(&hspi1, tx_cmd, 3, 10);
        // FAST_PIN_HIGH(AD_CS2_GPIO_Port,AD_CS2_Pin);    
    }

    // --- 4. LATCH DC VALUES (Software LDAC) ---
    // Send command to Board 1
    FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
    HAL_SPI_Transmit(&hspi1, SW_LDAC_all, 3, 10);
    FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

    // Send command to Board 2
    // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
    // HAL_SPI_Transmit(&hspi1, SW_LDAC_all, 3, 10);
    // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);
}

/**
 * @brief Generates the Lookup Table (LUT) for Multi-Frequency Waveforms.
 * Calculates exact SPI commands for every sample and stores them in RAM.
 */
void Generate_Waveforms(void) {
    float time_step = 1.0f / SAMPLE_RATE;

    for (int s = 0; s < NUM_SAMPLES; s++) {
        float t = (float)s * time_step; // Current time in seconds

        // --- BOARD 1 GENERATION ---
        for (int ch = 0; ch < 8; ch++) {
            WaveConfig_t cfg = Config_Waves_Board1[ch];
            
            // Math: V = Offset + Amp * sin(2*pi*f*t + phase)
            float volts = cfg.offset_volts + 
                          cfg.amp_volts * sinf(2.0f * 3.14159f * cfg.freq_hz * t + cfg.phase_rad);
            
            // Software Clipping for safety
            if (volts > V_MAX) volts = V_MAX;
            if (volts < V_MIN) volts = V_MIN;

            uint16_t code = VOLT_TO_CODE(volts);
            AD5766_BuildFrame(&Board1_Pattern[s][ch*3], AD5766_CMD_WR_INPUT_REG(ch), code);
        }

        // --- BOARD 2 GENERATION ---
        for (int ch = 0; ch < 8; ch++) {
            WaveConfig_t cfg = Config_Waves_Board2[ch];
            
            float volts = cfg.offset_volts + 
                          cfg.amp_volts * sinf(2.0f * 3.14159f * cfg.freq_hz * t + cfg.phase_rad);
            
            if (volts > V_MAX) volts = V_MAX;
            if (volts < V_MIN) volts = V_MIN;

            uint16_t code = VOLT_TO_CODE(volts);
            AD5766_BuildFrame(&Board2_Pattern[s][ch*3], AD5766_CMD_WR_INPUT_REG(ch), code);
        }
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  
  // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  // HAL_Delay(500);
  // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

  HAL_UART_Receive_IT(&huart5, &UART_OneByte, 1);
  char msg[] = "STM32 Ready via UART1\r\n";
  HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), 100);

  // 1. Ensure all Control Pins are Inactive (High)
  HAL_GPIO_WritePin(AD_CS1_GPIO_Port, AD_CS1_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(AD_CS2_GPIO_Port, AD_CS2_Pin, GPIO_PIN_SET);

  // 2. Initialize Chips (Hardware Reset & Range Setup)
  AD5766_Init_Chips();

  // 3. Calculate the Sine Wave Patterns in RAM
  Generate_Waveforms();

  // 4. Start the Timer Interrupt (The 80kHz Metronome)
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
  if (UART_NewCmdFlag == 1)
      {

          // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

          UART_NewCmdFlag = 0; // Abbassa flag

          char msg_resp[100];
          int cmd_ch;
          int cmd_mv; // Millivolt (Interi!)
          char cmd_type[10];

          // Parsing: "DC [ch] [millivolt]" -> Esempio "DC 0 2500"
          if (sscanf((char*)UART_RxBuffer, "%s %d %d", cmd_type, &cmd_ch, &cmd_mv) >= 3)
          {
              if (strcmp(cmd_type, "DC") == 0)
              {
                  if (cmd_ch >= 0 && cmd_ch < 8)
                  {
                      float volts = (float)cmd_mv / 1000.0f;
                      Config_DC_Volts_Board1[cmd_ch] = volts;

                      // --- Aggiorna Hardware ---
                      uint8_t dac_ch = cmd_ch + 8;
                      uint16_t code = VOLT_TO_CODE(volts);
                      uint8_t tx[3];
                      AD5766_BuildFrame(tx, AD5766_CMD_WR_INPUT_REG(dac_ch), code);

                      FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
                      SPI_Send_Fast(tx, 3);
                      FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);
                      
                      FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
                      SPI_Send_Fast(SW_LDAC_all, 3);
                      FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

                      // Risposta UART
                      snprintf(msg_resp, sizeof(msg_resp), "OK: DC Ch%d -> %d mV\r\n", cmd_ch, cmd_mv);
                      HAL_UART_Transmit(&huart5, (uint8_t*)msg_resp, strlen(msg_resp), 100);
                  }
              }
          }
          else
          {
               // Echo semplice se il comando non è riconosciuto
               snprintf(msg_resp, sizeof(msg_resp), "ECHO: %s\r\n", UART_RxBuffer);
               HAL_UART_Transmit(&huart5, (uint8_t*)msg_resp, strlen(msg_resp), 100);
          }
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
  htim2.Init.Period = 1125-1;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

/**
 * @brief Timer Interrupt. Fires every 12.5 microseconds (80 kHz).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
// Puntatori ai dati di questo istante temporale
        uint8_t *pBuff1 = Board1_Pattern[wave_index];
        uint8_t *pBuff2 = Board2_Pattern[wave_index];

        // --- INVIO DATI: CANALE PER CANALE ---
        // Dobbiamo "bussare" (CS Toggle) per ogni singolo canale
        for (int ch = 0; ch < 8; ch++) {
            // Calcola l'offset nel buffer (ch * 3)
            uint16_t offset = ch * 3;

            // --- BOARD 1: Invia Ch[x] ---
            FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
            SPI_Send_Fast(&pBuff1[offset], 3); // Manda solo 3 byte!
            FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin); // LATCH! Il chip memorizza Ch[x]

            // --- BOARD 2: Invia Ch[x] ---
            // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
            // SPI_Send_Fast(&pBuff2[offset], 3); // Manda solo 3 byte!
            // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin); // LATCH!
        }

        // --- UPDATE OUTPUTS (SW LDAC) ---
        // Ora che tutti gli 8 registri interni sono carichi, diamo il "GO" a tutti insieme.
        
        // Update Board 1
        FAST_PIN_LOW(AD_CS1_GPIO_Port, AD_CS1_Pin);
        SPI_Send_Fast(SW_LDAC_AC_Only, 3); 
        FAST_PIN_HIGH(AD_CS1_GPIO_Port, AD_CS1_Pin);

        // Update Board 2
        // FAST_PIN_LOW(AD_CS2_GPIO_Port, AD_CS2_Pin);
        // SPI_Send_Fast(SW_LDAC_AC_Only, 3);
        // FAST_PIN_HIGH(AD_CS2_GPIO_Port, AD_CS2_Pin);

        // --- AVANZAMENTO INDICE ---
        wave_index++;
        if (wave_index >= NUM_SAMPLES) {
            wave_index = 0;
        }
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // 1. Controlla se è il terminatore (Invio / \n o \r)
        if (UART_OneByte == '\n' || UART_OneByte == '\r')
        {
            if (UART_RxIndex > 0) // Se abbiamo ricevuto qualcosa prima dell'invio
            {
                UART_RxBuffer[UART_RxIndex] = '\0'; // Chiudi la stringa
                UART_NewCmdFlag = 1;                // Avvisa il main
                UART_RxIndex = 0;                   // Reset per la prossima volta
            }
        }
        else
        {
            // 2. Accumula il carattere nel buffer
            if (UART_RxIndex < UART_RX_SIZE - 1)
            {
                UART_RxBuffer[UART_RxIndex++] = UART_OneByte;
            }
            // Se il buffer è pieno, sovrascrivi l'ultimo (o gestisci errore), qui semplice
        }

        // 3. RI-ARMA L'INTERRUPT per il prossimo carattere
        HAL_UART_Receive_IT(&huart5, &UART_OneByte, 1);
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
