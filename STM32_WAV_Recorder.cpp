
#include "main.h"
#include "MyApp.h"


DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;




//int16_t PlayAudioBuf[kTotalAudioSampleSize]; // Converted to 16-bit samples


//------------------- micro_features_micro_model_settings.h ----------------------

constexpr int kMaxAudioSampleSize = 512;
constexpr int kAudioSampleFrequency = 16000;
constexpr int kFeatureSliceSize = 40;
constexpr int kFeatureSliceCount = 49;
constexpr int kFeatureElementCount = (kFeatureSliceSize * kFeatureSliceCount);
constexpr int kFeatureSliceStrideMs = 20;
constexpr int kFeatureSliceDurationMs = 30;

constexpr int kSilenceIndex = 0;
constexpr int kUnknownIndex = 1;
// If you modify the output categories, you need to update the following values.
constexpr int kCategoryCount = 4;
extern const char* kCategoryLabels[kCategoryCount];

 //---------------------------- End-------------------------------------------
int last_capture_index = 0;
volatile bool new_data_ready = false;


constexpr int kDFSDMSamplesPerSlot = 1024;


bool g_is_audio_initialized = false;
// An internal buffer able to fit 16x our sample size
int32_t dfsdm_dma_buffer[kDFSDMSamplesPerSlot];

constexpr int kAudioCaptureBufferSize = kDFSDMSamplesPerSlot * 16;
int16_t g_audio_capture_buffer[kAudioCaptureBufferSize];
// A buffer that holds our output
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
// Mark as volatile so we can check in a while loop to see if
// any samples have arrived yet.
volatile int32_t g_latest_audio_timestamp = 0;

int counter =0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void CaptureSamples(int half);
int16_t clamp(int32_t val, int16_t min, int16_t max);


//---------------------------------------
volatile uint8_t uart_tx_ready = 1;

volatile uint8_t tx_done_flag = 0;

uint8_t test_data[] = "Hello DMA UART!\r\n";

void TransmitAudioFrame(int16_t* audio_data, int sample_count) {

   HAL_UART_Transmit_DMA(&huart1, (uint8_t*)audio_data, sample_count * sizeof(int16_t));


}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_done_flag = 1; // Mark that TX is complete
    }
}



int16_t clamp(int32_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}


void CaptureSamples(int half) {

  int start_index = (half == 0) ? 0 : (kDFSDMSamplesPerSlot / 2); // 0 index or 512 index

  // Number of new samples per call
  int samples_to_copy = kDFSDMSamplesPerSlot / 2; // only half each time

  // Time (ms) of the latest samplef
  const int32_t time_in_ms = g_latest_audio_timestamp + (samples_to_copy / (kAudioSampleFrequency / 1000));

  const int32_t start_sample_offset = g_latest_audio_timestamp * (kAudioSampleFrequency / 1000);

  // Index in the ring buffer
  const int capture_index = start_sample_offset % kAudioCaptureBufferSize;

  // Clamp and copy with wrap-around into ring buffer
  for (int i = 0; i < samples_to_copy; ++i) {

		int32_t raw = dfsdm_dma_buffer[start_index + i];                       // Raw 32-bit DFSDM sample
		int16_t clamped = clamp(raw, -32768, 32767);             // Clamp to int16_t

		int dest_index = (capture_index + i) % kAudioCaptureBufferSize; // Wrap around
		g_audio_capture_buffer[dest_index] = clamped;

  }

 //  TransmitAudioFrame(g_audio_capture_buffer+capture_index, samples_to_copy);


  last_capture_index = capture_index;
  new_data_ready = true;

  // Mark the timestamp for next call
  g_latest_audio_timestamp = time_in_ms;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// External handles (defined by CubeMX)






/* DMA Half Transfer callback */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
    if (hdfsdm_filter == &hdfsdm1_filter0) {
       // StreamStereoHalfBuffer(0);

    	CaptureSamples(0);
    }
}

/* DMA Full Transfer callback */
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
    if (hdfsdm_filter == &hdfsdm1_filter0) {

    	 CaptureSamples(1);

       // StreamStereoHalfBuffer(1);
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
  MX_DFSDM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//  int32_t starttime =  LatestAudioTimestamp();
  //printf("This is audio_test.cpp \r\n");
//
//  HAL_Delay(2000);
//
//  int32_t current_time =  LatestAudioTimestamp();
//  printf("End time: %lu ms\r\n", current_time);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Start DFSDM recording via DMA */
  /* Start continuous DFSDM DMA for stereo microphones */
    HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, dfsdm_dma_buffer, kDFSDMSamplesPerSlot);
    while (1)
        {


  	  // printf("half flag: %ul, full flag: %ul, usart flag: %ul\r\n",  half_flag, full_flag,usart_flag);
  	   //HAL_Delay(1000);


            if (new_data_ready) {


                // Transmit a full frame starting from last_capture_index
                int16_t frame_buffer[kDFSDMSamplesPerSlot/2];

                for (int i = 0; i < kDFSDMSamplesPerSlot/2; ++i) {
                    int idx = (last_capture_index + i) % kAudioCaptureBufferSize;
                    frame_buffer[i] = g_audio_capture_buffer[idx];

                	counter++;

                }

                TransmitAudioFrame(frame_buffer, kDFSDMSamplesPerSlot/2);

                new_data_ready = false;
            }
        }

//  tx_done_flag = 1; // allow first transmission
//
//  while (1)
//  {
//      if (tx_done_flag)
//      {
//          tx_done_flag = 0;
//          HAL_UART_Transmit_DMA(&huart1, test_data, sizeof(test_data) - 1);
//          //HAL_Delay(1000);
//      }
//      // do other tasks here without blocking
//  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 200;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 25;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x03;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
