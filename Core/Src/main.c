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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_IDLE,       // รอการกดปุ่ม (วัดหัวใจ CH1)
  STATE_RECORDING,  // กำลังรับเสียง (วัดไมค์ CH2)
  STATE_PROCESSING, // จำลองการประมวลผล
  STATE_RESULT      // แสดงผลลัพธ์
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile SystemState_t currentState = STATE_IDLE;
static char uart_buf[64];
static uint32_t t_ms = 0;

// --- DMA BUFFER ---
// [0] = Microphone, [1] = Heartbeat
uint32_t adc_buffer[2];

// --- MICROPHONE VARIABLES ---
uint32_t mic_max = 0;
uint32_t mic_min = 4095;
uint32_t mic_val = 0;       // The final "Volume" value
uint32_t mic_last_reset = 0; // Timer for the 50ms window
int flag = 0;

// --- TALK DETECTION SETTINGS ---
#define MIC_THRESHOLD  500   // Volume above this = Talking (Tune this!)
#define HANG_TIME_MS   7000  // How long to wait before deciding speech stopped

// --- TALK DETECTION VARIABLES ---
uint8_t is_talking = 0;       // The final output: 1 = Talking, 0 = Quiet
uint32_t silence_timer = 0;   // Timer to track silence

/* ===== BPM Variables ===== */
static float dc = 0.0f; // DC tracker
static float y = 0.0f;  // ค่ากรองแล้ว (filt)
static float y_prev = 0.0f;
static float thr_ema = 0.0f;   // ฐานสำหรับคำนวณ threshold
static uint8_t peak_armed = 0; // เข้าโซนเหนือ threshold แล้ว รอหล่นข้าม low-threshold เพื่อนับ
static uint8_t refractory = 0; // กันเด้ง
static uint32_t ref_start_ms = 0;
static uint32_t last_peak_ms = 0;
static float bpm = 0.0f;

/* ===== Statistics Variables ===== */
float bpm_buffer[30]; // Circular buffer สำหรับ Baseline
int bpm_idx = 0;
int bpm_count_base = 0;

float baseline_mean = 0.0f; // u
float baseline_std = 0.0f;  // sigma

float response_sum = 0.0f;  // ผลรวม BPM ตอน Recording
int response_count = 0;     // n

/* ===== Other Config ===== */
#define ALPHA_DC 0.01f
#define ALPHA_LP 0.75f

#define THR_SCALE_HI 0.90f // threshold บน (เข้มขึ้นเพื่อลดนับซ้ำ)
#define THR_SCALE_LO 0.40f // threshold ล่าง = สัดส่วนของ threshold บน
#define REFRACT_MS 450     // กันเด้ง 300ms
#define IBI_MIN_MS 350     // 60000/171BPM ~ 350ms  (กัน BPM สูงเกิน)
#define IBI_MAX_MS 2000    // 30 BPM

#define BPM_SMOOTH_A 0.85f // smooth: bpm = A*bpm + (1-A)*bpm_raw
#define BUZZER_VOLUME 0.5f //[0.01-0.001]

#define NOTE_B3 247
#define NOTE_C4 700
#define TIM_FREQ 60000000
static volatile uint8_t beep_req = 0;    // ขอให้บี๊บ 1 ครั้ง
static volatile uint32_t beep_until = 0; // เวลาที่จะหยุดบี๊บ
#define BEEP_MS 60
int now = 0;
int prev = 0; // ระยะเวลาบี๊บ 60ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int Read_Mic_Polling(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// char rx_data[1] = 'A';
char state_str[20] = "idle"; // ตั้งค่าเริ่มต้นกันเหนียว
void Update_State_String(SystemState_t state){
	switch(state)
	    {
	        case STATE_IDLE:       sprintf(state_str, "IDLE");       break;
	        case STATE_RECORDING:  sprintf(state_str, "RECORDING");  break;
	        case STATE_PROCESSING: sprintf(state_str, "PROCESSING"); break;
	        case STATE_RESULT:     sprintf(state_str, "RESULT");     break;
	        default:               sprintf(state_str, "UNKNOWN");    break;
	    }
}


int presForFrequency(int frequency)
{
  if (frequency <= 0)
    return 0;
  /* PSC = TIM_FREQ / ((ARR+1) * f) - 1 */
  return (int)((TIM_FREQ / ((uint64_t)(htim1.Init.Period + 1) * (uint64_t)frequency)) - 1);
}
void Calculate_Baseline(void)
{
    if (bpm_count_base < 2) {
        baseline_mean = 80.0f; // ค่า Default กัน Error
        baseline_std = 5.0f;
        return;
    }

    // Mean (u)
    float sum = 0.0f;
    for(int i=0; i<bpm_count_base; i++) {
    	sum += bpm_buffer[i];
    }
    baseline_mean = sum / bpm_count_base;

    // StdDev (sigma)
    float sum_sq = 0.0f;
    for(int i=0; i<bpm_count_base; i++) {
        float diff = bpm_buffer[i] - baseline_mean;
        sum_sq += diff * diff;
    }
    baseline_std = sqrt(sum_sq / bpm_count_base);

    if(baseline_std == 0) baseline_std = 1.0f;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  now = HAL_GetTick();
  if (now - prev >= 50)
  {
    prev = now;
    if (currentState == STATE_IDLE)
    {

      Calculate_Baseline();
      currentState = STATE_RECORDING;
    }

  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) return;

  // อ่านค่า ADC จาก DMA Buffer
  // เราอ่านมารอไว้ก่อน แต่จะยังไม่คำนวณถ้าเวลายังไม่ถึง
  uint16_t raw_mic = (uint16_t)adc_buffer[0];
  uint16_t raw_heart = (uint16_t)adc_buffer[1];

  t_ms = HAL_GetTick();

  // ============================================================
  // [FIX] LIMIT CALCULATION RATE TO ~100Hz (Every 10ms)
  // ============================================================
  static uint32_t last_dsp_time = 0;

  // ถ้าผ่านไปไม่ถึง 10ms ให้ข้ามการคำนวณไปเลย (เพื่อลดความเร็ว Loop)
  if (t_ms - last_dsp_time < 10)
  {
      return;
  }
  last_dsp_time = t_ms; // อัปเดตเวลาล่าสุดที่คำนวณ

  // --- จากจุดนี้ไปคือ Code เดิมของคุณ ---
  // พอเราจำกัดเวลาเข้า Loop นี้ให้เหลือ 10ms ต่อครั้ง
  // สูตร ALPHA_DC และ ALPHA_LP ของคุณจะกลับมาทำงานได้ถูกต้องทันทีครับ

  // ==========================================
  // PART A: MICROPHONE LOGIC (Talking Detect)
  // ==========================================

  // 1. Update Min/Max
  if (raw_mic > mic_max) mic_max = raw_mic;
  if (raw_mic < mic_min) mic_min = raw_mic;

  // 2. Every 50ms check the volume
  // (Logic เดิมของคุณตรงนี้โอเคแล้ว แต่มันจะทำงานแม่นขึ้นเพราะ Loop นิ่งขึ้น)
  if (t_ms - mic_last_reset > 50)
  {
    mic_val = mic_max - mic_min;
    if(currentState == STATE_RECORDING ){
    	if (mic_val > MIC_THRESHOLD)
    	    {
    	      is_talking = 1;
    	      silence_timer = t_ms;
    	      flag = 1;
    	    }
    	else
    	    {
    	      if (t_ms - silence_timer > HANG_TIME_MS && flag != 0)
				  {
					is_talking = 0;
					flag = 0;
					currentState = STATE_PROCESSING;
				  }
    	    }
    }

    mic_max = 0;
    mic_min = 4095;
    mic_last_reset = t_ms;
  }

  // ==========================================
  // PART B: HEARTBEAT LOGIC
  // ==========================================

    float rawf = (float)raw_heart;
    float err = rawf - dc;
    dc += ALPHA_DC * err;
    float sig = rawf - dc;
    y += ALPHA_LP * (sig - y);

    float ay = (y >= 0) ? y : -y;
    thr_ema = 0.995f * thr_ema + 0.005f * ay;

    float thr_hi = THR_SCALE_HI * thr_ema;
    float thr_lo = THR_SCALE_LO * thr_hi;

    if (refractory && (t_ms - ref_start_ms) > REFRACT_MS)
    {
      refractory = 0;
    }

    float dy = y - y_prev;

    if (!refractory)
    {
      if (!peak_armed)
      {
        if (y > thr_hi)
        {
          peak_armed = 1;
        }
      }
      else
      {
        if (y < thr_lo && dy < 0.0f)
        {
          uint32_t now = t_ms;
          if (last_peak_ms != 0)
          {
            uint32_t ibi = now - last_peak_ms;

            // [OPTIONAL] ปรับช่วง IBI ให้กว้างขึ้นนิดหน่อยเผื่อหัวใจเต้นเร็ว/ช้าผิดปกติ
            if (ibi >= 300 && ibi <= 2000)
            {
              float bpm_raw = 60000.0f / (float)ibi;
              bpm = BPM_SMOOTH_A * bpm + (1.0f - BPM_SMOOTH_A) * bpm_raw;

              if (currentState == STATE_IDLE)
              {
                  bpm_buffer[bpm_idx] = bpm;
                  bpm_idx = (bpm_idx + 1) % 30;
                  if(bpm_count_base < 30) bpm_count_base++;
               }
               // อย่าลืม Logic นับ Response ใน State Recording
               else if (currentState == STATE_RECORDING)
               {
                  response_sum += bpm;
                  response_count++;
               }
            }
          }
          last_peak_ms = now;

          beep_req = 1;
          beep_until = t_ms + BEEP_MS;

          refractory = 1;
          ref_start_ms = now;
          peak_armed = 0;
        }
      }
    }

    y_prev = y;

    // ส่ง UART ทุกครั้งที่คำนวณเสร็จ (ซึ่งตอนนี้คือทุก 10ms = 100Hz พอดี ไม่รกเกินไป)
//    int n = snprintf(uart_buf, sizeof(uart_buf),
//                       "%lu,%u,%s,%.1f,%lu,%u\r\n",
//                       t_ms, raw_heart, state_str, bpm, mic_val, is_talking);
//
    int n = snprintf(uart_buf, sizeof(uart_buf),"%s,%d,%.1f,%d\r\n",state_str,-1,bpm, is_talking);
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, n, 10);
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, n, 10); // ปิดอันนี้ถ้าไม่ได้ใช้ จะได้ไม่หน่วง
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
  int state_timer = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
//  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(htim2.Init.Period * BUZZER_VOLUME));

  if (HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2) != HAL_OK)
  {
      // If this fails, blink LED rapidly to warn user
      while(1) {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(100);
      }
  }

  printf("System Initialized.\r\n");
  printf("Current State: IDLE. Press Blue Button to Record.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(currentState)
	  	    {
	  	        case STATE_IDLE:       sprintf(state_str, "idle");       break;
	  	        case STATE_RECORDING:  sprintf(state_str, "recording");  break;
	  	        case STATE_PROCESSING: sprintf(state_str, "processing"); break;
	  	        case STATE_RESULT:     sprintf(state_str, "result");     break;
	  	        default:               sprintf(state_str, "default");    break;
	  	    }
    uint32_t now = HAL_GetTick();
    if (beep_req)
        {
          if (now < beep_until)
          {
            // 1. ตั้งความถี่
            __HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(250));
            __HAL_TIM_SET_COUNTER(&htim2, 0);

            // 2. [เพิ่ม] เปิดเสียง (Volume 50%)
          }
          else
          {
            // 1. [สำคัญ] ปิดเสียง (Volume 0%)
        	__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0));
        	__HAL_TIM_SET_COUNTER(&htim2, 0);

            beep_req = 0;
          }
        }
    switch (currentState)
    {
    /* --- STATE 1: IDLE (วัดหัวใจทำงานอยู่เบื้องหลัง) --- */
      case STATE_IDLE:
    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // ปิดไฟ Recording

    	  // เปิดไฟ Idle
    	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

        break;

      /* --- STATE 2: RECORDING (วัดไมค์ IN2) --- */
      case STATE_RECORDING:
    	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // ดับไฟ Idle
    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // ติดไฟ Talking
    	  break;

      case STATE_PROCESSING:
    	HAL_ADC_Stop_DMA(&hadc1);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    	int n = snprintf(uart_buf, sizeof(uart_buf),"%s,%d,%.1f,%d\r\n", state_str,-1,0.0,0);
    	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, n, 10);
    	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, n, 10); // ปิดอันนี้ถ้าไม่ได้ใช้ จะได้ไม่หน่วง
    	HAL_Delay(100);
        if (state_timer == 0)
        {
          state_timer = now; // เริ่มจับเวลา
          // เริ่ม Interrupt วัดหัวใจ
//          HAL_ADC_Start_IT(&hadc1);
          // ไฟกระพริบถี่ๆ บอกสถานะ Processing
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);


        }


        // 2. รอจนครบ 10 วินาที (แบบไม่บล็อกโปรแกรมอื่น)
        if (now - state_timer > 10000)
        {
          // ครบ 10 วิ -> หยุดวัดหัวใจ
//          HAL_ADC_Stop_IT(&hadc1);

          // รีเซ็ต timer สำหรับรอบหน้า
          state_timer = 0;


          // ไปหน้าแสดงผล
          currentState = STATE_RESULT;
        }
        break;
      case STATE_RESULT:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

      float response_mean = 0;
    	if(response_count > 0){
    		response_mean = response_sum / response_count;
    	}

    	// Standard Error
    	float std_error = baseline_std / sqrt((float)response_count);
    	if(std_error == 0) {
    		std_error = 0.001f;
    	}

    	// Z-Score
    	float z_score = (response_mean - baseline_mean) / std_error;
    	if (z_score > 1.645f){
    		int n = snprintf(uart_buf, sizeof(uart_buf),"%s,%d,%.1f,%d\r\n", state_str,0,0.0,0);
    		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, n, 10); // ปิดอันนี้ถ้าไม่ได้ใช้ จะได้ไม่หน่วง
//    		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, n, 10);

    		__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(1000));
    		__HAL_TIM_SET_COUNTER(&htim2, 0);
    		HAL_Delay(1000);
    		__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0));
    		__HAL_TIM_SET_COUNTER(&htim2, 0);
    		HAL_Delay(1000);
    		__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(1000));
    		__HAL_TIM_SET_COUNTER(&htim2, 0);
    		HAL_Delay(1000);
    		__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0));
    		__HAL_TIM_SET_COUNTER(&htim2, 0);

    	}
    	else{
    		int n = snprintf(uart_buf, sizeof(uart_buf),"%s,%d,%.1f,%d\r\n", state_str,1,0.0,0);
//    		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, n, 10);
    		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, n, 10); // ปิดอันนี้ถ้าไม่ได้ใช้ จะได้ไม่หน่วง
    		__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(250));
    		__HAL_TIM_SET_COUNTER(&htim2, 0);
    		HAL_Delay(1000);
    		__HAL_TIM_SET_PRESCALER(&htim2, presForFrequency(0));
    		__HAL_TIM_SET_COUNTER(&htim2, 0);
    		HAL_Delay(1000);

    	}
    	HAL_Delay(5000);


    	HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);
        currentState = STATE_IDLE;
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 839;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 839;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
