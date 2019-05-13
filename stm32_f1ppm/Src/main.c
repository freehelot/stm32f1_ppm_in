/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * PPM Reader with DEBUG print RC channels
  * STM32F103C8T6 (BluePill) tested
  * This program gets PPM signal input from RC radio PPM out connection.
  * Timer 4 is used with chanels 1 and 2, connect PPM signal cable to PB6, GND to GND.
  * Debuged with ST-link, PB3 is SWdebug and reset is done via ST-link.
  * Up to 12 channels have been tested working with no problems on Taranis X9+, Turnigy 9x and arduino uno as ppm generator.
  * PPM signal triggers interrupts and channel 1 and channel 2 are used for detecting edges (rising or falling) and update channel values.
  * Counter reset of channels is done hardware wise since it's recommended over software reset (multiple interrupts, interrupt priority etc.)
  *
  *
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32_hal_legacy.h"
#include "stm32f1xx_hal_tim.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//number of ppm channels
//in future, add functionality to change number of channels with button/encoder or simply read

//channel length with STOP pulse, to be calculated in future, 30.5ms is for 12 channels
#define SIGNAL_LENGTH	30500
//max channels  to be calculated in future
#define MAX_PPM_CHANNELS (4)
//pulse lenght before timeout microsec, used to find STOP or to find tht the signal is busted
#define MAX_PULSE_WAIT_TIMEOUT_USEC (2500)

//ppm min and max will be used for checking PPM abnormalities and to round to nearest if the signal is lower/higher then we want
//#define PPM_MIN (950)
//#define PPM_MAX (2050)

//every channel init to 0
int PPM_CHANNEL_TIME[MAX_PPM_CHANNELS] = { 0 };
// indicates the system is just booted up
int is_first_start = 1;
// indicates that the first channel pulse is found
int is_in_sync = 0;
//edge which triggers value update/printf on every second edge(falling or rising)
int edge=1;
//current channel
int current_channel = 0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//if interrupt comes from TIM4 then do
	if(htim->Instance == TIM4)
	{

		//*****first iteration, not used in future, left for reference
		//uint32_t count= __HAL_TIM_GET_COUNTER(&htim4);
		//uint32_t usec = (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4)*10);
		//printf("%d\n",usec);

		//*******************************************************************************************************
		// multiplying everything with 10 since we have pulses every 10us, i find no reason to update every 1us
		//there can be errors in values ie. higher or lower value hence we allow 50-+ value for pause
		//

		//ppm_sig read the full signal value (0.3ms+Xms)min 1ms, max 2ms
		uint32_t ppm_sig=(HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1)*10);
		//ppm_pause reads the pause value which should be 0.3ms
		uint32_t ppm_pause=(HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2)*10);

		//***************************************************************************************
		//first iteration of the code, not working as intended, reset was done by software
		 //__HAL_TIM_SetCounter(&htim4, 0);
		//printf("Tim get compare=%d\n",usec);
		//printf("Tim counter=%d\n",count);
		//printf("SET COMPARE REGISTER: %d\r\n",__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,30));
		  //printf("Read captured value ch1: %d\r\n",HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1)*10);
		  //printf("Read captured value ch2: %d\r\n",HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2)*10);
		 // __HAL_TIM_SetCounter(&htim2, 0);
		// on first start just reset counter
		//*****************************************************************************************
		if(!is_first_start)			//checks for first start, not really needed but useful to avoid going through loops/checks if not needed in start
		{
			if(is_in_sync && edge)		//sync is checked with STOP pulse and edge is to make sure every second interrupt updates values and prints debug msg
			{
				//if(MAX_PULSE_WAIT_TIMEOUT_USEC <= usec && usec>350)
				//is max pulse timeout and ppm pause value is within acceptable parameters, it means we hit STOP
				//reset channel to 1
				if(MAX_PULSE_WAIT_TIMEOUT_USEC<=ppm_sig && (ppm_pause<350 || ppm_pause>250))
				{
					current_channel=1;
				}
				else
				//else the signal is channel value or invalid
				{
					//if ppm signal is longer then pause and less then timeout we can acquire the value and update channel
					if(ppm_sig>ppm_pause && ppm_sig<MAX_PULSE_WAIT_TIMEOUT_USEC)
					{

						PPM_CHANNEL_TIME[current_channel]=ppm_sig;
						//debug print msg, to be commented before use in future
						printf("PPM Ch time %d, current channel %d\n\n",PPM_CHANNEL_TIME[current_channel],current_channel);
						//increase current channel value since the next signal will be next channel
						current_channel++;

					//printf("PPM Ch time %d, current channel %d\n",PPM_CHANNEL_TIME[current_channel],current_channel);
					}

				}
			}
			else	//we wait for the first pulse
			{
				//is timeout is less then signal and less then whole length with
				if(MAX_PULSE_WAIT_TIMEOUT_USEC<=ppm_sig && (MAX_PULSE_WAIT_TIMEOUT_USEC<(SIGNAL_LENGTH+250)))
				{
					//the signal is in sync, we're receiving signal with time frame within acceptable parameters
					is_in_sync=1;

				}

				else if(MAX_PULSE_WAIT_TIMEOUT_USEC<=ppm_sig && (ppm_pause>350 || ppm_pause<250))
				{
					is_in_sync=0;
				}
			}
		}else{
			is_first_start=0;
		}
		//since interrupt is fired on every falling and rising edge, we don't need to update
		//values on both but only on one, hence we add variable edge that will simply
		//change value and update variables on every second interrupt
		if (edge==1){
			edge=0;
		}else
		{
			edge=1;
		}
		//disabled since we don't use tim counter, but individual channel values
		//__HAL_TIM_SET_COUNTER(&htim4,0);

	}
}
//int _write is used for debug by wire, needed for printing messages
int _write(int file, char *ptr, int len){
	int i;
	for(i = 0 ; i < len ; i++){
		ITM_SendChar((*ptr++));
	}
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //init channels
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);	//init channel 1 in TIM 4, enable interrupts
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);	//init channel 2 in TIM 4, enable interrupts


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
