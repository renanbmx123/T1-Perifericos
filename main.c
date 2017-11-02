/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fmc.h"

/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_ts.h"
#include "string.h"
#include "LCD_GRAPH.h"
#include "SHT15.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t raw_humi, raw_temp;
uint8_t count = 0;
float humi_val_real = 0.0;
float temp_val_real = 0.0;
float ad=0, X=0, vel;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	enum{Tplus,Tminus,Uplus,Uminus,Vel};
	uint8_t vetor[30]
					,tMax
					,tMin
					,Umax
					,Umin
					,vMax
					,select;
	int color[5];


	TS_StateTypeDef TsState;
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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER,LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER,LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_TS_Init(240, 320);
  ADC_ChannelConfTypeDef sConfig;

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);


  sConfig.Channel = ADC_CHANNEL_13;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  tMax = 30;
  tMin = 15;
  Umax = 75;
  Umin = 50;
  vMax = 50;
  select = Tplus;

  color[0] = LCD_COLOR_BLUE;
  color[1] = LCD_COLOR_BLUE;
  color[2] = LCD_COLOR_BLUE;
  color[3] = LCD_COLOR_BLUE;
  color[4] = LCD_COLOR_BLUE;
  SHT15_Init();
  BSP_LCD_SetTextColor(LCD_COLOR_DARKYELLOW);
  BSP_LCD_DrawHLine(10,40,220);

  BSP_LCD_SetTextColor(LCD_COLOR_DARKYELLOW);
  BSP_LCD_DrawHLine(10,250,220);

  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_DrawBitmap(160,120,(uint8_t*)Up);
  BSP_LCD_DrawBitmap(160,160,(uint8_t*)Down);
  BSP_LCD_DrawRect(160,159, 25,25);
  BSP_LCD_DrawRect(160,119, 25,25);

  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  sprintf((char*)vetor,"      Alarmes");
  BSP_LCD_DisplayStringAtLine(16,(uint8_t*)vetor);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if(count == 10){
	SHT15_Measure(&raw_temp, TEMP);
	SHT15_Measure(&raw_humi, HUMI);
	SHT15_Calculate(raw_temp, raw_humi, &temp_val_real, &humi_val_real); //Get the temperature measurement
	}
	ad=(X*3)/4095;
 	vel=ad*(100/3);

 	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	sprintf((char*)vetor,"T:%2.1f U:%2.1f%% V:%.2f",temp_val_real, humi_val_real, vel);
	BSP_LCD_DisplayStringAtLine(0,(uint8_t*)vetor);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	sprintf((char*)vetor,"T:%2.1f U:%2.1f%% V:%.2f",temp_val_real, humi_val_real, vel);
	BSP_LCD_DisplayStringAtLine(0,(uint8_t*)vetor);

	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(color[0]);
	sprintf((char*)vetor,"T.MAX=%2.1d",tMax);
	BSP_LCD_DisplayStringAtLine(3,(uint8_t*)vetor);
	BSP_LCD_SetTextColor(color[1]);
	sprintf((char*)vetor,"T.MIN=%2.1d",tMin);
	BSP_LCD_DisplayStringAtLine(5,(uint8_t*)vetor);
	BSP_LCD_SetTextColor(color[2]);
	sprintf((char*)vetor,"U.MAX=%2.1d",Umax);
	BSP_LCD_DisplayStringAtLine(7,(uint8_t*)vetor);
	BSP_LCD_SetTextColor(color[3]);
	sprintf((char*)vetor,"U.MIN=%2.1d",Umin);
	BSP_LCD_DisplayStringAtLine(9,(uint8_t*)vetor);
	BSP_LCD_SetTextColor(color[4]);
	sprintf((char*)vetor,"V.MAX=%dm/s",vMax);
	BSP_LCD_DisplayStringAtLine(11,(uint8_t*)vetor);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

	BSP_LCD_SetFont(&Font24);
//	sprintf((char*)vetor,"x:%d y:%d ad:%2.2f",TsState.X,TsState.Y,ad);
//	BSP_LCD_DisplayStringAtLine(18,(uint8_t*)vetor);

	/*------------------------------------------------*/
	/* --- Change color of alarm*/
	/*------------------------------------------------*/
	if((humi_val_real > Umax) | (humi_val_real < Umin) ){
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_DisplayChar(23,280,'U');
		BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
		BSP_LCD_DrawCircle(50,290,11);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillCircle(50,290,10);
	}
	else{
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_DisplayChar(23,280,'U');
		BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
		BSP_LCD_DrawCircle(50,290,11);
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_FillCircle(50,290,10);
	}
	if((temp_val_real > tMax) | (temp_val_real < tMin) ){
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_DisplayChar(93,280,'T');
		BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
		BSP_LCD_DrawCircle(120,290,11);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillCircle(120,290,10);
	}
	else{
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_DisplayChar(93,280,'T');
		BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
		BSP_LCD_DrawCircle(120,290,11);
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_FillCircle(120,290,10);
	}
	if(vel  >  vMax ){
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			BSP_LCD_DisplayChar(153,280,'V');
			BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
			BSP_LCD_DrawCircle(180,290,11);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_FillCircle(180,290,10);
		}
		else{
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			BSP_LCD_DisplayChar(153,280,'V');
			BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
			BSP_LCD_DrawCircle(180,290,11);
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_FillCircle(180,290,10);
		}


	/*------------------------------------------------*/

	if((TsState.X > 0) & (TsState.X < 140)){
		if ((TsState.Y >50) & (TsState.Y < 80)){
			color[1] = LCD_COLOR_BLUE;
	  		 color[2] = LCD_COLOR_BLUE;
	  		 color[3] = LCD_COLOR_BLUE;
	  		 color[4] = LCD_COLOR_BLUE;
	  		 color[0] = LCD_COLOR_RED;
	  		 select = Tplus;
		}
		else if ((TsState.Y >90) & (TsState.Y < 110)){
			color[0] = LCD_COLOR_BLUE;
	  		color[2] = LCD_COLOR_BLUE;
	  		color[3] = LCD_COLOR_BLUE;
	  		color[4] = LCD_COLOR_BLUE;
		  	color[1] = LCD_COLOR_RED;
	  		select = Tminus;
		}
		else if ((TsState.Y >140) & (TsState.Y < 150)){
			color[0] = LCD_COLOR_BLUE;
			color[1] = LCD_COLOR_BLUE;
			color[3] = LCD_COLOR_BLUE;
			color[4] = LCD_COLOR_BLUE;
			color[2] = LCD_COLOR_RED;
			select = Uplus;
		}
		else if ((TsState.Y >140) & (TsState.Y < 190)){
			color[0] = LCD_COLOR_BLUE;
			color[1] = LCD_COLOR_BLUE;
			color[2] = LCD_COLOR_BLUE;
			color[4] = LCD_COLOR_BLUE;
			color[3] = LCD_COLOR_RED;
			select = Uminus;
		}
		else if((TsState.Y >210) & (TsState.Y < 240)){
			color[0] = LCD_COLOR_BLUE;
			color[1] = LCD_COLOR_BLUE;
			color[2] = LCD_COLOR_BLUE;
			color[3] = LCD_COLOR_BLUE;
			color[4] = LCD_COLOR_RED;
			select = Vel;
		}
	}

	if(TsState.TouchDetected ==1)	{
		if((TsState.X > 140) & (TsState.X < 210)) 	{
			if ((TsState.Y >150) & (TsState.Y <180))	{
				if(select == Tplus)
					tMax--;
				if(select == Tminus)
					tMin--;
				if(select == Uplus)
					Umax--;
				if(select == Uminus)
					Umin--;
				if(select == Vel)
					vMax--;
			}
		}
		if ((TsState.X > 140) & (TsState.X < 210)){
			if ((TsState.Y >115) & (TsState.Y <150)){
				if(select == Tplus)
					tMax++;
				if(select == Tminus)
					tMin++;
				if(select == Uplus)
					Umax++;
				if(select == Uminus)
					Umin++;
				if(select == Vel)
					vMax++;
			}
		}
	}

	 HAL_Delay(100);
	 BSP_TS_GetState(&TsState);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 216;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (count > 10)
		count = 0;
	count ++;
 	TIM2->CCR1 = ad*(100/3) - 2;
	 HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,ADC_CHANNEL_13);
	X = HAL_ADC_GetValue(&hadc1);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
