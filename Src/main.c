
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
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//#include <GFXC.h>
//#include <oled.h>
#include "ssd1306.h"

#define DEG_2_8 256.0
#define DEG_2_23 8388608.0
#define DEG_2_18 262144.0
#define DEG_2_5 32.0
#define DEG_2_17 131072.0
#define DEG_2_7 128.0
#define DEG_2_21 2097152.0
#define DEG_2_15 32768.0
#define DEG_2_33 8589934592.0

#define PRESSURE_OVERSAMPLING 100

/* Private variables ---------------------------------------------------------*/
//TextParamStruct TS;

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private function prototypes -----------------------------------------------*/

uint8_t write_byte(uint8_t data)
{

	uint8_t data_out;
    uint8_t read_data;

	// wait for spi transmitter readiness
	while ((SPI2->SR & SPI_SR_TXE) == RESET );
	data_out = data;
    SPI2->DR = data_out;
    // wait while a transmission complete
	while ((SPI2->SR & SPI_SR_RXNE) == RESET );
    read_data = SPI2->DR;
	
	return read_data;

	
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(void)
{

	int i,j,k;

	char message[256];
	char timestamp[64];

  	RTC_TimeTypeDef sTime;
  	RTC_DateTypeDef sDate;

  	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  	HAL_Init();

  	/* Configure the system clock */
  	SystemClock_Config();

  	/* Initialize all configured peripherals */
  	MX_GPIO_Init();
  	MX_SPI2_Init();
	// enable spi2
	SPI2->CR1 |= SPI_CR1_SPE;
  	MX_I2C1_Init();
  	MX_USART1_UART_Init();
  	MX_SPI3_Init();
	// enable spi3
	SPI3->CR1 |= SPI_CR1_SPE;
  	MX_RTC_Init();
  	MX_TIM1_Init();
  	MX_TIM2_Init();
  	MX_TIM3_Init();
  	MX_TIM4_Init();
  	MX_USART2_UART_Init();

  	HAL_GPIO_WritePin(GPIOA, led0_Pin, GPIO_PIN_RESET); // turn led on
  	HAL_GPIO_WritePin(led_tft_GPIO_Port, led_tft_Pin, GPIO_PIN_SET); // turn tft led on
  	HAL_GPIO_WritePin(reset_tft_GPIO_Port, reset_tft_Pin, GPIO_PIN_SET); // tft reset high 
	
	/*
	ILI9163Init();
	ClrScrn();
	TS.Size = 1;
	TS.Font = StdFont;
	TS.XPos = 0;
	TS.YPos = 0;
	TS.TxtCol = Blue;
	TS.BkgCol = Black;

	PStr("Hello!", &TS);
	*/

	/*
	OLED_init();
	LCD_Clear();
	LCD_Goto(0,0);
	OLED_string("DIVE COMPUTER");
	LCD_Goto(0,2);
	OLED_string("STARTING...");
	*/


  	ssd1306_Init();
  	HAL_Delay(1000);
  	ssd1306_Fill(White);
  	ssd1306_UpdateScreen();
  	HAL_Delay(1000);
  	ssd1306_Fill(Black);
  	ssd1306_UpdateScreen();

  	HAL_Delay(1000);

  	ssd1306_SetCursor(0,0);
  	ssd1306_WriteString("DiveCmp", Font_16x26, White);
  	ssd1306_SetCursor(0,30);
  	ssd1306_WriteString("Start..", Font_16x26, White);
  	ssd1306_UpdateScreen();
	
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	uint8_t spi2_out_data_buffer[128];
	uint8_t spi2_in_data_buffer[128];

	uint8_t data_out;
    uint8_t read_data;


	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//                 RESET
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// reset spi2 cs pin
    spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi2_cs_pressure_Pin << 16); 	// reset
	// transmit 0x1e                             	
	write_byte( 0x1e);                         	
	// set spi2 cs pin                           	
    spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi2_cs_pressure_Pin ;	// set
	HAL_Delay(3);
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	uint16_t sensor_prom[7];

	for(i=1; i<7; i++)
	{
		//send read prom command
		// reset spi2 cs pin
    	spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi2_cs_pressure_Pin << 16); 	// reset
		// transmit command with address 
		write_byte( 0xa0 + (((uint8_t)i)<<1));

		// read ms byte
		sensor_prom[i] = write_byte(0x55);
		sensor_prom[i] <<= 8;
		// read ls byte
		sensor_prom[i] += write_byte(0x55);

		// set spi2 cs pin
    	spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi2_cs_pressure_Pin ;	// set
	}


	uint32_t pressure;
	uint32_t temperature;
	double dT;
	double actual_temperature;
	double OFF;
	double SENS;
	double P;

	HAL_Delay(1000);
  	
	ssd1306_Fill(Black);
  	ssd1306_UpdateScreen();

	while (1)
	{


		uint32_t aux_p = 0;

		for(i=0; i<PRESSURE_OVERSAMPLING; i++)
		{

			//send start conversion D1 OSR 1024 command
		    // reset spi2 cs pin
    		spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi2_cs_pressure_Pin << 16); 	// reset
		    // transmit command  
		    write_byte(0x44);
		    // set spi2 cs pin
    		spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi2_cs_pressure_Pin ;	// set
		    // pause 3 mS
		    HAL_Delay(3);
                                                         
		    //send read adc command
		    // reset spi2 cs pin
    		spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi2_cs_pressure_Pin << 16); 	// reset
		    // transmit command 
		    write_byte(0x00);
                                                         
		    // read ms byte
		    pressure = write_byte(0x55);
		    pressure <<= 8;
		    // read ls byte
		    pressure += write_byte(0x55);
		    pressure <<= 8;
		    // read ls byte
		    pressure += write_byte(0x55);
		    // set spi2 cs pin
    		spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi2_cs_pressure_Pin ;	// set

			aux_p += pressure;

		}

		pressure = aux_p/PRESSURE_OVERSAMPLING;

		//----------------------------------------------------
		
		//send start conversion D2 OSR 1024 command
		// reset spi2 cs pin
    	spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi2_cs_pressure_Pin << 16); 	// reset
		// transmit command  
		write_byte(0x54);
		// set spi2 cs pin
    	spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi2_cs_pressure_Pin ;	// set
		// pause 3 mS
		HAL_Delay(3);

		//send read adc command
		// reset spi2 cs pin
    	spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi2_cs_pressure_Pin << 16); 	// reset
		// transmit command 
		write_byte(0x00);

		// read ms byte
		temperature = write_byte(0x55);
		temperature <<= 8;
		// read ls byte
		temperature += write_byte(0x55);
		temperature <<= 8;
		// read ls byte
		temperature += write_byte(0x55);
		// set spi2 cs pin
    	spi2_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi2_cs_pressure_Pin ;	// set

		//---------------------------------------------------

		dT = (double)temperature - (double)sensor_prom[5]*DEG_2_8;
		actual_temperature = 2000 + (dT*((double)sensor_prom[6]))/DEG_2_23;

		OFF = ((double)sensor_prom[2])*DEG_2_18 + (((double)sensor_prom[4])*dT)/DEG_2_5;
		SENS = ((double)sensor_prom[1])*DEG_2_17 + (((double)sensor_prom[3])*dT)/DEG_2_7;


		double T2;
		double SENS2;
		double OFF2;


		if(actual_temperature >= 2000)
		{
			T2 = 0;
			SENS2 = 0;
			OFF2 = 0;
		}
		else 
		{
			T2 = 3.0 * dT * dT / DEG_2_33;
			double aux_dt = (actual_temperature - 2000);
			OFF2 = 3.0 * aux_dt * aux_dt / 8.0;
			SENS2 = 7.0 * aux_dt * aux_dt / 8.0;

			if(actual_temperature < -1500)
			{
				double aux_dt = actual_temperature + 1500;
				SENS2 = SENS2 + 3.0 * aux_dt * aux_dt;
			}
		}

		actual_temperature = actual_temperature - T2;
		
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;

		P = (((double)pressure*SENS)/DEG_2_21 - OFF)/DEG_2_15;
  	
	
  		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

		sprintf(timestamp, "%02x.%02x.%02x %02x:%02x:%02x   ", sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);
		HAL_UART_Transmit(&huart1, timestamp, strlen((const char *)timestamp), 500);
		

		sprintf(message, "press %06d   temp %04d\r\n", (int32_t)P, (int32_t)actual_temperature);
		//sprintf(message, "press = %u;   temp = %u;\r\n", pressure, temperature);
		HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);


		//LCD_Clear();
		//*
  		//ssd1306_SetCursor(0,0);
		//sprintf(timestamp, "%02x.%02x.%02x", sDate.Date, sDate.Month, sDate.Year);
  		//ssd1306_WriteString(timestamp, Font_11x18, White);
  		ssd1306_SetCursor(0,0);
		sprintf(timestamp, "%02x:%02x:%02x", sTime.Hours, sTime.Minutes, sTime.Seconds);
  		ssd1306_WriteString(timestamp, Font_11x18, White);
  		ssd1306_SetCursor(0,22);
		sprintf(message, "P %06d", (int32_t)P);
  		ssd1306_WriteString(message, Font_11x18, White);
  		ssd1306_SetCursor(0,44);
		sprintf(message, "T %04d", (int32_t)actual_temperature);
  		ssd1306_WriteString(message, Font_11x18, White);
  		ssd1306_UpdateScreen();

		HAL_GPIO_TogglePin(GPIOA, led0_Pin);
		//*/
		HAL_Delay(100);


	}

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
