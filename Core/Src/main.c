/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <stdio.h>
#include <string.h>
#include "APA102.h"
#include "INA260.h"
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

/* USER CODE BEGIN PV */

extern uint8_t rxL[SEN_rxSize];//Left Sensor receive buffer
extern uint8_t rxR[SEN_rxSize];//Right Sensor receive buffer
extern uint8_t rxE[ESP_rxSize];//ESP receive buffer
extern volatile uint8_t LC_Ready, RC_Ready, ESP_Ready;
volatile uint8_t timeToRead, timeToSend;
extern uint8_t effectDone;
const uint16_t effectDuration = 1000;
//0 volts, 1 amps, 2 wats, 3 energy;
float measures[4];
uint8_t valToSend[txLen];
uint8_t cnt, prevPower, prevGain, prevBright;
//uint32_t tL, tR, tL1, tR1;

//#define TIME_OUT					500
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance==TIM2) {
		timeToRead = 1;
	} else if(htim->Instance==TIM3){
		timeToSend = 1;
  }
}

static void UpdateAPA(){
	// Update only if we have new data from ESP
	if(prevPower && ESP_Ready){
	//Set color from OpenHab interface when frame_MQTT[Effect] is 0
		if(!frame_MQTT[Effect]){
			APA_setAllColor(frame_MQTT[LeftRed], frame_MQTT[LeftGreen], frame_MQTT[LeftBlue]);
			APA_update(0);
			APA_setAllColor(frame_MQTT[RightRed], frame_MQTT[RightGreen], frame_MQTT[RightBlue]);
			APA_update(1);
		}
		//Set up LED brightness and sensor gain
		if(prevBright != frame_MQTT[Intensity]){
			prevBright = frame_MQTT[Intensity];
			APA_setAllBrightness(prevBright);
		}
		if(prevGain != frame_MQTT[Gain]){
			prevGain = frame_MQTT[Gain];
			uint8_t sendGain[3] = {sMarker, prevGain, eMarker};
			HAL_UART_Transmit(&huart1, sendGain, 3, TIME_OUT);
			HAL_UART_Transmit(&huart2, sendGain, 3, TIME_OUT);
		}
		ESP_Ready = 0;
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(10000);
//	frame_MQTT[Power] = 1;
//	frame_MQTT[Mode] = 0;
//	frame_MQTT[Effect] = 1;
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
  valToSend[0] = 60;
  valToSend[17] = 62;
  ina260_start();
//	if(ina260_ready() == HAL_OK){
//		printf("INA ready in %d!!!\r\n", HAL_GetTick()-time);
//	}
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  APA_init();
	
  //tL = tR = HAL_GetTick();
  prevPower = prevGain = prevBright = 0;
  //Wait for ESP8266 to connect to WiFi and MQTT server
  //When done it will send the data stored in EEPROM
//  while (!ESP_Ready){
//		HAL_Delay(10);  
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//The Power state has changed
		if(prevPower != frame_MQTT[Power]){
			prevPower = frame_MQTT[Power];
			//The power is OFF
			if(!prevPower){
				APA_AllOff();
				ESP_Ready = 0;
				//Stop DMAs to save power?
			}
		}
		if(frame_MQTT[Power]){
				//If Power is ON
				// We are in sensor mode
				if(frame_MQTT[Mode]){
					//We have data from left/Right side sensor
					//0, 1, 2 Top Sensor - near debug interface
					//6, 7, 8 Bottom Sensor - near case wall
						if(LC_Ready){
							//tL = HAL_GetTick() - tL1;
							for(cnt = 0; cnt < FirstThird; cnt++){
								APA_setColor(cnt, rxL[0], rxL[1], rxL[2]);
								APA_setColor(cnt + FirstThird, rxL[3], rxL[4], rxL[5]);
								APA_setColor(cnt + SecondThird, rxL[6], rxL[7], rxL[8]);
							}
							APA_update(0);
							LC_Ready = 0;
							//tL1 = HAL_GetTick();
						}
						if(RC_Ready){
						//tr = HAL_GetTick() - tR1;
							for(cnt = 0; cnt < FirstThird; cnt++){
								APA_setColor(cnt, rxR[0], rxR[1], rxR[2]);
								APA_setColor(cnt + FirstThird, rxR[3], rxR[4], rxR[5]);
								APA_setColor(cnt + SecondThird, rxR[6], rxR[7], rxR[8]);
							}
							APA_update(1);
							RC_Ready = 0;
							//tR1 = HAL_GetTick();
						}
				} else {
					// Run lights effects
					if(frame_MQTT[Effect]){
						//If an effect was completed
						if(effectDone){
							//We start a new one
							effectDone = 0;
							switch(frame_MQTT[Effect]){
								case 1:
									FluidRainbowCycle(effectDuration);
								break;
								case 2: 
									SnowEffect(50);
								break;
								case 3: 
									PulseEffect(effectDuration);
								break;
								case 4: 
									FlashingEffect(effectDuration);
								break;
								case 5: 
									FadingEffect(effectDuration);
								break;
								case 6: 
									ColorWipe(effectDuration);
								break;
								case 7: 
									TheaterChase(frame_MQTT[LeftRed], frame_MQTT[LeftGreen], frame_MQTT[LeftBlue], effectDuration);
								break;
								case 8: 
									BreathEffect(effectDuration);
								break; 
								case 9: 
									WaveEffect(effectDuration);
								break; 
							}				
						}
					} 
				}
			}
		// We have data from ESP (MQTT)
		UpdateAPA();
		
		//End of APA stuff / Begin of INA260
		
		// Read INA260 power values
		if(timeToRead){
			if(ina260_conversion_ready()){
				ina260_get_voltage(&measures[0]);
				ina260_get_current(&measures[1]);
				ina260_get_power(&measures[2]);
				//Transform Power to Energy
				measures[3] += measures[2] * sHour;
				timeToRead = 0;
			}
		}
		//Send energy measurements to ESP to be displayed in OpenHab
		//Send the values every 30s to avoid loading Zigbee network
		if(timeToSend){
			//4x floats = 16bytes
			memcpy(valToSend+1, measures, 16);
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&valToSend, txLen);
			timeToSend = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

#ifdef  USE_FULL_ASSERT
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
