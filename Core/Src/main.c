/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int black=0;
int white=0;
int value=0;
int counter=0;
int a=0;
int b=0;
int enableB=0;
int pwmA=0;
int pwmB=0;
int *t;
int tmax;
int tmin=100000;
int arr[2];
int black1;
int white1;
int k;
int v;
int sum1;
int sum2;
int SensorError;
int * t ;
int * number;
int sens1;
int sens2;
int sens3;
int sens4;
int sens5;
int sens6;
int sens7;
int sens8;
int meanValue;
int keep;
int pid=0;
int p=0;
int i=0;
int d=0;
int pidErr;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int MeanSquareError();
int getValue(void);
int go(int enableA,int enableB);
void stop();
int MaxAndMin(int array[]);
int PID (int kp, int ki, double kd);
//int * getArrayValues(  );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init(void){
	
	//get measurment from floor
	//when engine is moving try to catch the biggest number and smallest number
	//go(20,0);
		HAL_GPIO_WritePin(GPIOE,InA_High_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,InA_Low_Pin,GPIO_PIN_RESET);
	
		HAL_GPIO_WritePin(GPIOA,InB_High_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,InB_Low_Pin,GPIO_PIN_RESET);
	
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		//number=getArrayValues();
		//MaxAndMin(number);
		arr[0] = black;
		arr[1] = white;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  //number=getArrayValues();
		//MaxAndMin(number);
		arr[0] = black1;
		arr[1] = white1;
		if(black>black1){
			black=black;
		}else{
			black=black1;
		}
		if(white>white1){
			white=white;
		}else{
			white=white1;
		}
}
int go(int enableA, int enableB){
	HAL_GPIO_WritePin(GPIOE,InA_High_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOE,InA_Low_Pin,GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(GPIOA,InB_High_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,InB_Low_Pin,GPIO_PIN_RESET);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,enableA);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,enableB);
	return 0;
	
}
int back(void){
	return 0;
}
void stop(){
		
}

//int * getArrayValues( void ){
//	int *arrForAll=(int*)malloc(sizeof(int)*8);
//	int Leds[8];
//	Leds[0]=Led_1_Pin;
//	Leds[1]=Led_2_Pin;
//	Leds[2]=Led_3_Pin;
//	Leds[3]=Led_4_Pin;
//	Leds[4]=Led_5_Pin;
//	Leds[5]=Led_6_Pin;
//	Leds[6]=Led_7_Pin;
//	Leds[7]=Led_8_Pin;
//	//initial values from the floor
//	for(int i=0;i<sizeof(arrForAll);i++){
//		arrForAll[i]=getValue(Leds[i]);
//	}
//	return arrForAll;
//}
int MaxAndMin(int array[]){
	v=0;
	arr[0]=0;
	k=0;
	arr[1]=0;
	int * t = (int*)malloc(sizeof(int)*8);
	t=array;
	for(int i=0;i<8;i++){
		for(int j=0;j<7;j++){
			if(t[i]>=t[j] && t[i]>=tmax){
				tmax=t[i];
				arr[0]=t[i];
				v=arr[0];
				if (v>=530000000 || v<=0 ){
					v=1000;
				}
			}else{
				break;
			}	
		}
		
	}
	for(int i=0;i<8;i++){
			for(int j=0;j<7;j++){
				if(t[i]<=t[j] && t[i]<=tmin){
					tmin=t[i];
					arr[1]=t[i];
					k=arr[1];
				}else{
					break;
			}	
		}
		
	}
}
int getValue(void){
	sens1=0;
  sens2=0;
  sens3=0;
  sens4=0;
  sens5=0;
  sens6=0;
  sens7=0;
  sens8=0;
	for(int i=0;i<5000;i++){
		if (HAL_GPIO_ReadPin (GPIOA, Led_1_Pin)==0) sens1++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_2_Pin)==0) sens2++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_3_Pin)==0) sens3++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_4_Pin)==0) sens4++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_5_Pin)==0) sens5++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_6_Pin)==0) sens6++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_7_Pin)==0) sens7++;
		if (HAL_GPIO_ReadPin (GPIOA, Led_8_Pin)==0) sens8++;}
		
		
	
	}	
	

int MeanSquareError(){
		// firtsly it takes all values. The middle one is the reference value for us
		// if the middle one is 0 and all others square distance to reference one
		// should be "0" if it is not zero that means there is some kind of error 
		// which is given by the formula : RSV(referenceSensorValue)=(Sensor4 + Sensor5)/2
		// 1/2*m(sum((RSV-Sensor[6,7,8])^2)(this is the for left part) - sum((RSV-Sensor[1,2,3])^2) which will decide which side robot will turn and
		// what is the instensity of the error if the error is zero which means robot goes very well)
		
		getValue();
		meanValue=(sens4+sens5)/2;
		sum1=(((sens1*8/1000)*3)^2)+(((sens2*4)*2/1000)^2)+(((sens3*2)/1000)^2);
		sum2=(((sens8*8/1000)*3)^2)+(((sens7*4)*2/1000)^2)+(((sens6*2)/1000)^2);
//			getValue();
//			if(sens4==0 && sens5==0){
//			
//			SensorError=5;
//			
//			}else if(sens4==0){
//					SensorError = 4;
//				if(sens3==0){
//					SensorError = 3;
//				}
//			
//			
//			}else if(sens3==0){
//				SensorError = 3;
//				if(sens2==0){
//					SensorError = 2;
//				}
//	
//			}else if(sens2==0){
//			SensorError = 2;
//				if(sens1==0){
//					SensorError = 1;
//				}
//			
//			}else if(sens1==0 || sens2!=0){
//			SensorError = 0;
//				
//			}
//			if(sens5==0){
//				SensorError = 5;
//				if(sens6==0){
//					SensorError = 6;
//				}
//			
//			}
//			if(sens6==0){
//			SensorError = 6;
//			if(sens7==0){
//					SensorError = 7;
//			}
//			}
//			if(sens7==0){
//			SensorError = 7;
//				if(sens8==0){
//					SensorError = 8;
//			}
//			}
//			if(sens8==0 || sens7!=0){
//			SensorError = 9;
//				
//			}
			SensorError=sum1-sum2;
			return SensorError;
		
		
		// if sum1 is bigger than sum2 the robot have to turn left otherwise 
		// it would turn right if the error is  much higher than we think pwm signal generator regulate itself
		// okay lets export error;
//		SensorError = (sum1 - sum2) * 1/2;
//	  if (sens8==0 && sens7!=0){
//			SensorError=-20000;
//			keep = sens8;
//		}
//		if (sens1==0 && sens2!=0){
//			SensorError=20000;
//			keep = sens1;
//		}
//			if (sens1!=0 && sens2!=0 && sens3!=0 && sens4!=0 && sens5!=0 && sens6!=0 && sens7!=0 && sens8!=0){
//				//keep where is the last zero place it is 8 or 1
//				if (keep == sens8){
//					SensorError=-30000;
//				}else if(keep == sens1){
//				SensorError=30000;
//				}
//			
//		}
		return SensorError;
}
int PID(int kp,	 int ki, double kd){
	MeanSquareError();
	if(pid==0){
		keep=0;
		i=0;
	}
	p=sum1/sum2;
	i=SensorError+i;
	d=SensorError-keep;
	keep=SensorError;
	pid=p*kp+i*ki+d*kd;
	return pid;
	
}
int selfLearnAlgo(int areaArray[]){
	//change Area arrays response to with respect to alpha values
	//in this section kp ki kd values will be selected by supervised algorithm 
	return 0 ;
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

  /* USER CODE BEGIN Ini1 */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,pwmA);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,pwmB);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(50);
		pidErr=PID(1,2,0.2);
		if(sens4==0){
			
			if(sens3 != 0){
				pidErr=0;
				pid=0;
			}
			}
		if(sens5==0){
			
			if(sens6 != 0){
				pidErr=0;
				pid=0;
			}
			}
		
		
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,2+pidErr);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,2-pidErr);
			
		
		
		
//		number=getArrayValues();
//		sens1=number[1];
//		sens2=number[2];
//		sens3=number[3];
//		sens4=number[4];
//		sens5=number[5];
//		sens6=number[6];
//		sens7=number[7];
//		sens8=number[8];
////		MaxAndMin(number);
////		MeanSquareError(number);
//		HAL_Delay(300);
		//init();
//		
//		while(HAL_GPIO_ReadPin(GPIOA,Led_1_Pin){
//			value++;
//		}
//		HAL_Delay(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, InA_High_Pin|InA_Low_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, InB_Low_Pin|InB_High_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Led_1_Pin Led_2_Pin Led_3_Pin Led_4_Pin
                           Led_5_Pin Led_6_Pin Led_7_Pin Led_8_Pin */
  GPIO_InitStruct.Pin = Led_1_Pin|Led_2_Pin|Led_3_Pin|Led_4_Pin
                          |Led_5_Pin|Led_6_Pin|Led_7_Pin|Led_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : InA_High_Pin InA_Low_Pin */
  GPIO_InitStruct.Pin = InA_High_Pin|InA_Low_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : InB_Low_Pin InB_High_Pin */
  GPIO_InitStruct.Pin = InB_Low_Pin|InB_High_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
