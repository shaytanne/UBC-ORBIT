
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
/* Libraries included -----------------------------add more if needed--------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/****** USER CODE BEGIN Includes */
#define DEBUG 0 //a debugging tool to check if the flag is turning on and off properly
/****** USER CODE END Includes */

/* Private variables */
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/****** USER CODE BEGIN PV */
/* Private variables ----------------------------ours-----------------------------*/
volatile short int flag = 0;  //this gets turned on/off by the interrupt
/****** USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);


/****** USER CODE BEGIN PFP */
/* Private function prototypes ---------------------ours--------------------------*/
uint8_t getIndex(char letter);
void num_print(uint8_t data);
/****** USER CODE END PFP */


/****** USER CODE BEGIN 0 */
/*-------------------------------in case we want to add anything before main-------------------*/
/****** USER CODE END 0 */


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /****** USER CODE BEGIN ---------------------variable declarations---------------- */
  char *callSign = "UBCORBIT";
  char printdata[3];
  uint8_t i, j;       //i is iterator for looping through digits of a single letter
                      //j is iterator for looping through all letters of the callsign
  uint8_t newLetter;  //flags when a single letter is done, can move to next one
  uint32_t morse[] = {  //a table of binary morse values for different characters
      /*A*/ 0b101110              , /*B*/ 0b1110101010         , /*C*/ 0b111010111010    , /*D*/ 0b11101010      ,
      /*E*/ 0b10                  , /*F*/ 0b1010111010         , /*G*/ 0b1110111010      , /*H*/ 0b10101010      ,
      /*I*/ 0b1010                , /*J*/ 0b10111011101110     , /*K*/ 0b1110101110      , /*L*/ 1011101010      ,
      /*M*/ 0b11101110            , /*N*/ 0b111010             , /*O*/ 0b111011101110    , /*P*/ 0b101110111010  ,
      /*Q*/ 0b11101110101110      , /*R*/ 0b10111010           , /*S*/ 0b101010          , /*T*/ 0b1110          ,
      /*U*/ 0b10101110            , /*V*/ 0b1010101110         , /*W*/ 0b1011101110      , /*X*/ 0b111010101110  ,
      /*Y*/ 0b11101011101110      , /*Z*/ 0b111011101010       , 
      /*end of letters/////////////////////////////////////////////////////////////////////////////////////////*/
      /*0*/ 0b11101110111011101110, /*1*/ 0b101110111011101110 , /*2*/ 0b1010111011101110, /*3*/ 0b10101011101110,
      /*4*/ 0b101010101110        , /*5*/ 0b1010101010         , /*6*/ 0b101010101110    , /*7*/ 0b11101110101010,
      /*8*/ 0b1110111011101010    , /*9*/ 0b111011101110111010 , /*space*/ 0b0000};

  /* Other characters in morse- if needed 

      ['.'] = 101110101110101110,
      [','] = 11101110101011101110,
      ['?'] = 1010111011101010,
      ['\''] = 10111011101110111010,
      ['!'] = 11101011101011101110,
      ['/'] = 11101010111010,
      ['('] = 1110101110111010,
      [')'] = 11101011101110101110,
      ['&'] = 101110101010,
      [':'] = 111011101110101010,
      [';'] = 111010111010111010,
      ['='] = 11101010101110,
      ['+'] = 10111010111010,
      ['-'] = 1110101010101110,
      ['_'] = 101011101110101110,
      ['\\'] = 1011101010111010,
      ['$'] = 101010111010101110,
      ['@'] = 101110111010111010,
      [' '] = 1110111011101110,
      */
  /****** USER CODE END  */

  /* MCU Configuration*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /****** USER CODE BEGIN Init */
  /****** USER CODE END Init */


  /* Configure the system clock */
  SystemClock_Config();


  /****** USER CODE BEGIN SysInit */
  /****** USER CODE END SysInit */


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();


  /******* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_UART_Transmit(&huart2, "Startnew\n\r", 11, 100);

  //initialization of variables for the first iteration of the while loop
  flag      = 0;  //callsign transmission interrupt OFF
  newLetter = 1;  //prevents the transmission of the 2 zero separation between letter elements
  j         = 0;  //start from the first letter of the callsign
  /******* USER CODE END 2 */


  
  /******* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1) {
    if (flag) //the flag we set in the file "stm32f4xx_it.c" 
    {
      if(strlen(callSign) > j){   //each time the iterrupt is triggered:
                                  // if j is < than the length of the callsign - transmit a letter
        if(newLetter){    //flag for starting a new letter

          //values in the morse array are 32 bits each, so fills in zeros to the left of the actual values
          //the for loop skips all the zeros and makes sure the first digit evaluated is a 1
          for (i = 19; !(morse[getIndex(callSign[j])] >> i & 0x01); i--); 
          newLetter = 0;  //clears the new letter flag
        }
        #if DEBUG  //debugging tool, to print the current value of i (before writing) to gcc, change value of DEBUG at top of file 
        num_print(i);
        #endif

          //write to output to the LED
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, morse[getIndex(callSign[j])] >> i & 0x01); 


          //if/else  for writing 1 or 0 to the terminal
          //helpful for development, not part of the operation of the LED (transceiver in the future)
          if ((morse[getIndex(callSign[j])] >> i) & 0x01)//if specific digit is 1 or 0, writes accordignly
            HAL_UART_Transmit(&huart2, "1", 1, 100);
          else
            HAL_UART_Transmit(&huart2, "0", 1, 100);



          if (i == 0) //resets i to 32 once we reach the last digit in the letter
            i = 32; 

          if(j == strlen(callSign)-1){  //when reaching end of the callsign-do 7 iterations of writing 0
            if(i == 25){                //when i reaches 25 (after 7 cycles)
              newLetter = 1;            //sets the new letter flag ON for the first letter of the new word
              j = 0;                    //resets the letter counter 
            }
          } 
          else{               //the case when the word is not done yet, does 2 cycles of writing 0 (separation between letters)
            if (i == 30){     //when i reaches 30 (after 2 cycles)
              newLetter = 1;  //sets the new letter flag ON 
              j++;            //increments the letter counter (moves to the next letter)
            }
          }
            
          #if DEBUG //debugging tool, to print the current value of i (after writing) to gcc, change value of DEBUG in 
          num_print(i); 
          #endif

          i--; //moves to the next digit in a letter
        
      }
      
      flag = 0; //resets the interrupt flag so that it can be triggered again 

      #if DEBUG //debugging tool, to print the current value of i (after writing) to gcc, change value of DEBUG in
      HAL_UART_Transmit(&huart2, "Flag unset\n\r", 13, 100);
      #endif
    } //end of if (flag)
  }   //end of while loop
  /******* USER CODE END WHILE */

  /******* USER CODE BEGIN 3 */
  /*---------------------------------anything else we want to add in main, but not in the while loop-----------------------*/
  /******* USER CODE END 3 */
}

/*----------------------------------other functions used by this program or the board-------------------------------*/

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  CHANGE TIMING HERE
// ~TIMING CONFIGURATION~ START  //
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 65355;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period =64;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 6;
// ~TIMING CONFIGURATION~ END  //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  CHANGE TIMING HERE
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/******* USER CODE BEGIN 4 */
// Functions that we want to add go here

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CHANGE THIS FUNCTOIN IF NEEDED TO SUPPORT MORE CHARACTERS IN MORSE/ DIFFERENT ENCODING
/*Function that takes a character (usually part of a string) & return its index in a conversion array using ASCII
  Example:  (from this code) the array morse[] contains characters A-Z, 0-9 and space
  Param:    char - a character
  Return:   uint_8  - the character's index in the conversion array
  Note:     this function is modified to work with the morse[] array, would need to be adjusted to
            work with different arrays  */
uint8_t getIndex(char letter)
{
  if (letter >= 'A' && letter <= 'Z') //the case for CAPITAL letter char
    return (letter - 'A');
  else if( letter == ' ')             //the case for the SPACE character
    return (letter - 'A' + 27);
  else                                //the case for digit characters
    return (letter - 'A' + 26);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ USEFUL FOR DEBUGGING
void num_print(uint8_t data)
{
  uint8_t out[3];

  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  sprintf(out, "%d", data);

  HAL_UART_Transmit(&huart2, out, 3, 100);
  return;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ USEFUL FOR DEBUGGING
void num_print_hex(uint32_t data)
{
  uint32_t out[3];

  HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
  sprintf(out, "%x", data);

  HAL_UART_Transmit(&huart2, out, 3, 100);
  return;
}
/****** USER CODE END 4 */

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
