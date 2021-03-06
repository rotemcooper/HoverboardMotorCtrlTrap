/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include <stdarg.h>

//#include "hd44780.h"

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
//LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
extern void consoleLog(char *message);

int cmd1;  // normalized input values. -1000 to 1000
int cmd2;
int cmd3;

typedef struct{
   int16_t steer;
   int16_t speed;
   //uint32_t crc;
} Serialcommand;

volatile Serialcommand command;
//rotemc
#define UART_RX_BUFF_SIZE 128
volatile char uart_rx_buff[UART_RX_BUFF_SIZE];
unsigned int uart_rx_index = 0;
//rotemc

uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage; // global variable for battery voltage

uint32_t inactivity_timeout_counter;

extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

int milli_vel_error_sum = 0;

//rotemc
extern DMA_HandleTypeDef hdma_usart2_rx;
extern void machine_main(void);

//uint32_t _millis;
//uint32_t millis(void) {
//    return _millis;
//}

//-------------------------------------------------------------------------------------------

/* Read a a character fromUART/Serialcommand.
** Optimize to use uart_peek
*/
int uart_get_char( char* c ) {
	unsigned int dma_count;
	unsigned int rx_index = uart_rx_index;	

	// If no data received, return 0.
	dma_count =  UART_RX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	if( dma_count == rx_index ) {
		return 0;
	}

	*c = uart_rx_buff[rx_index++];
	uart_rx_index = rx_index % UART_RX_BUFF_SIZE;
	
	return 1;
}

//-------------------------------------------------------------------------------------------

int uart_peek( char* c ) {
	unsigned int dma_count;
	unsigned int rx_index = uart_rx_index;	

	// If no data received, return 0.
	dma_count =  UART_RX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	if( dma_count == rx_index ) {
		return 0;
	}

	*c = uart_rx_buff[rx_index];
	return 1;
}

//-------------------------------------------------------------------------------------------

int uart_printf( const char *format, ...) {
    static char buffer[256];
    va_list args;
    va_start (args, format);
    int len = vsprintf (buffer, format, args);
    HAL_UART_Transmit( &huart2, (uint8_t*) buffer, len, 200);
    va_end (args);
    return len;
}

//-------------------------------------------------------------------------------------------

void poweroff(void) {
    if (abs(speed) < 20) {
        buzzerPattern = 0;
        enable = 0;
        for (int i = 0; i < 8; i++) {
            buzzerFreq = i;
            HAL_Delay(100);
        }
        HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
        while(1) {
          char c;
          if( uart_get_char(&c) ) {
            // uart_printf( "%c", c );
            if( c == 'n') {
              HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);
              NVIC_SystemReset();
            }
            //HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);
            //return;
          }
        }
    }
}

//rotemc
extern uint64_t isr_cnt;
extern int motorl_dir;
extern int v_tbl[];
extern int w_tbl[];
extern int u_tbl[];

extern int vt_tbl[];
extern int wt_tbl[];
extern int ut_tbl[];
extern int offset_pull;

//rotemc
float board_temp_adc_filtered;
float board_temp_deg_c;


int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    UART_Init();
  #endif

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

//rotemc - wait for power button PCF_I2C_ADDRESS
//while ( !HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) )
//{}

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  //int lastSpeedL = 0, lastSpeedR = 0;
  //int speedL = 0, speedR = 0;
  //float direction = 1;

  #ifdef CONTROL_PPM
    PPM_Init();
  #endif

  #ifdef CONTROL_NUNCHUCK
    I2C_Init();
    Nunchuck_Init();
  #endif

  #ifdef CONTROL_SERIAL_USART2
    UART_Control_Init();
    //rotemc
    //HAL_UART_Receive_DMA(&huart2, (uint8_t *)&command, 4);
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)uart_rx_buff, UART_RX_BUFF_SIZE);
    //rotemc

  #endif

  #ifdef DEBUG_I2C_LCD
    I2C_Init();
    HAL_Delay(50);
    lcd.pcf8574.PCF_I2C_ADDRESS = 0x27;
      lcd.pcf8574.PCF_I2C_TIMEOUT = 5;
      lcd.pcf8574.i2c = hi2c2;
      lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
      lcd.type = TYPE0;

      if(LCD_Init(&lcd)!=LCD_OK){
          // error occured
          //TODO while(1);
      }

    LCD_ClearDisplay(&lcd);
    HAL_Delay(5);
    LCD_SetLocation(&lcd, 0, 0);
    LCD_WriteString(&lcd, "Hover V2.0");
    LCD_SetLocation(&lcd, 0, 1);
    LCD_WriteString(&lcd, "Initializing...");
  #endif

  //rotemc
  board_temp_adc_filtered = (float)adc_buffer.temp;
  //float board_temp_deg_c;

  enable = 1;  // enable motors
  //_millis = HAL_GetTick(); //rotemc
  
  machine_main();
  return 0;

  //-----------------------------------------------------------------------
  

  //int self_test = 0;
  while(1) {
    HAL_Delay( DELAY_IN_MAIN_LOOP ); //delay in ms

    /*
    if( self_test > 0 ) {
      self_test--;
      if( self_test == 0 ) {
        pwml = 0;
      }
      else if( self_test%10 == 0 ) {
        if( self_test > 9000 ) {
          pwml = CLAMP( pwml-10, -350, 0);
        }
        else if( self_test > 8000 ) {
          pwml = CLAMP( pwml+10, -350, 0);
        }
        else if( self_test > 7000 ) {
          pwml = CLAMP( pwml-10, -350, 0);
        }
        else if( self_test > 6000 ) {
          pwml = CLAMP( pwml+10, -350, 0);
        }
      }
    }
    */

/*
    #ifdef CONTROL_NUNCHUCK
      Nunchuck_Read();
      cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis. Nunchuck joystick readings range 30 - 230
      cmd2 = CLAMP((nunchuck_data[1] - 128) * 8, -1000, 1000); // y - axis

      button1 = (uint8_t)nunchuck_data[5] & 1;
      button2 = (uint8_t)(nunchuck_data[5] >> 1) & 1;
    #endif

    #ifdef CONTROL_PPM
      cmd1 = CLAMP((ppm_captured_value[0] - 500) * 2, -1000, 1000);
      cmd2 = CLAMP((ppm_captured_value[1] - 500) * 2, -1000, 1000);
      button1 = ppm_captured_value[5] > 500;
      float scale = ppm_captured_value[2] / 1000.0f;
    #endif

    #ifdef CONTROL_ADC
      // ADC values range: 0-4095, see ADC-calibration in config.h
      cmd1 = CLAMP(adc_buffer.l_tx2 - ADC1_MIN, 0, ADC1_MAX) / (ADC1_MAX / 1000.0f);  // ADC1
      cmd2 = CLAMP(adc_buffer.l_rx2 - ADC2_MIN, 0, ADC2_MAX) / (ADC2_MAX / 1000.0f);  // ADC2

      // use ADCs as button inputs:
      button1 = (uint8_t)(adc_buffer.l_tx2 > 2000);  // ADC1
      button2 = (uint8_t)(adc_buffer.l_rx2 > 2000);  // ADC2

      timeout = 0;
    #endif

    #ifdef CONTROL_SERIAL_USART2
      cmd1 = CLAMP((int16_t)command.steer, -1000, 1000);
      cmd2 = CLAMP((int16_t)command.speed, -1000, 1000);
     */

      char output[32];
      char c;      
      if( uart_get_char(&c) > 0 ) {
        //rotemc HAL_UART_Transmit( &huart2, (uint8_t*)&c, sizeof(c), 200 );
        switch( c )
        {
          /*
          case '+':
            pwml += 10;
			      break;
            
          case '-':
            pwml -= 10;
			      break;
			  
		      case '0':
            pwml = 0;
		       	break;
          
          case 't':
            self_test = 10000;            
            break;
          
          case 'x':
            self_test = 0;
            pwml=0;            
            break;
          */
          case 'm':
            {
              char motor;
              char terminator;
              uint8_t LSBspeed;
              uint8_t MSBspeed;
              int16_t speed;
              uart_get_char( &motor );
              uart_get_char( (char*) &LSBspeed );
              uart_get_char( (char*) &MSBspeed );
              uart_get_char( &terminator );
              speed = (int16_t) ((((uint16_t) MSBspeed) << 8) | (uint16_t) LSBspeed);            
              //sprintf( output, " received m%c %o, %o, %d\n, ", motor, LSBspeed, MSBspeed, speed);
              //HAL_UART_Transmit( &huart2, output, strlen(output), 200);
              if( terminator == 'e' && motor == 'l' && speed > -800 && speed < 800 ) {
                pwml = speed;
              }
            }
            break;

          case 's':
          case 'q':
            poweroff();
            break;
          /*
          case 'a':
            offset_pull++;
            sprintf( output, " offset_pull = %d\n, ", offset_pull );
            HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            break;
          
          case 'z':
            offset_pull--;
            sprintf( output, " offset_pull = %d\n, ", offset_pull );
            HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            break;
          
          case 'd':
            sprintf( output, " Direction = %d\n, ", motorl_dir );
            HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            break;
          
          case 'p':
             HAL_UART_Transmit( &huart2, "\nN, ", 4, 200 );            
            for( int h=0; h<96; h++ ) {
              sprintf( output, "%d, ", h );
              HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            }

            HAL_UART_Transmit( &huart2, "\nv, ", 4, 200 );            
            for( int i=0; i<96; i++ ) {
              sprintf( output, "%d, ", v_tbl[i] );
              HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            }

            HAL_UART_Transmit( &huart2, "\nw, ", 4, 200 );            
            for( int j=0; j<96; j++ ) {
              sprintf( output, "%d, ", w_tbl[j] );
              HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            }

            HAL_UART_Transmit( &huart2, "\nu, ", 4, 200 );            
            for( int k=0; k<96; k++ ) {
              sprintf( output, "%d, ", u_tbl[k] );
              HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            }

            HAL_UART_Transmit( &huart2, "\nut, ", 5, 200 );
            for( int l=0; l<96; l++ ) {
              sprintf( output, "%d, ", ut_tbl[l] );
              HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            }
            
            HAL_UART_Transmit( &huart2, "\n", 1, 200);
            break;
          
          case '\n':
            sprintf( output, "\npwml=%d\n", pwml );
            HAL_UART_Transmit( &huart2, output, strlen(output), 200);
            break;
            */
        }
        
      }
      
      timeout = 0;
      
    //#endif

/*
    // ####### LOW-PASS FILTER #######
    steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
    speed = speed * (1.0 - FILTER) + cmd2 * FILTER;


    // ####### MIXER #######
    speedR = CLAMP(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT, -1000, 1000);
    speedL = CLAMP(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT, -1000, 1000);


    #ifdef ADDITIONAL_CODE
      ADDITIONAL_CODE;
    #endif
    */

//rotemc
/* 
    // ####### SET OUTPUTS #######
    if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50) && timeout < TIMEOUT) {
    #ifdef INVERT_R_DIRECTION
      pwmr = speedR;
    #else
      pwmr = -speedR;
    #endif
    #ifdef INVERT_L_DIRECTION
      pwml = -speedL;
    #else
      pwml = speedL;
    #endif    
    }
*/
   //rotemc
/*
    lastSpeedL = speedL;
    lastSpeedR = speedR;
*/

    if (inactivity_timeout_counter % 25 == 0) {
      // ####### CALC BOARD TEMPERATURE #######
      board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
      board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;
      
      // ####### DEBUG SERIAL OUT #######
      #ifdef CONTROL_ADC
        setScopeChannel(0, (int)adc_buffer.l_tx2);  // 1: ADC1
        setScopeChannel(1, (int)adc_buffer.l_rx2);  // 2: ADC2
      #endif
      //rotemc setScopeChannel(2, (int)speedR);  // 3: output speed: 0-1000
      //rotemc setScopeChannel(3, (int)speedL);  // 4: output speed: 0-1000
      setScopeChannel(4, (int)adc_buffer.batt1);  // 5: for battery voltage calibration
      setScopeChannel(5, (int)(batteryVoltage * 100.0f));  // 6: for verifying battery voltage calibration
      setScopeChannel(6, (int)board_temp_adc_filtered);  // 7: for board temperature calibration
      setScopeChannel(7, (int)board_temp_deg_c);  // 8: for verifying board temperature calibration
      consoleScope();
    }


    // ####### POWEROFF BY POWER-BUTTON #######
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && weakr == 0 && weakl == 0) {
      enable = 0;
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}
      poweroff();
    }

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && abs(speed) < 20) || (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && abs(speed) < 20)) {  // poweroff before mainboard burns OR low bat 3
      poweroff();
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      buzzerFreq = 4;
      buzzerPattern = 1;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
      //rotemc buzzerFreq = 5;
      //rotemc buzzerPattern = 42;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE) {  // low bat 2: fast beep
      //rotemc buzzerFreq = 5;
      //rotemc buzzerPattern = 6;
    } else if (BEEPS_BACKWARD && speed < -50) {  // backward beep
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else {  // do not beep
      buzzerFreq = 0;
      buzzerPattern = 0;
    }

/*
    // ####### INACTIVITY TIMEOUT #######
    if (abs(speedL) > 50 || abs(speedR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter ++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      poweroff();
    }
    */
  }
}

/** Check system health (temp, battery, etc.) and power off if needed.
*/
void main_health_check(void) {  
  timeout = 0;  
  // ####### POWEROFF BY POWER-BUTTON #######
  if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && weakr == 0 && weakl == 0) {
    enable = 0;
    while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}
    poweroff();   
  }

  // ####### CALC BOARD TEMPERATURE #######
  board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
  board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;
      
  // ####### BEEP AND EMERGENCY POWEROFF #######  
  if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && abs(speed) < 20) /* || (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && abs(speed) < 20)*/ ) {  // poweroff before mainboard burns OR low bat 3
    poweroff();
  } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
    buzzerFreq = 4;
    buzzerPattern = 1;
  } 
  else if( batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) ) {
    buzzerFreq = 5;
    buzzerPattern = 6;
  }
  
  else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
    //rotemc buzzerFreq = 5;
    //rotemc buzzerPattern = 42;
  } else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) /*&& batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE*/) {  // low bat 2: fast beep
    //rotemc buzzerFreq = 5;
    // rotemc buzzerPattern = 6;
  } else if (BEEPS_BACKWARD && speed < -50) {  // backward beep
    buzzerFreq = 5;
    buzzerPattern = 1;
  } else {  // do not beep
    buzzerFreq = 0;
    buzzerPattern = 0;
  }    
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


