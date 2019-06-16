
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"


volatile int posl = 0;
volatile int posr = 0;
volatile int pwml = 0;
volatile int pwmr = 0;
volatile int weakl = 0;
volatile int weakr = 0;

extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;

uint32_t buzzerFreq = 0;
uint32_t buzzerPattern = 0;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

const uint8_t hall_to_pos[8] = {
    0, //000
    0, //001
    2, //010
    1, //011
    4, //100
    5, //101
    3, //110
    0, //111
};

//rotemc
// LUT ( hall_tbl[prev_state][state] ) for mapping hall sensors transitions to ticks.
// +1 -> CW tick
// -1 -> CCW tick
// 00 -> no transition
// NA -> invalid transition
// There are ~90 ticks per revolution
#define NA 99
const signed char hall_tbl[6][6] = { {00, +1, 00, 00, 00, -1},
                                     {-1, 00, +1, 00, 00, 00},
                                     {00, -1, 00, +1, 00, 00},
                                     {00, 00, -1, 00, +1, 00},
                                     {00, 00, 00, -1, 00, +1},
                                     {+1, 00, 00, 00, -1, 00} };

//rotemc


inline void blockPWM(int dir, int pwm, int pos, int *u, int *v, int *w) {
  
  //rotemc
  //*u = pwm;
  //*v = pwm;
  //*w = pwm;
  //return;
  //rotemc

  if( dir > 0 ) {
    pwm -= 100; //rotemc 200;
  }  
  
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}

uint32_t buzzerTimer        = 0;

int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

int curl = 0;
// int errorl = 0;
// int kp = 5;
// volatile int cmdl = 0;

int last_pos = 0;
int timer = 0;
const int max_time = PWM_FREQ / 10;
volatile int vel = 0;

//rotemc --------------------------------------------------------

#define ALPHA 0
int ull=0, vll=0, wll=0;
int urr=0, vrr=0, wrr=0;

volatile uint64_t isr_cnt=0;
volatile uint64_t posl_isr_cnt=0;
volatile uint64_t posr_isr_cnt=0;

volatile uint64_t motorl_comm_isr_cnt=0;
volatile uint64_t motorr_comm_isr_cnt=0;

//volatile uint64_t posl_no_change_cntr=0;
//volatile uint64_t posr_no_change_cntr=0;

volatile uint posl_last=0;
volatile uint posr_last=0;

volatile int motorl_ticks=0;
volatile int motorr_ticks=0;

volatile int motorl_ticks_last=0;
volatile int motorr_ticks_last=0;

volatile int motorl_dir=1;
volatile int motorr_dir=1;

volatile uint motorl_speed=0;
volatile uint motorr_speed=0;

volatile uint motorl_comm_res=0;
volatile uint motorr_comm_res=0;

volatile int motorl_tbl_index=0;
volatile int motorr_tbl_index=0;

volatile uint motorl_tbl_index_next=0;
volatile uint motorr_tbl_index_next=0;

#define SIN_TBL_SIZE 96
#define ONE_3RD_SIN_TBL_SIZE (SIN_TBL_SIZE/3)
#define COMM_PER_HALL_TICK (SIN_TBL_SIZE/6)
const uint32_t sin_tbl[SIN_TBL_SIZE]= {	

  86718750, 89062500, 91796875, 94140625, 96093750, 97656250, 98828125, 99609375,
	99609375, 99609375, 99218750, 98046875, 96484375, 94921875, 92578125, 89843750,
  
	87109375, 83593750, 79687500, 75781250, 71484375, 66796875, 61718750, 56250000,
	50781250, 45312500, 39453125, 33203125, 26953125, 20703125, 14062500,  7812500,

	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,

	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,

	  781250,  7421875, 13671875, 20312500, 26562500, 32812500, 38671875, 44921875,
	49609375, 55468750, 60546875, 65625000, 70312500, 75000000, 78906250, 82812500,

  86328125, 89453125, 92187500, 94531250, 96484375, 97656250, 98828125, 99609375,
	99609375, 99609375, 98828125, 98046875, 96484375, 94531250, 92578125, 89843750

};

/*
const float sin_tbl_flt[SIN_TBL_SIZE]= {	

	0.86718750, 0.89062500, 0.91796875, 0.94140625, 0.96093750, 0.97656250, 0.98828125, 0.99609375,
	0.99609375, 0.99609375, 0.99218750, 0.98046875, 0.96484375, 0.94921875, 0.92578125, 0.89843750,
  
	0.87109375, 0.83593750, 0.79687500, 0.75781250, 0.71484375, 0.66796875, 0.61718750, 0.56250000,
	0.50781250, 0.45312500, 0.39453125, 0.33203125, 0.26953125, 0.20703125, 0.14062500, 0.07812500,

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

	0.0078125, 0.07421875, 0.13671875, 0.203125, 0.265625, 0.328125, 0.38671875, 0.44921875,
	0.49609375, 0.5546875, 0.60546875, 0.656250, 0.703125, 0.750000, 0.78906250, 0.82812500,

  0.86328125, 0.89453125, 0.92187500, 0.94531250, 0.96484375, 0.97656250, 0.98828125, 0.99609375,
	0.99609375, 0.99609375, 0.98828125, 0.98046875, 0.96484375, 0.94531250, 0.92578125, 0.89843750
};
*/

volatile int v_tbl[SIN_TBL_SIZE] = {0};
volatile int w_tbl[SIN_TBL_SIZE] = {0};
volatile int u_tbl[SIN_TBL_SIZE] = {0};
volatile int last_sin_idx=0;

volatile int vt_tbl[SIN_TBL_SIZE] = {0};
volatile int wt_tbl[SIN_TBL_SIZE] = {0};
volatile int ut_tbl[SIN_TBL_SIZE] = {0};

volatile int offset_pull=12;

inline void blockPWMsin(int dir, int pwm, int pos, int *u, int *v, int *w) {
  
  #if 0
  blockPWM( pwm, pos/COMM_PER_HALL_TICK, u, v, w);
  vt_tbl[last_sin_idx] = *v;
  wt_tbl[last_sin_idx] = *w;
  ut_tbl[last_sin_idx] = *u;
  #endif

  if( dir < 0 ) {
    pos = (pos + SIN_TBL_SIZE + offset_pull)%SIN_TBL_SIZE;
  }
  #if 1
  else if( dir > 0 ) {
    pwm -= 100;//rotemc 200;
  }
  #endif  

  // Integer table math
  if( pwm >= 0 ) {
    *v= (int) (((uint64_t) pwm * (uint64_t) sin_tbl[pos])/100000000);
    *u= (int) (((uint64_t) pwm * (uint64_t) sin_tbl[(pos +   ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE])/100000000);
    *w= (int) (((uint64_t) pwm * (uint64_t) sin_tbl[(pos + 2*ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE])/100000000);                        
  } else {
    pwm *= -1;
    *w= (int) (((uint64_t) pwm * (uint64_t) sin_tbl[pos])/100000000);
    *v= (int) (((uint64_t) pwm * (uint64_t) sin_tbl[(pos +   ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE])/100000000);
    *u= (int) (((uint64_t) pwm * (uint64_t) sin_tbl[(pos + 2*ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE])/100000000);                        
    pwm *= -1;
  }  

  #if 0
  v_tbl[last_sin_idx] = *v;
  w_tbl[last_sin_idx] = *w;
  u_tbl[last_sin_idx] = *u;
  last_sin_idx = (last_sin_idx+1)%SIN_TBL_SIZE;
  #endif

  /* // Float table math 
  if( pwm >= 0 ) {
    *v= (int) ((float) pwm * sin_tbl_flt[pos]);
    *u= (int) ((float) pwm * sin_tbl_flt[(pos + ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE]);
    *w= (int) ((float) pwm * sin_tbl_flt[(pos + 2*ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE]);
  } else {
    pwm *= -1;
    *w= (int) ((float) pwm * sin_tbl_flt[pos]);
    *v= (int) ((float) pwm * sin_tbl_flt[(pos + ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE]);
    *u= (int) ((float) pwm * sin_tbl_flt[(pos + 2*ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE]);
  }  
  */
  
}

//rotemc --------------------------------------------------------

//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  //rotemc
  isr_cnt++;

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  }

  //disable PWM when current limit is reached (current chopping)
  if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  }

  if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  static int ul=0, vl=0, wl=0;
  static int ur=0, vr=0, wr=0;  

  //determine next position based on hall sensors
  uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
  uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
  uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

  uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
  uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
  uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

  uint8_t halll = hall_ul * 1 + hall_vl * 2 + hall_wl * 4;
  posl          = hall_to_pos[halll];
  posl += 2;
  posl %= 6;

  uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;
  posr          = hall_to_pos[hallr];
  posr += 2;
  posr %= 6;
  
  //rotemc
  //------------------------------------------------------------------------------
  
  /*
  if( posl == posl_last ) {
    posl_no_change_cntr++;
    if( posl_no_change_cntr == 2000 && motorl_dir < 0 ) {      
      posl_last = (posl + 5)%6;
      //motorl_ticks_last--;
    }
  }
  */
  // -----------------------------------------------------------------------------

  // Sinusoidal commutation logic - left motor
  if( posl != posl_last ) {
    //posl_no_change_cntr = 0;
    motorl_tbl_index = posl*COMM_PER_HALL_TICK;    
    motorl_ticks += hall_tbl[posl_last][posl];
    posl_last = posl;

    if (motorl_ticks > motorl_ticks_last) {
      motorl_dir = 1;
    }
    else if (motorl_ticks < motorl_ticks_last) {
      motorl_dir = -1;
    }
    motorl_ticks_last = motorl_ticks;      

    motorl_speed = isr_cnt - posl_isr_cnt;
    posl_isr_cnt = isr_cnt;

    motorl_comm_res = motorl_speed/COMM_PER_HALL_TICK;
    motorl_comm_isr_cnt = isr_cnt + motorl_comm_res;

    motorl_tbl_index_next = (motorl_tbl_index + SIN_TBL_SIZE + COMM_PER_HALL_TICK*motorl_dir)%SIN_TBL_SIZE;  
  }
  else if ( isr_cnt == motorl_comm_isr_cnt &&
            motorl_tbl_index != motorl_tbl_index_next)
  {
    motorl_comm_isr_cnt = isr_cnt + motorl_comm_res;
    motorl_tbl_index = (motorl_tbl_index + SIN_TBL_SIZE + motorl_dir)%SIN_TBL_SIZE;    
  }

  if( isr_cnt < motorl_comm_isr_cnt + 500 ) {
    // Motor moving -> use sinusoidal commutation
    blockPWMsin(motorl_dir, pwml, motorl_tbl_index, &ul, &vl, &wl);
    LEFT_TIM->LEFT_TIM_U = CLAMP(ul, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_V = CLAMP(vl, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_W = CLAMP(wl, 10, pwm_res-10);
  }
  else {
    // Motor stoped -> use trapezoidal commutation
    blockPWM(motorl_dir, (pwml*1000)/2000, posl, &ul, &vl, &wl);
    LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);
  }
    
  // --------------------------------------------------------------------------------
  
  // Sinusoidal commutation logic - right motor
  if( posr != posr_last ) {
    //posr_no_change_cntr = 0;
    motorr_tbl_index = posr*COMM_PER_HALL_TICK;    
    motorr_ticks += hall_tbl[posr_last][posr];
    posr_last = posr;

    if (motorr_ticks > motorr_ticks_last) {
      motorr_dir = 1;
    }
    else if (motorr_ticks < motorr_ticks_last) {
      motorr_dir = -1;
    }
    motorr_ticks_last = motorr_ticks;      

    motorr_speed = isr_cnt - posr_isr_cnt;
    posr_isr_cnt = isr_cnt;

    motorr_comm_res = motorr_speed/COMM_PER_HALL_TICK;
    motorr_comm_isr_cnt = isr_cnt + motorr_comm_res;

    motorr_tbl_index_next = (motorr_tbl_index + SIN_TBL_SIZE + COMM_PER_HALL_TICK*motorr_dir)%SIN_TBL_SIZE;  
  }
  else if ( isr_cnt == motorr_comm_isr_cnt &&
            motorr_tbl_index != motorr_tbl_index_next)
  {
    motorr_comm_isr_cnt = isr_cnt + motorr_comm_res;
    motorr_tbl_index = (motorr_tbl_index + SIN_TBL_SIZE + motorr_dir)%SIN_TBL_SIZE;    
  }
/*
  if( isr_cnt < motorr_comm_isr_cnt + 500 ) {
    // Motor moving -> use sinusoidal commutation
    blockPWMsin(motorr_dir, pwmr, motorr_tbl_index, &ur, &vr, &wr);
    RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr, 10, pwm_res-10);
  }
  else {
    // Motor stoped -> use trapezoidal commutation
    blockPWM((pwmr*1000)/2000, posr, &ur, &vr, &wr);
    RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  }
  //blockPWM(pwmr, posr, &ur, &vr, &wr);
  //blockPWMsin(motorr_dir, pwmr, motorr_tbl_index, &ur, &vr, &wr);
  */
  //update PWM channels based on position
  //pwmr = CLAMP(pwmr, -200, 200);

  blockPWM(motorr_dir, (pwmr*1000)/2000, posr, &ur, &vr, &wr);
  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  
  // ------------------------------------------------------------------------------------
  
  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }  
}
