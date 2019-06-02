
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


inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  
  //rotemc
  //*u = pwm;
  //*v = pwm;
  //*w = pwm;
  //return;
  //rotemc
  
  
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
volatile uint64_t posr_isr_cnt_comm=0;

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
volatile uint motorr_res=0;

volatile int motorl_tbl_index=0;
volatile int motorr_tbl_index=0;

volatile uint motorl_tbl_index_next=0;
volatile uint motorr_tbl_index_next=0;

#define SIN_TBL_SIZE 96
#define ONE_3RD_SIN_TBL_SIZE (SIN_TBL_SIZE/3)
#define COMM_PER_HALL_TICK (SIN_TBL_SIZE/6)
const float sin_tbl[SIN_TBL_SIZE]= {
	0.86328125, 0.89453125, 0.92187500, 0.94531250, 0.96484375, 0.9765625, 0.98828125, 0.99609375,
	0.99609375, 0.99609375, 0.98828125, 0.98046875, 0.96484375, 0.9453125, 0.92578125, 0.89843750,

	0.86718750, 0.89062500, 0.91796875, 0.94140625, 0.96093750, 0.97656250, 0.98828125, 0.99609375,
	0.99609375, 0.99609375, 0.99218750, 0.98046875, 0.96484375, 0.94921875, 0.92578125, 0.89843750,
  
	0.87109375, 0.8359375, 0.79687500, 0.7578125, 0.714843750, 0.66796875, 0.6171875, 0.562500,
	0.50781250, 0.4531250, 0.39453125, 0.33203125, 0.26953125, 0.20703125, 0.1406250, 0.078125,

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

	0.0078125, 0.07421875, 0.13671875, 0.203125, 0.265625, 0.328125, 0.38671875, 0.44921875,
	0.49609375, 0.5546875, 0.60546875, 0.656250, 0.703125, 0.750000, 0.78906250, 0.82812500
};

volatile int v_tbl[SIN_TBL_SIZE] = {0};
volatile int w_tbl[SIN_TBL_SIZE] = {0};
volatile int u_tbl[SIN_TBL_SIZE] = {0};
volatile int last_sin_idx=0;

inline void blockPWMsin(int pwm, int pos, int *u, int *v, int *w) {
  *v= (int) ((float) pwm * sin_tbl[pos]);
  *w= (int) ((float) pwm * sin_tbl[(pos + ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE]);
  *u= (int) ((float) pwm * sin_tbl[(pos + 2*ONE_3RD_SIN_TBL_SIZE) % SIN_TBL_SIZE]);

  v_tbl[last_sin_idx] = *v;
  w_tbl[last_sin_idx] = *w;
  u_tbl[last_sin_idx] = *u;
  last_sin_idx = (last_sin_idx+1)%SIN_TBL_SIZE;
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

  int ul, vl, wl;
  int ur, vr, wr;  

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
  
  if( posl != posl_last ) {
    motorl_tbl_index = posl*COMM_PER_HALL_TICK;
    blockPWMsin(pwml, motorl_tbl_index, &ul, &vl, &wl);

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

    blockPWMsin(pwml, motorl_tbl_index, &ul, &vl, &wl);
  }  

  //------------------------------------------------------------------------------
  //rotemc

  blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);

  //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
  //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);


  // uint8_t buzz(uint16_t *notes, uint32_t len){
    // static uint32_t counter = 0;
    // static uint32_t timer = 0;
    // if(len == 0){
        // return(0);
    // }
    
    // struct {
        // uint16_t freq : 4;
        // uint16_t volume : 4;
        // uint16_t time : 8;
    // } note = notes[counter];
    
    // if(timer / 500 == note.time){
        // timer = 0;
        // counter++;
    // }
    
    // if(counter == len){
        // counter = 0;
    // }

    // timer++;
    // return(note.freq);
  // }


  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }

  //update PWM channels based on position
  blockPWM(pwml, posl, &ul, &vl, &wl);
  blockPWM(pwmr, posr, &ur, &vr, &wr);

/*
  int weakul, weakvl, weakwl;
  if (pwml > 0) {
    blockPWM(weakl, (posl+5) % 6, &weakul, &weakvl, &weakwl);
  } else {
    blockPWM(-weakl, (posl+1) % 6, &weakul, &weakvl, &weakwl);
  }
  ul += weakul;
  vl += weakvl;
  wl += weakwl;
*/

  //rotemc
  //ull = (ALPHA*ull+ul)/(ALPHA+1);
  //vll = (ALPHA*vll+vl)/(ALPHA+1);
  //wll = (ALPHA*wll+wl)/(ALPHA+1);
  //rotemc

/*
  int weakur, weakvr, weakwr;
  if (pwmr > 0) {
    blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {
    blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;
*/

  //rotemc
  //urr = (ALPHA*urr+ur)/(ALPHA+1);
  //vrr = (ALPHA*vrr+vr)/(ALPHA+1);
  //wrr = (ALPHA*wrr+wr)/(ALPHA+1);
  //rotemc

  
  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  
/*
  LEFT_TIM->LEFT_TIM_U = CLAMP(ull + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vll + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wll + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(urr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vrr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wrr + pwm_res / 2, 10, pwm_res-10);
  */
}
