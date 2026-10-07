#include "vs_wrc940_lightrover_e32.h"
#include "vs_wrc940_memmap.h"
#include "vs_wrc940_i2c.h"
#include "vs_wrc940_motor.h"
#include "vs_wrc940_serial.h"
#include "vs_wrc940_spi.h"
#include "vs_wrc940_wifi.h"
#include "vs_wrc940_ble.h"
#include "vs_wrc940_lidar.h"
#include <vector>
#include <string>
#include <math.h>
#define LEDC_CHANNEL_0     0
#define LEDC_TIMER_8_BIT   8
#define LEDC_BASE_FREQ     5000
#define LED_PIN            46

uint8_t bufValue = 0;
uint16_t waitTime = 0;
int waitStartTime = millis();
void ledInit(){
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  return;
}
void LED(int cmd){
  if(cmd){
    ledcWrite(LEDC_CHANNEL_0, 255);
  }else{
    ledcWrite(LEDC_CHANNEL_0, 0);
  }

  return;
}

const uint8_t io_pins[6] = {4, 5, 6, 7, 8, 9};
const uint8_t io_pwm_channels[6] = {3, 4, 5, 6, 7, 8}; 

void IoInit(){
  for(int i=0;i<6;i++){
    if(wrc940.u8Map(MU8_IO_MODE0+i)==1) pinMode(io_pins[i], INPUT);
    if(wrc940.u8Map(MU8_IO_MODE0+i)==2) pinMode(io_pins[i], INPUT_PULLUP);
    if(wrc940.u8Map(MU8_IO_MODE0+i)==3) pinMode(io_pins[i], INPUT_PULLDOWN);
    if(wrc940.u8Map(MU8_IO_MODE0+i)==4) pinMode(io_pins[i], OUTPUT);
    if(wrc940.u8Map(MU8_IO_MODE0+i)==6){
      ledcSetup(io_pwm_channels[i], 1000, 12); // 1kHz, 12bit
      ledcAttachPin(io_pins[i], io_pwm_channels[i]);
    }
  }
}

void chkIo(){
  int din=0;
  for(int i=0;i<6;i++){
    if(wrc940.u8Map(MU8_IO_MODE0+i)==1 || wrc940.u8Map(MU8_IO_MODE0+i)==2 || wrc940.u8Map(MU8_IO_MODE0+i)==3){
      din |= digitalRead(io_pins[i])<<i;
    }
    if(wrc940.u8Map(MU8_IO_MODE0+i)==4){
      digitalWrite(io_pins[i], ((wrc940.u8Map(MU8_IO_DO) >> i) & 0x01) ? HIGH : LOW);
    }
    if(wrc940.u8Map(MU8_IO_MODE0+i)==5){
      wrc940.u16Map(MU16_IO_AI0+2*i,analogRead(io_pins[i]));
    }
    if(wrc940.u8Map(MU8_IO_MODE0+i)==6){
      uint16_t duty = wrc940.u16Map(MU16_IO_PWM0 + (i * 2));
      if(duty > 4095) duty = 4095;
      ledcWrite(io_pwm_channels[i], duty);
    }
  }
  wrc940.u8Map(MU8_IO_DI,din);
}
void chkPadInput(){

  uint32_t button_on = 0;
  if(checkBTN(S_R1) || checkBTN(S_R2) || checkBTN(S_L1) || checkBTN(S_L2)){
    if(checkBTN(S_R1)){
      motor_param[MAX_SPEED] = 0.5;
      motor_param[MAX_RAD]   = 1.57;
    }else if(checkBTN(S_L1)){
      motor_param[MAX_SPEED] = std_motor_param[MAX_SPEED];
      motor_param[MAX_RAD]   = std_motor_param[MAX_RAD];
    }else if(checkBTN(S_R2)){
      motor_param[MAX_SPEED] = 1.0;
      motor_param[MAX_RAD]   = 3.14;
    }else if(checkBTN(S_L2)){
      motor_param[MAX_SPEED] = 1.40;
      motor_param[MAX_RAD]   = 6.28;
    }
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    aStick2V();

    button_on = 2;
      
  }else{
    setO_EN(ON_ON, &wrc940);
    setMortorParam2Std();
    ctl_v_com[M_L] = 0.0;
    ctl_v_com[M_R] = 0.0;
  }

  
  wrc940.s16Map(MS16_S_XS, 0);
  wrc940.s16Map(MS16_S_ZS, 0);

  if(checkBTN(CROSS_U)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) +   30);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) +   0);
    button_on = 1;

  }
  if(checkBTN(CROSS_D)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) -   30);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) +   0);
    button_on = 1;

  }
  if(checkBTN(CROSS_L)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) +   0);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) +   300);
    button_on = 1;

  }
  if(checkBTN(CROSS_R)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) +   0);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) -   300);
    button_on = 1;

  }

  if(checkBTN(TRIANGLE)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) +   10);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) +   0);
    button_on = 1;

  } 
  if(checkBTN(CROSS)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) -   10);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) +   0);
    button_on = 1;

  }
  if(checkBTN(SQUARE)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) +   0);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) +   100);
    button_on = 1;

  }
  if(checkBTN(CIRCLE)){
    setCtrlMode(MODE_POS);
    setO_EN(ON_ON, &wrc940);
    wrc940.s16Map(MS16_S_XS, wrc940.s16Map(MS16_S_XS) +   0);
    wrc940.s16Map(MS16_S_ZS, wrc940.s16Map(MS16_S_ZS) -   100);
    button_on = 1;

  }

  if(button_on == 1){
    memCom2V();
  }else if(button_on == 2){
    return;
  }else{
  }
}


/*******************************************
 * Interrupt timer state.
 */
hw_timer_t *interruptTimer = NULL;
bool isInterrupt = false;

volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);



  isInterrupt = true;

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);



}

void setupInterruptTimer(){
  timerSemaphore = xSemaphoreCreateBinary();
  interruptTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(interruptTimer, &onTimer, true);
  timerAlarmWrite(interruptTimer, 10000, true);
  timerAlarmEnable(interruptTimer);

}


/***************************************
 * Direct wheel-speed command helper.
 */
void wheelRun(int32_t spdL, int32_t spdR){
  double msSpdL = -1.0*spdL;
  double msSpdR = spdR;

  double roverV = (msSpdR + msSpdL)/2.0;
  double roverO = (msSpdR - msSpdL)/(2.0*ROVER_D);

  //Serial.println(roverV);

  setO_EN(ON_ON);
  wrc940.s16Map(MS16_S_XS, roverV);
  wrc940.s16Map(MS16_S_ZS, roverO);

  return;

}
void readEnc(int32_t *encL, int32_t *encR){
  *encL = -1*wrc940.s32Map(MS32_M_POS0);
  *encR = -1*wrc940.s32Map(MS32_M_POS1);

  return;
}
void clearEnc(){
  wrc940.u8Map(MU8_TRIG, (wrc940.u8Map(MU8_TRIG) | 0x80));

  return;
}
