#include "vs_wrc940_motor.h"
#include "vs_wrc940_memmap.h"
//#include "vs_wrc940_spi.h"
#include <Arduino.h>
#include <math.h>
#include <vector>
#include <string>
#include <cstdint>

double  ctl_v_com[2] = {0.0, 0.0};
double  v_com[2];
double motor_wheel_scale[2] = {1.0, 1.0};
int16_t m_com[2];

int32_t enc[2] = {0, 0};
int32_t old_enc[2] = {0, 0};


double sum_v[2][5] = {{0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}};
double avr_v[2] = {0.0, 0.0};
uint32_t old_micros = 0;


double v_enc[2]  = {0.0, 0.0};
double v_diff[2] = {0.0, 0.0};
double prev_v_diff[2] = {0.0, 0.0};
double prev2_v_diff[2] = {0.0, 0.0};
int16_t prev_m[2] = {0, 0};

uint32_t pid_time = 0;
uint32_t pos_time = 0;

float std_motor_param[] = { 0.1,
                            3.14,
                            520,
                            1.6,
                            0.2,
                            1.6,
                            30,
                            127,
                            1.5
                          };

float motor_param[] = { 0.1,
                        3.14,
                        520,
                        1.6,
                        0.2,
                        1.6,
                        30,
                        127,
                        1.5
                      };
int setO_EN(int data){
  setO_EN(data, &wrc940);

  return data;
}

int setO_EN(int data, Wrc940* memmap){

  //memmap->write1Byte(MU8_O_EN, data);
  memmap->u8Map(MU8_O_EN, data);

  return data;
}
int getO_EN(){
  int return_value = 0;
  return_value += getO_EN(&wrc940);

  return return_value;
}

int getO_EN(Wrc940* memmap){
  //memmap->readMemmap(MU8_O_EN, 1);

  return memmap->u8Map(MU8_O_EN);
}
int setCtrlMode(int ctrl_mode){
  setCtrlMode(ctrl_mode, &wrc940);

  return ctrl_mode;
}

int setCtrlMode(int ctrl_mode, Wrc940* memmap){
  if(getCtrlMode(memmap)){
    if(ctrl_mode == MODE_POS){
      setO_EN(OFF_OFF, memmap);
      //memmap->write4Byte(MS16_FB_PG0, KP_NORMAL);
      //memmap->write1Byte(MU8_TRIG, OFF_OFF);

      memmap->u32Map(MS16_FB_PG0, KP_NORMAL);
    }
  }else{
    if(ctrl_mode == MODE_PWM){
      setO_EN(OFF_OFF, memmap);
      //memmap->write4Byte(MS16_FB_PG0, KP_ZERO);
      //memmap->write1Byte(MU8_TRIG, ENC_RESET);

      memmap->u32Map(MS16_FB_PG0, KP_ZERO);
    } 
  }
  
  return ctrl_mode;
}
int getCtrlMode(){
  int return_value = 0;;
  return_value += getCtrlMode(&wrc940);

  return return_value;
}

int getCtrlMode(Wrc940* memmap){
  //memmap->readMemmap(MS16_FB_PG0, 4);
  if(memmap->u32Map(MS16_FB_PG0)){
    return MODE_POS;
  }else{
    return MODE_PWM;
  }
}
void aStick2V(){
  double pad_left_y;
  double pad_left_x;
  double pad_right_y;
  double pad_right_x;
  if(pad_data.left_stick.y >= motor_param[PAD_DEAD]){
    pad_left_y = (double)pad_data.left_stick.y - motor_param[PAD_DEAD];
  }else if(pad_data.left_stick.y <= -1.0*motor_param[PAD_DEAD]){
    pad_left_y = (double)pad_data.left_stick.y + motor_param[PAD_DEAD];
  }else{
    pad_left_y = 0.0;
  }

  if(pad_data.left_stick.x >= motor_param[PAD_DEAD]){
    pad_left_x = (double)pad_data.left_stick.x - motor_param[PAD_DEAD];
  }else if(pad_data.left_stick.x <= -1.0*motor_param[PAD_DEAD]){
    pad_left_x = (double)pad_data.left_stick.x + motor_param[PAD_DEAD];
  }else{
    pad_left_x = 0.0;
  }

  if(pad_data.right_stick.y >= motor_param[PAD_DEAD]){
    pad_right_y = (double)pad_data.right_stick.y - motor_param[PAD_DEAD];
  }else if(pad_data.right_stick.y <= -1.0*motor_param[PAD_DEAD]){
    pad_right_y = (double)pad_data.right_stick.y + motor_param[PAD_DEAD];
  }else{
    pad_right_y = 0.0;
  }

  if(pad_data.right_stick.x >= motor_param[PAD_DEAD]){
    pad_right_x = (double)pad_data.right_stick.x - motor_param[PAD_DEAD];
  }else if(pad_data.right_stick.x <= -1.0*motor_param[PAD_DEAD]){
    pad_right_x = (double)pad_data.right_stick.x + motor_param[PAD_DEAD];
  }else{
    pad_right_x = 0.0;
  }

  if(fabs(pad_right_y) > fabs(pad_left_y)){
    pad_left_y = pad_right_y;
  }else{
    pad_right_y = pad_left_y;
  }

  if(fabs(pad_right_x) > fabs(pad_left_x)){
    pad_left_x = pad_right_x;
  }else{
    pad_right_x = pad_left_x;
  }

  if(pad_right_y >= 0.0){
    pad_right_x = -1.0*pad_right_x;
  }

  double diff_max_dead = motor_param[PAD_MAX] - motor_param[PAD_DEAD];
  ctl_v_com[M_L] = -1.0*((motor_param[MAX_SPEED]*(pad_left_y/diff_max_dead)) - (motor_param[MAX_RAD]*ROVER_D*(pad_right_x/diff_max_dead)));
  ctl_v_com[M_R] = ((motor_param[MAX_SPEED]*(pad_left_y/diff_max_dead)) + (motor_param[MAX_RAD]*ROVER_D*(pad_right_x/diff_max_dead)));
  applyWheelSpeedScale();

  

  if(fabs(ctl_v_com[M_L]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[M_R]) > motor_param[MAX_SPEED]){
    double ctl_v_com_max = fabs(ctl_v_com[0]);
    int i;
    for(i = 1; i < 2; i++){
      if(ctl_v_com_max < fabs(ctl_v_com[i])){
        ctl_v_com_max = fabs(ctl_v_com[i]);
      }
    }

    for(i = 0; i < 2; i++){
       ctl_v_com[i] /= (ctl_v_com_max/motor_param[MAX_SPEED]);
    }
  }
}
void applyWheelSpeedScale(){
  ctl_v_com[M_L] *= motor_wheel_scale[M_L];
  ctl_v_com[M_R] *= motor_wheel_scale[M_R];
}

void memCom2V(){
  double mem_com_x = wrc940.s16Map(MS16_S_XS)/1000.0;  
  double mem_com_z = wrc940.s16Map(MS16_S_ZS)/1000.0;  //[m/s]

  setMortorParam2Std();

  ctl_v_com[M_L] = -1.0*(mem_com_x - ROVER_D*mem_com_z);
  ctl_v_com[M_R] = (mem_com_x + ROVER_D*mem_com_z);
  applyWheelSpeedScale();

  if(fabs(ctl_v_com[M_L]) > motor_param[MAX_SPEED] || fabs(ctl_v_com[M_R]) > motor_param[MAX_SPEED]){
    double ctl_v_com_max = fabs(ctl_v_com[0]);
    int i;
    for(i = 1; i < 2; i++){
      if(ctl_v_com_max < fabs(ctl_v_com[i])){
        ctl_v_com_max = fabs(ctl_v_com[i]);
      }
    }

    for(i = 0; i < 2; i++){
       ctl_v_com[i] /= (ctl_v_com_max/motor_param[MAX_SPEED]);
    }
  }


}
void ctl2Vcom(){
  int i;
  double v_diff[2];
  for(i = 0; i < 2; i++){
    v_diff[i] = ctl_v_com[i] - v_com[i];
  }
  double max_v_diff = fabs(v_diff[M_L]);
  for(i = 1; i < 2; i++){
    if(fabs(v_diff[i]) > max_v_diff){
      max_v_diff = fabs(v_diff[i]);
    }
  }
  static double prev_micros = micros();
  double new_micros = micros();
  double elapsed_time = (new_micros - prev_micros);
  if(elapsed_time < 0){
    elapsed_time = (new_micros + (UINT64_MAX - prev_micros));
  }
  if(elapsed_time > 50000.0){
    elapsed_time = 50000.0;
  }
  double add_v = 0.0;
  if(max_v_diff != 0.0){
    for(i = 0; i < 2; i++){
      add_v = ((motor_param[MAX_ACC] * (v_diff[i]/max_v_diff)*elapsed_time)/1000000.0);
      if(fabs(v_diff[i]) >= fabs(add_v)){
        v_com[i] = v_com[i] + add_v;
      }else{
        v_com[i] = ctl_v_com[i];
      }      
    }
  }


  
  for(i = 0; i < 2; i++){
    if(ctl_v_com[i] == 0.0 && fabs(v_com[i]) <= 0.001){
      v_com[i] = 0.0;
    }
  }  

  prev_micros = new_micros;

}
void getEncoderValue(){

  wrc940.readMemmap(MS32_M_POS0, 8);
  //wrc940.readMemmap(MS32_M_POS1, 4);
}
void getRoverV(){
    
  getEncoderValue();
  enc[M_L] = (int32_t)wrc940.s32Map(MS32_M_POS0);
  enc[M_R] = (int32_t)wrc940.s32Map(MS32_M_POS1);

  uint32_t new_micros;
  new_micros = micros();

  int i;
  for(i = 0; i < 2; i++){
    v_enc[i] = (((double)(enc[i]-old_enc[i])/ENC_COUNTS_PER_TURN)*TIRE_CIRCUMFERENCE)/(double)((new_micros-old_micros)/1000.0);//[m/s]
    old_enc[i] = enc[i];
  }

  old_micros = new_micros;

  for(int i = 0; i < 2; i++){
    if(fabs(v_enc[i]) > 2*motor_param[MAX_SPEED]){
      v_enc[i] = sum_v[i][0];
    }
    
    for(int j = 4; j > 0; j--){
      sum_v[i][4] += sum_v[i][j-1];
      sum_v[i][j] =  sum_v[i][j-1];
    }

    sum_v[i][4] += v_enc[i];
    avr_v[i]    =  sum_v[i][4]/5.0;
    sum_v[i][0] =  v_enc[i];
  }
}

void pidControl(){

  setCtrlMode(MODE_POS);

  if(chkWDT(&wrc940)){
    setO_EN(OFF_OFF);
    ctl_v_com[M_L] = 0.0;
    ctl_v_com[M_R] = 0.0;
  }else{
    setO_EN(ON_ON);
  }

  getRoverV();  

  ctl2Vcom();
  for(int i = 0; i < 2; i++){
    if(fabs(v_enc[i]) > 2*motor_param[MAX_SPEED]){
      v_enc[i] = sum_v[i][0];
    }
    
    for(int j = 4; j > 0; j--){
      sum_v[i][4] += sum_v[i][j-1];
      sum_v[i][j] =  sum_v[i][j-1];
    }

    sum_v[i][4] += v_enc[i];
    avr_v[i]    =  sum_v[i][4]/5.0;
    sum_v[i][0] =  v_enc[i];

    v_diff[i] = v_com[i] - avr_v[i];
    m_com[i] = prev_m[i] + (int)(((v_diff[i])*motor_param[K_I]
                + (v_diff[i] - prev_v_diff[i])*motor_param[K_P] + ((v_diff[i] - prev_v_diff[i])-(prev_v_diff[i] - prev2_v_diff[i]))*motor_param[K_D]) * motor_param[K_V2MP]);

    if(m_com[i] > 4096){
      m_com[i] = 4096;
    }else if(m_com[i] < -4096){
      m_com[i] = -4096;
    }

    prev2_v_diff[i] = prev_v_diff[i];
    prev_v_diff[i] = v_diff[i];
    prev_m[i] = m_com[i];
  }
/*  if(v_com[M_L] == 0.0 && v_com[M_R] == 0.0){
    m_com[M_L] = 0;
    m_com[M_R] = 0;
    prev_m[M_L] = 0;
    prev_m[M_R] = 0;
  }
*/

  if(v_com[M_L] == 0.0 && v_com[M_R] == 0.0 && fabs(avr_v[M_L]) < 0.005 && fabs(avr_v[M_R]) < 0.005){
    m_com[M_L] = 0;
    m_com[M_R] = 0;
    prev_m[M_L] = 0;
    prev_m[M_R] = 0;
  }

  wrc940.s16Map(MS16_T_OUT0, m_com[M_L]);
  wrc940.s16Map(MS16_T_OUT1, m_com[M_R]);
  
}
double buf_enc_com[2];
void posControl(){

  setCtrlMode(MODE_POS);
  setO_EN(ON_ON);
  ctl2Vcom();
  getRoverV();

  //Serial.println(v_com[M_L]);
  double e_v_com[2];
  e_v_com[M_L] = v_com[M_L]*ENC_PER_MM*1000.0;
  e_v_com[M_R] = v_com[M_R]*ENC_PER_MM*1000.0;
  static double prev_micros = micros();
  double new_micros = micros();
  double elapsed_time = (new_micros - prev_micros);
  if(elapsed_time < 0){
    elapsed_time = (new_micros + (UINT32_MAX - prev_micros));
  }
  if(elapsed_time > 50000.0){
    elapsed_time = 50000.0;
  }
  buf_enc_com[M_L] += (e_v_com[M_L]*elapsed_time)/1000000.0;
  buf_enc_com[M_R] += (e_v_com[M_R]*elapsed_time)/1000000.0;
  if(wrc940.checkWriteFlag(MS32_A_POS0)){
    wrc940.s32Map(MS32_A_POS0, (int32_t)wrc940.s32Map(MS32_A_POS0)+(int32_t)buf_enc_com[M_L]);
  }else{
    wrc940.s32Map(MS32_A_POS0, (int32_t)buf_enc_com[M_L]);
  }
  if(wrc940.checkWriteFlag(MS32_A_POS1)){
    wrc940.s32Map(MS32_A_POS1, (int32_t)wrc940.s32Map(MS32_A_POS1)+(int32_t)buf_enc_com[M_R]);
  }else{
    wrc940.s32Map(MS32_A_POS1, (int32_t)buf_enc_com[M_R]);
  }
  if((int32_t)buf_enc_com[M_L] > 0){
    if(wrc940.s16Map(MS16_T_OUT0) <= 0){
      wrc940.s16Map(MS16_T_OUT0, 800); //
    }
  }else if((int32_t)buf_enc_com[M_L] < 0){
    if(wrc940.s16Map(MS16_T_OUT0) >= 0){
      wrc940.s16Map(MS16_T_OUT0, -800); //
    }
  }
  if((int32_t)buf_enc_com[M_R] > 0){
    if(wrc940.s16Map(MS16_T_OUT1) <= 0){
      wrc940.s16Map(MS16_T_OUT1, 800); //
    }
  }else if((int32_t)buf_enc_com[M_R] < 0){
    if(wrc940.s16Map(MS16_T_OUT1) >= 0){
      wrc940.s16Map(MS16_T_OUT1, -800); //
    }
  }

  buf_enc_com[M_L] -=  (int32_t)buf_enc_com[M_L];
  buf_enc_com[M_R] -=  (int32_t)buf_enc_com[M_R];

  wrc940.u8Map(MU8_TRIG, (wrc940.u8Map(MU8_TRIG) | 0x03));

  prev_micros = new_micros;

}
void resetOdom(){
  if(!(wrc940.u8Map(MU8_TRIG) & 0x80)){
    return;
  }

  wrc940.s32Map(MS32_WP_PX, 0x00000000);
  wrc940.s32Map(MS32_WP_PY, 0x00000000);
  wrc940.s16Map(MS16_WP_TH, 0x00000000);

  old_enc[M_L] = 0;
  old_enc[M_R] = 0;
  enc[M_L] = 0;
  enc[M_R] = 0;
  wrc940.write1Byte(MU8_TRIG, 0x0c);
  wrc940.u8Map(MU8_TRIG, wrc940.u8Map(MU8_TRIG) & 0x7F);

  return;

}

uint8_t chkWDT(Wrc940* memmap){
  memmap->readMemmap(MU16_WDT, 2);
  if(memmap->u16Map(MU16_WDT) == 0xffff){
    return 1;
  }
  return 0;
}
void setMortorParam2Std(){

  int i;
  for(i = 0; i < 7; i++){
    motor_param[i] = std_motor_param[i];
  }

  return;
}

