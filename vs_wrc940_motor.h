/**
 * Differential-drive motor control helpers.
 */
#ifndef WRC940_MOTOR_H
#define WRC940_MOTOR_H

#include "vs_wrc940_memmap.h"
#include "vs_wrc940_spi.h"
#include <vector>
#include <string>

#define MODE_PWM      1
#define MODE_POS      0
/*
#define OFF_OFF    0
#define ON_OFF     1
#define OFF_ON     2
#define ON_ON      3
*/
#define OFF_OFF    0
#define ON_OFF     5
#define OFF_ON     6
#define ON_ON      7
#define PEN_ON     4
#define ENC_RESET  0x0c

#define KP_NORMAL  0x08000800
#define KP_ZERO    0x00000000

#define M_L        0 
#define M_R        1  
 

const double ROVER_TREAD = 0.116;  // Distance between left and right wheel contact points [m]
const double ROVER_D = ROVER_TREAD / 2.0;
const double TIRE_DIAMETER_MM = 60.0;
const double TIRE_CIRCUMFERENCE = TIRE_DIAMETER_MM * PI;
const double ENC_PULSES_PER_MOTOR_TURN = 3.0;
const double ENC_QUADRATURE_MULTIPLIER = 4.0;
const double GEAR_RATIO = 99.002;
const double ENC_COUNTS_PER_TURN =
  ENC_PULSES_PER_MOTOR_TURN * ENC_QUADRATURE_MULTIPLIER * GEAR_RATIO;
const double ENC_PER_MM = ENC_COUNTS_PER_TURN/TIRE_CIRCUMFERENCE;
extern float std_motor_param[];
extern float motor_param[];

enum MotorList{
  F_L = 0,
  F_R = 1,
  R_L = 2,
  R_R = 3
};

enum IndexOfMotorParam{
  MAX_SPEED = 0,
  MAX_RAD   = 1,
  K_V2MP    = 2,
  K_P       = 3,
  K_I       = 4,
  K_D       = 5,
  PAD_DEAD  = 6,
  PAD_MAX   = 7,
  MAX_ACC   = 8
};

extern double  ctl_v_com[2];
extern double v_com[2];
extern double motor_wheel_scale[2];
extern int16_t m_com[2];
extern int16_t prev_m[2];
extern double v_enc[2];
extern double v_diff[2];
extern double avr_v[2];
extern double prev_v_diff[2];

extern int32_t enc[2];
extern int32_t old_enc[2];

extern int currentFlag;

extern uint32_t pid_time;

int setO_EN(int data);
int setO_EN(int data, Wrc940* memmap);
int getO_EN();
int getO_EN(Wrc940* memmap);
int setCtrlMode(int ctrl_mode);
int setCtrlMode(int ctrl_mode, Wrc940* memmap);
int getCtrlMode();
int getCtrlMode(Wrc940* memmap);
void aStick2V();
void memCom2V();
void applyWheelSpeedScale();
void ctl2Vcom();
void getRoverV();
void pidControl();
void posControl();
void resetOdom();
uint8_t chkWDT(Wrc940* memmap);
void setMortorParam2Std();

#endif
