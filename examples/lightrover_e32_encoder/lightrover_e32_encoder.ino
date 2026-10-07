#include <vs_wrc940_lightrover_e32.h>
#include <Arduino.h>


/*******************************************
 * Setup
 */
void setup()
{
  std_motor_param[MAX_SPEED] = 0.1;
  std_motor_param[MAX_RAD]   = 2.51;
  std_motor_param[K_P]       = 1.4;
  std_motor_param[K_I]       = 0.2;
  std_motor_param[K_D]       = 1.2;
  Serial.begin(115200);
  i2cMasterInit();
  wrc940.initMemmap(8.0);
  setupInterruptTimer();
  delay(100);
  clearEnc();
  delay(50);

}


/*******************************************
 * Main loop
 */
void loop(){

  int initialEncL = 0;
  int initialEncR = 0;

  readEnc(&initialEncL, &initialEncR);

  //Serial.println(initialEncL);

  wheelRun(-20, 20);

  int encL = 0;
  int encR = 0;
  while(1){
    readEnc(&encL, &encR);

    encL -= initialEncL;
    encR -= initialEncR;

    Serial.println(encR/ENC_PER_MM);

    if(encL/ENC_PER_MM < -50.0 || encR/ENC_PER_MM > 50.0){
      break;
    }

    delay(10);
  }

  wheelRun(0, 0);
  delay(5000);
  

}
