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

  wheelRun(-20,20);
  delay(2000);

  wheelRun(0,0);
  delay(1000);

  wheelRun(20,-20);
  delay(2000);

  wheelRun(0,0);
  delay(1000);

}




