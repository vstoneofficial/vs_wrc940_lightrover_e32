#include <Wire.h>
#include "vs_wrc940_i2c.h"
#include "vs_wrc940_memmap.h"
void i2cMasterInit(){
  pinMode(1, INPUT);
  pinMode(2, INPUT);

  uint32_t frequency=100000;
  Wire.begin(1, 2, frequency);

  delay(100);

  return;
}
void i2cSlaveInit(uint8_t addr){
  pinMode(1, INPUT);
  pinMode(2, INPUT);

  Wire.begin(addr);

  delay(100);

  return;
}

