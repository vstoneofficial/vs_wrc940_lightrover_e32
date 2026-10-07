#ifndef WRC940_I2C_H
#define WRC940_I2C_H

#include <Arduino.h>
#include <Wire.h>
#include "vs_wrc940_memmap.h"

void i2cMasterInit();
void i2cSlaveInit(uint8_t addr);





#endif /* I2C_H */
