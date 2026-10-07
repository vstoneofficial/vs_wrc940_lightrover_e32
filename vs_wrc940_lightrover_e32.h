#ifndef WRC940_DROVER_H
#define WRC940_DROVER_H


#include "vs_wrc940_memmap.h"
#include "vs_wrc940_i2c.h"
#include "vs_wrc940_motor.h"
#include "vs_wrc940_serial.h"
#include "vs_wrc940_spi.h"
#include "vs_wrc940_wifi.h"
#ifndef WRC940_USE_NIMBLE
#include "vs_wrc940_ble.h"
#endif
#include "vs_wrc940_lidar.h"
#include <vector>
#include <string>

enum wrc940StatusCode{
    NO_INPUT     = 0,
    HTTP_ACCES   = 1,
    HTTP_PAD     = 3,
    HTTP_SYSTEM  = WIFI_VIN,
    HTTP_BAD     = BAD_REQUEST,
    SERIAL_ACCES = 6,
    BLE_ACCES    = 7,
    BTC_ACCES    = 8,
    ROS_CTRL     = 9,
    PAD_INPUT    = 10

};

extern uint16_t waitTime;
extern int waitStartTime;

extern bool isInterrupt;

void ledInit();
void LED(int cmd);
void IoInit();
void chkIo();
void chkPadInput();

void IRAM_ATTR onTimer();
void setupInterruptTimer();

void wheelRun(int32_t spdL, int32_t spdR);
void readEnc(int32_t *encL, int32_t *encR);
void clearEnc();

#endif /* WRC940_DROVER_H */
