/**
 * Memory map definitions used by ESP32.
 * 
 */
#ifndef WRC940_MEMMAP_H
#define WRC940_MEMMAP_H

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// Memory map size.
#define MAP_SIZE 0x1A0

// Memory map access macros.
#define R_MU8(x)  (*(uint8_t  *)(&wrc940.memMap[x]))
#define R_MU16(x) (*(uint16_t *)(&wrc940.memMap[x]))
#define R_MU32(x) (*(uint32_t *)(&wrc940.memMap[x]))

#define R_MS8(x)  (*(int8_t   *)(&wrc940.memMap[x]))
#define R_MS16(x) (*(int16_t  *)(&wrc940.memMap[x]))
#define R_MS32(x) (*(int32_t  *)(&wrc940.memMap[x]))

extern SemaphoreHandle_t xMutexHandle;
extern const TickType_t  xTicksToWait; // Semaphore wait time [ms].

// Command received over Serial/Bluetooth/Wi-Fi.
struct cmd{
  uint8_t cmdType;    // Command type: r=read, w=write.
  uint8_t addr;       // First memory map address to read/write.
  uint8_t *value;     // Values to write.
  uint8_t valueCount; // Number of bytes to write.
  uint8_t readLength; // Number of bytes to read.
};
enum memmapAddr{
  MU16_SYSNAME  = 0x00, // R: system name.
  MU16_FIRMREV  = 0x02, // R: firmware revision.
  MU32_TRIPTIME = 0x04, // R: elapsed time since startup [ms].
  MU16_WDT      = 0x08, // R/W: watchdog timeout [ms].
  MU8_MODE      = 0x0d, // R: DIP switch state detected at startup, bits 0-3.
  MU16_POWOFF_T = 0x0e, // R/W: power-off delay [ms].
  MU8_O_EN      = 0x10, // R/W: output enable flags, bit0=CH0, bit1=CH1, bit2=CH2.
  MU8_TRIG      = 0x11, // R/W: trigger flags for position, waypoint, and encoder reset.
  MU16_SD_VI    = 0x12, // R/W: shutdown voltage threshold.
  MU16_OD_DI    = 0x14, // R/W: DI mask that disables outputs when the selected DI is active.
  MU16_SPD_T0   = 0x16, // R/W: reference interval for speed calculation [ms].
  MU16_MOVE_T0  = 0x18, // R/W: integration time.
  MS16_FB_PG0   = 0x20, // R/W: proportional gain.
  MS16_FB_PG1   = 0x22, // R/W: proportional gain.
  MU16_FB_ALIM0 = 0x24, // R/W: acceleration limit.
  MU16_FB_ALIM1 = 0x26, // R/W: acceleration limit.
  MU16_FB_DLIM0 = 0x28, // R/W: deceleration limit.
  MU16_FB_DLIM1 = 0x2a, // R/W: deceleration limit.
  MU16_FB_OLIM0 = 0x2c, // R/W: output limit, 100% = 0x1000.
  MU16_FB_OLIM1 = 0x2e, // R/W: output limit, 100% = 0x1000.
  MU16_FB_PCH0  = 0x30, // R/W: punch output, 100% = 0x1000.
  MU16_FB_PCH1  = 0x32, // R/W: punch output, 100% = 0x1000.
  MS32_T_POS0   = 0x40, // R/W: target encoder value.
  MS32_T_POS1   = 0x44, // R/W: target encoder value.
  MS32_A_POS0   = 0x48, // R/W: value added to T_POS0.
  MS32_A_POS1   = 0x4c, // R/W: value added to T_POS1.
  MS16_T_OUT0   = 0x50, // R/W: output offset for CH0.
  MS16_T_OUT1   = 0x52, // R/W: output offset for CH1.
  MS16_T_OUT2   = 0x54, // R/W: output offset for CH2.
  MS32_M_POS0   = 0x60, // R: measured encoder value.
  MS32_M_POS1   = 0x64, // R: measured encoder value.
  MS16_M_SPD0   = 0x68, // R: measured speed calculated from M_POS0 delta.
  MS16_M_SPD1   = 0x6a, // R: measured speed calculated from M_POS1 delta.
  MS16_M_OUT0   = 0x6c, // R: calculated motor output value, 100% = 0x1000.
  MS16_M_OUT1   = 0x6e, // R: calculated motor output value, 100% = 0x1000.
  MS32_WP_PX    = 0x80, // R: NanoRover world-frame X position [um].
  MS32_WP_PY    = 0x84, // R: NanoRover world-frame Y position [um].
  MS16_WP_TH    = 0x88, // R: NanoRover world-frame yaw angle [mrad].
  MU16_M_VI     = 0x90, // R: input voltage, 3.3 V x 4 = 13.2 V at 0x0fff.
  MS32_P_DIS    = 0xa0, // R/W: position-control travel distance [um].
  MS16_P_RAD    = 0xa4, // R/W: position-control turn angle [mrad].
  MS16_P_ACC    = 0xa6, // R/W: position-control acceleration [mm/s^2].
  MS16_P_SPD    = 0xa8, // R/W: position-control travel speed [mm/s].
  MU8_P_STTS    = 0xaa, // R: position-control state, 0x01=moving, 0x00=waypoint reached.
  MS16_S_XS     = 0xac, // R/W: velocity-control linear speed [mm/s].
  MS16_S_ZS     = 0xae, // R/W: velocity-control yaw rate [mrad/s].

  // User IO mode registers for IO4-IO9.
  // Mode values: 0=disabled, 1=digital input, 2=input pull-up,
  // 3=input pull-down, 4=digital output, 5=analog input, 6=PWM output.
  MU8_IO_MODE0  = 0xB0, // R/W: IO4 mode.
  MU8_IO_MODE1  = 0xB1, // R/W: IO5 mode.
  MU8_IO_MODE2  = 0xB2, // R/W: IO6 mode.
  MU8_IO_MODE3  = 0xB3, // R/W: IO7 mode.
  MU8_IO_MODE4  = 0xB4, // R/W: IO8 mode.
  MU8_IO_MODE5  = 0xB5, // R/W: IO9 mode.
  MU8_IO_DO     = 0xB6, // R/W: digital output state, bits 0-5 correspond to IO4-IO9.
  MU8_IO_DI     = 0xB7, // R: digital input state, bits 0-5 correspond to IO4-IO9.
  MU16_IO_AI0   = 0xB8, // R: IO4 analog raw value.
  MU16_IO_AI1   = 0xBA, // R: IO5 analog raw value.
  MU16_IO_AI2   = 0xBC, // R: IO6 analog raw value.
  MU16_IO_AI3   = 0xBE, // R: IO7 analog raw value.
  MU16_IO_AI4   = 0xC0, // R: IO8 analog raw value.
  MU16_IO_AI5   = 0xC2, // R: IO9 analog raw value.
  MU16_IO_PWM0  = 0xC4, // R/W: IO4 PWM duty.
  MU16_IO_PWM1  = 0xC6, // R/W: IO5 PWM duty.
  MU16_IO_PWM2  = 0xC8, // R/W: IO6 PWM duty.
  MU16_IO_PWM3  = 0xCA, // R/W: IO7 PWM duty.
  MU16_IO_PWM4  = 0xCC, // R/W: IO8 PWM duty.
  MU16_IO_PWM5  = 0xCE, // R/W: IO9 PWM duty.
};

// Initial memory map values for 0x10-0x8f.
extern const uint8_t initialMemmap[MAP_SIZE];



// Wrc940 memory map class.
class Wrc940{
  public:
  Wrc940();
  Wrc940(uint8_t addr);

  int sendWriteMapTime;

  void initMemmap(double cutOffLevel);
  void memMapClean();
  virtual int readMemmap(uint8_t addr, uint8_t readLength);
  int readAll();
  int writeMemmap(uint8_t addr, uint8_t data[], uint8_t writeLength);
  int write4Byte(uint8_t addr, int32_t data);
  int write2Byte(uint8_t addr, int16_t data);
  int write1Byte(uint8_t addr, uint8_t data);
  uint8_t getAddr();

  void checkMsg(String rcvMsg);
  void checkMsg(String rcvMsg, WiFiClient* client);
  void checkMsg(String rcvMsg, uint8_t viaBle);
  void checkMsg(String rcvMsg, WiFiClient* client, uint8_t viaBlt);
  void sendEnc2dev();
  void sendEnc2dev(uint8_t viaBlt);
  void sendEnc2dev(WiFiClient* client);
  void sendEnc2dev(WiFiClient* client, uint8_t viaBlt);
  void clearEnc();

  void sendMap2pc(struct cmd rcvCmd);
  void sendMap2pc(struct cmd rcvCmd, WiFiClient* client);
  void sendMap2pc(struct cmd rcvCmd, WiFiClient* client, uint8_t viaBle);
  void setWriteMapViaMsg(struct cmd rcvCmd);
  virtual void sendWriteMap();
  


  int8_t   s8Map(uint8_t addr);
  int8_t   s8Map(uint8_t addr, int8_t data);
  uint8_t  u8Map(uint8_t addr);
  uint8_t  u8Map(uint8_t addr, uint8_t data);
  int16_t  s16Map(uint8_t addr);
  int16_t  s16Map(uint8_t addr, int16_t data);
  uint16_t u16Map(uint8_t addr);
  uint16_t u16Map(uint8_t addr, uint16_t data);
  int32_t  s32Map(uint8_t addr);
  int32_t  s32Map(uint8_t addr, int32_t data);
  uint32_t u32Map(uint8_t addr);
  uint32_t u32Map(uint8_t addr, uint32_t data);

  uint8_t checkWriteFlag(uint8_t addr);

  double getVin();

  protected:
  uint8_t devAddr;
  uint8_t memMap[MAP_SIZE];
  uint8_t writeFlag[MAP_SIZE];
};

extern Wrc940 wrc940;
//extern Wrc940 wrc022;

//extern static char hexToA[16];
uint8_t cToHex(uint8_t c);
String int2HexLittleString(int data, uint8_t length);
String int2HexBigString(int data, uint8_t length);


int checkI2cAddrOfMsg(String rcvMsg, int rcvMsgCount);
int checkI2cAddrOfMsg(String rcvMsg, int rcvMsgCount, uint8_t viaBle);

uint8_t setRoverParam(String rcvMsg);
uint8_t getRoverParam(String rcvMsg); 

#endif /* MEMMAP_H */


