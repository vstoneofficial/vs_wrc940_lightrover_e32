#include "vs_wrc940_spi.h"
#include <Arduino.h>
#include <SPI.h>

const uint8_t CMD_config_mode_enter[] =     {0x01,0x43,0x00,0x01, 0x00,0x00,0x00,0x00, 0x00};
const uint8_t CMD_config_mode_exit[] =      {0x01,0x43,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00};
const uint8_t CMD_config_mode_exit2[] =     {0x01,0x43,0x00,0x00, 0x5a,0x5a,0x5a,0x5a, 0x5a};
const uint8_t CMD_set_mode_and_lock[] =     {0x01,0x44,0x00,0x01, 0x03,0x00,0x00,0x00, 0x00};
const uint8_t CMD_query_model_and_mode[] =  {0x01,0x45,0x00,0x5a, 0x5a,0x5a,0x5a,0x5a, 0x5a};
const uint8_t CMD_vibration_enable[] =      {0x01,0x4d,0x00,0x00, 0x01,0xff,0xff,0xff, 0xff};
const uint8_t CMD_vibration_disnable[] =    {0x01,0x4d,0x00,0xff, 0xff,0xff,0xff,0xff, 0xff};
const uint8_t CMD_query_DS2_analog_mode[] = {0x01,0x41,0x00,0x5a, 0x5a,0x5a,0x5a,0x5a, 0x5a};
const uint8_t CMD_set_DS2_native_mode[] =   {0x01,0x4f,0x00,0xff, 0xff,0x03,0x00,0x00, 0x00};
const uint8_t CMD_read_data[] =             {0x01,0x42,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00};
const uint8_t CMD_read_data2[] =            {0x01,0x42,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
                                                    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00};

uint8_t PAD_SCLK = 16;
uint8_t PAD_MISO = 18;
uint8_t PAD_MOSI = 17;
uint8_t PAD_SS   = 15;
const uint32_t PAD_SPI_CLK = 2500000;

uint8_t pad_buf[30];
uint8_t prev_pad_buf[30];

int pad_time = 0;

uint8_t pad_transfer_delay_us = 0;
uint8_t pad_spi_bus = FSPI;

Pad pad_data;
spi_t* spi_pad=NULL;

static void clearPadData(){
  pad_data.button = 0;
  pad_data.right_stick.x = 0;
  pad_data.right_stick.y = 0;
  pad_data.left_stick.x = 0;
  pad_data.left_stick.y = 0;
}

void spiInit(){

  pad_transfer_delay_us = 0;

  pad_spi_bus = FSPI;
  PAD_SCLK = 15;
  PAD_MISO = 17;
  PAD_MOSI = 16;
  PAD_SS   = 18;
  pad_transfer_delay_us = 10;

  spi_pad = spiStartBus(pad_spi_bus, PAD_SPI_CLK, SPI_MODE2, SPI_LSBFIRST);

  pinMode(PAD_MISO, INPUT);
  pinMode(PAD_SS, OUTPUT);
  digitalWrite(PAD_SS, HIGH);

  spiAttachSCK(spi_pad, PAD_SCLK);
  spiAttachMISO(spi_pad, PAD_MISO);
  spiAttachMOSI(spi_pad, PAD_MOSI);

  int i = 0;
  for(i = 0; i < sizeof(pad_buf); i++){
    pad_buf[i] = 0x00;
  }
  clearPadData();
  
  checkAN();
}
void rwPad(uint8_t* sendData, uint8_t* rcvData, uint16_t dataLength){
  int i = 0;
  
  digitalWrite(PAD_SS, LOW);
  for(i = 0; i < dataLength; i++){
    spiTransferBytes(spi_pad, (uint8_t* )&sendData[i], (uint8_t* )&rcvData[i], 1);
    delayMicroseconds(10);
  }
  digitalWrite(PAD_SS, HIGH);

}
int updatePad(){

  if(millis() - pad_time < 15){
    return 0;
  }
  pad_time = millis();

  rwPad((uint8_t*)CMD_read_data, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_read_data));
  if(checkAN()){
    return 0;
  }
  if(memcmp(pad_buf, prev_pad_buf, sizeof(CMD_read_data)) == 0){
    return 0;
  }

  int i;
  for(i = 0; i<sizeof(CMD_read_data); i++){
    prev_pad_buf[i] = pad_buf[i];
  }

  
  getButton();
  getAnalogStick();

  return 1;
}
int checkAN(){
  static uint32_t last_config_attempt_ms = 0;
  const uint32_t config_retry_interval_ms = 1000;

  if(pad_buf[ANF_ADDR] != 0x73){
    clearPadData();

    if(millis() - last_config_attempt_ms < config_retry_interval_ms){
      return 1;
    }
    last_config_attempt_ms = millis();

    //Serial.println("setANALOG");
    delay(15);
    rwPad((uint8_t*)CMD_config_mode_enter, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_config_mode_enter));

    delay(15);
    rwPad((uint8_t*)CMD_set_mode_and_lock, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_set_mode_and_lock));

    delay(15);
    rwPad((uint8_t*)CMD_config_mode_exit, (uint8_t*)pad_buf, (uint16_t)sizeof(CMD_config_mode_exit));
    delay(15);



    return 1;
  }

  return 0;
}
void getButton(){
  pad_data.button = ~((pad_buf[BTN_ADDR] << 8) | pad_buf[BTN_ADDR +1]);
}
void getAnalogStick(){
  int8_t tmp[4];
  int i;
  
  for(i = 0; i < 4; i++){
    tmp[i] = (int8_t)pad_buf[ANS_ADDR+i];
    if((uint8_t)tmp[i] <= 0x7F){
      tmp[i] = 127 - tmp[i];
    }else{
      tmp[i] = -(128 + tmp[i]);
    }
  }

  pad_data.right_stick.x = -tmp[0];
  pad_data.right_stick.y = tmp[1];
  pad_data.left_stick.x  = -tmp[2];
  pad_data.left_stick.y  = tmp[3];

}
uint8_t checkBTN(uint16_t comparisonData){
  
  if(pad_data.button & comparisonData){
    return 1;
  }else{
    return 0;
  } 
}
void VS_C2ToSerial(){
  int i = 0;
  
  for(i = 0; i < 30; i++){
    Serial.print(pad_buf[i],HEX);
    Serial.print(' ');  
  }
  Serial.println();
}
bool existsPadInput(){
  if(pad_data.button){
    return true;
  }
  if(pad_data.right_stick.x != 0 || pad_data.right_stick.y != 0 || pad_data.left_stick.x != 0 || pad_data.left_stick.y != 0){
    return true;
  }

  return false;
  
}



