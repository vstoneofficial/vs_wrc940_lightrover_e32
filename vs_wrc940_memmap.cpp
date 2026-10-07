#include "vs_wrc940_memmap.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <sstream>
#include "vs_wrc940_lightrover_e32.h"

#define POS_OF_I2CADDR 1

SemaphoreHandle_t xMutexHandle = NULL;
const TickType_t  xTicksToWait = portTICK_RATE_MS;


const uint8_t initialMemmap[MAP_SIZE] = {0x00, 0x00, 0xed, 0x05, 0xff, 0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x10, 0x00, 0x10,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//itoa[a] = 'A'
//itoa[3] = '3'
static char hexToA[] = "0123456789ABCDEF";
// '1' => 0x01, 'a' => 0x0a
uint8_t cToHex(uint8_t c){
  if (isDigit(c)) {
    return (c - 0x30);
  }
  if (isUpperCase(c)) {
    return (c - 0x37);
  }
  if (isLowerCase(c)) {
    return (c - 0x57);
  }
  return 0;
}

String int2HexLittleString(int data, uint8_t length) {
  std::stringstream ss;

	ss << std::hex << data;
	std::string bigMes = ss.str();
	std::string littleMes;

	if (bigMes.length() % 2) {
		bigMes.insert(0, "0");
	}

	int i;
	for (i = bigMes.length() / 2; i < length; i++) {
		bigMes.insert(0, "00");
	}
	

	for (i = bigMes.length() - 2; i >= 0; i -= 2) {
		littleMes.append(bigMes, i, 2);
	}
	//printf("%s\n", littleMes.c_str());

  String str = littleMes.c_str();

	return str;
}

String int2HexBigString(int data, uint8_t length) {
  std::stringstream ss;

	ss << std::hex << data;
	std::string bigMes = ss.str();

	if (bigMes.length() % 2) {
		bigMes.insert(0, "0");
	}

	int i;
	for (i = bigMes.length() / 2; i < length; i++) {
		bigMes.insert(0, "00");
	}

  String str = bigMes.c_str();
	
	return str;
}

Wrc940::Wrc940(){}
Wrc940::Wrc940(uint8_t addr){
  devAddr = addr;
}
void Wrc940::initMemmap(double cutOffLevel){
  uint16_t cutOffHex = (uint16_t)((cutOffLevel/29.7)*0xfff);
  Wrc940::writeMemmap(MU8_O_EN, (uint8_t*)&initialMemmap, 0xF0);
  Wrc940::readAll();
  //Wrc940::write2Byte(MU16_SD_VI, cutOffHex);
  Wrc940::u16Map(MU16_SD_VI, cutOffHex); 
  sendWriteMapTime = millis();

}
void Wrc940::memMapClean(){  
  int i = 0;
  for(i = 0; i<MAP_SIZE; i++){
    memMap[i] = 0x00; 
  }

  return;
}
int Wrc940::readMemmap(uint8_t addr, uint8_t readLength){

  int i = 0;

  //if(xStatus == pdTRUE){

  unsigned char readAddr = 0x00;
  if(addr < 0x01){
    readAddr = 0xFF;
  }else{
    readAddr = addr - 0x01;
  }
  Wire.beginTransmission(devAddr);
  Wire.write(readAddr);
  Wire.endTransmission();
  Wire.requestFrom((int)devAddr, (int)(readLength+1));
  while(!Wire.available()){
  }
  Wire.read();
  while (Wire.available()){
    memMap[i+ addr] = Wire.read();
    i++;
  }

  return i;
}
int Wrc940::readAll(){

  readMemmap(0x00,64);
  readMemmap(0x40,64);
  readMemmap(0x80,20);

  return 1;
}
int Wrc940::writeMemmap(uint8_t addr, uint8_t data[], uint8_t writeLength){
  
  int i;

  uint8_t writeByte;

  if(writeLength == 0){
    return -1;
  }

  Wire.beginTransmission(devAddr);
  Wire.write(addr);
  for(i = 0; i < writeLength ; i++){
    writeByte = data[i];
    Wire.write(writeByte);
  }
  Wire.endTransmission();
  
  return i;
}
int Wrc940::write4Byte(uint8_t addr, int32_t data){

  int i;
  uint8_t writeByte;

  Wire.beginTransmission(devAddr);
  Wire.write(addr);
  for(i = 0; i <= 3; i++){
    writeByte = data >> i * 8;
    Wire.write(writeByte);
  }
  Wire.endTransmission();
  
  return 1;
}
int Wrc940::write2Byte(uint8_t addr, int16_t data){

  uint8_t upperByte = data >> 8;
  uint8_t lowerByte = data;
  Wire.beginTransmission(devAddr);
  Wire.write(addr);
  Wire.write(lowerByte);
  Wire.write(upperByte);
  Wire.endTransmission();

  return 1;
}
int Wrc940::write1Byte(uint8_t addr, uint8_t data){

  Wire.beginTransmission(devAddr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();

  return 1;
}
uint8_t Wrc940::getAddr(){
  return devAddr;
}
void Wrc940::checkMsg(String rcvMsg){
  checkMsg(rcvMsg, NULL, NULL);
}
void Wrc940::checkMsg(String rcvMsg, WiFiClient* client){
  checkMsg(rcvMsg, client, NULL);
}
void Wrc940::checkMsg(String rcvMsg, uint8_t viaBlt){
  checkMsg(rcvMsg, NULL, viaBlt);
}

void Wrc940::checkMsg(String rcvMsg, WiFiClient* client, uint8_t viaBlt){
  if(rcvMsg.length()%2 && rcvMsg[0] != 'E' && rcvMsg[0] != 'e' && rcvMsg[0] != 'C' && rcvMsg[0] != 'c' && rcvMsg[0] != 'S' && rcvMsg[0] != 's' 
    && rcvMsg[0] != 'P' && rcvMsg[0] != 'p' && rcvMsg[0] != 'L' && rcvMsg[0] != 'l'){
    //Serial.println("ERR: The number of characters is odd");
    return;
  }
  if(rcvMsg[0] == 'r' || rcvMsg[0] == 'R' || rcvMsg[0] == 'w' || rcvMsg[0] == 'W'){
    struct cmd rcvCmd;
    
    rcvCmd.cmdType = rcvMsg[0];
    if(!isSpace(rcvMsg[1])){
      ///Serial.println("ERR: Cmd format");
      return;
    }
    if(isHexadecimalDigit(rcvMsg[2]) && isHexadecimalDigit(rcvMsg[3])){
      rcvCmd.addr = ((cToHex(rcvMsg[2]) << 4) | (cToHex(rcvMsg[3])));
    }
    if(!isSpace(rcvMsg[4])){
      ///Serial.println("ERR: Cmd format");
      return;
    }
    if(rcvCmd.cmdType == 'r' || rcvCmd.cmdType == 'R'){
      if(rcvMsg.length() == 8){
        if(isHexadecimalDigit(rcvMsg[5]) && isHexadecimalDigit(rcvMsg[6])){
          rcvCmd.readLength = ((cToHex(rcvMsg[5]) << 4) | (cToHex(rcvMsg[6])));
          sendMap2pc(rcvCmd, client, viaBlt);
          return;
        }
      }else{
        ///Serial.println("ERR: Read format");
        return;
      }
    }else{
      
      int i = 5;
      int j = 0;
      uint8_t value[MAP_SIZE];
      while(rcvMsg[i] != '\n'){
        if(isHexadecimalDigit(rcvMsg[i]) && isHexadecimalDigit(rcvMsg[i+1])){
          value[j] = ((cToHex(rcvMsg[i]) << 4) | (cToHex(rcvMsg[i+1])));
          i += 2;
          j++;
                    
        }else{
          rcvCmd.valueCount = 0;
          ///Serial.println("ERR: Write format");
          return;
        }
      }
      rcvCmd.valueCount = j;
      rcvCmd.value = value;

      setWriteMapViaMsg(rcvCmd);
      return;
    }
  }else if(rcvMsg[0] == 'E'/* || rcvMsg[0] == 'e'*/){
    sendEnc2dev(client, viaBlt);
  }else if(rcvMsg[0] == 'C'){
    clearEnc();
  }

  return;
}
void Wrc940::sendEnc2dev(){
  sendEnc2dev(NULL, NULL);
}

void Wrc940::sendEnc2dev(uint8_t viaBlt){
  sendEnc2dev(NULL, viaBlt);
}

void Wrc940::sendEnc2dev(WiFiClient* client){
  sendEnc2dev(client, NULL);
}

void Wrc940::sendEnc2dev(WiFiClient* client, uint8_t viaBlt){
  int32_t encL;
  int32_t encR;
  String msg = "";

  encL = s32Map(MS32_M_POS0);
  encR = s32Map(MS32_M_POS1);

  msg = int2HexBigString(encL, 4);
  msg += " ";
  msg += int2HexBigString(encR, 4);
  
  Serial.println(msg);

  if(viaBlt == VIA_BLE){
    //ble::sendMultiByte(msg, i*2);
  }
  
  return;

}
void Wrc940::clearEnc(){
  wrc940.u8Map(MU8_TRIG ,wrc940.u8Map(MU8_TRIG) | 0x80);
  return;
}

void Wrc940::sendMap2pc(struct cmd rcvCmd){
  sendMap2pc(rcvCmd, NULL, NULL);
}

void Wrc940::sendMap2pc(struct cmd rcvCmd, WiFiClient* client){
  sendMap2pc(rcvCmd, client, NULL);
}

void Wrc940::sendMap2pc(struct cmd rcvCmd, WiFiClient* client, uint8_t viaBlt){
  if(rcvCmd.cmdType == 'r' || rcvCmd.cmdType == 'R'){
    int i = 0;
    int j = 0;
    char msg[MAP_SIZE];

    if(rcvCmd.readLength == 0x00){
      readAll();

      msg[2] = '\0';

      Serial.println("     0|  1|  2|  3|  4|  5|  6|  7|  8|  9|  A|  B|  C|  D|  E|  F");

      for(i = 0; i < 16; i++){
        Serial.print(i,HEX);
        Serial.print("0");
        Serial.print(": ");

        for(j = 0; j < 16; j++){
          msg[0]= hexToA[memMap[(i*16)+j] >> 4];
          msg[1]= hexToA[memMap[(i*16)+j] & 0x0f];
          Serial.print(msg);
          Serial.print("  ");
        }
        Serial.println();
      }
      Serial.println();
      
    }else{
      if(rcvCmd.addr < 0x00){
        if(rcvCmd.addr + rcvCmd.readLength > 0x90){
          rcvCmd.readLength -= rcvCmd.addr + rcvCmd.readLength - 0x90;
        }
        //Serial.println(rcvCmd.addr);
        //Serial.println(rcvCmd.readLength);
        readMemmap(rcvCmd.addr, rcvCmd.readLength);
      }
      
      for(i = 0; i < rcvCmd.readLength; i++){
          msg[i*2]= hexToA[memMap[rcvCmd.addr + i] >> 4];
          msg[(i*2)+1]= hexToA[memMap[rcvCmd.addr + i] & 0x0f];
      }
      msg[i*2] = '\0';
      Serial.print(msg);
      Serial.println();
      
      if(client != NULL){
        client->println("HTTP/1.1 200 OK");
        client->println("Content-type:text/html");
        client->println();
        client->print(msg);
        client->println();
      }

      if(viaBlt == VIA_BLE){

      }

      
      
      
    }
  }
}

void Wrc940::setWriteMapViaMsg(struct cmd rcvCmd){
  if(rcvCmd.cmdType == 'w' || rcvCmd.cmdType == 'W'){
    int i;
    for(i = 0; i < rcvCmd.valueCount; i++){
      u8Map(rcvCmd.addr +i, *(rcvCmd.value +i));
    }
    
  }
}
void Wrc940::sendWriteMap(){

  uint16_t i;
  uint8_t headAddr;
  uint8_t length;
  for(i = 0x12; i < 0x90; i++){
    headAddr = i; 
    length = 0;
    while(writeFlag[i]){
      writeFlag[i] = 0x00;
      length++;
      i++;
      if(i >= 0x90){
        break;
      }      
    }
    writeMemmap(headAddr,&memMap[headAddr], length);
  }
  for(i = 0xe; i < 0x12; i++){
    headAddr = i; 
    length = 0;
    while(writeFlag[i]){
      writeFlag[i] = 0x00;
      length++;
      i++;
      if(i >= MAP_SIZE){
        break;
      }      
    }
    writeMemmap(headAddr,&memMap[headAddr], length);
  }
}
int8_t Wrc940::s8Map(uint8_t addr){
  return (*(int8_t *)(&memMap[addr]));
}
int8_t Wrc940::s8Map(uint8_t addr, int8_t data){
  (*(int8_t *)(&memMap[addr])) = data;
  (*(uint8_t *)(&writeFlag[addr])) = 0x01;
  return (*(int8_t *)(&memMap[addr]));
}
uint8_t Wrc940::u8Map(uint8_t addr){
  return (*(uint8_t *)(&memMap[addr]));
}
uint8_t Wrc940::u8Map(uint8_t addr, uint8_t data){
    (*(uint8_t *)(&memMap[addr])) = data;
    (*(uint8_t *)(&writeFlag[addr])) = 0x01;
  return (*(uint8_t *)(&memMap[addr]));
}
int16_t Wrc940::s16Map(uint8_t addr){
  return (*(int16_t *)(&memMap[addr]));
}
int16_t Wrc940::s16Map(uint8_t addr, int16_t data){
    (*(int16_t *)(&memMap[addr])) = data;
    (*(uint16_t *)(&writeFlag[addr])) = 0x0101;
  return (*(int16_t *)(&memMap[addr]));
}
uint16_t Wrc940::u16Map(uint8_t addr){
  return (*(uint16_t *)(&memMap[addr]));
}
uint16_t Wrc940::u16Map(uint8_t addr, uint16_t data){
    (*(uint16_t *)(&memMap[addr])) = data;
    (*(uint16_t *)(&writeFlag[addr])) = 0x0101;
  return (*(uint16_t *)(&memMap[addr]));
}
int32_t Wrc940::s32Map(uint8_t addr){
  return (*(int32_t *)(&memMap[addr]));
}
int32_t Wrc940::s32Map(uint8_t addr, int32_t data){
    (*(int32_t *)(&memMap[addr])) = data;
    (*(uint32_t *)(&writeFlag[addr])) = 0x01010101;
  return (*(int32_t *)(&memMap[addr]));
}
uint32_t Wrc940::u32Map(uint8_t addr){
  return (*(uint32_t *)(&memMap[addr]));
}
uint32_t Wrc940::u32Map(uint8_t addr, uint32_t data){
    (*(uint32_t *)(&memMap[addr])) = data;
    (*(uint32_t *)(&writeFlag[addr])) = 0x01010101;
  return (*(uint32_t *)(&memMap[addr]));
}
uint8_t Wrc940::checkWriteFlag(uint8_t addr){
  return writeFlag[addr];
}
double Wrc940::getVin(){
  uint16_t memmapV;
  double vin;
  readMemmap(MU16_M_VI , 0x02);
  memmapV = Wrc940::u16Map(MU16_M_VI);

  vin = ((double)memmapV / 0x0fff)*13.2;

  return vin;
}

int checkI2cAddrOfMsg(String rcvMsg, int rcvMsgCount){
  return checkI2cAddrOfMsg(rcvMsg, rcvMsgCount, NULL);
}

int checkI2cAddrOfMsg(String rcvMsg, int rcvMsgCount, uint8_t viaBlt){
  if(rcvMsg[rcvMsgCount] != '\n'){
    //Serial.println("ERR: no \\n");
    return 0;
  
  }else if( rcvMsg[0] == 'E' || rcvMsg[0] == 'e' || rcvMsg[0] == 'C' || rcvMsg[0] == 'c' || rcvMsg[0] == 'S' || rcvMsg[0] == 's' 
          || rcvMsg[0] == 'P' || rcvMsg[0] == 'p' || rcvMsg[0] == 'L' || rcvMsg[0] == 'l' ){
    //Serial.println("get spiffs command");
    wrc940.checkMsg(rcvMsg, viaBlt);
      
  }else if(rcvMsgCount >= 7 ){
    uint8_t i2cAddr = 0xff;
    if(isHexadecimalDigit(rcvMsg[POS_OF_I2CADDR]) && isHexadecimalDigit(rcvMsg[POS_OF_I2CADDR+1])){
      i2cAddr = ((cToHex(rcvMsg[POS_OF_I2CADDR]) << 4) | (cToHex(rcvMsg[POS_OF_I2CADDR+1])));
      rcvMsg.remove(1, 2);
    }else if(rcvMsg[POS_OF_I2CADDR] == ' '){
      i2cAddr = 0x10;        
    }
    if(i2cAddr == 0x10){
      wrc940.checkMsg(rcvMsg, viaBlt);
    }else{
      ///Serial.print("ERR: invalid I2C addr");
      //printf(" 0x%2x \n", i2cAddr);
      return 0;
    }
      
  }else if(rcvMsgCount == 0){
    //Serial.println('\n');
    return 0;
      
  }else{
    //Serial.println("ERR: Msg length too short");
    return 0;
  }

  return 1;    
  
}
uint8_t setRoverParam(String rcvMsg){

  int i = 0;
  String name = "";
  String value = "";
  while(!isSpace(rcvMsg[i])){
    name += rcvMsg[i];
    i++;
  }
  i++;
  while(i <= rcvMsg.length()){
    value += rcvMsg[i];
    i++;
  }

  return 1;
}

uint8_t getRoverParam(String rcvMsg){

  uint8_t tmp[256];
  int i;
  for(i = 0; i < 256; i++){
    tmp[i] = 0;
  }
  tmp[0] = '\n';

  String msg = (char*)tmp;

  if(msg != "null"){
    Serial.println(msg);
    return 1;
  }

  return 0;

}
Wrc940 wrc940(0x10);

