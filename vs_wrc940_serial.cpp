#include "vs_wrc940_serial.h"
#include "vs_wrc940_memmap.h"
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator>  
void serialInit(){
  Serial.begin(115200);
  return;
}
std::string rcvMsgViaSerial(SERIAL_BUF_SIZE, '\0');
std::string rcvMsg4ROS(SERIAL_BUF_SIZE, '\0');
int persMsgViaSerial(){
  int return_val = 0;

  static int16_t rosMsgBytes = 0;
  char tmp;

  while(Serial.available()){
  if(rcvMsgViaSerial.empty()){
    tmp = Serial.read();
    rcvMsgViaSerial.push_back(tmp);
  }
  if(rcvMsgViaSerial[0] == 'r' || rcvMsgViaSerial[0] == 'R' || rcvMsgViaSerial[0] == 'w' || rcvMsgViaSerial[0] == 'W'
    || rcvMsgViaSerial[0] == 'E' ||rcvMsgViaSerial[0] == 'C' || rcvMsgViaSerial[0] == 'S' || rcvMsgViaSerial[0] == 'P'|| rcvMsgViaSerial[0] == 'L'){
    while(Serial.available()){
      tmp = Serial.read();

      if(tmp == 'r' || tmp == 'R' || tmp == 'w' || tmp == 'W'
        /*|| tmp == 'E' || tmp == 'C' */|| tmp == 'S' || tmp == 'P' || tmp == 'L'){
        rcvMsgViaSerial.clear();
      }

      if(rcvMsgViaSerial.length() >= SERIAL_BUF_SIZE){
        rcvMsgViaSerial.clear();
      }

      rcvMsgViaSerial.push_back(tmp);
      if(tmp == '\n'){
        //Serial.print(rcvMsgViaSerial.c_str());
        return_val += checkI2cAddrOfMsg(rcvMsgViaSerial.c_str(), rcvMsgViaSerial.length()-1);
        
        rcvMsgViaSerial.clear();
        break;
      }
    }

    

  }else if(rcvMsgViaSerial[0] == 0xff){
    while(rosMsgBytes==0){
      if(!Serial.available()){
        return return_val;
      }

      tmp = Serial.read();
      rcvMsgViaSerial.push_back(tmp);

      if(rcvMsgViaSerial.length() >= 4){
        rosMsgBytes = (int)rcvMsgViaSerial[2];
        rosMsgBytes += (int)rcvMsgViaSerial[3] << 8;
        rosMsgBytes += 8;

        if(rosMsgBytes > SERIAL_BUF_SIZE){
          rosMsgBytes = SERIAL_BUF_SIZE;
        }

        Serial.println(rosMsgBytes);
      }
    }
    while(rcvMsgViaSerial.length() < rosMsgBytes){
      if(!Serial.available()){
        return return_val;
      }

      tmp = Serial.read();
      rcvMsgViaSerial.push_back(tmp);

    }
    if(rcvMsg4ROS.length()+rosMsgBytes < SERIAL_BUF_SIZE){
      std::reverse_copy(rcvMsgViaSerial.begin(), rcvMsgViaSerial.end(), std::back_inserter(rcvMsg4ROS));
      rcvMsgViaSerial.clear();
      rosMsgBytes = 0;
    }else{
      rcvMsg4ROS.clear();
      std::reverse_copy(rcvMsgViaSerial.begin(), rcvMsgViaSerial.end(), std::back_inserter(rcvMsg4ROS));
      rcvMsgViaSerial.clear();
      rosMsgBytes = 0;

    }

    return return_val;
    

  }else{
    rcvMsgViaSerial.clear();
  }


  }

  return return_val;

}
int readMsgViaSerial(){
  return persMsgViaSerial();
}
int readMsg4ROS(){
  if(rcvMsg4ROS.empty()){
    return -1;
  }

  int return_val = rcvMsg4ROS[rcvMsg4ROS.length()-1];

  rcvMsg4ROS.pop_back();

  return return_val;


}


