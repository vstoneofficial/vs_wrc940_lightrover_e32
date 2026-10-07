/**
 * Serial transport helpers for the WRC940 memory map protocol.
 */
#ifndef WRC940_SERIAL_H
#define WRC940_SERIAL_H

#include "vs_wrc940_memmap.h"
#include <iostream>
#include <vector>
#include <algorithm>    
#include <iterator>  

#define SERIAL_BUF_SIZE 512

extern std::string rcvMsgViaSerial;
extern std::string rcvMsg4ROS;

void serialInit();
int persMsgViaSerial();
int readMsgViaSerial();
int readMsg4ROS();

#endif /* SERIAL_H */
