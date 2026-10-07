/**
 * LiDAR acquisition and raw scan publishing helpers.
 */
#ifndef WRC940_LIDAR_H
#define WRC940_LIDAR_H

#include "vs_wrc940_memmap.h"
#include <Arduino.h>
#include <LDS_YDLIDAR_X2_X2L.h>

#define LIDAR_PWM_FREQ    10000
#define LIDAR_PWM_BITS    11
#define LIDAR_PWM_CHANNEL    2 // ESP32 PWM channel for LiDAR motor speed control
#define LIDAR_SCAN_POINTS 360
#define LIDAR_CHUNK_POINTS 180
#define LIDAR_DEG_PER_POINT (360.0f / LIDAR_SCAN_POINTS)
#define DEG_TO_RAD_F 0.017453292519943295f

typedef struct std_msgs__msg__UInt16MultiArray std_msgs__msg__UInt16MultiArray;

extern std_msgs__msg__UInt16MultiArray pubmsg_lidar_raw_0;
extern std_msgs__msg__UInt16MultiArray pubmsg_lidar_raw_1;
extern volatile bool lidar_scan_ready;

void lidarInit();
int lidar_serial_read_callback();
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length);
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,bool scan_completed);
void lidar_info_callback(LDS::info_t code, String info);
void lidar_error_callback(LDS::result_t code, String aux_info);
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin);
void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed);
void lidarAct();

#endif /* SERIAL_H */
