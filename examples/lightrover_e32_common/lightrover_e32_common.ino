/**
 * Drover common sketch
 *
 * This sketch controls the rover from external interfaces and publishes
 * odometry, sensor status, and LiDAR raw data to ROS 2 via micro-ROS.
 */

/*******************************************************************
 * Wireless interface setting
 *
 * Select one interface with WRC940_WIRELESS_MODE.
 *   WIRELESS_OFF: wireless control disabled
 *   USE_WIFI    : Wi-Fi control
 *   USE_BLE     : BLE control
 *   USE_BTC     : Bluetooth Classic control
 */
#define WIRELESS_OFF 0
#define USE_WIFI     1
#define USE_BLE      2
#define USE_BTC      3
#define WRC940_WIRELESS_MODE   USE_WIFI

/*******************************************************************
 * ROS 2 transport setting
 *
 * Select one transport with WRC_ROS_SERIAL_MODE.
 *   MODE_OFF   : micro-ROS disabled
 *   MODE_WIFI  : micro-ROS over Wi-Fi
 *   MODE_SERIAL: micro-ROS over USB serial
 *
 * Set WRC_ROS_DOMAIN_ID to the same value as ROS_DOMAIN_ID on the PC.
 */
#define MODE_OFF    0
#define MODE_WIFI   1
#define MODE_SERIAL 2
#define WRC_ROS_SERIAL_MODE MODE_WIFI
#define ROS2_SERIAL_BAUDRATE 921600UL
#define WRC_ROS_DOMAIN_ID 0

/*******************************************************************
 * LiDAR setting
 *
 * Set LIDAR_USE to LIDAR_ON when using LiDAR, or LIDAR_OFF when unused.
 */
#define LIDAR_OFF 0
#define LIDAR_ON  1
#define LIDAR_USE LIDAR_ON

/*******************************************************************
 * Watchdog timer setting
 *
 * If communication from an external controller stops for WDTIME ms,
 * the motor output is stopped. Set 0 to disable ESP32-side timeout clear.
 */
#define WDTIME 500

#define WRC940_USE_NIMBLE
#include <vs_wrc940_lightrover_e32.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/init_options.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>

#include <WiFi.h>
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <algorithm>

/*******************************************
 * IO setting
 *
 * ioSet corresponds to IO4-IO9.
 *   0: disabled
 *   1: digital input
 *   2: digital input pull-up
 *   3: digital input pull-down
 *   4: digital output
 *   5: analog input
 *   6: PWM output
 */
const int8_t ioSet[6]={0,0,0,0,0,0};
const bool ioOn=std::any_of(ioSet, ioSet + 6, [](int x) {
    return x >= 1;
});

/*******************************************
 * Wi-Fi setting
 */
char* ui_path = "/index.html";  // Path of the HTML controller UI
char* ssid     = "SSID";  // Wi-Fi SSID (2.4 GHz only)
char* password = "password";  // Wi-Fi password

/*******************************************
 * ROS 2 agent setting for Wi-Fi transport
 */
char*    ROSserverIP = "192.168.1.1";  // IP address of the PC running the micro-ROS Agent
uint16_t serverPort = 8888;  // micro-ROS Agent UDP port

/*******************************************
 * ROS 2 entities
 */
rcl_publisher_t publisher_odo;
rcl_publisher_t publisher_sensor;
rcl_publisher_t publisher_lidar_0;
rcl_publisher_t publisher_lidar_1;
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
static const rmw_qos_profile_t sub_last_one_profile = 
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

geometry_msgs__msg__Twist pubmsg_odo;
geometry_msgs__msg__Twist submsg_twist;
std_msgs__msg__Int16MultiArray pubmsg_sensor;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} ros2state;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCINITCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("ROS INIT ERR: "); Serial.println(#fn); return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

const int publish_rate = 50;      // Publish rate [Hz]
const int ros_agent_ping_interval_ms = 500;
const int ros_agent_ping_timeout_ms = 100;
const int ros_agent_ping_attempts = 3;
const int ros_agent_check_interval_ms = 2000;
const int ros_agent_check_timeout_ms = 20;

bool ros2_support_created = false;
bool ros2_node_created = false;
bool ros2_subscriber_created = false;
bool ros2_publisher_odo_created = false;
bool ros2_publisher_sensor_created = false;
bool ros2_publisher_lidar_0_created = false;
bool ros2_publisher_lidar_1_created = false;
bool ros2_timer_created = false;
bool ros2_executor_created = false;
bool ros2_sensor_memory_created = false;
bool ros2_lidar_raw_0_memory_created = false;
bool ros2_lidar_raw_1_memory_created = false;

micro_ros_utilities_memory_conf_t sensor_msg_memory_conf(){
  micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_ros2_type_sequence_capacity = 2;
  conf.max_basic_type_sequence_capacity = 2;
  return conf;
}

micro_ros_utilities_memory_conf_t lidar_raw_msg_memory_conf(){
  micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_ros2_type_sequence_capacity = 0;
  conf.max_basic_type_sequence_capacity = LIDAR_CHUNK_POINTS;
  return conf;
}

void error_loop(){
  while(1){
    static bool led_brink = true;
    LED(led_brink);
    led_brink = !led_brink;
    Serial.println("ROS ERR");
    delay(1000);
  }
}

void wait_agent(){
  static int64_t last_time = -1;
  if (last_time == -1) { last_time = uxr_millis();}

  if(uxr_millis() - last_time > ros_agent_ping_interval_ms){
#if (WRC_ROS_SERIAL_MODE == MODE_WIFI)
    if(WiFi.status() != WL_CONNECTED){
      ros2state = WAITING_AGENT;
      last_time = uxr_millis();
      return;
    }
#endif
    ros2state = (RMW_RET_OK == rmw_uros_ping_agent(ros_agent_ping_timeout_ms, ros_agent_ping_attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
    last_time = uxr_millis();
  }
}

void check_agent(){
  static int64_t last_time = -1;
  if (last_time == -1) { last_time = uxr_millis();}

  if(uxr_millis() - last_time > ros_agent_check_interval_ms && ros2state == AGENT_CONNECTED){
    ros2state = (RMW_RET_OK == rmw_uros_ping_agent(ros_agent_check_timeout_ms, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    last_time = uxr_millis();

    if(ros2state == AGENT_CONNECTED){
      Serial.println("ROS CA-> OK");
    }else{
      Serial.println("ROS CA-> DISCONNECTED");
    }
  }
}

bool ros2_init(){
  allocator = rcl_get_default_allocator();
  Serial.println("FIN rcl_get_default_allocator");

  // Create init options and set ROS 2 Domain ID.
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCINITCHECK(rcl_init_options_init(&init_options, allocator));
  RCINITCHECK(rcl_init_options_set_domain_id(&init_options, WRC_ROS_DOMAIN_ID));
  RCINITCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCSOFTCHECK(rcl_init_options_fini(&init_options));
  ros2_support_created = true;
  Serial.print("FIN rclc_support_init domain_id=");
  Serial.println(WRC_ROS_DOMAIN_ID);

  // create node
  RCINITCHECK(rclc_node_init_default(&node, "megarover", "", &support));
  ros2_node_created = true;
  Serial.println("FIN rclc_node_init_default");

  // create subscriber
  RCINITCHECK(rclc_subscription_init(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "rover_twist",
    &sub_last_one_profile));
  ros2_subscriber_created = true;
  Serial.println("FIN rclc_subscription_init_default");

  RCINITCHECK(rclc_publisher_init_best_effort(
  &publisher_odo,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "rover_odo"));
  ros2_publisher_odo_created = true;
  Serial.println("FIN publisher_odo");

  RCINITCHECK(rclc_publisher_init_best_effort(
    &publisher_sensor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "rover_sensor"));
  ros2_publisher_sensor_created = true;
  Serial.println("FIN publisher_sensor");

  RCINITCHECK(rclc_publisher_init_best_effort(
    &publisher_lidar_0,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "rover_lidar_raw_0"));
  ros2_publisher_lidar_0_created = true;
  Serial.println("FIN publisher_lidar_0");

  RCINITCHECK(rclc_publisher_init_best_effort(
    &publisher_lidar_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "rover_lidar_raw_1"));
  ros2_publisher_lidar_1_created = true;
  Serial.println("FIN publisher_lidar_1");

  // create timer,
  RCINITCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000/publish_rate),
    timer_callback));
  ros2_timer_created = true;
  

  // create executor
  const uint8_t num_handles = 2; // total number of handles = #subscriptions + #timers
  Serial.println("START rclc_executor_init");
  RCINITCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  ros2_executor_created = true;
  Serial.println("FIN rclc_executor_init");
  RCINITCHECK(rclc_executor_add_subscription(&executor, &subscriber, &submsg_twist, &subscription_callback, ON_NEW_DATA));
  RCINITCHECK(rclc_executor_add_timer(&executor, &timer));

  // init message memory
  micro_ros_utilities_memory_conf_t conf = sensor_msg_memory_conf();
  if(!micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        &pubmsg_sensor,
        conf)) 
  {
    Serial.println("ROS INIT ERR: micro_ros_utilities_create_message_memory");
    return false;
  }
  ros2_sensor_memory_created = true;
  Serial.println("FIN sensor memory");

  micro_ros_utilities_memory_conf_t lidar_raw_conf = lidar_raw_msg_memory_conf();
  if(!micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        &pubmsg_lidar_raw_0,
        lidar_raw_conf))
  {
    Serial.println("ROS INIT ERR: LiDAR raw 0 memory");
    return false;
  }
  ros2_lidar_raw_0_memory_created = true;
  Serial.println("FIN lidar raw 0 memory");

  if(!micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        &pubmsg_lidar_raw_1,
        lidar_raw_conf))
  {
    Serial.println("ROS INIT ERR: LiDAR raw 1 memory");
    return false;
  }
  ros2_lidar_raw_1_memory_created = true;
  Serial.println("FIN lidar raw 1 memory");

  //pubmsg_lidar_raw_0.layout.dim.size = 0;
  pubmsg_lidar_raw_0.layout.data_offset = 0;
  pubmsg_lidar_raw_0.data.size = LIDAR_CHUNK_POINTS;
  //pubmsg_lidar_raw_1.layout.dim.size = 0;
  pubmsg_lidar_raw_1.layout.data_offset = 0;
  pubmsg_lidar_raw_1.data.size = LIDAR_CHUNK_POINTS;

  Serial.println("FIN ros2_init");
  return true;
}


//destroy entities when re-connect
void destroy_entities(){
  if(ros2_support_created){
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }

  if(ros2_lidar_raw_1_memory_created){
    micro_ros_utilities_memory_conf_t lidar_raw_conf = lidar_raw_msg_memory_conf();
    micro_ros_utilities_destroy_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
      &pubmsg_lidar_raw_1,
      lidar_raw_conf);
    ros2_lidar_raw_1_memory_created = false;
  }

  if(ros2_lidar_raw_0_memory_created){
    micro_ros_utilities_memory_conf_t lidar_raw_conf = lidar_raw_msg_memory_conf();
    micro_ros_utilities_destroy_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
      &pubmsg_lidar_raw_0,
      lidar_raw_conf);
    ros2_lidar_raw_0_memory_created = false;
  }

  if(ros2_sensor_memory_created){
    micro_ros_utilities_memory_conf_t conf = sensor_msg_memory_conf();
    micro_ros_utilities_destroy_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      &pubmsg_sensor,
      conf);
    ros2_sensor_memory_created = false;
  }

  if(ros2_executor_created){
    rclc_executor_fini(&executor);
    ros2_executor_created = false;
  }
  if(ros2_timer_created){
    rcl_timer_fini(&timer);
    ros2_timer_created = false;
  }
  if(ros2_subscriber_created){
    rcl_subscription_fini(&subscriber, &node);
    ros2_subscriber_created = false;
  }
  if(ros2_publisher_lidar_1_created){
    rcl_publisher_fini(&publisher_lidar_1, &node);
    ros2_publisher_lidar_1_created = false;
  }
  if(ros2_publisher_lidar_0_created){
    rcl_publisher_fini(&publisher_lidar_0, &node);
    ros2_publisher_lidar_0_created = false;
  }
  if(ros2_publisher_sensor_created){
    rcl_publisher_fini(&publisher_sensor, &node);
    ros2_publisher_sensor_created = false;
  }
  if(ros2_publisher_odo_created){
    rcl_publisher_fini(&publisher_odo, &node);
    ros2_publisher_odo_created = false;
  }
  if(ros2_node_created){
    rcl_node_fini(&node);
    ros2_node_created = false;
  }
  if(ros2_support_created){
    rclc_support_fini(&support);
    ros2_support_created = false;
  }
}

//timer_callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pubmsg_odo.linear.x = ((v_enc[M_R] + (-1.0*v_enc[M_L]))/2.0);
    pubmsg_odo.angular.z = ((v_enc[M_R] - (-1.0*v_enc[M_L]))/ROVER_TREAD);
    //pubmsg_odo.linear.z  = lift.s16Map(MS16_L_MPM)/10000.0;
    //pubmsg_odo.linear.y = wrc940.s16Map(MS16_S_XS);
    //pubmsg_odo.angular.y = wrc940.s16Map(MS16_S_ZS);
    rcl_ret_t rc = rcl_publish(&publisher_odo, &pubmsg_odo, NULL);
    if(rc == RCL_RET_OK){

    }

    //pubmsg_sensor.data.data[0] = wrc940.u16Map(MU16_M_DI);
    pubmsg_sensor.data.data[1] = (uint16_t)(wrc940.u16Map(MU16_M_VI));
    pubmsg_sensor.data.size = pubmsg_sensor.data.capacity;
    RCSOFTCHECK(rcl_publish(&publisher_sensor, &pubmsg_sensor, NULL));

    if(lidar_scan_ready){
      lidar_scan_ready = false;
      rcl_ret_t rc0 = rcl_publish(&publisher_lidar_0, &pubmsg_lidar_raw_0, NULL);
      rcl_ret_t rc1 = rcl_publish(&publisher_lidar_1, &pubmsg_lidar_raw_1, NULL);
      if(rc0 != RCL_RET_OK || rc1 != RCL_RET_OK){
        Serial.print("publish lidar rc0=");
        Serial.print(rc0);
        Serial.print(" rc1=");
        Serial.println(rc1);
      }
    }
  }
}


//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  wrc940.s16Map(MS16_S_XS, (int16_t)(msg->linear.x*1000));
  wrc940.s16Map(MS16_S_ZS, (int16_t)(msg->angular.z*1000));
  setCtrlMode(MODE_POS);
  setO_EN(ON_ON);
  wrc940.u16Map(MU16_WDT, WDTIME);
}

/*******************************************
 * Setup
 */
void setup()
{

  // Motor control parameters.
  std_motor_param[MAX_SPEED] = 0.1;     // Maximum linear speed [m/s]
  std_motor_param[MAX_RAD]   = 2.51;    // Maximum angular speed [rad/s]
  std_motor_param[K_P]       = 1.4;     // P gain
  std_motor_param[K_I]       = 0.2;     // I gain
  std_motor_param[K_D]       = 1.2;     // D gain
  std_motor_param[PAD_DEAD]  = 30.0;    // Controller dead zone
  std_motor_param[PAD_MAX]   = 127.0;   // Controller maximum input
  // Compensate for the difference in speed between the left and right drive trains.
  // A larger right value makes the robot steer further to the left while driving forward.
  motor_wheel_scale[M_L] = 1.00;
  motor_wheel_scale[M_R] = 1.00;
  
  delay(5000);

  Serial.begin(115200);
  Serial.println("Start");
  

    // Configure micro-ROS transport
  #if (WRC_ROS_SERIAL_MODE == MODE_WIFI)
    // using ROS over Wi-Fi
    set_microros_wifi_transports(ssid, password, ROSserverIP, serverPort);
  #else
    // using ROS via USB-Serial
    set_microros_transports();
    delay(500);
  #endif

  ros2state = WAITING_AGENT;

  // Wireless controller setup when ROS is not used
  /*#if (WRC940_WIRELESS_MODE == USE_WIFI)
    wifiInit((char* )ssid, (char* )password);
    serverInit((char* )ui_path);       // Start the web controller server
  #endif*/

  Serial.println("ROS setuped");

  delay(50);

  ledInit();
  delay(10);
  Serial.println("LED setuped");

  if(LIDAR_USE){
    lidarInit();
    delay(10);
    Serial.println("LiDAR setuped");
  }

  // I2C setup
  i2cMasterInit();
  delay(10);
  Serial.println("I2C setuped");

  spiInit();
  delay(10);
  Serial.println("SPI setuped");

  // Initialize memory map.
  wrc940.initMemmap(4.0);

  if(ioOn){
    for(int i=0;i<6;i++){
      wrc940.u8Map(MU8_IO_MODE0+i,ioSet[i]);
    }
    IoInit();
    delay(10);
    Serial.println("IO setuped");
  }

  // Start sync timer task
  delay(200);

  setupInterruptTimer(); 
  
  delay(100);

  // Initialize encoder values.
  clearEnc();
  delay(50);


}

/*******************************************
 * Main loop
 */
void loop(){

  int code = NO_INPUT;    // Current input/source status
  LED(0);

  if(LIDAR_USE){
    lidarAct();
  }

  if(ioOn){
    chkIo();
  }

  if(Serial.available()){                                                                // Serial input received
    code = SERIAL_ACCES;
    LED(1);                                                                // Turn LED on while handling input
    
    if(!persMsgViaSerial()){                                                          // Parse serial input
      // Invalid serial access.
      code = NO_INPUT;                                                                // Ignore invalid serial access
    }    
  }

  if(isInterrupt){
    isInterrupt = false;

    if(updatePad() || existsPadInput()){                                                  // Gamepad input received
      LED(1);                                                                // Turn LED on while handling input
      code = PAD_INPUT;                                                                   // Mark source as gamepad input
      chkPadInput();                                                                      // Apply gamepad input
    }else{
      memCom2V();
    }

    // Run motor control.
    if(wrc940.u8Map(MU8_P_STTS) == 0x01 || wrc940.u8Map(MU8_TRIG) & 0x10 || getCtrlMode() == MODE_POS){
      posControl();                   // Position control
    }else{
      pidControl();                   // Velocity PID control
    } 



    // Send pending memory map writes from ESP32 to STM32
    wrc940.sendWriteMap();      

  }

  if(WRC_ROS_SERIAL_MODE != MODE_OFF){
  switch (ros2state) {
    case WAITING_AGENT:
      wait_agent();
      LED(0);
      //printf("Wait starting your micro-ros Agent\n");
      break;
    case AGENT_AVAILABLE:
      ros2state = (true == ros2_init()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (ros2state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      if (ros2state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      ros2state = WAITING_AGENT;
      break;
    default:
      break;
  }
  }

  (void)code;

  delay(1);

}




