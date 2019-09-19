// Arduino MAVLink Simulation
//
// (c) 2019 David Wyss
//
// Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink

#include <mavlink.h>


// Parameter setup
// These parameters may be entered manually, parsed from another protocol or simulated locally.
// If your input protocol does not provide values for all of these, parameters may be left at their default values.
// All parameters set here are passed onward through serial output in MAVLink format.

//Basic UAV Parameters
uint8_t system_id = 1;        // MAVLink system ID. Leave at 0 unless you need a specific ID.
uint8_t component_id = 0;     // Should be left at 0. Set to 190 to simulate mission planner sending a command
uint8_t system_type = 1;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
uint8_t autopilot_type = 0;   // Autopilot type. Usually set to 0 for generic autopilot with all capabilities
uint8_t system_mode = 64;     // Flight mode. 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode
uint32_t custom_mode = 0;     // Usually set to 0          
uint8_t system_state = 4;     // 0 = unknown, 3 = standby, 4 = active
uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter

// Flight parameters
float roll = 0;         // Roll angle in degrees
float pitch = 0;        // Pitch angle in degrees
float yaw = 0;          // Yaw angle in degrees
int16_t heading = 0;      // Geographical heading angle in degrees
float lat = 47.379945;    // GPS latitude in degrees (example: 47.123456)
float lon = 8.539970;     // GPS longitude in degrees
float alt = 0.0;        // Relative flight altitude in m
float groundspeed = 0.0; // Groundspeed in m/s
float airspeed = 0.0;    // Airspeed in m/s
float climbrate = 0.0;    // Climb rate in m/s, currently not working
float throttle = 0.0;     // Throttle percentage

// GPS parameters
int16_t gps_sats = 0;    // Number of visible GPS satellites
int32_t gps_alt = 0.0;  // GPS altitude (Altitude above MSL)
float gps_hdop = 100.0;     // GPS HDOP
uint8_t fixType = 0;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix

// Battery parameters
float battery_remaining = 0.0;  // Remaining battery percentage
float voltage_battery = 0.0;    // Battery voltage in V
float current_battery = 0.0;    // Battery current in A


void setup() {
  
   // Enable serial output. Default: 57600 baud, can be set higher if needed
   Serial.begin(57600);
}



//Main loop: Send generated MAVLink data via serial output
void loop() {
  
  // Send MAVLink heartbeat
  command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);

  //Send battery status
  command_status(system_id, component_id, battery_remaining, voltage_battery, current_battery);

  // Send GPS and altitude data
  command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, gps_alt, heading, groundspeed, gps_hdop, gps_sats);

  // Send HUD data (speed, heading, climbrate etc.)
  command_hud(system_id, component_id, airspeed, groundspeed, heading, throttle, alt, climbrate);

  // Send attitude data to artificial horizon
  command_attitude(system_id, component_id, upTime, roll, pitch, yaw);

  //Delay: send messages at about 10Hz
  delay(100);
}



/************************************************************
* @brief Sends a MAVLink heartbeat message, needed for the system to be recognized
* @param Basic UAV parameters, as defined above
* @return void
*************************************************************/

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message
  Serial.write(buf, len);
}


/************************************************************
* @brief Send some system data parameters (battery, etc)
* @param 
* @return void
*************************************************************/
       
void command_status(uint8_t system_id, uint8_t component_id, float battery_remaining, float voltage_battery, float current_battery) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, voltage_battery * 1000.0, current_battery * 100.0, battery_remaining, 0, 0, 0, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}


/************************************************************
* @brief Sends current geographical location (GPS position), altitude and heading
* @param lat: latitude in degrees, lon: longitude in degrees, alt: altitude, heading: heading
* @return void
*************************************************************/

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat * 10000000.0, lon * 10000000.0, alt * 1000.0, gps_hdop * 100.0, 65535, groundspeed, 65535, gps_sats);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Send globalgps command
  command_globalgps(system_id, component_id, upTime, lat, lon, alt, gps_alt, heading);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

/************************************************************
* @brief Send some core parameters such as speed to the MAVLink ground station HUD
* @param 
* @return void
*************************************************************/
       
void command_hud(int8_t system_id, int8_t component_id, float airspeed, float groundspeed, int16_t heading, float throttle, float alt, float climbrate) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_vfr_hud_pack(system_id, component_id, &msg, airspeed, groundspeed, heading, throttle, alt * 1000.0, climbrate);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

/************************************************************
* @brief Send attitude and heading data to the primary flight display (artificial horizon)
* @param roll: roll in degrees, pitch: pitch in degrees, yaw: yaw in degrees, heading: heading in degrees
* @return void
*************************************************************/
       
void command_attitude(int8_t system_id, int8_t component_id, int32_t upTime, float roll, float pitch, float yaw) {

  //Radian -> degree conversion rate
  float radian = 57.2958;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_attitude_pack(system_id, component_id, &msg, upTime, roll/radian, pitch/radian, yaw/radian, 0, 0, 0); 

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}



/************************************************************
* @brief Sends Integer representation of location, only for ijnternal use (do not call directly)
* @param lat: latitude, lon: longitude, alt: altitude, gps_alt: altitude above MSL, heading: heading
* @return void
*************************************************************/
       
void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading) {

  int16_t velx = 0; //x speed
  int16_t vely = 0; //y speed
  int16_t velz = 0; //z speed


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, lat * 10000000.0, lon * 10000000.0, gps_alt * 1000.0, alt * 1000.0, velx, vely, velz, heading);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}
