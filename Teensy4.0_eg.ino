#define HWSERIAL Serial4
#define debug Serial3

#include "src/ardupilotmega/mavlink.h"

int sysid = 1;                   ///< ID 20 for this airplane
int compid = MAV_COMP_ID_ONBOARD_COMPUTER;     ///< The component sending the message is the IMU, it could be also a Linux process
int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
  
      //uint8_t system_type = MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
  
uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

void setup() {
  // put your setup code here, to run once:
  HWSERIAL.begin(9600);
  debug.begin(57600);

  

}

void loop() {
      // mavlink_message_t msg;
      // uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      // mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
      
      // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      // HWSERIAL.write(buf, len);
      

      static uint8_t conf;
      (conf<255)?conf++:conf=0;
      
      mavlink_message_t msg1;
      uint8_t buf1[MAVLINK_MAX_PACKET_LEN];
      mavlink_msg_command_long_pack(1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg1,
                              1 , 1, MAV_CMD_SET_MESSAGE_INTERVAL, conf, MAVLINK_MSG_ID_ATTITUDE, 0, 0, 0, 0, 0, 0);

      uint16_t len1 = mavlink_msg_to_send_buffer(buf1,&msg1);
      debug.write(" msg send ");
      debug.write(buf1,len1);
      
      
      mavlink_message_t msg2;
      mavlink_status_t  read_stat;

      //memset(&msg2,0,sizeof(msg2));
      while(HWSERIAL.available()>0) 
      {
        uint8_t c = HWSERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg2, &read_stat)) {
            switch (msg2.msgid) 
            {
                case MAVLINK_MSG_ID_ATTITUDE: {
                  mavlink_attitude_t attitude;
                  mavlink_msg_attitude_decode(&msg2, &attitude);

                  // Access the attitude data
                  float roll = attitude.roll;
                  float pitch = attitude.pitch;
                  float yaw = attitude.yaw;

                  // Do something with the attitude data
                  // ...
                  
                  Serial.println("Roll: "+String(roll)+" Pitch: "+String(pitch)+" Yaw: "+String(yaw));
                  break;
                }
            }
        }
      }   
      delay(250);   
}
