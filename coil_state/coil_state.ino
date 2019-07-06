#include <ros.h>
#include <std_msgs/Int8.h>
const uint8_t IND = 4, IR = 5;
std_msgs::Int8 STATE;
ros::NodeHandle nh;
uint8_t _state;
ros::Publisher coil_pub("coil_state", &STATE);
void setup() {
  pinMode(IND, INPUT);
  pinMode(IR, INPUT);
  nh.initNode();
  nh.advertise(coil_pub);
}

void loop() {
  if(digitalRead(IND) && digitalRead(IR)){
    _state = 1;
  }else if(digitalRead(IND) && !digitalRead(IR)){
    _state = 2;
  }else if(!digitalRead(IND) && !digitalRead(IR)){
    _state = 0;
  }
  STATE.data = _state;
  coil_pub.publish(&STATE);
  nh.spinOnce();
  delay(20);
}
