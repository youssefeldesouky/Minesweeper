#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
const uint8_t IND = A0, IR = 5;
std_msgs::Int8 STATE;
std_msgs::Int16 coil_state;
ros::NodeHandle nh;
uint8_t _state=0;
ros::Publisher coil_pub("coil_state", &STATE);
ros::Publisher coil_state_pub("coil_value", &coil_state);
void setup() {
  pinMode(IND, INPUT);
  pinMode(IR, INPUT_PULLUP);
  nh.initNode();
  nh.advertise(coil_pub);
  nh.advertise(coil_state_pub);
  Serial.begin(9600);
}

void loop() {  
  if(analogRead(A0) >=900 && digitalRead(IR)){
    _state = 1;
  }
  else if(analogRead(A0) >= 900 && !digitalRead(IR)){
    _state = 2;
  }
  else if(analogRead(A0) < 900 && digitalRead(IR)){
    _state = 0;
  }
  else 
  {
    _state = 0;
    }
   Serial.println(_state);
  STATE.data = _state;
  coil_state.data = digitalRead(A0);
  coil_state_pub.publish(&coil_state);
  coil_pub.publish(&STATE);
  nh.spinOnce();
  delay(20);
}
