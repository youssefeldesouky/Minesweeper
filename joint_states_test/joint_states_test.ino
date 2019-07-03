//Arduino Uno is a total piece of shit
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

const uint8_t L = 2, R = 3;
uint32_t l_ticks, r_ticks;
const uint8_t maxTicks = 60;  //PPR
double l_pos, r_pos;
uint8_t l_sign = 1, r_sign = 1;
int16_t left_cmd, right_cmd;

void signCB(const std_msgs::Int8MultiArray &data){
  l_sign = data.data[0];
  r_sign = data.data[1];
}

void leftCmdCB(const std_msgs::Int16 &data){
  left_cmd = (int16_t)data.data;
}

void rightCmdCB(const std_msgs::Int16 &data){
  right_cmd = (int16_t)data.data;
}

ros::Subscriber<std_msgs::Int8MultiArray> sign_sub("sign", &signCB);
ros::Subscriber<std_msgs::Int16> left_cmd_sub("left_cmd", &leftCmdCB);
ros::Subscriber<std_msgs::Int16> right_cmd_sub("left_cmd", &rightCmdCB);
std_msgs::Float32MultiArray joint_states;
ros::Publisher js_pub("joint_positions", &joint_states);

void setup() {
  pinMode(L, INPUT_PULLUP);
  pinMode(R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L), leftENC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R), rightENC, CHANGE);
  nh.initNode();
  nh.subscribe(sign_sub);
  nh.subscribe(left_cmd_sub);
  nh.subscribe(right_cmd_sub);
  joint_states.data_length = 4;
  nh.advertise(js_pub);
}

void loop() {
  /*
  
    insert mapping functionality here.

  */
  l_pos = ((double)l_ticks/maxTicks) * 2 * PI;
  r_pos = ((double)r_ticks/maxTicks) * 2 * PI;
  float positions[4] = {l_pos, r_pos, l_pos, r_pos};
  joint_states.data = positions;
  js_pub.publish(&joint_states);
  nh.spinOnce();
  delay(20);
}

void leftENC(){
  l_ticks += l_sign;
}

void rightENC(){
  r_ticks += r_sign;
}
//Receive data from 2 encoders (2 Interrupts)
//Control 4 Motors with 2 control signals (one for each side)
//tick-meters could overflow
