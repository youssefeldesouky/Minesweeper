 #include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
const uint8_t L = 19, R = 20;
int32_t l_ticks, r_ticks;
const uint16_t maxTicks = 84;  //PPR
double l_pos, r_pos;
int8_t l_sign = 1, r_sign = 1;

void signCB(const std_msgs::Int8MultiArray &data){
  l_sign = data.data[0];
  r_sign = data.data[1];
}

ros::Subscriber<std_msgs::Int8MultiArray> sign_sub("sign", &signCB);
std_msgs::Float32MultiArray joint_states;
ros::Publisher js_pub("joint_positions", &joint_states);


void setup() {
  // put your setup code here, to run once:
  pinMode(L, INPUT_PULLUP);
  pinMode(R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L), leftENC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R), rightENC, CHANGE);
  nh.initNode();
  nh.subscribe(sign_sub);
  joint_states.data_length = 4;
  nh.advertise(js_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
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
