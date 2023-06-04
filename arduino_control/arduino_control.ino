//Arduino Uno is a total piece of shit
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#define L_EN_F 4
#define R_EN_F 5 
#define L_EN_B 6
#define R_EN_B 7
#define L_PWM_F 8
#define L_PWM_B 9
#define R_PWM_B 10
#define R_PWM_F 11
uint8_t L_PWM = 0 , R_PWM = 0 ;
ros::NodeHandle nh;
uint8_t l_pwm=0,r_pwm=0;
const uint8_t L = 2, R = 3;

int32_t l_ticks, r_ticks;
const uint16_t maxTicks = 50;  //PPR
double l_pos, r_pos;
int8_t l_sign = 1, r_sign = 1;
int16_t left_cmd, right_cmd;


void leftCmdCB(const std_msgs::Int16 &data){
  left_cmd = (int16_t)data.data;
}

void rightCmdCB(const std_msgs::Int16 &data){
  right_cmd = (int16_t)data.data;
}

ros::Subscriber<std_msgs::Int16> left_cmd_sub("left_cmd", &leftCmdCB);
ros::Subscriber<std_msgs::Int16> right_cmd_sub("right_cmd", &rightCmdCB);
std_msgs::Int16 right_msg;
std_msgs::Int16 left_msg;

void setup() {
  pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);c
  pinMode(12,OUTPUT);

  nh.initNode();
  nh.subscribe(left_cmd_sub);
  nh.subscribe(right_cmd_sub);
digitalWrite(R_EN_F,LOW);
digitalWrite(R_EN_B,LOW);
digitalWrite(L_EN_F,LOW);
digitalWrite(L_EN_B,LOW);
}

void loop() {
  Motor_Drive();
  nh.spinOnce();
  delay(20);
}

//Receive data from 2 encoders (2 Interrupts)
//Control 4 Motors with 2 control signals (one for each side)
//tick-meters could overflow
void Motor_Drive (void)
{
  digitalWrite(L_EN_F,LOW);
  digitalWrite(R_EN_F,LOW);
  digitalWrite(L_EN_B,LOW);
  digitalWrite(R_EN_B,LOW);
  analogWrite(L_PWM_B,0);
  analogWrite(R_PWM_B,0);
  analogWrite(L_PWM_F,0);
  analogWrite(R_PWM_F,0);
  //delay(30);
  l_pwm= map(abs(left_cmd), 4, 183, 80, 255);
  r_pwm = map(abs(right_cmd), 4, 183, 80, 255);
  
  if (right_cmd < 0 ) // right side reverse direction 
  {
   digitalWrite(R_EN_F,HIGH);  
   digitalWrite(R_EN_B,HIGH);
   //from RPM to PWM where x is the pwm vaule
   analogWrite(R_PWM_B,abs(r_pwm));
    }
  else if (right_cmd > 0) //right side forward
  {
    
   digitalWrite(R_EN_B,HIGH);  
   digitalWrite(R_EN_F,HIGH);  
   //from RPM to PWM where x is the pwm vaule
   analogWrite(R_PWM_F,abs(r_pwm));   
    }
      else 
    {
    r_pwm = l_pwm = 0;
    digitalWrite(R_EN_B,LOW);  
    digitalWrite(R_EN_F,LOW);     
      }
  
  if (left_cmd  < 0 ) // left side reverse direction 
  {
   digitalWrite(L_EN_F,HIGH);  
   digitalWrite(L_EN_B,HIGH);
   //from RPM to PWM where x is the pwm vaule
   analogWrite(L_PWM_B,abs(l_pwm));
    }
  else if (left_cmd  > 0) //left side forward
  {
   digitalWrite(L_EN_B,HIGH);  
   digitalWrite(L_EN_F,HIGH);  
   //from RPM to PWM where x is the pwm vaule
   
   analogWrite(L_PWM_F,abs(l_pwm));   
    }
    else 
    {
     l_pwm = r_pwm = 0;
    digitalWrite(L_EN_B,LOW);  
    digitalWrite(L_EN_F,LOW); 
      }
}
  
