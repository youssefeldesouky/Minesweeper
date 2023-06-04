#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>

Servo myservo;
int angle=0, angle_old = 0;
uint16_t angle_values[] = {0, 20, 40, 60, 80, 100, 120, 140, 160, 180};
int magnet=0;
int dc_motor=0;
bool angle_lock = false;
const int magnet_pin=11;
#define EnA 10
#define EnB 5
#define In1 9
#define In2 8
#define In3 7
#define In4 6

ros::NodeHandle nh;

void craneCB(const std_msgs::Int8MultiArray &data){
  angle = data.data[0];
  magnet = data.data[1];
  dc_motor = data.data[2];
}

ros::Subscriber<std_msgs::Int8MultiArray> crane_sub("crane", &craneCB);

void setup() {
  myservo.attach(4); 
  nh.initNode();
  nh.subscribe(crane_sub);

  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(EnA, HIGH);
  digitalWrite(EnB, HIGH);

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
}

void loop() {
  if(angle != angle_old){
    angle_lock = false;
    angle_old = angle;
  }
  // put your main code here, to run repeatedly:
  if(!angle_lock){
    myservo.write(angle_values[angle + 4]);
    angle_lock = true;
  }
  if(dc_motor==0)
  { 
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);  
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
  }
  else if(dc_motor>0)
  { 
    digitalWrite(In2, LOW);  
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
    digitalWrite(In1, HIGH);
  }
  else if(dc_motor<0)
  { 
    digitalWrite(In2, HIGH);  
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
    digitalWrite(In1, LOW);
  }
    if(magnet==0)
  { 
    digitalWrite(magnet_pin, LOW);  
  
  }
    else if(magnet==1)
  { 
    digitalWrite(magnet_pin, HIGH);  
  
  }
  nh.spinOnce();
  delay(20);
}

uint16_t Servo_val(int angle){
  
}
