#include <ros.h>
#include <std_msgs/Int8.h>
#include <TimerOne.h>
const uint8_t IND = 4, IR = 5;
std_msgs::Int8 STATE;
ros::NodeHandle nh;
uint8_t _state;
volatile uint8_t coil[6]= {0};
volatile uint8_t counter = 0;
volatile uint8_t coil_state = 0 ;
int IR_state = 1 ;
ros::Publisher coil_pub("coil_state", &STATE);

void setup() 
{
  pinMode(IND, INPUT);
  pinMode(IR, INPUT_PULLUP);
  nh.initNode();
  nh.advertise(coil_pub);
  Timer1.initialize(200000);
  Timer1.attachInterrupt(Coil_read);
  Serial.begin(9600);
}

void loop() 
{
 
  IR_state = digitalRead(IR);
  
  if( coil_state == 1 && IR_state == 1)
  {
    _state = 1;
  }
  else if(coil_state == 1 && IR_state == 0)
  {
    _state = 2;
  }
  else 
  {
    _state = 0;
  }
  STATE.data = _state;
  coil_pub.publish(&STATE);
  nh.spinOnce();
  delay(20);
}
void Coil_read (void)
{
  if (counter < 6)
  {
    coil[counter]=digitalRead(IND);
    counter++;;
    }
    else
  {
 
  counter = 0 ;
    }

if (coil[0] == 1 && coil[1]  == 1 && coil[2] == 1  && coil[3]  == 1 && coil[4]  == 1 && coil[5]  == 1)
{
  coil_state = 1;
  }
  else 
  {
    coil_state = 0;
   }
    Serial.println(coil_state);
  }
