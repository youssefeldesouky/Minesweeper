#include <ros.h>
#include <std_msgs/Int8.h>
#include <TimerOne.h>
const uint8_t IND = 4, IR = 8;
std_msgs::Int8 STATE, IR_STATE;
ros::NodeHandle nh;
uint8_t _state;
volatile uint8_t coil[6]= {0};
volatile uint8_t counter = 0;
volatile uint8_t coil_state = 0 ;
int IR_state = 1 ;
ros::Publisher coil_pub("coil_state", &STATE);
ros::Publisher ir_pub("ir_state", &IR_STATE);
bool lock = false;

void setup() 
{
  pinMode(IND, INPUT);
  pinMode(IR, INPUT_PULLUP);
  nh.initNode();
  nh.advertise(coil_pub);
  nh.advertise(ir_pub);
  Timer1.initialize(200000);
  Timer1.attachInterrupt(Coil_read);
  //Serial.begin(9600);
}

void loop() 
{
 
  

  if (coil_state == 1)
  {
    for (int i=0;i < 300 && coil_state == 1;++i)
    {
      IR_state = digitalRead(IR); 
      if (IR_state == 0)
      {
        _state = 2;
        lock = true;
      break;
      }     
      delay(10);
    }
    
  }
    else{ 
    _state = 0;

    }
  
 /* if( coil_state == 1 && IR_state == 1)
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
  }*/
  STATE.data = _state;
  IR_STATE.data = IR_state;
  coil_pub.publish(&STATE);
  ir_pub.publish(&IR_STATE);
  nh.spinOnce();


    Serial.println(_state);

  delay(20);
}



void Coil_read (void)
{
  if (counter < 6)
  {
    coil[counter]=digitalRead(IND);
    counter++;
    }
    else
  {
 
  counter = 0 ;
    }

if (coil[0] == 1 && coil[1]  == 1 && coil[2] == 1  && coil[3]  == 1 && coil[4]  == 1 && coil[5]  == 1)
{
  coil_state = 1;
  if (_state == 0)
  _state = 1; 
  }
  else 
  {
    coil_state = 0;
   }
    //Serial.println(coil_state);
  }
