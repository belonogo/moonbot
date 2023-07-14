#include <ros.h> 
#include <std_msgs/Empty.h> 
#include <geometry_msgs/Twist.h> 
#include <std_msgs/String.h> 
#define ROBOT_WIDTH 150 
 
int pot; 
int out; 
 
#define PWM1 5 
#define AIN1 7 
#define AIN2 8 
 
#define PWM2 6 
#define BIN1 4 
#define BIN2 9 
 
#define EN1 A0 
#define EN2 A1 
 
int pwm = 128; 
int command = 0; 
 
class NewHardware : public ArduinoHardware 
{ 
  public: 
    NewHardware(): ArduinoHardware(&Serial, 115200) {}; 
}; 
  
ros::NodeHandle_<NewHardware>  nh; 
  
void motor1SetSpeed(int spd = 0) { 
  bool dir = 0; 
  if (spd < 0) { 
    dir = 1; 
    spd = 255-spd; 
  } else { 
    dir = 0; 
  } 
  digitalWrite(A0, HIGH); 
  if (dir == 1) { 
    digitalWrite(AIN1, 1); 
    digitalWrite(AIN2, 0); 
    spd = spd; 
    analogWrite(PWM1, spd); 
  } else { 
    digitalWrite(AIN1, 0); 
    digitalWrite(AIN2, 1); 
    spd = spd; 
    analogWrite(PWM1, spd); 
  } 
} 
 
void motor2SetSpeed(int spd1 = 0) { 
  bool dir1 = 0; 
  if (spd1 < 0) { 
    dir1 = 0; 
    spd1 = 255-spd1; 
  } else { 
    dir1 = 1; 
  } 
  digitalWrite(A1, HIGH); 
  if (dir1 == 1) { 
    digitalWrite(BIN1, 1); 
    digitalWrite(BIN2, 0); 
    analogWrite(PWM2, spd1); 
  } else { 
    digitalWrite(BIN1, 0); 
    digitalWrite(BIN2, 1); 
    analogWrite(PWM2, spd1); 
  } 
} 

String data;

void messageCb(const geometry_msgs::Twist& cmd_vel)  
{  
    long lasttim=millis(); 
    double vel_x=cmd_vel.linear.x; 
    double vel_th=cmd_vel.angular.z; 
    double left_vel=(vel_x-vel_th*ROBOT_WIDTH/2000)*255; 
    double right_vel=(vel_x+vel_th*ROBOT_WIDTH/2000)*255; 
    motor1SetSpeed((int)left_vel); 
    motor2SetSpeed((int)right_vel);
    data = String(left_vel) + " " + String(right_vel);
}  
 
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb ); 
std_msgs::String str_msg; 
ros::Publisher chatter("chatter", &str_msg); 
 
void setup() { 
  pinMode(BIN1, OUTPUT); 
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT); 
  pinMode(BIN2, OUTPUT); 
 
  pinMode(PWM1, OUTPUT); 
  pinMode(PWM2, OUTPUT); 
 
  pinMode(A0, OUTPUT); 
  pinMode(A1, OUTPUT); 
  Serial3.begin(9600); 
 
  nh.initNode();  
  nh.subscribe(sub); 
  nh.advertise(chatter); 
} 
void loop() { 
  str_msg.data = data.c_str();
  chatter.publish( &str_msg ); 
  nh.spinOnce();  
  delay(1);  
}