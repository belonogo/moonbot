#include <ros.h>  
#include <std_msgs/Empty.h>  
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>  
#include <TroykaIMU.h>
#include <std_msgs/Int64.h>
#include <Servo.h>
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
 
Gyroscope gyroscope; 
 
geometry_msgs::Vector3 imuData; 
std_msgs::String str_msg; 
std_msgs::UInt32 left_encoder;
std_msgs::UInt32 right_encoder;
std_msgs::UInt32 water_data;
String data; 
 
ros::Publisher chatter("chatter", &str_msg); 
ros::Publisher imuDataPublisher("imu_data", &imuData); 
ros::Publisher leftEncoderPublisher("wheel_ticks_left", &left_encoder);
ros::Publisher rightEncoderPublisher("wheel_ticks_right", &right_encoder);
ros::Publisher water("water", &water_data); 

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
  } 
  else {  
    dir = 0;  
  }  
  digitalWrite(A0, HIGH);  
  if (dir == 1) {  
    digitalWrite(AIN1, 1);  
    digitalWrite(AIN2, 0);  
    spd = spd;  
    analogWrite(PWM1, spd);  
  } 
  else {  
    digitalWrite(AIN1, 0);  
    digitalWrite(AIN2, 1);
    analogWrite(PWM1, spd);  
  }  
}  
  
void motor2SetSpeed(int spd1 = 0) {  
  bool dir1 = 0;  
  if (spd1 < 0) {  
    dir1 = 0;  
    spd1 = 255-spd1;  
  } 
  else {  
    dir1 = 1;  
  }  
  digitalWrite(A1, HIGH);  
  if (dir1 == 1) {  
    digitalWrite(BIN1, 1);  
    digitalWrite(BIN2, 0);  
    analogWrite(PWM2, spd1);  
  } 
  else {  
    digitalWrite(BIN1, 0);  
    digitalWrite(BIN2, 1);  
    analogWrite(PWM2, spd1);  
  }  
}  
 
void messageCb(const geometry_msgs::Twist &cmd_vel)   
{   
    long lasttim=millis();  
    double vel_x=cmd_vel.linear.x;  
    double vel_th=cmd_vel.angular.z;  
    double left_vel=(vel_x-vel_th*ROBOT_WIDTH/2000)*255*3;  
    double right_vel=(vel_x+vel_th*ROBOT_WIDTH/2000)*255*3; 
    left_vel = min(160, left_vel); 
    right_vel = min(160, right_vel); 
    motor1SetSpeed((int)left_vel);  
    motor2SetSpeed((int)right_vel); 
    data = String(left_vel) + "    " + String(right_vel); 
}

Servo myservo;

// функция-обработчик изменеий топика serv_info
void servInfoAdapter(const std_msgs::Int64 &servInfo)   
{   
    // берем значение из servInfo и подаем на сервопривод
    myservo.write(servInfo.data);
}
 
volatile uint32_t right_enc_cnt = 0; 
volatile uint32_t left_enc_cnt = 0; 
 
void enc_right() 
{ 
  right_enc_cnt++; 
} 
 
void enc_left() 
{ 
  left_enc_cnt++; 
}
 
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);
// создаем подписчика на топик serv_info. !!!! топик появляется только после запуска ноды питон скрипта
// обработчиком изменения данных в топике будет функция servInfoAdapter
ros::Subscriber<std_msgs::Int64> servSub("serv_info", &servInfoAdapter);

void setup() {  
  myservo.attach(52);

  pinMode(BIN1, OUTPUT);  
  pinMode(AIN1, OUTPUT);  
  pinMode(AIN2, OUTPUT);  
  pinMode(BIN2, OUTPUT);  
  
  pinMode(PWM1, OUTPUT);  
  pinMode(PWM2, OUTPUT);  
  
  pinMode(A0, OUTPUT);  
  pinMode(A1, OUTPUT);  
  Serial3.begin(9600); 
 
  gyroscope.begin(); 
  
  nh.initNode();
  nh.subscribe(sub); 
  nh.subscribe(servSub); // подписываем на топик serv_info
  nh.advertise(chatter);
  nh.advertise(imuDataPublisher); 
  nh.advertise(leftEncoderPublisher);
  nh.advertise(rightEncoderPublisher);
  nh.advertise(water);
  
  attachInterrupt(digitalPinToInterrupt(18), enc_right, RISING); 
  attachInterrupt(digitalPinToInterrupt(2), enc_left, RISING); 
} 
 
void loop() {  
  str_msg.data = data.c_str(); 
  imuData.x = gyroscope.readRotationDegX(); 
  imuData.y = gyroscope.readRotationDegY(); 
  imuData.z = gyroscope.readRotationDegZ(); 

  left_encoder.data = left_enc_cnt;
  right_encoder.data = right_enc_cnt;

  water_data.data = analogRead(A8);
  
  chatter.publish(&str_msg); 
  imuDataPublisher.publish(&imuData); 
  leftEncoderPublisher.publish(&left_encoder);
  rightEncoderPublisher.publish(&right_encoder);
  water.publish(&water_data);

  nh.spinOnce();  
  delay(1);   
}
