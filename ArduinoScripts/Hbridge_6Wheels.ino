/*
 * rosserial PubSub Example
 * Control teleoperado de robots tipo semana i 
 * 
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 * 
 * rostopic pub /cmd_vel geometry_msgs/Twist "linear"
 */


#include <stdint.h>
#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

/*********H_BRIDGE_1*********/
//Motor 1
#define motor_1_ena 2 //Enable PWM, motor 1
#define motor_1_in1 22
#define motor_1_in2 23

//Motor 2
#define motor_2_enb 3 //Enable PWM, motor 2
#define motor_2_in3 24
#define motor_2_in4 25
/*********H_BRIDGE_1*********/

/*********H_BRIDGE_2*********/
//Motor 3
#define motor_3_ena 4 //Enable PWM, motor 3
#define motor_3_in1 26
#define motor_3_in2 27

//Motor 4
#define motor_4_enb 5 //Enable PWM, motor 4
#define motor_4_in3 28
#define motor_4_in4 29
/*********H_BRIDGE_2*********/

/*********H_BRIDGE_3*********/
//Motor 5
#define motor_5_ena 6 //Enable PWM, motor 5
#define motor_5_in1 30
#define motor_5_in2 31

//Motor 6
#define motor_6_enb 7 //Enable PWM, motor 6
#define motor_6_in3 32
#define motor_6_in4 33
/*********H_BRIDGE_3*********/

//ROS NODE HANDLER//
ros::NodeHandle  nh;

//-------------------------Define robot's variables-------------------------------------- 
#define WHEELRAD 0.06 //The radius of the wheel (m) 
#define WHEELDIST 0.5 //Distance between wheels (m) 
#define INV_WHEELRAD 16.66666666666666666666666666666666667 //The radius of the wheel (m) 

volatile float wl, wr; 

//-----------------------------Subscriber--------------------------------
void cmd_vel_cb( const geometry_msgs::Twist& vel_msg){ 
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led 
  set_wheel_speeds(vel_msg.linear.x, vel_msg.angular.z); 
} 

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb); 

//-----------------------------Publisher---------------------------------
//----Messagges----------------------------------------------------------
std_msgs::Float32 wr_msg; //right wheel speed ros message 
std_msgs::Float32 wl_msg; //left wheel speed ros message

ros::Publisher wr_pub("wr", &wr_msg); 
ros::Publisher wl_pub("wl", &wl_msg);  

void set_wheel_speeds(float v, float w) { 
  /***Equations for a differential drive mobile robot. 
  * We are avoiding divisions to make it faster 
  ***/ 
  //Right wheel angular speed [rad/s] 
  wr = (2.0 * v + WHEELDIST * w) * 0.5 * INV_WHEELRAD; //Right wheel angular speed [rad/s] 
  wl = (2.0 * v - WHEELDIST * w) * 0.5 * INV_WHEELRAD; //Left wheel angular speed [rad/s] 
} 

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

void setup() {
  
  /*********H_BRIDGE_1*********/
  //Motor1
  pinMode(motor_1_ena, OUTPUT);
  pinMode(motor_1_in1, OUTPUT);
  pinMode(motor_1_in2, OUTPUT);

    //Motor2
  pinMode(motor_2_enb, OUTPUT);
  pinMode(motor_2_in3, OUTPUT);
  pinMode(motor_2_in4, OUTPUT);

  // Set initial rotation direction
  analogWrite(motor_1_ena, 127);
  digitalWrite(motor_1_in1, LOW);
  digitalWrite(motor_1_in2, LOW);
  analogWrite(motor_2_enb, 127);
  digitalWrite(motor_2_in3, LOW);
  digitalWrite(motor_2_in4, LOW);
  /*********H_BRIDGE_1*********/

  /*********H_BRIDGE_2*********/
  //Motor3
  pinMode(motor_3_ena, OUTPUT);
  pinMode(motor_3_in1, OUTPUT);
  pinMode(motor_3_in2, OUTPUT);

    //Motor4
  pinMode(motor_4_enb, OUTPUT);
  pinMode(motor_4_in3, OUTPUT);
  pinMode(motor_4_in4, OUTPUT);

  // Set initial rotation direction
  analogWrite(motor_3_ena, 127);
  digitalWrite(motor_3_in1, LOW);
  digitalWrite(motor_3_in2, LOW);
  analogWrite(motor_4_enb, 127);
  digitalWrite(motor_4_in3, LOW);
  digitalWrite(motor_4_in4, LOW);
  /*********H_BRIDGE_2*********/

  /*********H_BRIDGE_3*********/
  //Motor5
  pinMode(motor_5_ena, OUTPUT);
  pinMode(motor_5_in1, OUTPUT);
  pinMode(motor_5_in2, OUTPUT);

    //Motor6
  pinMode(motor_6_enb, OUTPUT);
  pinMode(motor_6_in3, OUTPUT);
  pinMode(motor_6_in4, OUTPUT);

  // Set initial rotation direction
  analogWrite(motor_5_ena, 127);
  digitalWrite(motor_5_in1, LOW);
  digitalWrite(motor_5_in2, LOW);
  analogWrite(motor_6_enb, 127);
  digitalWrite(motor_6_in3, LOW);
  digitalWrite(motor_6_in4, LOW);
  /*********H_BRIDGE_3*********/

  //NODE HANDLER
  pinMode(13, OUTPUT); 
  nh.initNode(); 
  nh.advertise(wr_pub); 
  nh.advertise(wl_pub); 
  nh.subscribe(cmd_vel_sub); 
}

void loop(){ 
  if (!(millis() % 100)) {
  /*  
  *  Enter here every 100 ms 
  *  Add your code here  
  *  str_pub.publish(&str_msg); //Example publish data 
  */  
  wr_msg.data = wr; 
  wl_msg.data = wl; 
  wr_pub.publish( &wr_msg ); 
  wl_pub.publish( &wl_msg );
  control(wr,wl);
  }
  nh.spinOnce(); 
} 

void control(float r, float l){
    right_side_control(r);
    left_side_control(l);
}

void right_side_control(float n){
  int s = 0;
  analogWrite(motor_1_ena, n);
  analogWrite(motor_3_ena, n); 
  analogWrite(motor_5_ena, n); 

  if(n == 0){
    digitalWrite(motor_1_in1, LOW);
    digitalWrite(motor_1_in2, LOW);
    digitalWrite(motor_3_in1, LOW);
    digitalWrite(motor_3_in2, LOW);
    digitalWrite(motor_5_in1, LOW);
    digitalWrite(motor_5_in2, LOW);
  } else {
    if(n>0) s=1;
    else    s=0;
    digitalWrite(motor_1_in1, s);
    digitalWrite(motor_1_in2, !s);
    digitalWrite(motor_3_in1, s);
    digitalWrite(motor_3_in2, !s);
    digitalWrite(motor_5_in1, s);
    digitalWrite(motor_5_in2, !s);
  }
}

void left_side_control(float n){
  int s = 0;
  analogWrite(motor_2_enb, n);
  analogWrite(motor_4_enb, n); 
  analogWrite(motor_6_enb, n); 

  if(n == 0){
    digitalWrite(motor_2_in3, LOW);
    digitalWrite(motor_2_in4, LOW);
    digitalWrite(motor_4_in3, LOW);
    digitalWrite(motor_4_in4, LOW);
    digitalWrite(motor_6_in3, LOW);
    digitalWrite(motor_6_in4, LOW);
  } else {
    if(n>0) s=1;
    else    s=0;
    digitalWrite(motor_2_in3, s);
    digitalWrite(motor_2_in4, !s);
    digitalWrite(motor_4_in3, s);
    digitalWrite(motor_4_in4, !s);
    digitalWrite(motor_6_in3, s);
    digitalWrite(motor_6_in4, !s);
  }
}