#include <stdint.h>
#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
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


//funciton definitions
void motor1_rot(int);
void motor2_rot(int);
void motor3_rot(int);
void motor4_rot(int);
void motor5_rot(int);
void motor6_rot(int);
void serialControl(char,int&);


char FORWARD  = 'w';
char LEFT     = 'a';
char RIGHT    = 'd';
char BACKWARD = 'z';
char STOP     = 's';
char SPEEDUP  = 'r';
char SPEEDDOWN= 'f';



//ROS NODE HANDLER//
ros::NodeHandle  nh;

void messageCb( const sensor_msgs::Joy& joy){

  char cad;
  int speed = 250;
  
  float updown_arrow = joy.axes[7];
  float lr_arrow     = joy.axes[6];

  if     (updown_arrow >=  0.5) cad = FORWARD;
  else if(updown_arrow <= -0.5) cad = BACKWARD;
  else if(lr_arrow     >=  0.5) cad = LEFT;
  else if(lr_arrow     <= -0.5) cad = RIGHT;
  else cad = STOP;
  
  serialControl(cad,speed);
  //control_direction();
  //control_speed();
  
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

////////////////////


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
  analogWrite(motor_1_ena, 250);
  digitalWrite(motor_1_in1, LOW);
  digitalWrite(motor_1_in2, LOW);
  analogWrite(motor_2_enb, 250);
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
  analogWrite(motor_3_ena, 250);
  digitalWrite(motor_3_in1, LOW);
  digitalWrite(motor_3_in2, LOW);
  analogWrite(motor_4_enb, 250);
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
  analogWrite(motor_5_ena, 250);
  digitalWrite(motor_5_in1, LOW);
  digitalWrite(motor_5_in2, LOW);
  analogWrite(motor_6_enb, 250);
  digitalWrite(motor_6_in3, LOW);
  digitalWrite(motor_6_in4, LOW);
  /*********H_BRIDGE_3*********/

//  //Serial COM
//  Serial.begin(9600);
//  Serial.setTimeout(5);//default: 1000, changed to run in realtime
//  delay(30);

  //NODE HANDLER
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);

//  //Begin serial control
//  serialControl();
//
//  //stop motors and finish execution
//  motor1_rot(-1);
//  motor2_rot(-1);
//  while(1); 
}


//////////    FUNTION DECLARATIONS    //////////
//MOTOR 1, n:1 forward, n:0 backwards, n:-1 stop
void motor1_rot(int n){
  if(n == -1) {//
    digitalWrite(motor_1_in1, LOW);
    digitalWrite(motor_1_in2, LOW);
   }else{
  digitalWrite(motor_1_in1, n);
  digitalWrite(motor_1_in2, !n);
  }
}

//MOTOR 2, n:1 forward, n:0 backwards, n:-1 stop
void motor2_rot(int n){
  if(n == -1) {//
    digitalWrite(motor_2_in3, LOW);
    digitalWrite(motor_2_in4, LOW);
   }else{
  digitalWrite(motor_2_in3, n);
  digitalWrite(motor_2_in4, !n);
  }
}

//MOTOR 3, n:1 forward, n:0 backwards, n:-1 stop
void motor3_rot(int n){
  if(n == -1) {//
    digitalWrite(motor_3_in1, LOW);
    digitalWrite(motor_3_in2, LOW);
   }else{
  digitalWrite(motor_3_in1, !n);
  digitalWrite(motor_3_in2, n);
  }
}

//MOTOR 4, n:1 forward, n:0 backwards, n:-1 stop
void motor4_rot(int n){
  if(n == -1) {//
    digitalWrite(motor_4_in3, LOW);
    digitalWrite(motor_4_in4, LOW);
   }else{
  digitalWrite(motor_4_in3, !n);
  digitalWrite(motor_4_in4, n);
  }
}

//MOTOR 5, n:1 forward, n:0 backwards, n:-1 stop
void motor5_rot(int n){
  if(n == -1) {//
    digitalWrite(motor_5_in1, LOW);
    digitalWrite(motor_5_in2, LOW);
   }else{
  digitalWrite(motor_5_in1, n);
  digitalWrite(motor_5_in2, !n);
  }
}

//MOTOR 6, n:1 forward, n:0 backwards, n:-1 stop
void motor6_rot(int n){
  if(n == -1) {//
    digitalWrite(motor_6_in3, LOW);
    digitalWrite(motor_6_in4, LOW);
   }else{
  digitalWrite(motor_6_in3, n);
  digitalWrite(motor_6_in4, !n);
  }
}


//Serial Control function//
void serialControl(char cad, int& speed){
//  char cad;
//  int speed = 127;

      switch (cad) {
        case 'w' : //Forward
                     /*****LEFT WHEELS*****/
                     motor1_rot(1);
                     motor2_rot(1);
                     motor5_rot(1);
                     /*****LEFT WHEELS*****/

                     /*****RIGHT WHEELS*****/
                     motor3_rot(1);
                     motor4_rot(1);
                     motor6_rot(1);
                     /*****RIGHT WHEELS*****/
                     //Serial.println("Forward");
                     break;
        case 'a' ://Rotate left
                     /*****LEFT WHEELS*****/
                     motor1_rot(0);
                     motor2_rot(1);
                     motor5_rot(0);
                     /*****LEFT WHEELS*****/

                     /*****RIGHT WHEELS*****/
                     motor3_rot(1);
                     motor4_rot(1);
                     motor6_rot(0);
                     /*****RIGHT WHEELS*****/                                          
                     //Serial.println("Rotate left");
                     break;
        case 'd' ://Rotate right
                     /*****LEFT WHEELS*****/
                     motor1_rot(1);
                     motor2_rot(0);
                     motor5_rot(1);
                     /*****LEFT WHEELS*****/

                     /*****RIGHT WHEELS*****/
                     motor3_rot(0);
                     motor4_rot(0);
                     motor6_rot(1);
                     /*****RIGHT WHEELS*****/                     

                     //Serial.println("Rotate right");
                     break;
        case 'z' ://Backwards
                     /*****LEFT WHEELS*****/
                     motor1_rot(0);
                     motor2_rot(0);
                     motor5_rot(0);
                     /*****LEFT WHEELS*****/

                     /*****RIGHT WHEELS*****/
                     motor3_rot(0);
                     motor4_rot(0);
                     motor6_rot(0);
                     /*****RIGHT WHEELS*****/  
                     //Serial.println("Backwards");
                     break;
        case 's' ://Stop
                     /*****LEFT WHEELS*****/
                     motor1_rot(-1);
                     motor2_rot(-1);
                     motor5_rot(-1);
                     /*****LEFT WHEELS*****/

                     /*****RIGHT WHEELS*****/
                     motor3_rot(-1);
                     motor4_rot(-1);
                     motor6_rot(-1);
                     /*****RIGHT WHEELS*****/  
                     //Serial.println("Stop");
                     break;                     
        case 'r' ://speed up
                    if(speed<(255-20)) speed = speed + 20;
                    speed = speed + 20;
                    analogWrite(motor_1_ena, speed);
                    analogWrite(motor_2_enb, speed);
                    analogWrite(motor_3_ena, speed);
                    analogWrite(motor_4_enb, speed);
                    analogWrite(motor_5_ena, speed);
                    analogWrite(motor_6_enb, speed);                                        
                    //Serial.println("Speed up");
                    //delay(2000);
                     break;
        case 'f' ://speed down
                    if(speed>20) speed = speed - 20;
                    analogWrite(motor_1_ena, speed);
                    analogWrite(motor_2_enb, speed);
                    analogWrite(motor_3_ena, speed);
                    analogWrite(motor_4_enb, speed);
                    analogWrite(motor_5_ena, speed);
                    analogWrite(motor_6_enb, speed);  
                    //Serial.println("Speed down");
                    //delay(2000);
                    break;
        default :   break;
  }
}
