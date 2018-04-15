/*
 * rosserial Publisher Example
 *test of Publishing Multi Array
 */
#include <stdio.h>
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#define datalength 4294967295

#define MOTOR_L 1
#define MOTOR_R 2
#define CORRECTION 10
#define FINTHRESH 1

#define SERVO_PIN 7
//#define SERVO_RED_PIN 7
//#define SERVO_BLUE_PIN 6

#define HIDE_BOTH 0
#define SHOW_RED 1
#define SHOW_BLUE 2

#define SHOW_RED_NUM 90
#define SHOW_BLUE_NUM 50
#define SHOW_BOTH_NUM 160

Servo hand_red,hand_blue,hand;

volatile double pw[5]={0,0,0,0,0},targetSpeed[6]={0,0,0,0,0,0};
double speed[5]={}, n=90;
long Enc_a[5]={},Enc_b[5]={};///18000
double p_gain=2.5,i_gain=0.03,d_gain=0.2, pre_pid[5]={};
double e2[5]={},e4[5]={};
//0.95 2.4 0.4
static int h=0,l=0;
long plus=2000000000;
long minus=-2000000000;
int state=0;

long enc_buf_l,enc_buf_r = 0;

uint8_t addr1 = 0x26;
uint8_t addr2 = 0x27;


IseMotorDriver m1 = IseMotorDriver(addr1);
IseMotorDriver m2 = IseMotorDriver(addr2);



ros::NodeHandle  nh;
std_msgs::Int32MultiArray encarr;
ros::Publisher enc_pub("enc", &encarr);

void motorLCallBack(const std_msgs::Int16& msg){
  targetSpeed[MOTOR_L] = msg.data;
}
void motorRCallBack(const std_msgs::Int16& msg){
  targetSpeed[MOTOR_R] = msg.data;
}

void servoCallBack(const std_msgs::Int16& srv){
  /*if(srv.data == 0){
    hand.write(SHOW_BLUE_NUM);
  }else if(srv.data == 1){
    hand.write(SHOW_BOTH_NUM);
  }else if(srv.data == 2){
    hand.write(SHOW_RED_NUM);
  }*/
}

ros::Subscriber<std_msgs::Int16>mr("mr",motorRCallBack);
ros::Subscriber<std_msgs::Int16>ml("ml",motorLCallBack);
ros::Subscriber<std_msgs::Int16>servo("servo",servoCallBack);

void encorder(){
      
    Enc_b[MOTOR_L] = -m1.encorder();
    Enc_b[MOTOR_R] = m2.encorder();
    delay(100);
    //-2,147,483,648 ～ 2,147,483,647
    //encorder foward +4-
    Enc_a[MOTOR_L] = -m1.encorder();
    Enc_a[MOTOR_R] = m2.encorder();
    speed[MOTOR_L]=(Enc_a[MOTOR_L]-Enc_b[MOTOR_L])/n;
    speed[MOTOR_R]=(Enc_a[MOTOR_R]-Enc_b[MOTOR_R])/n;    
}

double delta[10] = {};
void cal(){
  double pid;
  double gain;
  double e1[5];
  double e3[5];
  for(int i=1;i<3;i++){
    e1[i]=double(targetSpeed[i]-speed[i]); 
    e2[i]=e2[i]+e1[i];
    e3[i]=e1[i]-e4[i];
    e4[i]=e1[i];
    pid = p_gain*e1[i]+i_gain*e2[i]+d_gain*e3[i];
//    delta[i] += pid;
//    pw[i] += delta[i];
    pw[i] = speed[i] + pid;

    //test
    /*
    if (e1[i] > 0) {
       pw[i] +=1;
    } else if (e1[i] < 0) {
      pw[i] -= 1;
    }*/

    if (targetSpeed[i] == 0) {
       pw[i] = 0;
       e2[i] = 0;
       delta[i] = 0;
    }
    /*
    if (e2[i]>1000) {
       e2[i] = 1000;
    }
    if (e2[i]<-1000) {
       e2[i] = -1000;
    }*/
    

    if(abs(pw[i])>100){
      pw[i]=100*pw[i]/abs(pw[i]);
    }
  }
}

void setup()
{
  encarr.data_length=6;
  encarr.data=(int32_t*)malloc(sizeof(int32_t*)*6);
  Wire.begin();
  nh.initNode();
  nh.advertise(enc_pub);
  nh.subscribe(ml);
  nh.subscribe(mr);
  nh.subscribe(servo);
  //hand.attach(SERVO_PIN);
  //hand_red.attach(SERVO_RED_PIN);
  //hand_blue.attach(SERVO_BLUE_PIN);
}

int count = 0;

void loop()
{
  if (count == 0){
  encorder();
  cal();

  if (pw[1] != 0 || pw[2] != 0){
    m1.setSpeed(-1*int(pw[1]));
    m2.setSpeed(int(pw[2]));
  }else if(pw[1] == 0 && pw[2] == 0){
    if(speed[1] < -FINTHRESH && speed[2] < -FINTHRESH){
      m1.setSpeed(-CORRECTION);
      m2.setSpeed(CORRECTION);
    }else if(speed[1] > FINTHRESH && speed[2] > FINTHRESH){
      m1.setSpeed(CORRECTION);
      m2.setSpeed(-CORRECTION);
    }else{
      m1.setSpeed(0);
      m2.setSpeed(0);
    }
  }
  
  count = 1;
  }
  for(int j=0;j<2;j++){
    encarr.data[j] = speed[j+1];
  }
  for(int j=0;j<2;j++){
    encarr.data[j+2] = pw[j+1];
  }
  for(int j=0;j<2;j++){
    encarr.data[j+4] = targetSpeed[j+1];
  }
 
  enc_pub.publish( &encarr );  
  count --;
  nh.spinOnce();
  //delay(100);
}

