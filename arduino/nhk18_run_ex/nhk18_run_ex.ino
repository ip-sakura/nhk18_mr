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
#include <stdlib.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#define datalength 4294967295

#define MOTOR_L 1
#define MOTOR_R 2

volatile int pw[5]={0,0,0,0,0},w[6]={0,0,0,0,0,0};
long enc[5]={},Enc[5]={},Enc_a[5]={},Enc_b[5]={},n=5;///18000
double p_gain=0.01,i_gain=0,d_gain=0,e1[5]={},e2[5]={},e3[5]={},e4[5]={};
//0.95 2.4 0.4
static int h=0,l=0;
long plus=2000000000;
long minus=-2000000000;
int state=0;

uint8_t addr1 = 0x26;
uint8_t addr2 = 0x27;


IseMotorDriver m1 = IseMotorDriver(addr1);
IseMotorDriver m2 = IseMotorDriver(addr2);



ros::NodeHandle  nh;
std_msgs::Int32MultiArray encarr;
ros::Publisher enc_pub("enc", &encarr);

void motorLCallBack(const std_msgs::Int16& pw){
  w[MOTOR_L] = pw.data;
}
void motorRCallBack(const std_msgs::Int16& pw){
  w[MOTOR_R] = pw.data;
}

ros::Subscriber<std_msgs::Int16>mr("mr",motorRCallBack);
ros::Subscriber<std_msgs::Int16>ml("ml",motorLCallBack);

void encorder(){
      
    Enc_b[MOTOR_L] = -m1.encorder();
    Enc_b[MOTOR_R] = -m2.encorder();
    delay(10);
    //-2,147,483,648 ～ 2,147,483,647
    //encorder foward +4-
    Enc_a[MOTOR_L] = -m1.encorder();
    Enc_a[MOTOR_R] = -m2.encorder();

  for(int i=1;i<3;i++){ 

    if(Enc_b[i]>plus){
      if(Enc_a[i]<=0&&Enc_b[i]>0){
        enc[i]=(datalength+Enc_a[i]-Enc_b[i])/n;
      }
      else{
        enc[i]=(Enc_a[i]-Enc_b[i])/n;
      }
    }
      
    //encorder back -4+ 
    else if(Enc_b[i]<minus){
      if(Enc_a[i]>=0&&Enc_b[i]<0){
        enc[i]=(-datalength+Enc_a[i]-Enc_b[i])/n;
      }
      else{
        enc[i]=(Enc_a[i]-Enc_b[i])/n;
      }
    }
    
    else{
      enc[i]=(Enc_a[i]-Enc_b[i])/n;
    }
    enc[i]=enc[i]/3;
  }
}
void cal(){
  for(int i=1;i<3;i++){
    
    e1[i]=w[i]-enc[i]; 
    e2[i]=e2[i]+e1[i];
    e3[i]=e1[i]-e4[i];
    e4[i]=e1[i];
    pw[i]+=p_gain*e1[i]+i_gain*e2[i]+d_gain*e3[i];
    if(abs(pw[i])>100){
      pw[i]=100*pw[i]/abs(pw[i]);
    }
    else{
    }
    // pw[i]=w[i];
  }
}

void setup()
{
  encarr.data_length=6;
  encarr.data=(int32_t*)malloc(sizeof(int32_t*)*6);
  Wire.begin();
  //Serial.begin(9600);
  nh.initNode();
  //delay(1000);
  //delay(1000);
  //delay(1000);
  nh.advertise(enc_pub);
  nh.subscribe(ml);
  nh.subscribe(mr);
}

void loop()
{
  encorder();
  cal();
  m1.setSpeed(pw[1]);
  m2.setSpeed(pw[2]);
  
  for(int j=0;j<2;j++){
    encarr.data[j] = enc[j+1];
  }
  for(int j=0;j<2;j++){
    encarr.data[j+2] = pw[j+1];
  }
  for(int j=0;j<2;j++){
    encarr.data[j+4] = w[j+1];
  }


  
  enc_pub.publish( &encarr );
  nh.spinOnce();
  delay(10);
}

