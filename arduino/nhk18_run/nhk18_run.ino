#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

uint8_t addrr = 0x27;
uint8_t addrl = 0x26;

IseMotorDriver mdl = IseMotorDriver(addrl);
IseMotorDriver mdr = IseMotorDriver(addrr);
int l = 0;
int r = 0;

void motorLCallBack(const std_msgs::Int16& pw){
  l = pw.data;
  mdl.setSpeed(l);
}
void motorRCallBack(const std_msgs::Int16& pw){
  r = pw.data;
  mdr.setSpeed(r);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16>mr("mr",motorRCallBack);
ros::Subscriber<std_msgs::Int16>ml("ml",motorLCallBack);


void setup(){
  Wire.begin();
  nh.initNode();
  nh.subscribe(ml);
  nh.subscribe(mr);
}

void loop() {
  nh.spinOnce();
  delay(100);
}
