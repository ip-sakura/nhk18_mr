#include <ros.h>
#include <Wire.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

uint8_t addrr = 0x26;
uint8_t addrl = 0x27;

IseMotorDriver mdl = IseMotorDriver(addrl);
IseMotorDriver mdr = IseMotorDriver(addrr);
int l = 0;
int r = 0;

void setup(){
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  mdr.setSpeed(50);
  int a = mdr.encorder();
  int c = mdl.encorder();
  delay(10);
  int b = mdr.encorder();
  int d = mdl.encorder();
  Serial.print(b-a);
  Serial.print(":");
  Serial.println(d-c);
  delay(10);
}
