#include <Stepper.h>
#include <ros.h>
#include <std_msgs/Int8.h>

#define WHEEL_DIAMETER 40//だいたい
#define STEP 1.8
#define STEPS 200
#define PW 100
#define RPM 30 //分あたりの回転数
#define PULSE_ROLL (WHEEL_DIAMETER * 3.14 * STEP / 360)//１パルスあたり

static const int motor_cw[2] = {3, 4};
static const int motor_ccw[2] = {2, 5};

Stepper stepper_a(STEPS, motor_cw[0], motor_ccw[0]);
Stepper stepper_b(STEPS, motor_cw[1], motor_ccw[1]);

void stpaCallback(const std_msgs::Int8& a){
     int input_pulse = a.data/PULSE_ROLL;
     stepper_a.step(input_pulse);
}
void stpbCallback(const std_msgs::Int8& b){
     int input_pulse = b.data/PULSE_ROLL;
     stepper_b.step(input_pulse);

}
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int8> stpa("stpa",stpaCallback);
ros::Subscriber<std_msgs::Int8> stpb("stpb",stpbCallback);

void stp_motor_init(){
  for(int i = 0; i < 2; i++){
    pinMode(motor_cw[i], OUTPUT);
    pinMode(motor_ccw[i], OUTPUT);
  }
  stepper_a.setSpeed(RPM);
  stepper_b.setSpeed(RPM);
}

void setup(){
  nh.initNode();
  stp_motor_init();
  nh.subscribe(stpa);
  nh.subscribe(stpb);
}

void loop(){
  delay(10);
  nh.spinOnce();
}
