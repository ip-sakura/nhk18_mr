#include <ros.h>
#include <std_msgs/Int8.h>

#define WHEEL_DIAMETER 40//だいたい
#define STEP 1.8
#define STEPS 200// 360 / STEP (360度の回転に何パルス必要か
#define RPM 30 //分あたりの回転数
#define PULSE_ROLL (WHEEL_DIAMETER * 3.14 * STEP / 360)//１パルスあたり
#define STEPPING_MOTOR_SUM 2
#define DEFAULTPW 80


void stpaCallback(const std_msgs::Int16& a){
     int input_pulse = a.data/PULSE_ROLL;
}
void stpbCallback(const std_msgs::Int16& b){
     int input_pulse = b.data/PULSE_ROLL;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> stpa("stpa",stpaCallback);
ros::Subscriber<std_msgs::Int16> stpb("stpb",stpbCallback);

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
