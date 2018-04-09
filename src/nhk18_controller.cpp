#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"

#define STOP 0
//足回り状態
#define FORWARD 1
#define BACK 2
#define TURNL 3
#define TURNR 4
//ステピ状態
#define STPUP 1
#define STPDW 2

#define THRESHOLD 0.5
#define TOPPOWER 90
#define LRGAP 5 //壁伝い走行用の回転差
#define STPX 150//ラックを持ち上げる際の変位（mm）
#define SUPRESS 0.1

int status = STOP;//状態
int status_buf = STOP;//直前の状態
int stp_status = STOP;//ステピの状態
int stp_status_buf = STOP;//直前のステピの状態
int r_ispushed,l_ispushed;
int sw = 0;
int span_ms = 2;//速度？積算のタイムスパン
int cnt = 0;//タイムスパン用カウンタ

float delta = 0.8;//PID制御用の係数 pgain

int motorpw_l=0;
int motorpw_r=0;

std_msgs::Int16 mpwsender_l,mpwsender_r;//最終的にpubされるmotorpw
std_msgs::Int16 stpsender_a,stpsender_b;

void set_motor_speed(int& motor_pw,int target_pw){//PID制御でmotorpwに積算する
  
  int gap = delta * (target_pw - motor_pw);
  if(gap == 0){
    if(target_pw-motor_pw > 0)gap = 1;
    else if (target_pw-motor_pw < 0)gap = -1;
    else gap = 0;
  }
  
  motor_pw += gap;
}

void set_stp_move(){//mm単位でどれだけ回すかmsgsに格納
  if(stp_status == STPUP){
    stpsender_a.data = STPX;
    stpsender_b.data = STPX;
    ROS_INFO("stp cw \n");
  }else if(stp_status == STPDW){
    stpsender_a.data = -STPX;
    stpsender_b.data = -STPX;
    ROS_INFO("stp ccw \n");
  }//else if(stp_status == STOP){
  //stpsender_a.data = 0;
  //stpsender_b.data = 0;
  //ROS_INFO("stp stop");
}


void set_motor_status(){//statusに応じてmotorpwを変化させる
  if(cnt == span_ms){
    if(status == FORWARD){
      set_motor_speed(motorpw_l,TOPPOWER);
      set_motor_speed(motorpw_r,TOPPOWER);
    }else if(status == BACK){
      set_motor_speed(motorpw_l,-TOPPOWER);
      set_motor_speed(motorpw_r,-TOPPOWER);
    }else if(status == STOP){
      set_motor_speed(motorpw_l,0);
      set_motor_speed(motorpw_r,0);
    }
    cnt = 0;
  }
  cnt++;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  if(joy->axes[1] >= THRESHOLD){
    status = FORWARD;
  }else if(joy->axes[1] <= -THRESHOLD){
    status = BACK;
  }else{
    status = STOP;
  }

  if(joy->buttons[5]){//Rボタン（右寄り走行）
    r_ispushed = 1;
  }else r_ispushed = 0;
  if(joy -> buttons[4]){//Lボタン（左寄り走行）
    l_ispushed = 1;
  }else l_ispushed = 0;

  if(status != status_buf){
    status_buf = status;//状態が切り替わった直後の初期化処理
    cnt = 0;
  }

  if(joy -> axes[5] >= THRESHOLD){//十字キー上下でステピ動かす予定
    stp_status = STPUP;
  }else if(joy -> axes[5] <= -THRESHOLD){
    stp_status = STPDW;
  }//else {
  //stp_status = STOP;
  //}
}


int main (int argc, char **argv){
  r_ispushed = 0;
  l_ispushed = 0;
  ros::init(argc,argv,"nhk18_controller");
  ros::NodeHandle nh;
  ros::Subscriber joy = nh.subscribe("joy", 1000, joyCallback);
  ros::Publisher mr_pub = nh.advertise<std_msgs::Int16>("mr",1000);
  ros::Publisher ml_pub = nh.advertise<std_msgs::Int16>("ml",1000);
  ros::Publisher stpa_pub = nh.advertise<std_msgs::Int16>("stpa",1000);
  ros::Publisher stpb_pub = nh.advertise<std_msgs::Int16>("stpb",1000);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    set_motor_status();
    mpwsender_r.data = motorpw_r;
    mpwsender_l.data = motorpw_l;
    if(l_ispushed == 0 && r_ispushed == 1){//L,Rボタンに応じて片方に寄る走行をさせる
      if(mpwsender_l.data >= 0)
	mpwsender_l.data += (mpwsender_l.data * SUPRESS);
      else mpwsender_l.data -= (mpwsender_l.data * SUPRESS);
    }else if(l_ispushed == 1 && r_ispushed == 0){
      if(mpwsender_r.data >= 0)
	mpwsender_r.data += (mpwsender_r.data * SUPRESS);
      else mpwsender_r.data -= (mpwsender_r.data * SUPRESS);
    }

    mr_pub.publish(mpwsender_l);
    ml_pub.publish(mpwsender_r);

    ROS_INFO("L:%d",mpwsender_l.data);
    ROS_INFO("R:%d\n",mpwsender_r.data);


    if(stp_status != stp_status_buf){
      stp_status_buf = stp_status;
      set_stp_move();
      stpa_pub.publish(stpsender_a);
      stpb_pub.publish(stpsender_b);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
