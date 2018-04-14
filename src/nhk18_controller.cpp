#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#define STOP 0
//足回り状態
#define FORWARD 1
#define BACK 2
#define TURNL 3
#define TURNR 4
//ステピ状態
#define STPUP 1
#define STPDW 2

//エンコーダ用
#define DELTA_L 0
#define DELTA_R 1
#define DGAP 2

#define THRESHOLD 0.1
#define TOPPOWER 80
#define MINPOWER 50

#define L_OR_R_PLUS 5//Lbutton Rbutton ni taiou addition

#define STP0 10 //ステピ初期位置
#define STPX 150 - STP0//ラックを持ち上げる際の変位（mm）

#define SUPRESS 0.1//使わない
#define SUPRESS_COR 0.5 //補正用サプレッサー 使わない
#define SUPRESSER 0.15//コントローラーからの出力にかけてarduinoに渡す

int status = STOP;//状態
int status_buf = STOP;//直前の状態
int stp_status = STOP;//ステピの状態
int stp_status_buf = STOP;//直前のステピの状態
int r_ispushed,l_ispushed,a_ispushed;
int sw = 0;
int span_ms = 1;//速度？積算のタイムスパン
int cnt = 0;//タイムスパン用カウンタ

float delta = 0.8;//PID制御用の係数 pgain

int motorpw_l=0;
int motorpw_r=0;
int extra_correction = 0;//補正用
float gap_ratio = 0;//エンコーダからとってきた差を割合で表す。
bool vlr_max = 0; //=DELTA_L(0) or DELTA_R(1)で考える

std_msgs::Int16 mpwsender_l,mpwsender_r;//最終的にpubされるmotorpw
std_msgs::Int16 stpsender_a,stpsender_b;
std_msgs::Float64 ex;

void set_motor_speed(int& motor_pw,int target_pw){//PID制御でmotorpwに積算する
  
  /* int gap = delta * (target_pw - motor_pw);
  if(gap == 0){
    if(target_pw-motor_pw > 0)gap = 1;
    else if (target_pw-motor_pw < 0)gap = -1;
    else gap = 0;
  }
  
  motor_pw += gap;*/
}

void get_correction(){
  /*if(motorpw_l>0){
    if(gap_ratio > 0){
      extra_correction--;
    }else{
      extra_correction++;
    }
  }else if(motorpw_l<0){
    if(gap_ratio > 0){
      extra_correction++;
    }else{
      extra_correction--;
    }
    }*/
  
    /*if(vlr_max == DELTA_L){
      //set_correction(extra_correction,-(int)(motorpw_l * gap_ratio));
      extra_correction--;
    }else if(vlr_max == DELTA_R){
      //set_correction(extra_correction,(int)motorpw_r * gap_ratio);
      extra_correction++;
      }*/
    
  

  /*if(gap_ratio > 0){
    if(mpwsender_l.data > 0 && mpwsender_r.data > 0){
      if(extra_correction < 0)extra_correction = 0;
      extra_correction += 1;
    }else if(mpwsender_l.data < 0 && mpwsender_r.data < 0){
      if(extra_correction > 0)extra_correction = 0;
      extra_correction -= 1;
    }
    }*/
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

  if(a_ispushed){
    stpsender_a.data = STP0;
    a_ispushed = 0;
  }
}


void set_motor_status(){//statusに応じてmotorpwを変化させる
  /*if(cnt == span_ms){
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
  cnt++;*/
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  if(joy->axes[1] >= THRESHOLD){
    status = FORWARD;
    motorpw_l = joy->axes[1] * 100 * SUPRESSER;
    motorpw_r = joy->axes[1] * 100 * SUPRESSER;
  }else if(joy->axes[1] <= -THRESHOLD){
    status = BACK;
    motorpw_l = joy->axes[1] * 100 * SUPRESSER;
    motorpw_r = joy->axes[1] * 100 * SUPRESSER;
  }else{
    status = STOP;
    motorpw_l = 0;
    motorpw_r = 0;
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

  if(joy->buttons[0]){
    a_ispushed = 1;
  }
    
}

int max(int l, int r){
  if(l>0 && r>0){
    if(l>=r){
      vlr_max = DELTA_L;
      return l;
    }else{
      vlr_max = DELTA_R;
      return r;
    }
  }else if(l<0 && r<0){
    if(l<=r){
      vlr_max = DELTA_L;
      return l;
    }else{
      vlr_max = DELTA_R;
      return r;
    }
  }
}

void encCallback(const std_msgs::Int32MultiArray& lmr){
  /*int vl = lmr.data[DELTA_L];
  int vr = lmr.data[DELTA_R];
  int gap = lmr.data[DGAP];

  //ROS_INFO("encCallback");

  if(vl == 0 || vr == 0){
    gap_ratio = 0;
    extra_correction = 0;
  }else gap_ratio = (float)gap / (float)max(vl,vr);
  
  //if(gap_ratio < 0)gap_ratio *= -1;
  ex.data = extra_correction;*/
}

int main (int argc, char **argv){
  r_ispushed = 0;
  l_ispushed = 0;
  ros::init(argc,argv,"nhk18_controller");
  ros::NodeHandle nh;
  ros::Subscriber joy = nh.subscribe("joy", 1000, joyCallback);
  ros::Subscriber v_lmr = nh.subscribe("enc",1000, encCallback);
  ros::Publisher mr_pub = nh.advertise<std_msgs::Int16>("mr",1000);
  ros::Publisher ml_pub = nh.advertise<std_msgs::Int16>("ml",1000);
  ros::Publisher stpa_pub = nh.advertise<std_msgs::Int16>("stpa",1000);
  ros::Publisher stpb_pub = nh.advertise<std_msgs::Int16>("stpb",1000);
  ros::Rate loop_rate(50);

  ros::Publisher extra = nh.advertise<std_msgs::Float64>("ex",1000);

  while(ros::ok()){
    //set_motor_status();
    mpwsender_r.data = motorpw_r;
    mpwsender_l.data = motorpw_l;

    //get_correction();
    extra.publish(ex);

    //mpwsender_l.data += extra_correction;

    //if(vlr_max == DELTA_L)mpwsender_r.data += extra_correction;//補正
    //else if (vlr_max == DELTA_R)mpwsender_l.data += extra_correction;
    
    if(l_ispushed == 0 && r_ispushed == 1){//L,Rボタンに応じて片方に寄る走行をさせる
      if(mpwsender_r.data >= 0)
	mpwsender_r.data += L_OR_R_PLUS;//(mpwsender_l.data * SUPRESS);
      else mpwsender_r.data -= L_OR_R_PLUS;//(mpwsender_l.data * SUPRESS);
    }else if(l_ispushed == 1 && r_ispushed == 0){
      if(mpwsender_l.data >= 0)
	mpwsender_l.data += L_OR_R_PLUS;//(mpwsender_r.data * SUPRESS);
      else mpwsender_l.data -= L_OR_R_PLUS * 3;//(mpwsender_r.data * SUPRESS);
    }

    mr_pub.publish(mpwsender_r);
    ml_pub.publish(mpwsender_l);

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
