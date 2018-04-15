#include <ros.h>

#include <std_msgs/Int8.h>

#include <std_msgs/Int16.h>


#define WHEEL_DIAMETER 25//だいたい

#define STEP 1.8

#define STEPS 200// 360 / STEP (360度の回転に何パルス必要か

#define RPM 30 //分あたりの回転数

#define PULSE_ROLL (WHEEL_DIAMETER * 3.14 * STEP / 360)//１パルスあたり

#define STEPPING_MOTOR_SUM 2

#define DEFAULTPW 80

#define PULSEPIN_A 2

#define DIRPIN_A 3

#define PULSEPIN_B 4

#define DIRPIN_B 5

#define BOTH 0
#define ONLY_A 1
#define ONLY_B 2
volatile int input_pulse = 0;

ros::NodeHandle nh;
int join = 0;

void stpaCallback(const std_msgs::Int16& a){
     
     input_pulse = a.data/PULSE_ROLL;
     
     
}
void stpJoinCallback(const std_msgs::Int16& j){
     join = j.data;
     
}
ros::Subscriber<std_msgs::Int16> stpa("stpa",stpaCallback);
ros::Subscriber<std_msgs::Int16>stpa_join_sub("stp_join",stpJoinCallback);
void pulse(){
  
  static  boolean output = HIGH;
 
  
  digitalWrite(PULSEPIN_A,output);//パルスをHIGH
  digitalWrite(PULSEPIN_B,output);
  
  output = !output;
  
     
    
    
  
  }

  void pulse_A(){
    static boolean output_A = HIGH;

    digitalWrite(PULSEPIN_A,output_A);

    output_A = !output_A;
  }

  void pulse_B(){
    static boolean output_B = HIGH;

    digitalWrite(PULSEPIN_B,output_B);
    output_B = !output_B;
  }







void test(int b){
     int i,j = 0;
     static int count = 0;

     if(b>=0){
      Serial.print("+");
      
      digitalWrite(DIRPIN_A,LOW);
      digitalWrite(DIRPIN_B,LOW);
      
      
      pulse();

      count++;
  
     
      
      
      }

     if(b<0){
      Serial.print("-");
      b= -b;
      digitalWrite(DIRPIN_A,HIGH);
      digitalWrite(DIRPIN_B,HIGH);
      
      
      pulse();
  
      count++;
      
      
      }

     if(count>b*2){
      
      count = 0;

      while(1){
        nh.spinOnce();

        static int num2 = 0;
        int num1 = input_pulse;

        if(num1 != num2 ){
          num2 = num1;

          break;
          
          }
        
        num2 = num1;
        
        
        }
      
      
     
      }


}

void test_A(int b){
  int i,j = 0;
     static int count_A = 0;

     if(b>=0){
      Serial.print("+");
      
      digitalWrite(DIRPIN_A,LOW);
      
      
      pulse_A();

      count_A++;
  
     
      
      
      }

     if(b<0){
      Serial.print("-");
      b= -b;
      digitalWrite(DIRPIN_A,HIGH);
      
      
      pulse_A();
  
      count_A++;
      
      
      }

     if(count_A>b*2){
      
      count_A = 0;

      while(1){
        nh.spinOnce();

        static int num2_A = 0;
        int num1_A = input_pulse;

        if(num1_A != num2_A ){
          num2_A = num1_A;

          break;
          
          }
        
        num2_A = num1_A;
        
        
        }
      
      
     
     }
}




void test_B(int b){
  int i,j = 0;
     static int count_B = 0;

     if(b>=0){
      Serial.print("+");
      
      digitalWrite(DIRPIN_B,LOW);
      
      
      pulse_B();

      count_B++;
  
     
      
      
      }

     if(b<0){
      Serial.print("-");
      b= -b;
      digitalWrite(DIRPIN_B,HIGH);
      
      
      pulse_B();
  
      count_B++;
      
      
      }

     if(count_B>b*2){
      
      count_B = 0;

      while(1){
        nh.spinOnce();

        static int num2_B = 0;
        int num1_B = input_pulse;

        if(num1_B != num2_B ){
          num2_B = num1_B;

          break;
          
          }
        
        num2_B = num1_B;
        
        
        }
      
      
     
      }
}






void setup(){
  pinMode(DIRPIN_A,OUTPUT);
  pinMode(PULSEPIN_A,OUTPUT);
  digitalWrite(DIRPIN_A,HIGH);
  digitalWrite(PULSEPIN_A,LOW);
  pinMode(DIRPIN_B,OUTPUT);
  pinMode(PULSEPIN_B,OUTPUT);
  digitalWrite(DIRPIN_B,HIGH);
  digitalWrite(PULSEPIN_B,LOW);
  Serial.begin(9600);
  
  
  nh.initNode();

  nh.subscribe(stpa);
  nh.subscribe(stpa_join_sub);


}



void loop(){
  Serial.print(1);


  if(join == 0){
    test(input_pulse);
  }else if(join == 1){
    test_A(input_pulse);
  }else if(join == 2){
    test_B(input_pulse);
  }


  delayMicroseconds(1400);

  nh.spinOnce();

}
