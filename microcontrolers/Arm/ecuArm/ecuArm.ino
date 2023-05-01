#include "Wire.h"

#define I2C_DEV_ADDR 0x55
/* Kinematic variable definition*/

#define origin2waterDistance            30.4


#define START_COMMAND                   0x55 /* binary ->  0101 0101*/
#define END_COMMAND                     0xAA /* binary ->  1010 1010/


#define JOINT1_ZERO_POSITION            60.5
#define JOINT2_ZERO_POSITION            60.5
#define JOINT3_ZERO_POSITION            60.5


#define JOINT1_ENCODER_RESOLUTION       50
#define JOINT2_ENCODER_RESOLUTION       50
#define JOINT3_ENCODER_RESOLUTION       50


#define JOINT1_LOW_LIMIT_HW                0   /*First Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT1_HIGH_LIMIT_HW               90  /*First Joint movement limit in terms of pulses*/
#define JOINT2_LOW_LIMIT_HW                0   /*Second Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT2_HIGH_LIMIT_HW               90  /*Second Joint movement limit in terms of pulses*/
#define JOINT3_LOW_LIMIT_HW                0   /*Second Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT3_HIGH_LIMIT_HW               90  /*Secon Joint movement limit in terms of pulses*/


#define JOINT1_LOW_LIMIT_SW                 0   /*First Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT1_HIGH_LIMIT_SW               90  /*First Joint movement limit in terms of pulses*/
#define JOINT2_LOW_LIMIT_SW                0   /*Second Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT2_HIGH_LIMIT_SW               90  /*Second Joint movement limit in terms of pulses*/
#define JOINT3_LOW_LIMIT_SW                0   /*Second Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT3_HIGH_LIMIT_SW               90  /*Secon Joint movement limit in terms of pulses*/


#define KP_1                            1
#define KI_1                            1
#define KD_1                            1
#define POSITIVE_DIR_PIN_1              5
#define NEGATIVE_DIR_PIN_1              5
#define ZERO_POS_1                      1.57


#define KP_2 1
#define KI_2                            1
#define KD_2                            1
#define POSITIVE_DIR_PIN_2                5
#define NEGATIVE_DIR_PIN_2                5
#define ZERO_POS_2                      1.57

#define KP_3                            1
#define KI_3                            1
#define KD_3                            1
#define POSITIVE_DIR_PIN_3                5
#define NEGATIVE_DIR_PIN_3                5
#define ZERO_POS_3                      1.57


#define PI                             3.14159265

#define PID_INTERVAL 5
uint32_t i = 0;
long time_now=millis();
long time_last=millis();


class DCmotor_Encoder{
  private: 
    int joint_desired;
    int joint_current;
    unsigned char positive_dir_pin=0;
    unsigned char negative_dir_pin=0;
    
    const unsigned int min_actuator_signal=0;
    const unsigned int max_actuator_signal=0;
    
    float kp, kd, ki;
    float joint_error_i=0;

    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;
    
    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;

    
  public:
    DCmotor_Encoder(float kpV, float kiV, float kdV, unsigned char positive_dir_pinV,unsigned char negative_dir_pinV,float joint_low_limit_hwV,
                   float  joint_high_limit_hwV, float joint_low_limit_swV, float joint_high_limit_swV){
      kp=kpV;
      kd=kdV;
      ki=kiV;
      positive_dir_pin=positive_dir_pinV;
      negative_dir_pin=negative_dir_pinV;
      joint_low_limit_hw = joint_low_limit_hwV;
      joint_high_limit_hw =joint_high_limit_hwV;
      joint_low_limit_sw = joint_low_limit_swV;
      joint_high_limit_sw = joint_high_limit_swV;
    }
    void move2position(float deltaTime){
      float joint_error = joint_desired - joint_current;
      float joint_error_d = joint_error / deltaTime;
      joint_error_i += joint_error * deltaTime;
      float joint_control= kp * joint_error + kd * joint_error_d + ki * joint_error_i;
      moveDirection();
      moveMotor(satureControl(joint_control));
    }
    void moveMotor(int duty_cycle){
      ledcWrite(0, duty_cycle);
    }
    void moveDirection(){
      if ( (joint_desired - joint_current) > 0){
        positiveMovement();
      }
      else if ( (joint_desired - joint_current) < 0){
        negativeMovement();
      }
      else{
        stopMovement();        
      }
    }
    /*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
    void positiveMovement(){
      digitalWrite(negative_dir_pin,LOW);
      digitalWrite(positive_dir_pin,HIGH);
    }
    void negativeMovement(){
      digitalWrite(positive_dir_pin,LOW);
      digitalWrite(negative_dir_pin,HIGH);
    }
    void stopMovement(){
      digitalWrite(positive_dir_pin,LOW);
      digitalWrite(negative_dir_pin,LOW);
    }
    unsigned int satureControl(float control_action){
      if ( control_action > max_actuator_signal){
        return max_actuator_signal;
      }
      if ( control_action < min_actuator_signal){
        return min_actuator_signal;
      }
      return int(control_action);
    }
    void setJointDesired( int desired_pulses){
      joint_desired  = desired_pulses;
    }
    
};
class manipulator3dof{
  private:

    float origin2water=origin2waterDistance;
    DCmotor_Encoder first_motor=DCmotor_Encoder(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT1_LOW_LIMIT_HW,JOINT1_HIGH_LIMIT_HW,JOINT1_LOW_LIMIT_SW,JOINT1_HIGH_LIMIT_SW);
    DCmotor_Encoder second_motor= DCmotor_Encoder(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT2_LOW_LIMIT_HW,JOINT2_HIGH_LIMIT_HW,JOINT2_LOW_LIMIT_SW,JOINT2_HIGH_LIMIT_SW);
    DCmotor_Encoder third_motor= DCmotor_Encoder(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT3_LOW_LIMIT_HW,JOINT3_HIGH_LIMIT_HW,JOINT3_LOW_LIMIT_SW,JOINT3_HIGH_LIMIT_SW);


    float x_desired, y_desired, z_desired; /*Set thorugh i2c communication intereruption*/
    
    /*Encoder resolutions in pulses per revolution*/
    unsigned int joint1_encoder_resolution = JOINT1_ENCODER_RESOLUTION;
    unsigned int joint2_encoder_resolution = JOINT2_ENCODER_RESOLUTION;
    unsigned int joint3_encoder_resolution = JOINT1_ENCODER_RESOLUTION;

   /**/
    float theta,radius,z_height;

  public:
    manipulator3dof (){
      
    }
    /*Assumes corrdinate frame located in the center of the robotic arm*/
    void inverse_kinematics(float x, float y, float z){
      theta =atan2(x,y);
      radius =sqrt(pow(x,2)+pow(y,2));
      z_height = z + origin2water; 
    }

    void reachPosition(float deltaTime){
      
      first_motor.move2position(deltaTime);
      second_motor.move2position(deltaTime);
      third_motor.move2position(deltaTime);
      
    }
  /*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
  void setTargetJoints(){
    int pulsesJoint1 =(theta - ZERO_POS_1)* joint1_encoder_resolution/ (2* PI);   /*  pulses per revolution [pulses/rev]*  */ 
    int pulsesJoint2 =(radius - ZERO_POS_2)* joint2_encoder_resolution;   /*  pulses per revolution [pulses/rev]*  */ 
    int pulsesJoint3 =(z_height - ZERO_POS_3)* joint3_encoder_resolution;   /*  pulses per revolution [pulses/rev]*  */ 
    first_motor.setJointDesired(pulsesJoint1);
    second_motor.setJointDesired(pulsesJoint2);
    third_motor.setJointDesired(pulsesJoint3);
  }
    
};
manipulator3dof titanicCrane;
void onRequest(){
  
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}
void readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
}
void onReceive(int len){
  /*Receive x,y,z values*/
  if (len==8){
    int tempByte = Wire.read();
    if (startCommand == START_COMMAND){
      float xref=(float(readUint16data())- pow(2,15))/1000;
      float yref=(float(readUint16data())- pow(2,15))/1000;
      float zref=(float(readUint16data())- pow(2,15))/1000;
      tempByte = Wire.read();
      if(tempByte == END_COMMAND){
        titanicCrane.inverse_kinematics( xref,  yref,  zref);
        titanicCrane.setTargetJoints(); 
      }      
    }
    else{
      while(Wire.available()){
        Wire.read();
      }
    }
    
    
  }
  /*Command with gripper action*/


  /*Command with just arm action*/
  Serial.printf("onReceive[%d]: ", len);
  
  while(Wire.available()){
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  

}
/*Main Loop*/
void loop() {
  time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();
  }

}
