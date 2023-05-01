#include "motorsWencoder.h"
#include "Arduino.h"

DCmotor_Encoder::DCmotor_Encoder(float kpV, float kiV, float kdV, unsigned char positive_dir_pinV,unsigned char negative_dir_pinV,float joint_low_limit_hwV,
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
      min_actuator_signal=0;
      max_actuator_signal=0;
      joint_error_i=0;
}
void DCmotor_Encoder::move2position(float deltaTime){
      float joint_error = joint_desired - joint_current;
      float joint_error_d = joint_error / deltaTime;
      joint_error_i += joint_error * deltaTime;
      float joint_control= kp * joint_error + kd * joint_error_d + ki * joint_error_i;
      moveDirection();
      moveMotor(satureControl(joint_control));
}
void DCmotor_Encoder::moveMotor(int duty_cycle){
      ledcWrite(0, duty_cycle);
}
void DCmotor_Encoder:: moveDirection(){
  if ( (joint_desired - joint_current) > 0){
    DCmotor_Encoder::positiveMovement();
  }
  else if ( (joint_desired - joint_current) < 0){
    DCmotor_Encoder::negativeMovement();
  }
  else{
    DCmotor_Encoder::stopMovement();        
  }
}
/*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
void DCmotor_Encoder:: positiveMovement(){
  digitalWrite(negative_dir_pin,LOW);
  digitalWrite(positive_dir_pin,HIGH);
}
void DCmotor_Encoder::negativeMovement(){
  digitalWrite(positive_dir_pin,LOW);
  digitalWrite(negative_dir_pin,HIGH);
}
void DCmotor_Encoder::stopMovement(){
  digitalWrite(positive_dir_pin,LOW);
  digitalWrite(negative_dir_pin,LOW);
}
unsigned int DCmotor_Encoder::satureControl(float control_action){
  if ( control_action > max_actuator_signal){
    return max_actuator_signal;
  }
  if ( control_action < min_actuator_signal){
    return min_actuator_signal;
  }
  return int(control_action);
}
void DCmotor_Encoder::setJointDesired( int desired_pulses){
  joint_desired  = desired_pulses;
}
    