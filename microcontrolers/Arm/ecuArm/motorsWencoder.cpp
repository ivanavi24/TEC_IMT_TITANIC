#include "motorsWencoder.h"
#include "Arduino.h"
#include "isr.h"
#include "mode_configuration.h"
//#include "isr.h"


#define PI            3.1415926535
DCmotor_Encoder::DCmotor_Encoder(MotorDCEncoderParams motorParams){
      

      joint_current=ZERO_VAL_INITIALIZER;
      joint_desired=ZERO_VAL_INITIALIZER;
      joint_error_i=ZERO_VAL_INITIALIZER;


      avg_vel_pps_current = ZERO_VAL_INITIALIZER;
      joint_velocity_desired = ZERO_VAL_INITIALIZER;
      joint_velocity_error_i =ZERO_VAL_INITIALIZER;

      max_actuator_signal = motorParams.max_actuator_pwm_signal;
      min_actuator_signal = motorParams.min_actuator_pwm_signal;


      pwm_channel = motorParams.pwm_channel;
      pwm_frquency = motorParams.pwm_frquency;
      pwm_resolution = motorParams.pwm_resolution;


      kp = motorParams.kp;
      kd = motorParams.kd;
      ki = motorParams.ki;
      kp_v = motorParams.kp_v;
      kd_v = motorParams.kd_v;
      ki_v = motorParams.ki_v;
    
      
      joint_low_limit_hw = motorParams.joint_low_limit_hw;
      joint_high_limit_hw =motorParams.joint_high_limit_hw;
      joint_low_limit_sw = motorParams.joint_low_limit_sw;
      joint_high_limit_sw = motorParams.joint_high_limit_sw;
      
      encoderA = motorParams.encoderA;
      encoderB = motorParams.encoderB;
      positive_dir_pin = motorParams.positive_dir_pin;
      negative_dir_pin = motorParams.negative_dir_pin;
      pwm_pin = motorParams.pwm_pin;
      
      encoder_resolution = motorParams.encoder_resolution;
      average_pulses = motorParams.average_pulses;

      
      joint_error_i=0;
      
      My_timer = timerBegin(1, 80, true);
}

void DCmotor_Encoder::setPositionGains(float kpV, float kdV, float kiV){
  kp = kpV;
  kd = kdV;
  ki = kiV;
  
}
void DCmotor_Encoder::setVelocityGains(float kpV, float kdV, float kiV){
  kp_v = kpV;
  kd_v = kdV;
  ki_v = kiV;
}

void DCmotor_Encoder::move2position(float deltaTime){
      float joint_error = joint_desired - joint_current;
      if(abs(joint_error)< desired_joint_threshold )
      {
        reach_desired_joint = true;
      }
      else{
        reach_desired_joint = false;
      }
      float joint_error_d = joint_error / deltaTime;
      joint_error_i += joint_error * deltaTime;
      float joint_control= kp * joint_error + kd * joint_error_d + ki * joint_error_i;
      moveDirection();
      moveMotor(satureControl(abs(joint_control)));
}
float DCmotor_Encoder::getMotorRPM(){
  return float(avg_vel_pps_current) / float(encoder_resolution) * SECONDS_IN_MINUTES;
}

float DCmotor_Encoder::getMotorFrequency(){
  return avg_vel_pps_current;
}
PIN DCmotor_Encoder::getEncoderA(){
  return encoderA;
}

void DCmotor_Encoder::moveWVelocity(float deltaTime){
      float joint_vel_error = joint_velocity_desired - getMotorRPM();
      float joint_v_error_d = joint_vel_error / deltaTime;
      joint_velocity_error_i += joint_vel_error * deltaTime;
      float joint_control_v= kp_v * joint_vel_error + kd_v * joint_v_error_d + ki_v * joint_velocity_error_i;
      moveDirectionVelocity();
      Serial.printf("Deseado: %f  actual  %f\n",joint_velocity_desired, getMotorRPM());
      //Serial.printf("El error es  %f\n",joint_vel_error);
      //Serial.printf("La accion de control %f\n",joint_control_v);
      Serial.printf("La accion de control final %i\n",satureControl(joint_control_v));
      moveMotor(satureControl(joint_control_v));
}
void DCmotor_Encoder::moveMotor(int duty_cycle){
      ledcWrite(pwm_channel, duty_cycle);
    }
void DCmotor_Encoder:: moveDirectionVelocity(){
      if ( joint_velocity_desired > 0){
    DCmotor_Encoder::positiveMovement();
      }
      else if ( joint_velocity_desired < 0){
    DCmotor_Encoder::negativeMovement();
      }
      else{                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    DCmotor_Encoder::stopMovement();        
      }
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
      positive_movement = true;   
}  
void DCmotor_Encoder::negativeMovement(){
      digitalWrite(positive_dir_pin,LOW);
      digitalWrite(negative_dir_pin,HIGH);
      positive_movement = false;
}
    
void DCmotor_Encoder::stopMovement(){
      digitalWrite(positive_dir_pin,LOW);
      digitalWrite(negative_dir_pin,LOW);
    }
int DCmotor_Encoder::satureControl(float control_action){
      if ( float(control_action) > max_actuator_signal){
        return max_actuator_signal;
      }
      if ( float(control_action) < min_actuator_signal){
        return min_actuator_signal;
      }
      return int(control_action);
    }
void DCmotor_Encoder::setJointDesired( float desired_revolutions){
      joint_desired  = desired_revolutions*encoder_resolution;
      joint_error_i= ZERO_VAL_INITIALIZER;
      reach_desired_joint = false;
}
void DCmotor_Encoder::setJointDesiredFromAngle( float desired_angle){
      joint_desired  = desired_angle/360*encoder_resolution; //desired pulses
      joint_error_i= ZERO_VAL_INITIALIZER;
      reach_desired_joint = false;
}
void DCmotor_Encoder::setVelocityDesiredRPM( float desired_velocity){
      joint_velocity_desired  = desired_velocity;
      Serial.printf("La velocidad %f \n",joint_velocity_desired);
      Serial.printf("Las ganacias son  KP:%f  KI:%f KV:%f \n",kp_v,ki_v,kd_v);
      //joint_velocity_error_i= ZERO_VAL_INITIALIZER;
}
void DCmotor_Encoder::updateCurrentJoint(){
  static unsigned int pulseCounter = 0;
  static int pulseCounterDirection = 0;
  static long lastTime = 0;
  pulseCounter++;
  #if (DC_MOTOR_2ENCODERS==MOD_ON)
  int channelB_state= digitalRead(encoderB);
  if (channelB_state)
  {
    joint_current++;
    pulseCounterDirection++;
  }

  else
  {
    joint_current--;
    pulseCounterDirection--;
  }

  #else
  if (positive_movement)
  {
    joint_current++;
    pulseCounterDirection++;
  }

  else
  {
    joint_current--;
    pulseCounterDirection--;
  }


  #endif
  
  
  if(pulseCounter>=average_pulses)//(pulseCounter>=average_pulses
  {
    long currentTime =millis();
    long deltaTime = currentTime - lastTime ;
    avg_vel_pps_current =  double(pulseCounterDirection) / deltaTime*1000; //Average velocity in pulses per second HZ
    pulseCounter = 0;
    pulseCounterDirection = 0;
    lastTime = millis();
    zero_velocity_flag = false;
  }
  
}
void DCmotor_Encoder::setLimitSwitchReferencePoint(float revolutions){
  joint_current = revolutions*float(encoder_resolution);
}


void DCmotor_Encoder:: initializePWM(){
  pinMode(pwm_pin,OUTPUT);
  pinMode(positive_dir_pin,OUTPUT);
  pinMode(negative_dir_pin,OUTPUT);
  ledcSetup(pwm_channel, pwm_frquency, pwm_resolution);
  ledcAttachPin(pwm_pin, pwm_channel);
  
}
void DCmotor_Encoder::initilizeEncoders(){
  pinMode(encoderA,INPUT);
  timerAttachInterrupt(My_timer, &ISR__LOW_VELOCITY_JOINT2, true);
  timerAlarmWrite(My_timer, 50000, true); 
  timerAlarmEnable(My_timer); 
#if (#if (DC_MOTOR_2ENCODERS==MOD_ON))
    pinMode(encoderB,INPUT);
#endif
  
}
void DCmotor_Encoder::displayGainValues(){
  Serial.printf("Kp: %f Kd: %f Ki: %f  \n",kp,kd,ki);
}

int DCmotor_Encoder::getJointCurrentVal(){
  return joint_current;
}
int DCmotor_Encoder::getDesiredJointVal(){
  return joint_desired;
}

void DCmotor_Encoder::zeroVelocityCase()
{
  if (zero_velocity_flag)
  {

    avg_vel_pps_current=0;
  }
  zero_velocity_flag = true;
}


float DCmotor_Encoder::getVelocityDesired()
{
  return joint_velocity_desired;
}
