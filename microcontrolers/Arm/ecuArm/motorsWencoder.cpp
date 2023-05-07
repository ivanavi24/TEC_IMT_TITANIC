#include "motorsWencoder.h"


#include "Arduino.h"

DCmotor_Encoder::DCmotor_Encoder(MotorEncoderParams motorParams){
      

      joint_current=ZERO_VAL_INITIALIZER;
      joint_desired=ZERO_VAL_INITIALIZER;
      joint_error_i=ZERO_VAL_INITIALIZER;


      avg_vel_pps_current = ZERO_VAL_INITIALIZER;
      joint_velocity_desired = ZERO_VAL_INITIALIZER;
      joint_velocity_error_i =ZERO_VAL_INITIALIZER;

      min_actuator_signal = motorParams.max_actuator_pwm_signal;
      max_actuator_signal = motorParams.min_actuator_pwm_signal;
      kp = motorParams.kp;
      kd = motorParams.kd;
      ki = motorParams.ki;
      positive_dir_pin=motorParams.positive_dir_pin;
      negative_dir_pin=motorParams.negative_dir_pin;
      
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

      min_actuator_signal=0;
      max_actuator_signal=0;
      joint_error_i=0;

      //initializePWM( channel, freq, resolution);
}
void DCmotor_Encoder::move2position(float deltaTime){
      float joint_error = joint_desired - joint_current;
      float joint_error_d = joint_error / deltaTime;
      joint_error_i += joint_error * deltaTime;
      float joint_control= kp * joint_error + kd * joint_error_d + ki * joint_error_i;
      moveDirection();
      moveMotor(satureControl(joint_control));
}
void DCmotor_Encoder::moveWVelocity(float deltaTime){
      float joint_vel_error = getMotorRPM() - avg_vel_pps_current;
      float joint_v_error_d = joint_vel_error / deltaTime;
      joint_velocity_error_i += joint_vel_error * deltaTime;
      float joint_control_v= kp_v * joint_vel_error + kd_v * joint_v_error_d + ki_v * joint_velocity_error_i;
      moveDirectionVelocity();
      moveMotor(satureControl(joint_control_v));
}
void DCmotor_Encoder::moveMotor(int duty_cycle){
      ledcWrite(pwm_pin, duty_cycle);
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
      joint_desired  = desired_pulses*encoder_resolution;
      joint_error_i= ZERO_VAL_INITIALIZER;
}
void DCmotor_Encoder::setVelocityDesiredRPM( float desired_velocity){
      joint_velocity_desired  = desired_velocity;
      joint_velocity_error_i= ZERO_VAL_INITIALIZER;
}
void DCmotor_Encoder::updateCurrentJoint(){
  static unsigned int pulseCounter = 0;
  static int pulseCounterDirection = 0;
  static long lastTime = 0;
  pulseCounter++;
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
  
  if(pulseCounter>=average_pulses)
  {
    float deltaTime = millis() - lastTime ;
    avg_vel_pps_current =  pulseCounterDirection / deltaTime; //Average velocity in pulses per second HZ
    pulseCounter = 0;
    pulseCounterDirection = 0;
    lastTime = millis();
  }
  
}
float DCmotor_Encoder::getMotorRPM(){
  return float(avg_vel_pps_current) / float(encoder_resolution) * SECONDS_IN_MINUTES;
}

void DCmotor_Encoder:: initializePWM(unsigned char ledchannel, unsigned int freq, unsigned char resolution){
  ledcSetup(ledchannel, freq, resolution);
  ledcAttachPin(pwm_pin, ledchannel);
}

void DCmotor_Encoder::displayGainValues(){
  Serial.printf("Kp: %f Kd: %f Ki: %f  \n",kp,kd,ki);
}

