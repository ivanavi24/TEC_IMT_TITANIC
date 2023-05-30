#include "motorStep.h"
#include "mode_configuration.h"
#include "isr.h"    

#define PI            3.1415926535
Step_motor::Step_motor(MotorStepParams motorParams){
      

      phase1 = motorParams.IN1_PIN;
      phase2 = motorParams.IN2_PIN;
      phase3 = motorParams.IN3_PIN;
      phase4 = motorParams.IN4_PIN;
      limit_switch_high = motorParams.limit_switch_high;
      limit_switch_low = motorParams.limit_switch_low;

      min_micros_between_step = motorParams.min_micros_between_step;
      max_micros_between_step = motorParams.max_micros_between_step;
   
      joint_low_limit_hw = motorParams.joint_low_limit_hw;
      joint_high_limit_hw =motorParams.joint_high_limit_hw;
      joint_low_limit_sw = motorParams.joint_low_limit_sw;
      joint_high_limit_sw = motorParams.joint_high_limit_sw;
    
      steps_per_revolution = motorParams.steps_per_revolution;
      joint_current=ZERO_VAL_INITIALIZER;
      joint_desired=ZERO_VAL_INITIALIZER;
      joint_error_i=ZERO_VAL_INITIALIZER;    
      step_sequence[0]= phase1;
      step_sequence[1]= phase3;
      step_sequence[2]= phase2;
      step_sequence[3]= phase4;
      My_timer = timerBegin(0, 80, true);
}



void Step_motor::move2position(){

      moveDirection();
      //moveMotor(satureControl(abs(0)));
}
float Step_motor::getMotorRPM(){
  return float(0) / float(steps_per_revolution) * SECONDS_IN_MINUTES;
}


void Step_motor::moveWVelocity(float deltaTime){
}
void Step_motor::moveMotor(){
  static  int sequence_counter =0;
  for (int pin=0; pin<STEP_MOTOR_CHANNELS;pin++)
  {
    digitalWrite(step_sequence[pin],LOW);
  }
  digitalWrite(step_sequence[current_pin],HIGH);
  digitalWrite(step_sequence[(current_pin+1)%(STEP_MOTOR_CHANNELS)],HIGH);
  current_pin = current_pin >= (STEP_MOTOR_CHANNELS-1)? 0: current_pin + 1;  
  if (positiveMovementFlag){
    joint_current++;
  }
  else if (negativeMovementFlag){
    joint_current--;
  }
  if(joint_current == joint_desired)
  {
    
#if (!(CRANE_Z_AXIS_BEHAVIOR==CRANE_Z_AXIS_INTERMITENT))
    stopMovement();

#else
  
  reachFlag=true;  
  

#endif
  }
#if (CRANE_Z_AXIS_BEHAVIOR==CRANE_Z_AXIS_INTERMITENT)
    if(reachFlag){
    sequence_counter++;
    if (sequence_counter >(Z_AXIS_STOP_STEPS*float(steps_per_revolution))){
      if(negativeMovementFlag){
        positiveMovement();
      }
      else if (positiveMovementFlag){
        negativeMovement();
      }
      sequence_counter=0;
    }
  }

#endif   
  
    
}

void Step_motor:: moveDirection(){
      if ( (joint_desired - joint_current) > 0){
    Step_motor::positiveMovement();
    
      }
      else if ( (joint_desired - joint_current) < 0){
    Step_motor::negativeMovement();
      }
      else{
    Step_motor::stopMovement();        
      }
  }
    /*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
void Step_motor:: positiveMovement(){
      step_sequence[1] = phase3;
      step_sequence[3] = phase4;
      positiveMovementFlag=true;
      negativeMovementFlag=false;
}
void Step_motor::negativeMovement(){
      step_sequence[1] = phase4;
      step_sequence[3] = phase3;
      positiveMovementFlag=false;
      negativeMovementFlag=true;
}
void Step_motor::stopMovement(){
   if(timerAttached)
   {
      timerDetachInterrupt(My_timer);
      timerAttached = false;
   }   
  for (int pin=0; pin<STEP_MOTOR_CHANNELS;pin++){
    digitalWrite(step_sequence[pin],LOW);
  }
#if (CRANE_Z_AXIS_BEHAVIOR==CRANE_Z_AXIS_LOCKED)
  digitalWrite(step_sequence[0],HIGH);
#endif
    
}
int Step_motor::satureControl(float control_action){

}
void Step_motor::setJointDesired( float desired_revolutions){
      joint_desired  = float(desired_revolutions)*float(steps_per_revolution);//desired_revolutions*float(steps_per_revolution)*STEP_MOTOR_CHANNELS;
      joint_error_i= ZERO_VAL_INITIALIZER;
      moveDirection();
      timerAlarmEnable(My_timer);
      timerAttachInterrupt(My_timer, &ISR__TIME_JOINT3, true);
      timerAttached = true;
      reachFlag=false;
}
void Step_motor::setJointDesiredFromAngle( float desired_angle){
      joint_desired  = desired_angle/360*float(steps_per_revolution); //desired pulses
      joint_error_i= ZERO_VAL_INITIALIZER;
      moveDirection();
      timerAlarmEnable(My_timer);
      timerAttachInterrupt(My_timer, &ISR__TIME_JOINT3, true);
      timerAttached = true;
      reachFlag=false;
}
void Step_motor::setVelocityDesiredRPM( float desired_velocity){
      joint_velocity_desired_RPM  = desired_velocity;
}
void Step_motor::updateCurrentJoint(int value){
  joint_current = joint_current + value;
}
void Step_motor::setLimitSwitchReferencePoint(unsigned char value){
  if (value){
    joint_current = joint_high_limit_hw/(2*PI)*float(steps_per_revolution);
  }
  else if (!value){ /*Set joint current value to zero val*/
    joint_current = ZERO_VAL_INITIALIZER;
  }

}

void Step_motor::initilizePINS(){

  pinMode(phase1,OUTPUT);
  pinMode(phase2,OUTPUT);
  pinMode(phase3,OUTPUT);
  pinMode(phase4,OUTPUT);
  pinMode(limit_switch_high,INPUT);
  pinMode(limit_switch_low,INPUT);
  joint_current=0;
  
  timerAttachInterrupt(My_timer, &ISR__TIME_JOINT3, true);
  timerAlarmWrite(My_timer, 1600, true);  
  //timerAlarmEnable(My_timer);
  
}
int Step_motor::getJointCurrentVal(){
  return joint_current;
}
int Step_motor::getDesiredJointVal(){
  return joint_desired;
}

float Step_motor::getMotorFrequency(){
  return 0;
}

void Step_motor::resetCurrentJoint(){
  joint_current=0;
  Serial.print("Joint Current: ");
  Serial.println(joint_current);
}