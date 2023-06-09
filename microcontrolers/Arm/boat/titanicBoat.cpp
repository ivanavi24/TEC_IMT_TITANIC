
#include "joint_right.h"
#include "joint_left.h"
#include "mode_configuration.h"
#include "isr.h"
#include "stateMachine.h"

#include "titanicBoat.h"
#include "math.h"
#include "Arduino.h"


#define PI                          3.14159265
#define RADIUS_MIN          0.3
#define RADIUS_MAX          0.5
#define THETA_MIN          0.3
#define THETA_MAX          0.5


#define OBJECT_RIGHT        (0.5 *100) //cm
#define OBJECT_CENTER       (0.5 *100) //cm
#define OBJECT_LEFT       (0.5 *100) //cm

#define STRAIGHT_LINE_OBSTACLE    500

#define STEER_SLOPE               10

#define LS_J1_H                   34
#define LS_J1_L                   35
#define LS_J2_H                   36
#define LS_J2_L                   39
#define LS_J3_H                   39
#define LS_J3_L                   39

extern ultrasonicSensor soundSystem;
extern stateMachine sequenceMachine;

boat::boat ():right_motor(joint_right),
left_motor(joint_left){


  //My_timer = timerBegin(MAIN_ROUTINE_TIMER, 80, true);
  LimitSwitch_j1_high =LS_J1_H;
  LimitSwitch_j1_low = LS_J1_L;
   
  LimitSwitch_j2_high = LS_J2_H;
  LimitSwitch_j2_low = LS_J2_L;

  LimitSwitch_j3_high = LS_J3_H;
  LimitSwitch_j3_low = LS_J3_L;

  



}
/*Getter Methods*/
DCmotor_Encoder boat::get_right_motor(){
  return right_motor;
}
DCmotor_Encoder boat::get_left_motor(){
  return left_motor;
}


/*Setter methods*/
void boat::setTargetRPM(unsigned char index){
  float velocity;
  Serial.printf("Desired RPM MOTOR %u: ",index);      //Prompt User for input
  while (Serial.available()==0)  {
  }
  velocity = Serial.parseFloat();
  Serial.println(velocity);  
  switch (index)
  {
  case MOTOR1:
    right_motor.setVelocityDesiredRPM(velocity);
    break;
  case MOTOR2:
    left_motor.setVelocityDesiredRPM(velocity);
    break;
  default:
    break;
  } 
}


void boat::set_target_position(float x, float y, float z){
#if (SERIAL_PRINT_CONTROL_ESP32)
  float xD,yD,zD; 
  Serial.printf("Target x[m]: ");      //Prompt User for input
  while (Serial.available()==0)  {
  }
  xD = Serial.parseFloat();
  Serial.printf("%f\n",xD);
  Serial.printf("Target y[m]: ");      //Prompt User for input
  while (Serial.available()==0)  {
  }
  yD = Serial.parseFloat();  
  Serial.printf("%f\n",yD);
  Serial.printf("Target z[m]: ");      //Prompt User for input
  while (Serial.available()==0)  {
  }
  zD = Serial.parseFloat();
  Serial.printf("%f\n",zD);
  if(inverse_kinematics(x,y,z))
  {
    setTargetJoints();
  } 
#else
  if(inverse_kinematics(x,y,z))
  {
    setTargetJoints();
  } 
#endif
  
}
void boat::reachPosition(){
  
  right_motor.move2position(MAIN_ROUTINE_CONTROL_TIME);
  left_motor.move2position(MAIN_ROUTINE_CONTROL_TIME);
  if (right_motor.reach_desired_joint & left_motor.reach_desired_joint)
  {
    craneState new_state = sequenceMachine.determineNextState();
    sequenceMachine.changeState(new_state);
  }
  
}

void boat::moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution){
  int dCycle = (pwm[0]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 1 es: %d \n",dCycle);
  right_motor.moveMotor(dCycle);
  dCycle = (pwm[1]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 2 es: %d \n",dCycle);
  left_motor.moveMotor(dCycle);
  dCycle = (pwm[2]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 2 es: %d \n",dCycle);

}
void boat::updateMotors(unsigned char index){
  switch (index)
  {
  case MOTOR1:
    right_motor.updateCurrentJoint();
    break;
  case MOTOR2:
    left_motor.updateCurrentJoint();
    break;
  default:
    break;
  }
  
}

void boat::adjustMotorGains(unsigned char index,bool velocityGains){
  float kp,kd,ki;
  char gainsLetter ='P"';
  if (velocityGains){
    gainsLetter = 'V';
  }
  Serial.printf("Motor%u kp %u: ",index,gainsLetter);      //Prompt User for input
  while (Serial.available()==0)  {
  }
  kp = Serial.parseFloat();
  Serial.printf("%f\n",kp);
  Serial.printf("Motor%u kd %u: ",index,gainsLetter); 
  while (Serial.available()==0)  {
  }
  kd = Serial.parseFloat();  
  Serial.printf("%f\n",kd);
  Serial.printf("Motor%u ki %u: ",index,gainsLetter); 
  while (Serial.available()==0)  {
  }
  ki = Serial.parseFloat();
  Serial.printf("%f\n",ki);
  if(!velocityGains){
    switch (index)
    {
    case MOTOR1:
      right_motor.setPositionGains(kp,kd,ki);
      break;
    case MOTOR2:
      left_motor.setPositionGains(kp,kd,ki);
      break;
    default:
      break;
    }

  }
  else{
    switch (index)
    {
    case MOTOR1:
      right_motor.setVelocityGains(kp,kd,ki);
      break;
    case MOTOR2:
      left_motor.setVelocityGains(kp,kd,ki);
      break;
    default:
      break;
    }
  }
  

}
void boat::displayEncodersFrequency(unsigned char index){
  Serial.printf("Motor 1 frequency: %f\n",right_motor.getMotorFrequency());
  Serial.printf("Motor 2 frequency: %f\n",left_motor.getMotorFrequency());
  Serial.println("");
}
void boat::printMotorGains(unsigned char index){
  right_motor.displayGainValues();
  left_motor.displayGainValues();
}
void boat::initializeVars(){



  right_motor.initializePWM();

  left_motor.initializePWM();


  pinMode(LimitSwitch_j1_high,INPUT);
  pinMode(LimitSwitch_j1_low,INPUT); 
  pinMode(LimitSwitch_j2_high,INPUT);
  pinMode(LimitSwitch_j2_low,INPUT); 
  pinMode(LimitSwitch_j3_high,INPUT);
  pinMode(LimitSwitch_j1_low,INPUT); 

  attachInterrupt(LimitSwitch_j1_high, ISR__LIMIT_SWITCH_H_JOINT1, CHANGE);
  attachInterrupt(LimitSwitch_j1_low, ISR__LIMIT_SWITCH_L_JOINT1, CHANGE);
  attachInterrupt(LimitSwitch_j2_high, ISR__LIMIT_SWITCH_H_JOINT1, CHANGE);
  attachInterrupt(LimitSwitch_j2_low, ISR__LIMIT_SWITCH_L_JOINT1, CHANGE);
  attachInterrupt(LimitSwitch_j3_high, ISR__LIMIT_SWITCH_H_JOINT3, CHANGE);
  attachInterrupt(LimitSwitch_j1_low, ISR__LIMIT_SWITCH_L_JOINT3, CHANGE);

 // timerAttachInterrupt(My_timer, &ISR__TIME_POSIITON, true);
  //timerAlarmWrite(My_timer, MAIN_ROUTINE_CONTROL_TIME, true);  
  //timerAlarmEnable(My_timer);
  
  
};

void boat::moveMotor(unsigned char index, float deltaTime){
  switch (index)
  {
  case MOTOR1:
    right_motor.move2position(deltaTime);
    break;
  case MOTOR2:
    left_motor.move2position(deltaTime);
    break;
  case MOTOR3:
    break;
  default:
    break;
  }
}
void boat::moveMotorVel(unsigned char index, float deltaTime){
  switch (index)
  {
  case MOTOR1:
    right_motor.moveWVelocity(deltaTime);
    break;
  case MOTOR2:
    left_motor.moveWVelocity(deltaTime);
    break;
  default:
    break;
  }
}

void boat::stopMotorMovement(unsigned char index){
  switch (index)
  {
  case MOTOR1:
    right_motor.stopMovement();
    break;
  case MOTOR2:
    left_motor.stopMovement();
    break;
  default:
    break;
  }

}


void boat::setZeroVelocityMotors(unsigned char index)
{
  switch (index)
  {
  case MOTOR1:
    break;
  case MOTOR2:
    left_motor.zeroVelocityCase();
    break;
  case MOTOR3:
    break;
  default:
    break;
  }

}
void boat::stopAllMotors(){
  right_motor.stopMovement();
  left_motor.stopMovement();

}


void boat::compareReferenceandCurrent(unsigned char index){

  switch (index)
  {
  case MOTOR1:
    Serial.printf("%u Current Joint: %i Desired Joint %i \n",MOTOR1,right_motor.getJointCurrentVal(),right_motor.getDesiredJointVal());
  case MOTOR2:
    Serial.printf("%u Current Joint: %i Desired Joint %i \n",MOTOR2,left_motor.getJointCurrentVal(),left_motor.getDesiredJointVal());
    break;
  default:
    break;
  }
}



void boat::craneMovement()
{
  Serial.println("Reactive navigation");
  reachPosition();
  if (sequenceMachine.getCurrentState() == sailing)
  {
    
  }
  else if (sequenceMachine.getCurrentState() == recolection)
  {

    
  }
  
}

void boat::reactiveNavigation()
{
  bool objectRight = soundSystem.d_right < OBJECT_RIGHT;
  bool objectCenter = soundSystem.d_center < OBJECT_CENTER;
  bool objectLeft = soundSystem.d_left< OBJECT_LEFT ;
  if(objectRight | objectLeft |  objectCenter)
  {

    Serial.println("Reactive navigation");
    float steerLeft = 0;
    float steerRight = 0;
    long error_side = soundSystem.d_right-soundSystem.d_left; //more distance to the right so steer left motor

    if(error_side>0)
    {
      steerLeft = STRAIGHT_LINE_OBSTACLE + abs(error_side)*STEER_SLOPE;    
      steerRight = STRAIGHT_LINE_OBSTACLE - abs(error_side)*STEER_SLOPE;

    }
    else
    {
      steerLeft = STRAIGHT_LINE_OBSTACLE - abs(error_side)*STEER_SLOPE;    
      steerRight = STRAIGHT_LINE_OBSTACLE + abs(error_side)*STEER_SLOPE;

    }
    Serial.printf("PWM Right  %i   PWM Left %i \n",left_motor.satureControl(steerLeft),right_motor.satureControl(steerRight));
    left_motor.moveMotor(left_motor.satureControl(steerLeft));
    right_motor.moveMotor(right_motor.satureControl(steerRight));
    
    
  }
  else
  {
    left_motor.moveMotor(STRAIGHT_LINE_OBSTACLE);
    right_motor.moveMotor(STRAIGHT_LINE_OBSTACLE);
    //stopAllMotors();
  }
  
 
}
