
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"
#include "mode_configuration.h"
#include "isr.h"

#include "Crane3dof.h"
#include "math.h"
#include "Arduino.h"


#define PI                          3.14159265
#define RADIUS_MIN          0.3
#define RADIUS_MAX          0.5
#define THETA_MIN          0.3
#define THETA_MAX          0.5



Crane3dof::Crane3dof ():first_motor(joint1),
second_motor(joint2),
third_motor(joint3){
  origin2water=0;
  x_desired=0;
  y_desired=0;
  z_desired=0;
  theta=0;
  radius=0;
  z_height=0;

}
/*Getter Methods*/
DCmotor_Encoder Crane3dof::get_first_motor(){
  return first_motor;
}
DCmotor_Encoder Crane3dof::get_second_motor(){
  return second_motor;
}
Step_motor Crane3dof::get_third_motor(){
  return third_motor;
}

/*Setter methods*/
void Crane3dof::setTargetRPM(unsigned char index){
  float velocity;
  Serial.printf("Desired RPM MOTOR %u: ",index);      //Prompt User for input
  while (Serial.available()==0)  {
  }
  velocity = Serial.parseFloat();
  Serial.println(velocity);  
  switch (index)
  {
  case MOTOR1:
    first_motor.setVelocityDesiredRPM(velocity);
    break;
  case MOTOR2:
    second_motor.setVelocityDesiredRPM(velocity);
    break;
  case MOTOR3:
    third_motor.setVelocityDesiredRPM(velocity);
    break;
  default:
    break;
  } 
}
void Crane3dof::setTargetAngle(unsigned char index){
  float angle;
  Serial.printf("Desired Angle MOTOR%u [degrees]: ",index);      //Prompt User for input
  while (Serial.available()==0) {
  }
  angle = Serial.parseFloat();
  Serial.print(angle);  
  Serial.println(" .");
  switch (index)
  {
  case MOTOR1:
    first_motor.setJointDesiredFromAngle(angle);
    break;
  case MOTOR2:
    second_motor.setJointDesiredFromAngle(angle);
    break;
  case MOTOR3:
    third_motor.setJointDesiredFromAngle(angle);
    break;
  default:
    break;
  }
}

/*Assumes corrdinate frame located in the center of the robotic arm*/
bool Crane3dof::inverse_kinematics(float x, float y, float z){
  float thetaWorld =atan2(x,y);
  float radiusWorld =sqrt(pow(x,2)+pow(y,2));
  float z_world= z + origin2water; 
  if ( abs (thetaWorld) > (PI / 2) ){
    if( thetaWorld >0){
      thetaWorld -= PI;
    }
    else if ( thetaWorld <0){
      thetaWorld += PI;
    }
    //some flag here
    radiusWorld = -radiusWorld ;

  }  
  if (restrict2Workspace(thetaWorld, radiusWorld, z_world))
  {
    theta = thetaWorld;
    radius = radiusWorld;
    z_height = z_world;
#if (SERIAL_PRINT_CONTROL_ESP32)
    Serial.printf("IK Coords set to theta:%f  radius:%f height:%f \n", theta,radius,z_height);
#endif
    return true; 
  }
#if (SERIAL_PRINT_CONTROL_ESP32)
    Serial.printf("Given coordinates were out of Cranes workspace \n");
#endif
  return false;
}
bool Crane3dof::restrict2Workspace(float thetaW, float radiusW, float zW){
  bool radiusFlag = false;
  bool thetaFlag = false;
  if( (radiusW>RADIUS_MIN) && (radiusW<RADIUS_MAX)){radiusFlag = true;}
  if( (thetaW>THETA_MIN) && (thetaW<THETA_MAX)){radiusFlag = true;}
  if (radiusFlag && thetaFlag){
    return true;
  }
  return false;
}
void Crane3dof::set_target_position(float x, float y, float z){
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
void Crane3dof::reachPosition(float deltaTime){
  
  first_motor.move2position(deltaTime);
  second_motor.move2position(deltaTime);
  third_motor.move2position();
  
}
/*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
void Crane3dof::setTargetJoints(){
  float pulsesJoint1 =(theta - ZERO_POS_1)/ (2* PI);   /*  [radians]/[radians] = revolutions */ 
  float pulsesJoint2 =(radius - ZERO_POS_2+INIT_POS_HALF_LINE)*REVOLUTIONS_PER_METER_2;   /*  [meters]*[rev/meter] = revolutions  */ 
  float pulsesJoint3 =(z_height - ZERO_POS_3)*REVOLUTIONS_PER_METER_3;   /*  [meters]*[rev/meter] = revolutions  */ 
  first_motor.setJointDesired(pulsesJoint1);
  second_motor.setJointDesired(pulsesJoint2);
  third_motor.setJointDesired(pulsesJoint3);
}
void Crane3dof::moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution){
  int dCycle = (pwm[0]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 1 es: %d \n",dCycle);
  first_motor.moveMotor(dCycle);
  dCycle = (pwm[1]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 2 es: %d \n",dCycle);
  second_motor.moveMotor(dCycle);
  dCycle = (pwm[2]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 2 es: %d \n",dCycle);
  //third_motor.moveMotor(dCycle); 
}
void Crane3dof::updateMotors(unsigned char index){
  switch (index)
  {
  case MOTOR1:
    first_motor.updateCurrentJoint();
    break;
  case MOTOR2:
    second_motor.updateCurrentJoint();
    break;
  case MOTOR3:
    third_motor.moveMotor();
    break;
  default:
    break;
  }
  
}

void Crane3dof::adjustMotorGains(unsigned char index,bool velocityGains){
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
      first_motor.setPositionGains(kp,kd,ki);
      break;
    case MOTOR2:
      second_motor.setPositionGains(kp,kd,ki);
      break;
    default:
      break;
    }

  }
  else{
    switch (index)
    {
    case MOTOR1:
      first_motor.setVelocityGains(kp,kd,ki);
      break;
    case MOTOR2:
      second_motor.setVelocityGains(kp,kd,ki);
      break;
    default:
      break;
    }
  }
  

}
void Crane3dof::displayEncodersFrequency(unsigned char index){
  Serial.printf("Motor 1 frequency: %f\n",first_motor.getMotorFrequency());
  Serial.printf("Motor 2 frequency: %f\n",second_motor.getMotorFrequency());
  Serial.printf("Motor 3 frequency: %f\n",third_motor.getMotorFrequency());
  Serial.println("");
}
void Crane3dof::printMotorGains(unsigned char index){
  first_motor.displayGainValues();
  second_motor.displayGainValues();
}
void Crane3dof::initializeVars(){

  /*first_motor.initilizeEncoders();
  first_motor.initializePWM();
  first_motor.stopMovement();
  attachInterrupt(first_motor.getEncoderA(), ISR__ENCODER_JOINT1, FALLING);
  */
  second_motor.initilizeEncoders();
  second_motor.initializePWM();
  attachInterrupt(second_motor.getEncoderA(), ISR__ENCODER_JOINT2, FALLING);
  
  //third_motor.initilizeEncoders();
  //attachInterrupt(third_motor.getEncoderA(), ISR__ENCODER_JOINT3, FALLING);
  
  
};

void Crane3dof::moveMotor(unsigned char index, float deltaTime){
  switch (index)
  {
  case MOTOR1:
    first_motor.move2position(deltaTime);
    break;
  case MOTOR2:
    second_motor.move2position(deltaTime);
    break;
  case MOTOR3:
    break;
  default:
    break;
  }
}
void Crane3dof::moveMotorVel(unsigned char index, float deltaTime){
  switch (index)
  {
  case MOTOR1:
    first_motor.moveWVelocity(deltaTime);
    break;
  case MOTOR2:
    second_motor.moveWVelocity(deltaTime);
    break;
  case MOTOR3:
    third_motor.moveWVelocity(deltaTime);
  default:
    break;
  }
}
void Crane3dof::moveAllMotors(float deltaTime){
  first_motor.move2position(deltaTime);
  second_motor.move2position(deltaTime);
  if (first_motor.reach_desired_joint & second_motor.reach_desired_joint & third_motor.reach_desired_joint)
  {
    sequenceMachine.changeState();
  }
}
void Crane3dof::stopMotorMovement(unsigned char index){
  switch (index)
  {
  case MOTOR1:
    first_motor.stopMovement();
    break;
  case MOTOR2:
    second_motor.stopMovement();
    break;
  case MOTOR3:
    third_motor.stopMovement();
    break;
  default:
    break;
  }

}
void Crane3dof::motorsLimitSwitchesHandler(unsigned char index, unsigned char value){
  float lw_rev_reference;
  switch (index)
  {
  case MOTOR1:
    lw_rev_reference = (value*JOINT1_HIGH_LIMIT_HW + (1-value)*JOINT1_LOW_LIMIT_HW)/(2*PI);
    first_motor.setLimitSwitchReferencePoint(lw_rev_reference );
    break;
  case MOTOR2:
    lw_rev_reference = (value*JOINT2_HIGH_LIMIT_HW + (1-value)*JOINT2_LOW_LIMIT_HW)*REVOLUTIONS_PER_METER_2;
    second_motor.setLimitSwitchReferencePoint(lw_rev_reference);
    break;
  case MOTOR3:
    lw_rev_reference = (value*JOINT3_HIGH_LIMIT_HW + (1-value)*JOINT3_LOW_LIMIT_HW)*REVOLUTIONS_PER_METER_3;
    third_motor.setLimitSwitchReferencePoint(lw_rev_reference);
    break;
  default:
    break;
  }
}

void Crane3dof::setZeroVelocityMotors(unsigned char index)
{
  switch (index)
  {
  case MOTOR1:
    break;
  case MOTOR2:
    second_motor.zeroVelocityCase();
    break;
  case MOTOR3:
    break;
  default:
    break;
  }

}
void Crane3dof::stopAllMotors(){
  first_motor.stopMovement();
  second_motor.stopMovement();
  third_motor.stopMovement();
}


void Crane3dof::compareReferenceandCurrent(unsigned char index){

  switch (index)
  {
  case MOTOR1:
    Serial.printf("Current Joint: %i Desired Joint %i \n",first_motor.getJointCurrentVal(),first_motor.getDesiredJointVal());
  case MOTOR2:
    Serial.printf("Current Joint: %i Desired Joint %i \n",second_motor.getJointCurrentVal(),second_motor.getDesiredJointVal());
    break;
  case MOTOR3:
    Serial.printf("Current Joint: %i Desired Joint %i \n",third_motor.getJointCurrentVal(),third_motor.getDesiredJointVal());
  default:
    break;
  }
}

float Crane3dof::getXdesired(){return x_desired;}
float Crane3dof::getYdesired(){return y_desired;};
float Crane3dof::getZdesired(){return z_desired;};