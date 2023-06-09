
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
  while (Serial.available()==0)  {
  }
  angle = Serial.parseFloat();
  Serial.println(angle);  
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
void Crane3dof::inverse_kinematics(float x, float y, float z){
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
    restrict2Workspace(thetaWorld, radiusWorld, z_world, 1);
  }
  restrict2Workspace(thetaWorld, radiusWorld, z_world, 0);
}
void Crane3dof::restrict2Workspace(float thetaW, float radiusW, float zW,bool craneMecanismFlag){
  bool radiusFlag = false;
  bool thetaFlag = false;
  if( (radiusW>RADIUS_MIN) && (radiusW<RADIUS_MAX)){radiusFlag = true;}
  if( (thetaW>THETA_MIN) && (thetaW<THETA_MAX)){radiusFlag = true;}
  if (radiusFlag && thetaFlag){
    theta = thetaW;
    radius = radiusW;
    z_height = zW;

  }
  
}
void Crane3dof::reachPosition(float deltaTime){
  
  first_motor.move2position(deltaTime);
  second_motor.move2position(deltaTime);
  third_motor.move2position(deltaTime);
  
}
/*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
void Crane3dof::setTargetJoints(){
  float pulsesJoint1 =(theta - ZERO_POS_1)/ (2* PI);   /*  [radians]/[radians] = revolutions */ 
  float pulsesJoint2 =(radius - ZERO_POS_2)*REVOLUTIONS_PER_METER_2;   /*  [meters]*[rev/meter] = revolutions  */ 
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
  third_motor.moveMotor(dCycle); 
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
  default:
    break;
  }
  
}
void Crane3dof::jointExtremePosition(unsigned char index,unsigned char value){
  switch (index)
  {
  case 1:
    first_motor.setReferencePoint(value);
    break;
  case 2:
    second_motor.setReferencePoint(value);
    break;
  case 3:
    //third_motor.setReferencePoint(value);
    break;
  default:
    break;
  }
}

void Crane3dof::adjustMotorGains(unsigned char index){
  float kp,kd,ki;
  Serial.printf("Motor%u kp: ",index);      //Prompt User for input
  while (Serial.available()==0)  {
  }
  kp = Serial.parseFloat();
  Serial.printf("%f\n",kp);
  Serial.printf("Motor%u kd: ",index); 
  while (Serial.available()==0)  {
  }
  kd = Serial.parseFloat();  
  Serial.printf("%f\n",kd);
  Serial.printf("Motor%u ki: ",index); 
  while (Serial.available()==0)  {
  }
  ki = Serial.parseFloat();
  Serial.printf("%f\n",ki);
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
void Crane3dof::displayEncodersFrequency(unsigned char index){
  Serial.printf("Motor 1 frequency: %f\n",first_motor.getMotorFrequency());
  Serial.printf("Motor 2 frequency: %f\n",first_motor.getMotorRPM());
  Serial.printf("Motor 3 frequency: %f\n",third_motor.getMotorFrequency());
  Serial.println("");
}
void Crane3dof::printMotorGains(unsigned char index){
  first_motor.displayGainValues();
  second_motor.displayGainValues();
}
void Crane3dof::initializeVars(){

  first_motor.initilizeEncoders();
  first_motor.initializePWM();
  first_motor.stopMovement();
  attachInterrupt(first_motor.getEncoderA(), ISR__ENCODER_JOINT1, FALLING);

  second_motor.initilizeEncoders();
  second_motor.initializePWM();
  attachInterrupt(second_motor.getEncoderA(), ISR__ENCODER_JOINT2, FALLING);
  
  //third_motor.initilizeEncoders();
  //attachInterrupt(third_motor.getEncoderA(), ISR__ENCODER_JOINT3, FALLING);
  
  
};

