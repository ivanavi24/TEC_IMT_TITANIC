

#include "i2cComm.h"
#include "Crane3dof.h"
#include "stateMachine.h"
#include "mode_configuration.h"
//#include "isr.h"


#define PID_INTERVAL 20
uint32_t i = 0;
long time_now=millis();
long time_last=millis();
int desiredPrintTime = 10000; //print every 5 seconds
int countTimes4DesiredTime = desiredPrintTime/ PID_INTERVAL;
int userInterfaceCounter=0;
Crane3dof titanicCrane;
stateMachine sequenceMachine;
void setup() {
  
  i2c_setSlave();
  //titanicCrane.initializeVars();
  titanicCrane.get_third_motor().initilizePINS();
  Serial.begin(115200);

}
/*Main Loop*/
void loop() {
  time_now=millis();
#if (CURRENT_OPERATION_MODE==NORMAL_FUNCTION)
  
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    //titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();

  }
#elif (CURRENT_OPERATION_MODE==CALIBRATE_MOTORS_POS)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.moveMotor(CURRENT_SELECTED_MOTOR,time_now-time_last);
    userInterfaceCounter++;
    if(userInterfaceCounter >= countTimes4DesiredTime){
      Serial.printf("Current Joint: %i Desired Joint %i \n",titanicCrane.get_third_motor().getJointCurrentVal(),titanicCrane.get_third_motor().getDesiredJointVal());
      titanicCrane.stopMotorMovement(CURRENT_SELECTED_MOTOR);
      titanicCrane.adjustMotorGains(CURRENT_SELECTED_MOTOR,false);
      titanicCrane.setTargetAngle(CURRENT_SELECTED_MOTOR);
      
      userInterfaceCounter=0;
      
      
    }
    time_last=millis();
}
#elif (CURRENT_OPERATION_MODE==REACH_GIVEN_POSITION_SERIAL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.moveAllMotors(time_now-time_last);
    userInterfaceCounter++;
    if(userInterfaceCounter >= countTimes4DesiredTime){
      titanicCrane.stopAllMotors();
      titanicCrane.set_target_position(0,0,0);//Dummy vars
      userInterfaceCounter=0;
      
      
    }
    
    time_last=millis();
}

#elif (CURRENT_OPERATION_MODE==CALIBRATE_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.moveMotorVel(CURRENT_SELECTED_MOTOR, time_now-time_last);
    //Serial.printf("RPM : %f Desired: %f\n",titanicCrane.get_second_motor().getMotorRPM(),titanicCrane.get_second_motor().getVelocityDesired());
    userInterfaceCounter++;
    if(userInterfaceCounter >= countTimes4DesiredTime){
      titanicCrane.adjustMotorGains(CURRENT_SELECTED_MOTOR,true);
      titanicCrane.setTargetRPM(CURRENT_SELECTED_MOTOR);
      userInterfaceCounter=0;
    }
    time_last=millis();
  }
#elif (CURRENT_OPERATION_MODE==DISPLAY_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/
    userInterfaceCounter++;
    if(userInterfaceCounter >= countTimes4DesiredTime){
      titanicCrane.displayEncodersFrequency(CURRENT_SELECTED_MOTOR);
      Serial.printf("RPM : %f\n",titanicCrane.get_second_motor().getMotorRPM());
      userInterfaceCounter=0;
    }
    
    time_last=millis();

  }

#endif
}
