

#include "i2cComm.h"
#include "stateMachine.h"
#include "mode_configuration.h"
#include "titanicBoat.h"
#include "ultrasonicSensor.h"
//#include "isr.h"


#define PID_INTERVAL 20
uint32_t i = 0;
long time_now=millis();
long time_last=millis();
int desiredPrintTime = 10000; //print every 5 seconds
int countTimes4DesiredTime = desiredPrintTime/ PID_INTERVAL;
int userInterfaceCounter=0;


boat rose;
ultrasonicSensor soundSystem;
stateMachine sequenceMachine;
void setup() {
  
  i2c_setSlave();
  rose.initializeVars();
  //titanicCrane.get_third_motor().initilizePINS();
  Serial.begin(115200);

}
/*Main Loop*/
void loop() {
  time_now=millis();
#if (CURRENT_OPERATION_MODE==NORMAL_FUNCTION)
  
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  

    time_last=millis();

  }


#elif (CURRENT_OPERATION_MODE==CALIBRATE_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    userInterfaceCounter++;
    if(userInterfaceCounter >= countTimes4DesiredTime){
      userInterfaceCounter=0;
    }
    time_last=millis();
  }
#elif (CURRENT_OPERATION_MODE==DISPLAY_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/
    userInterfaceCounter++;
    if(userInterfaceCounter >= countTimes4DesiredTime){
      Serial.printf("RPM : %f\n",titanicCrane.get_second_motor().getMotorRPM());
      userInterfaceCounter=0;
    }
    
    time_last=millis();

  }

#elif (CURRENT_OPERATION_MODE==DISPLAY_DISTANCES)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/
    userInterfaceCounter++;
    soundSystem.updateDistances();
    if(userInterfaceCounter >= countTimes4DesiredTime){
      Serial.printf("Left : %d Center : %d Right : %d  \n",double(soundSystem.d_left),double(soundSystem.d_center),double(soundSystem.d_right));
      userInterfaceCounter=0;
    }
    
    time_last=millis();

  }


#endif


}
