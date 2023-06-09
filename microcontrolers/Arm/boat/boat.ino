

#include "i2cComm.h"
#include "stateMachine.h"
#include "mode_configuration.h"
#include "titanicBoat.h"
#include "ultrasonicSensor.h"
//#include "isr.h"


#define PID_INTERVAL 20
#define ULTRASONIC_TIME_PRINT 1000
uint32_t i = 0;
long time_now=millis();
long time_last=millis();
int desiredPrintTime = 10000; //print every 5 seconds
int countTimes4DesiredTime = desiredPrintTime/ PID_INTERVAL;
int userInterfaceCounter=0;

int counterBaby=0;
boat rose;
ultrasonicSensor soundSystem;
stateMachine sequenceMachine;
void setup() {
  
  i2c_setSlave();
  rose.initializeVars();
  soundSystem.initialize();
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
    
    if(userInterfaceCounter >=( ULTRASONIC_TIME_PRINT/PID_INTERVAL)){
      Serial.print("Left ");Serial.print(soundSystem.d_left);
      Serial.print(" Center ");Serial.print(soundSystem.d_center);
      Serial.print(" Right ");Serial.println(soundSystem.d_right);
      soundSystem.updateDistances();
      rose.reactiveNavigation();
      //Serial.printf("Counter value %i\n",counterBaby);
      userInterfaceCounter=0;
    }
    
    time_last=millis();

  }


#endif


}
