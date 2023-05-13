#include "i2cComm.h"
#include "Crane3dof.h"
#include "mode_configuration.h"
#include "isr.h"


#define PID_INTERVAL 2000
uint32_t i = 0;
long time_now=millis();
long time_last=millis();
int desiredPrintTime = 10000; //print every 5 seconds
int countTimes = desiredPrintTime/ PID_INTERVAL;
int counter=0;
Crane3dof titanicCrane;
void setup() {
  
  i2c_setSlave();
  titanicCrane.initializeVars();
  pinMode(27,OUTPUT);
  ledcSetup(0, 30000, 8);
  ledcAttachPin(27, 0);
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
    titanicCrane.get_first_motor().move2position(time_now-time_last );
    
    
    counter++;
    if(counter >= countTimes){
      titanicCrane.get_first_motor().stopMovement();
      counter=0;
      titanicCrane.adjustMotorGains();
      titanicCrane.setTargetAngle();
      
      Serial.printf("Current Joint: %i Desired Joint %i \n",titanicCrane.get_first_motor().getJointCurrentVal(),titanicCrane.get_first_motor().getDesiredJointVal());
      titanicCrane.get_first_motor().move2position(time_now - time_last);
      Serial.printf("PWM: %i\n",titanicCrane.get_first_motor().controlActionval);
      
    }
    time_last=millis();
  }
#elif (CURRENT_OPERATION_MODE==CALIBRATE_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.displayEncodersFrequency();
    titanicCrane.get_first_motor().moveWVelocity(time_now-time_last);
    time_last=millis();
    counter++;
    if(counter >= countTimes){
      counter=0;
      titanicCrane.setTargetRPM();
    }

  }
#elif (CURRENT_OPERATION_MODE==DISPLAY_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.displayEncodersFrequency();
    time_last=millis();

  }

#endif
}
