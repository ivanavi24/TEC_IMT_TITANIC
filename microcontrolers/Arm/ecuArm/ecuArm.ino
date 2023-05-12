#include "i2cComm.h"
#include "Crane3dof.h"
#include "mode_configuration.h"
#include "isr.h"


#define PID_INTERVAL 5000
uint32_t i = 0;
long time_now=millis();
long time_last=millis();

Crane3dof titanicCrane;
void setup() {
  Serial.begin(115200);
  i2c_setSlave();
  

}
/*Main Loop*/
void loop() {
#if (CURRENT_OPERATION_MODE==NORMAL_FUNCTION)
  time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    //titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();

  }
#elif (CURRENT_OPERATION_MODE==CALIBRATE_MOTORS)
#elif (CURRENT_OPERATION_MODE==DISPLAY_MOTORS_VEL)
if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.displayEncodersFrequency();
    time_last=millis();

  }

#endif
}


