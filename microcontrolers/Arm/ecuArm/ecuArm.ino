#include "i2cComm.h"
#include "Crane3dof.h"


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
  time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    //titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();

  }

}
