cmd 
#include "Wire.h"
#include "i2cComm.h"


void setup() {
  Serial.begin(115200);
  i2c_setSlave();
  

}
/*Main Loop*/
void loop() {
  
}
