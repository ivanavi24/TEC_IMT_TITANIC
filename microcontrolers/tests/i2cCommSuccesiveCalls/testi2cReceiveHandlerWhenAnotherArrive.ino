
#include "Wire.h"

#define I2C_DEV_ADDR 0x55


#define START_COMMAND                   0x55 /* binary ->  0101 0101*/
#define END_COMMAND                     0xAA /* binary ->  1010 1010*/

int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
  
}
void onReceive(int len){
  
  /*Receive x,y,z values*/
  Serial.printf("Received con call %d: ", len);

  for (int i=0; i<5;i++){
    delay(1000);
    Serial.printf("Iteration: [%d] Bytes available: %d: ",i, Wire.available());
    Serial.println();
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  

}
/*Main Loop*/
void loop() {

}
