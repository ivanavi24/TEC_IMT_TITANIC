
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
#define maxValueCommd1     (pow(2,15)-1)/1000
#define minValueCommd1     (-pow(2,15)+1)/1000
void onReceive(int len){
  
  /*Receive x,y,z values*/
  if (len==(8+1)){
    Wire.read();//Get rid of address register
    int tempByte = Wire.read();
    Serial.println(tempByte);
    if (tempByte == START_COMMAND){
      float xref=(float(readUint16data()))/(pow(2,16)-1)*(maxValueCommd1-minValueCommd1)+minValueCommd1;
      float yref=(float(readUint16data()))/(pow(2,16)-1)*(maxValueCommd1-minValueCommd1)+minValueCommd1;
      float zref=(float(readUint16data()))/ (pow(2,16)-1)*(maxValueCommd1-minValueCommd1)+minValueCommd1;
      Serial.print(" X:  ");Serial.printf("%f",xref);Serial.print("  Y: ");Serial.printf("%f",yref);Serial.print(" Z: ");Serial.printf("%f\n",zref);
      tempByte = Wire.read();
      if(tempByte == END_COMMAND){
        Serial.println("Communication success");
      }      
    }
  }
  /*Command with gripper action*/


  /*Command with just arm action*/
  Serial.printf("onReceive[%d]: ", len);
  
  while(Wire.available()){
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
 // Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  

}
/*Main Loop*/
void loop() {
  /*time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    Control action  
    titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();
  }*/
  
}
