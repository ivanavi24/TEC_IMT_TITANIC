#include "Wire.h"
#include "i2c_config.h"
#include "i2cComm.h"
#include "Arduino.h"



void i2c_onRequest(){
  
  /*Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");*/
}
int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
}
void i2c_onReceive(int len){
  
  /*Receive x,y,z values*/
  if (len==(ARM_COMMAND_LENGTH+EXTRA_MSG_OFFSET)){
    Wire.read();//Get rid of address register
    int tempByte = Wire.read();
    Serial.println(tempByte);
    if (tempByte == START_COMMAND){
      float xref=(float(readUint16data()))/(pow(2,16)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
      float yref=(float(readUint16data()))/(pow(2,16)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
      float zref=(float(readUint16data()))/ (pow(2,16)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
      Serial.print(" X:  ");Serial.printf("%f",xref);Serial.print("  Y: ");Serial.printf("%f",yref);Serial.print(" Z: ");Serial.printf("%f\n",zref);
      float pwmArr[3]={xref,yref,zref};
      titanicCrane.moveMotors(pwmArr, MIN_VALUE_CMD_1,MAX_VALUE_CMD_1, 16);
      tempByte = Wire.read();
      if(tempByte == END_COMMAND){
        Serial.println("Communication success");
      }      
    }
  }
 
}
void i2c_setSlave(){
  //Serial.setDebugOutput(true);
  Wire.onReceive(i2c_onReceive);
  Wire.onRequest(i2c_onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

