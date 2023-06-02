#include "Wire.h"
#include "i2c_config.h"
#include "i2cComm.h"
#include "Arduino.h"

#include "Crane3dof.h"
extern Crane3dof titanicCrane;

unsigned int valor=90;
void i2c_onRequest(){
  
  /*Wire.write(valor++);
  Wire.write(valor++);
  Wire.write(valor++);
  Wire.write(valor++);
  //Wire.print(" Packets.");*/
  Serial.println("onRequest");
}
int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
}
void i2c_onReceive(int len){

  int tempByte = Wire.read(); //read one extra due to format
  tempByte = Wire.read();
  switch (tempByte)
  {
  case START_COMMAND_WORLD_POS:

    float xref=(float(readUint16data()))/(pow(2,BIT_LEN_POS_DATA)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
    float yref=(float(readUint16data()))/(pow(2,BIT_LEN_POS_DATA)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
    float zref=(float(readUint16data()))/ (pow(2,BIT_LEN_POS_DATA)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
    Serial.print(" X:  ");Serial.printf("%f",xref);Serial.print("  Y: ");Serial.printf("%f",yref);Serial.print(" Z: ");Serial.printf("%f\n",zref);
    float pwmArr[3]={xref,yref,zref};
    titanicCrane.moveMotors(pwmArr, MIN_VALUE_CMD_1,MAX_VALUE_CMD_1, BIT_SIZE_FORMAT);
    tempByte = Wire.read();
    if(tempByte == END_COMMAND_WORLD_POS){
      Serial.println("Communication success");
    } 
    break;
  }

    
  case START_COMMAND_LIMIT_SWITCHES:
  {
    unsigned char limit_switches_au8[8]; 
    tempByte = Wire.read();
    for (int limitSwitch=0;limitSwitch<8;limitSwitch++){
      limit_switches_au8[limitSwitch] = (tempByte & (1<<limitSwitch))>>limitSwitch;  
      Serial.printf("El valor del limitSwitch %i es %u\n",limitSwitch,limit_switches_au8[limitSwitch]);
    }
    tempByte = Wire.read();
    for (int limitSwitch=0;limitSwitch<8;limitSwitch++){
      limit_switches_au8[limitSwitch] = (tempByte && (1<<limitSwitch))>>limitSwitch;  
    }
    if(tempByte == END_COMMAND_LIMIT_SWITCHES){
      Serial.println("Communication success");
    } 
    break;  
  }
    
  default:
    break;
  }
  /*Receive x,y,z values*/

}
void i2c_setSlave(){
  //Serial.setDebugOutput(true);
  Wire.onReceive(i2c_onReceive);
  Wire.onRequest(i2c_onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

