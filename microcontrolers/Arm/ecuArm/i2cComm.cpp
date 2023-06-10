#include "Wire.h"
#include "i2c_config.h"
#include "i2cComm.h"
#include "Arduino.h"
#include "stateMachine.h"

#include "Crane3dof.h"
extern Crane3dof titanicCrane;
extern stateMachine sequenceMachine;
unsigned int valor=90;
void i2c_onRequest(){
  
  /*Return servo pos*/
  /*Return if esp32 is free to perform new sequence*/
  Serial.println("Data 1");

  Wire.write(titanicCrane.cameraScanningState);
  Wire.write(titanicCrane.getCameraServoPanPos());
  Wire.write(titanicCrane.getCameraServoTiltPos());
  Wire.write(titanicCrane.getCraneThetaPos());
  //Wire.write(0x4);
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
  {
    float xref=(float(readUint16data()))/(pow(2,BIT_LEN_POS_DATA)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
    float yref=(float(readUint16data()))/(pow(2,BIT_LEN_POS_DATA)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
    float zref=(float(readUint16data()))/ (pow(2,BIT_LEN_POS_DATA)-1)*(MAX_VALUE_CMD_1-MIN_VALUE_CMD_1)+MIN_VALUE_CMD_1;
    Serial.print(" X:  ");Serial.printf("%f",xref);Serial.print("  Y: ");Serial.printf("%f",yref);Serial.print(" Z: ");Serial.printf("%f\n",zref);
    tempByte = Wire.read();
    
    if(tempByte == END_COMMAND){
      titanicCrane.inverse_kinematics(xref,yref,zref);
      titanicCrane.cameraScanningState = scaning_towards;
      titanicCrane.current_state_index_camera = 0;
      sequenceMachine.changeState(arm2target);
      Serial.println("Arm moving to target position");
    } 
    break;
  }

  case START_COMMAND_SAILING:
  {
    tempByte = Wire.read();
    if(tempByte == END_COMMAND){
      /*En softly the scanning sequence*/
      titanicCrane.cameraScanningState =scaning_towards;
      titanicCrane.current_state_index_camera = 0;
      sequenceMachine.changeState(sailing);
      Serial.println("Saling to avoid colitions");
    } 
    break;  
  }
  case START_COMMAND_SCANING:
  {
    tempByte = Wire.read();
    if(tempByte == END_COMMAND){
      sequenceMachine.changeState(scaning);
      Serial.println("Starting scaning to serch for flowee");
    } 
    break;  
  }


  
  case START_COMMAND_LIMIT_SWITCHES:
  {
    unsigned int motor =1;
    unsigned int sumer = 0;
    tempByte = Wire.read();
    for (int limitSwitch=1;limitSwitch<7;limitSwitch++){
      if((tempByte & (1<<limitSwitch))>>limitSwitch)
      {
        motor = motor + sumer;
        Serial.printf("Motor %i  value: %i\n",motor,limitSwitch%2);
        titanicCrane.motorsLimitSwitchesHandler(motor,tempByte%2);
        sumer = (limitSwitch+1) %2;
        
      }
      //Serial.printf("El valor del limitSwitch %i es %u\n",limitSwitch,limit_switches_au8[limitSwitch]);
    }
    tempByte = Wire.read();
    if(tempByte == END_COMMAND){
      Serial.println("Limit Switch command receives succesfully");
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

