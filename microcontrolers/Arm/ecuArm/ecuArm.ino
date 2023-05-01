#include "Wire.h"
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"
#include  "i2c_config.h"
#include "motorsWencoder.h"


/* Kinematic variable definition*/

#define origin2waterDistance            30.4







#define PI                             3.14159265

#define PID_INTERVAL 5
uint32_t i = 0;
long time_now=millis();
long time_last=millis();



class manipulator3dof{
  private:

    float origin2water=origin2waterDistance;
    DCmotor_Encoder first_motor=DCmotor_Encoder(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT1_LOW_LIMIT_HW,JOINT1_HIGH_LIMIT_HW,JOINT1_LOW_LIMIT_SW,JOINT1_HIGH_LIMIT_SW);
    DCmotor_Encoder second_motor= DCmotor_Encoder(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT2_LOW_LIMIT_HW,JOINT2_HIGH_LIMIT_HW,JOINT2_LOW_LIMIT_SW,JOINT2_HIGH_LIMIT_SW);
    DCmotor_Encoder third_motor= DCmotor_Encoder(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT3_LOW_LIMIT_HW,JOINT3_HIGH_LIMIT_HW,JOINT3_LOW_LIMIT_SW,JOINT3_HIGH_LIMIT_SW);


    float x_desired, y_desired, z_desired; /*Set thorugh i2c communication intereruption*/
    
    /*Encoder resolutions in pulses per revolution*/
    unsigned int joint1_encoder_resolution = JOINT1_ENCODER_RESOLUTION;
    unsigned int joint2_encoder_resolution = JOINT2_ENCODER_RESOLUTION;
    unsigned int joint3_encoder_resolution = JOINT1_ENCODER_RESOLUTION;

   /**/
    float theta,radius,z_height;

  public:
    manipulator3dof (){
      
    }
    /*Assumes corrdinate frame located in the center of the robotic arm*/
    void inverse_kinematics(float x, float y, float z){
      theta =atan2(x,y);
      radius =sqrt(pow(x,2)+pow(y,2));
      z_height = z + origin2water; 
    }

    void reachPosition(float deltaTime){
      
      first_motor.move2position(deltaTime);
      second_motor.move2position(deltaTime);
      third_motor.move2position(deltaTime);
      
    }
  /*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
  void setTargetJoints(){
    int pulsesJoint1 =(theta - ZERO_POS_1)* joint1_encoder_resolution/ (2* PI);   /*  pulses per revolution [pulses/rev]*  */ 
    int pulsesJoint2 =(radius - ZERO_POS_2)* joint2_encoder_resolution;   /*  pulses per revolution [pulses/rev]*  */ 
    int pulsesJoint3 =(z_height - ZERO_POS_3)* joint3_encoder_resolution;   /*  pulses per revolution [pulses/rev]*  */ 
    first_motor.setJointDesired(pulsesJoint1);
    second_motor.setJointDesired(pulsesJoint2);
    third_motor.setJointDesired(pulsesJoint3);
  }
    
};
manipulator3dof titanicCrane;
void onRequest(){
  
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}
int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
}
void onReceive(int len){
  /*Receive x,y,z values*/
  if (len==8){
    int tempByte = Wire.read();
    if (tempByte == START_COMMAND){
      float xref=(float(readUint16data())- pow(2,15))/1000;
      float yref=(float(readUint16data())- pow(2,15))/1000;
      float zref=(float(readUint16data())- pow(2,15))/1000;
      tempByte = Wire.read();
      if(tempByte == END_COMMAND){
        titanicCrane.inverse_kinematics( xref,  yref,  zref);
        titanicCrane.setTargetJoints(); 
      }      
    }
    else{
      while(Wire.available()){
        Wire.read();
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
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  

}
/*Main Loop*/
void loop() {
  time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();
  }

}
