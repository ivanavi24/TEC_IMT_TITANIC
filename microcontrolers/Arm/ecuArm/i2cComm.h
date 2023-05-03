
#include "Crane3dof.h"
extern Crane3dof titanicCrane;

#ifndef I2CCOMM_H
#define I2CCOMM_H


int readUint16data();
void i2c_onReceive(int len);
void i2c_onRequest();
void i2c_setSlave();
#endif