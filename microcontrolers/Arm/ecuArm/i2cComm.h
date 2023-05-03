#ifndef I2CCOMM_H
#define I2CCOMM_H
#include "Wire.h"
#include "i2c_config.h"

int readUint16data();
void i2c_onReceive(int len);
void i2c_onRequest();
void i2c_setSlave();
#endif