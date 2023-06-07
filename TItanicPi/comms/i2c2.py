# i2ctest.py
# A brief demonstration of the Raspberry Pi I2C interface, using the Sparkfun
# Pi Wedge breakout board and a SparkFun MCP4725 breakout board:
# https://www.sparkfun.com/products/8736

import smbus
import time



def sendI2Cbyte(i2cBus,address,byte):
    i2cBus.write_byte(address,byte)
    pass

def receiveI2Cbyte():
    pass

def initializeI2Cbus(master:bool=True,channel:int=1,slaveAddress:int=0x25):
    if master:
        bus = smbus.SMBus(channel)
    time.sleep(1)
    return bus
