Summary: Test to check if onReceivei2c handler could be interrupted by other i2c receive handlers

Description:in receive handler of esp32, wire.available() was used to check the available bytes ofg i2c bufffer.Delay was addes inside this function to hive time another decive to sent new iec messages meanwhile. i2c messages were sent by the raspberry pi.

Results: 
1) onReceivei2c handler of esp32 was not interrupted by subsequent i2c calls. Instead, succesive i2c messages were enqueued and attended in respective order

2) wire.available() always return the same value inside onReceivei2c handler, meaning that variables and data inside this scope function has its own memory and static. New messages arrive like described in one

3) Even though data is not read with the Wire.read() instruction, the buffer is empty when a new call to handler is performed