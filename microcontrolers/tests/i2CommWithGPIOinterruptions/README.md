Summary: Test to check if whether GPIO interruptions(possibily of an encoder) interfere with i2c message sequence

Description:in receive handler of esp32, a delay was added so we can manually trigger a GPIO interruption. GPIO was triggered with a button, and connected to a LED to see output. I2C messages were called multiple times to trigger different onReceive handlers while also triggering GPIO interruptions.

Observations: lengthy code could not be inside GPIO ISR, or watchDog is triggered. Example: if  Serial.println() is inside the ISR there will be errors int he Serial Terminal, since the code is to lenghty

Results: 
1) LED was able to be turned off/on even when the code execution where in the i2cHanlder. (Possibily this instruction has greater priority)

2) Multiple i2c messages were triggered and results were the same as i2cCommSuccesiveCalls. When GPIO were triggered among this calls, results were consistent with results in point 1

