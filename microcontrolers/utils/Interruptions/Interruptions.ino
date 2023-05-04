struct Button {
	const uint8_t PIN;
	uint32_t numberKeyPresses;
	bool pressed;
};

Button button1 = {35, 0, false};
Button button2 = {34, 0, false};
Button button3 = {39, 0, false};
Button button4 = {36, 0, false};
Button button5 = {32, 0, false};
Button button6 = {33, 0, false};
#define PID_INTERVAL 3000
uint32_t i = 0;
long time_now=millis();
long time_last=millis();

void IRAM_ATTR isr1() {
	button1.numberKeyPresses++;

}
void IRAM_ATTR isr2() {
	button2.numberKeyPresses++;
	
}

void IRAM_ATTR isr3() {
	button3.numberKeyPresses++;

}

void IRAM_ATTR isr4() {
	button4.numberKeyPresses++;
	
}

void IRAM_ATTR isr5() {
	button5.numberKeyPresses++;
	
}

void IRAM_ATTR isr6() {
	button6.numberKeyPresses++;
	
}

void setup() {
	Serial.begin(115200);
	pinMode(button1.PIN, INPUT);//INPUT_PULLUP
  pinMode(button2.PIN, INPUT);
  pinMode(button3.PIN, INPUT);
  pinMode(button4.PIN, INPUT);
  pinMode(button5.PIN, INPUT);
  pinMode(button6.PIN, INPUT);
	attachInterrupt(button1.PIN, isr1, FALLING);
  attachInterrupt(button2.PIN, isr2, FALLING);
  attachInterrupt(button3.PIN, isr3, FALLING);
  attachInterrupt(button4.PIN, isr4, FALLING);
  attachInterrupt(button5.PIN, isr5, FALLING);
  attachInterrupt(button6.PIN, isr6, FALLING);
  
}

void loop() {

  time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    //titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();
    Serial.printf("Button 1 has been pressed %u times\n", button1.numberKeyPresses);
    Serial.printf("Button 2 has been pressed %u times\n", button2.numberKeyPresses);
    Serial.printf("Button 3 has been pressed %u times\n", button3.numberKeyPresses);
    Serial.printf("Button 4 has been pressed %u times\n", button4.numberKeyPresses);
    Serial.printf("Button 5 has been pressed %u times\n", button5.numberKeyPresses);
    Serial.printf("Button 6 has been pressed %u times\n", button6.numberKeyPresses);

  }
}