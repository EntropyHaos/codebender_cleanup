void setup() {
 //Connection speed must be same as app.
 Serial.begin(57600);
}

void loop() {
	involtSend(0, analogRead(A0));
	delay(2);
}

/*
  INVOLT FUNCTIONS
  ------------------------------------------------------
  involtSend - send int to app.
  ------------------------------------------------------
*/

void involtSend(int pinNumber, int sendValue){
  Serial.print('A'); 
  Serial.print(pinNumber); 
  Serial.print('V'); 
  Serial.print(sendValue); 
  Serial.println('E');
  Serial.flush();
};
