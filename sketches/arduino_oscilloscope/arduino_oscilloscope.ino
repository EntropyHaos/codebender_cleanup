/*
https://codebender.cc/sketch:404147
https://www.google.com/search?sourceid=chrome-psyapi2&rlz=1C1CHBF_enUS704US704&ion=1&espv=2&ie=UTF-8&q=arduino%20oscilloscope&oq=arduino%20oscilloscope&aqs=chrome.0.69i59j0l2j69i65j0l2.2406j0j8
http://www.instructables.com/id/Arduino-Oscilloscope-poor-mans-Oscilloscope/
https://www.google.com/search?sourceid=chrome-psyapi2&rlz=1C1CHBF_enUS704US704&ion=1&espv=2&ie=UTF-8&q=processing%20console%20log&oq=processing%20consol&aqs=chrome.2.0j69i57j0l4.11111j0j8
https://processing.org/reference/print_.html


*/


#define ANALOG_IN 0
#define BUTTON_ONE_PIN 2
#define BUTTON_TWO_PIN 3

int button_one_state = 0;
int button_two_state = 0;

bool signal_on = false;

void setup() {
  Serial.begin(9600); 
  //Serial.begin(115200);
  
  pinMode(BUTTON_ONE_PIN, INPUT);
  pinMode(BUTTON_TWO_PIN, INPUT);
}

void send_signal()
{
  int val = analogRead(ANALOG_IN);                                              
  Serial.write( 0xff );                                                         
  Serial.write( (val >> 8) & 0xff );                                            
  Serial.write( val & 0xff );
	
}

void loop()
{
	if (digitalRead(BUTTON_TWO_PIN) == HIGH && !(signal_on))
	{
		while (digitalRead(BUTTON_TWO_PIN) == HIGH)
		{
			signal_on = true;
		}
	}
	
	if (digitalRead(BUTTON_TWO_PIN) == HIGH && (signal_on))
	{
		while (digitalRead(BUTTON_TWO_PIN) == HIGH)
		{
			signal_on = false;
		}
	}

	if (signal_on == false && button_one_state != digitalRead(BUTTON_ONE_PIN))
	{
		button_one_state = digitalRead(BUTTON_ONE_PIN);
		if (button_one_state == HIGH)
		{
			send_signal();
		}
	}
	if (signal_on)
	{
		send_signal();
	}
}