#include "Arduino.h"

#define FRONT_LED_PIN 10
#define REAR_LED_PIN  9


void setup_led_pins()
{
	pinMode(FRONT_LED_PIN, OUTPUT);
	pinMode(REAR_LED_PIN, OUTPUT);

	digitalWrite(FRONT_LED_PIN, LOW);
	digitalWrite(REAR_LED_PIN, LOW);
}


void front_led_control(bool value)
{
	if (value)
	{
		digitalWrite(FRONT_LED_PIN, HIGH);
	}
	else
	{
		digitalWrite(FRONT_LED_PIN, LOW);
	}
}

void rear_led_control(bool value)
{
	if (value)
	{
		digitalWrite(REAR_LED_PIN,  HIGH);
	}
	else
	{
		digitalWrite(REAR_LED_PIN, LOW);
	}
}

void signal_error_with_led()
{
	for (int i = 0; i < 5; i++)
	{

		front_led_control(true);
		rear_led_control(false);
		delay(100);
		front_led_control(false);
		rear_led_control(true);
		delay(100);
	}

}