#include "Arduino.h"
#include "haos_smartcar_buzzer.h"

#define BUZZER_PIN 45

int melody[] = {NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};
int noteDurations[] = {4, 8, 8, 4,4,4,4,4 };

boolean diable_buzzer;

void setup_buzzer_pin()
{
	pinMode(BUZZER_PIN, OUTPUT);
}

void buzzer_on()
{
	digitalWrite(BUZZER_PIN, HIGH);
}

void buzzer_off()
{
	digitalWrite(BUZZER_PIN, LOW);
}

void buzz(int ms_to_buzz)
{
	buzzer_on();
	delay(ms_to_buzz);
	buzzer_off();
}

void turn_buzzer_on()
{
	buzzer_on();
}

void turn_buzzer_off()
{
	buzzer_off();
}

void buzz_error()
{
	buzz(500);	
}

void buzz_ready()
{
	buzz(100);	
}

void buzz_signal_recieved()
{
	buzz(50);
	delay(50);
	buzz(50);
}

void buzz_a_test()
{
	buzz(1);
	delay(1000);
	buzz(5);	
	delay(1000);
	buzz(10);
}

// Source : https://www.google.com/search?sourceid=chrome-psyapi2&ion=1&espv=2&ie=UTF-8&q=Arduino%20Mega%202560%20buzzer%20tones&oq=Arduino%20Mega%202560%20buzzer%20tones&aqs=chrome..69i57.9194j0j8

void buzz_a_scale_sweep()
{
	//tone(pin, frequency, duration)
	tone(BUZZER_PIN, NOTE_C6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_D6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_E6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_F6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_G6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_A6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_B6, 5000);
	delay(500);
	tone(BUZZER_PIN, NOTE_C7, 5000);
	delay(500);
}

// Source : https://www.google.com/search?sourceid=chrome-psyapi2&ion=1&espv=2&ie=UTF-8&q=Arduino%20Mega%202560%20buzzer%20tones&oq=Arduino%20Mega%202560%20buzzer%20tones&aqs=chrome..69i57.9194j0j8

void play_melody() {

  // iterate over the notes of the melody:

  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate note duration, take one second.divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.

    int noteDuration = 1000/noteDurations[thisNote];

    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note’s duration + 30% seems to work well:

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // stop the tone playing:
    noTone(8);
  }
}

