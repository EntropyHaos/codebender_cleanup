
 /*
 * Started with public domain code found at : http://playground.arduino.cc/Learning/MFRC522
 *
 * Modified by Benjamin Haos for easier readability and duel NFC reader.
 *
 * Pin layout should be as follows:
 * NFC			Arduino		Wire
 * Reader		Uno			Color
 * Pin			Pin
 * ------------------------------------------------------------
 * 			[NFC One]
 * RST			9			White
 * SDA			10			Grey

 * 			[NFC Two]
 * RST			5			White
 * SDA			6			Grey

 * 			[Both NFC]
 * MOSI			11			Purple
 * MISO			12			Blue
 * SCK			13			Green
 *
 * 			[Arduino Uno]
 *
 * GREEN LED	8			Orange
 * YELLOW LED	2			Yellow
 * RED LED		3			Red
 *
 * Speaker +	7			Red     
 * Speaker +	G			Black     
 *
 * Piezo Sns+	A0			See : http://www.ttiinc.com/object/te-connectivity-sensor-piezo-dt-series
 * Piezo Sns-   G
 */


#include <SPI.h>
#include <MFRC522.h>

#include "pitches.h"

int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4 };
#define speaker_pin 7
void play_song()
{

// iterate over the notes of the melody:
	for (int thisNote = 0; thisNote < 8; thisNote++)
	{

// to calculate the note duration, take one second
// divided by the note type.
//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(speaker_pin, melody[thisNote], noteDuration);

// to distinguish the notes, set a minimum time between them.
// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
// stop the tone playing:
		noTone(speaker_pin);
	}
	
}


#define RST_PIN_1 9
#define SS_PIN_1 10
MFRC522 mfrc522_1(SS_PIN_1, RST_PIN_1);	// Create MFRC522 instance 1.

#define RST_PIN_2 5
#define SS_PIN_2 6
MFRC522 mfrc522_2(SS_PIN_2, RST_PIN_2);	// Create MFRC522 instance 2.

const int PIEZO_PIN = A0; // Piezo output

int red_signal_led = 3;
int ylw_signal_led = 2;
int grn_signal_led = 8;

int relay_signal_pin = 4;
int hello_delay = 500;

int count_times_read = 0;
int max_count_before_reset = 100;

float manually_set_threshold_to_activate = 0.40;

void say_hello()
{
	digitalWrite(red_signal_led, LOW);
	delay(hello_delay);
	digitalWrite(red_signal_led, HIGH);
	digitalWrite(ylw_signal_led, LOW);
	delay(hello_delay);
	digitalWrite(ylw_signal_led, HIGH);
	digitalWrite(grn_signal_led, LOW);
	delay(hello_delay);
	digitalWrite(grn_signal_led, HIGH);
}

void setup_signal_led_set()
{
	pinMode(red_signal_led, OUTPUT);
	digitalWrite(red_signal_led, HIGH);
	pinMode(ylw_signal_led, OUTPUT);
	digitalWrite(ylw_signal_led, HIGH);
	pinMode(grn_signal_led, OUTPUT);
	digitalWrite(grn_signal_led, HIGH);
}

void turn_off_all_led()
{
	digitalWrite(red_signal_led, HIGH);
	digitalWrite(ylw_signal_led, HIGH);
	digitalWrite(grn_signal_led, HIGH);
}

void signal_red()
{
	turn_off_all_led();
	digitalWrite(red_signal_led, LOW);
}

void signal_yellow()
{
	turn_off_all_led();
	digitalWrite(ylw_signal_led, LOW);
}

void signal_green()
{
	turn_off_all_led();
	digitalWrite(grn_signal_led, LOW);
}


void setup_relay()
{
	pinMode(relay_signal_pin, OUTPUT);
	digitalWrite(relay_signal_pin, HIGH);
}

void switch_relay(boolean on_if_true)
{
	if (on_if_true)
	{
		// Turn on relay.
		digitalWrite(relay_signal_pin, LOW);
	}
	else
	{
		// Turn off relay.
		digitalWrite(relay_signal_pin, HIGH);
	}
}
/*
*/

void cycle_lights()
{
	for (int i = 0; i < 10; i++)
	{
		signal_red();
		delay(100);
		signal_yellow();
		delay(100);
		signal_green();
		delay(100);
	}

}

bool keys_are_present = false;

bool wallet_is_present = false;

String key_fob_serial = "116251252244";
String wallet_card_serial = "013091220001";



void setup_nfc()
{
	SPI.begin();			// Init SPI bus
	mfrc522_1.PCD_Init();	// Init NFC card reader one.
	mfrc522_2.PCD_Init();	// Init NFC card reader two.

}

void test_values(String value_one, String value_two)
{

	if ((value_one == key_fob_serial && value_two == wallet_card_serial) || (value_two == key_fob_serial && value_one == wallet_card_serial))
	{
		keys_are_present = true;
		wallet_is_present = true;
	}
	else if ((value_one == key_fob_serial || value_two == key_fob_serial) && (value_one != wallet_card_serial && value_two == wallet_card_serial))
	{
		keys_are_present = true;
		wallet_is_present = false;
	}
	else if ((value_one == wallet_card_serial || value_two == wallet_card_serial)  && (value_one != key_fob_serial && value_two == key_fob_serial))
	{
		wallet_is_present = true;
		keys_are_present = false;
	}
	else
	{
		wallet_is_present = false;
		keys_are_present = false;

	}
}

int num_times_to_cycle_nfc = 10;

void do_activity_done_when_device_senses_activity_be_done_has_been_requested()
{
	cycle_lights();
	String found_serial_one = "";
	String found_serial_two = "";

	mfrc522_1.PCD_Init();	// Init NFC card reader one.
	mfrc522_2.PCD_Init();	// Init NFC card reader two.


	for (int i = 0; i < num_times_to_cycle_nfc; i++ )
	{
		if ( mfrc522_1.PICC_IsNewCardPresent()  )
		{
			Serial.print(" - reader 1 noticed new card, value = ");
			if( mfrc522_1.PICC_ReadCardSerial() )
			{
				mfrc522_2.PICC_HaltA();
				found_serial_one = getCardValue(mfrc522_1.uid.uidByte, mfrc522_1.uid.size);
				//mfrc522_1.PICC_HaltA();
			}
			Serial.println("" + String(found_serial_one)); 
			//return;
		}

		if ( mfrc522_2.PICC_IsNewCardPresent() )
		{
			Serial.print(" - reader 2 noticed new card, value = ");
			if( mfrc522_2.PICC_ReadCardSerial() )
			{
				mfrc522_1.PICC_HaltA();
				found_serial_two = getCardValue(mfrc522_2.uid.uidByte, mfrc522_2.uid.size);
				//mfrc522_2.PICC_HaltA();
			}
			Serial.println("" + String(found_serial_two)); 
			//return;
		}
	}

	test_values(found_serial_one, found_serial_two);
}

String getCardValue(byte *buffer, byte bufferSize)
{

	String return_String = "";
	for (byte i = 0; i < bufferSize; i++)
	{
		if (buffer[i] < 100)
		{
			return_String += "0";
		}
		if (buffer[i] < 10)
		{
			return_String += "0";
		}

		return_String += String(buffer[i]);
	}
	return return_String;
}


void setup()
{
	setup_signal_led_set();
	say_hello();
	setup_nfc();
	Serial.begin(9600);
	Serial.println("Setep complete...");
	setup_relay();
	
	//play_song();
	run_check_on_nfc();
}


void run_check_on_nfc()
{
	signal_yellow();
	do_activity_done_when_device_senses_activity_be_done_has_been_requested();
}

void loop()
{
	// Read Piezo ADC value in, and convert it to a voltage

	int piezoADC = analogRead(PIEZO_PIN);
	float piezoV = piezoADC / 1023.0 * 5.0;

	if (piezoV > manually_set_threshold_to_activate)
	{
		run_check_on_nfc();
		//signal_yellow();
		//do_activity_done_when_device_senses_activity_be_done_has_been_requested();
	}
	else
	{
		//Serial.println(piezoV); // Print the voltage.
		if (keys_are_present && wallet_is_present)
		{
			signal_green();
			switch_relay(true);
		}
		else
		{
			signal_red();
			switch_relay(false);
		}
	}
}




