#include "Arduino.h"

#define NUM_TX_BYTES 5
#define NUM_RX_BYTES 17


unsigned char TX_buf[NUM_TX_BYTES] = {0x76, 0x00, 0xF0, 0x00, 0xF0};
unsigned char TX_stop_buf[NUM_TX_BYTES] = {0x76, 0x00, 0x0F, 0x00, 0x0F};
unsigned char RX_buf[NUM_RX_BYTES];

boolean ultrasonic_result;

void setup_ultrasonic()
{
	ultrasonic_result = false;
	Serial1.begin(115200);
}

void ultrasonic_sensor_read()
{
	ultrasonic_result = false;
	Serial1.write(TX_buf, NUM_TX_BYTES);
}

void serialEvent1()
{
	unsigned char z, tmp = 0;
	
	Serial1.readBytes((char *)RX_buf, NUM_RX_BYTES);
	
	if ( (RX_buf[0] == 0x76) && (RX_buf[1] == 0x00) && (ultrasonic_result == false) )
	{
		for (z = 2; z < NUM_RX_BYTES-1; z++)
			tmp += RX_buf[z];
		
		tmp = tmp & 0xFF;
		
		if (RX_buf[NUM_RX_BYTES-1] == tmp)
		{
			Serial.println("FRONT");
			
			for (z=4; z < 11; z++)
			{
				Serial.print(" F");
				Serial.print(z-4);
				Serial.print(": ");
				Serial.print(RX_buf[z]);
			}
			
			Serial.println("\nBACK");
	
			for (z=11; z < NUM_RX_BYTES-1; z++)
			{
				Serial.print(" B");
				Serial.print(z-11);
				Serial.print(": ");
				Serial.print(RX_buf[z]);
			}
		}
		
		ultrasonic_result = true;
		
		Serial1.write(TX_stop_buf, NUM_TX_BYTES);
	}
}
