#include "a_haos_linetracer.h"

#define S_DIN 42
#define S_SCLK 43
#define S_SYNCN 44
#define IN_SEN_EN 26

// Arduino Mega 2560 InfraRed sensors pin assignments ararays.
//int arduino_mega_2560_analog_pins_for_infrared[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int arduino_mega_2560_analog_pins_for_infrared[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int arduino_mega_2560_digitl_pins_for_infrared[8] = {30,31,32,33,34,35,36,37};

// The arrays to set the pin assignments to.
int SensorAnalog[8];
int SensorDigitl[8];

int dac_val_min_wav_ret[8]; 
int dac_val_max_wav_ret[8];

boolean record_analog;
boolean digital_pins_are_setup;

boolean duel_read_mode;

void initialize_pins() {
	pinMode(IN_SEN_EN,OUTPUT);
	pinMode(S_DIN,OUTPUT);
	pinMode(S_SCLK,OUTPUT);
	pinMode(S_SYNCN,OUTPUT);
	digitalWrite(S_SCLK,LOW);
	digitalWrite(S_SYNCN,HIGH);
	digitalWrite(IN_SEN_EN,HIGH);	
}

// This is used for writing to the DAC control.
void DAC_setting(unsigned int data)
{
    int z;
    
    digitalWrite(S_SCLK,HIGH);
    delayMicroseconds(1);
    
    digitalWrite(S_SCLK,LOW);
    delayMicroseconds(1);
   
    digitalWrite(S_SYNCN,LOW);
    delayMicroseconds(1);
    
    for(z = 15; z >= 0; z--)
    {
        digitalWrite(S_DIN,(data>>z)&0x1);
        
        digitalWrite(S_SCLK,HIGH);
        delayMicroseconds(1);
        
        digitalWrite(S_SCLK,LOW);
        delayMicroseconds(1);
    }
    digitalWrite(S_SYNCN, HIGH);
}

void DAC_CH_Write(unsigned int ch, unsigned int da)
{
    unsigned int data = ((ch << 12) & 0x7000) | ((da << 4) & 0x0FF0);
    DAC_setting(data);
}

void send_signal_for_write_through_mode_to_ir_sensor_package()
{
	DAC_setting(0x9000);
}


int num_request_recieved = 0;

void toggle_record_analog_to_record_digital_and_visa_versa()
{
	record_analog = !record_analog;	
}


void write_sensor_to_array(int infra_sensor_read_request_array[8])
{
	num_request_recieved++;
	//Serial.println("#" + String(num_request_recieved) + " Read Request Recieved!");
	//Serial.print("Recording : ");
	if (record_analog)
	{
		//Serial.println("Analogue.");
	}
	else
	{
		//Serial.println("Digital.");
	}
	
	int analog_val=0;
	int digitl_val=0;	

	for(int z = 7; z >= 0; z--)
	{
		analog_val = analogRead(arduino_mega_2560_analog_pins_for_infrared[z]);
		if (digital_pins_are_setup)
		{
			digitl_val = digitalRead(arduino_mega_2560_digitl_pins_for_infrared[z]);
		}
		else
		{
			digitl_val = analogRead(arduino_mega_2560_digitl_pins_for_infrared[z]);
		}
		
		//Serial.print("[A:" + String(analog_val) +"], ");
		//Serial.println("[D:" + String(digitl_val) +"] ");
		
		if (record_analog)
		{
			infra_sensor_read_request_array[z] = analog_val;
		}
		else
		{
			infra_sensor_read_request_array[z] = digitl_val;
		}
	}
}

void initilize_dac_min_max_arrays()
{
	for (int z = 0; z < 8; z++)
	{
		dac_val_min_wav_ret[z] = 250;	// MIN LITE-WAVE RETURNED (BLACK)
		dac_val_max_wav_ret[z] = 750;	// MAX LITE-WAVE RETURNED (WHITE)
	}
}
void initialize_infrared_sensor_pack()
{
	initialize_pins();	
	initilize_dac_min_max_arrays();
	record_analog = true;
	digital_pins_are_setup = false;
}


void setup_ports_to_recieve_digital_output_from_sensors()
{
	for (int z = 0; z < 8; z++)
	{
	    pinMode(arduino_mega_2560_digitl_pins_for_infrared[z], INPUT);
	}
}	

void set_digital_cutoff_point_values()
{
	
	int cutoffs_values[]={0, 0, 0, 0, 0, 0, 0, 0};
    
    send_signal_for_write_through_mode_to_ir_sensor_package();
    
    for (int z = 0; z < 8; z++)
    {
        int mean_val = (dac_val_min_wav_ret[z] + dac_val_max_wav_ret[z])/2; //10-bit
        cutoffs_values[z]=mean_val;
        DAC_CH_Write(z, mean_val >> 2); //should be 8-bit
    }
    for (int z = 0; z < 8; z++)
    {
    	//Serial.println("[CU" + String(z) + ":" + String(cutoffs_values[z])  + "]");
    }
    
    digital_pins_are_setup = true;
}

void config_digital_infrared_sensors()
{
	setup_ports_to_recieve_digital_output_from_sensors();
	//send_signal_for_write_through_mode_to_ir_sensor_package();


	set_digital_cutoff_point_values();
	record_analog = false;
}

int sensor_values[8];

void infrared_sensor_read()
{
    write_sensor_to_array(sensor_values);	
}

void infrared_sensor_read(int array_to_read_values_into[8])
{
    write_sensor_to_array(array_to_read_values_into);	
}

void serial_print_array_values()
{
	//Serial.println("[Sen][Wht][Blk]");
	for(int z = 7; z >= 0; z--)
	{
		 


		 //Serial.print("["+ String(sensor_values[z]) +"], ");
		 //Serial.print("["+ String(dac_val_max_wav_ret[z]) +"], ");
		 //Serial.print("["+ String(dac_val_min_wav_ret[z]) +"]");
		 
		 ////Serial.print("["+ String(arduino_mega_2560_analog_pins_for_infrared[z]) +"]");
		 ////Serial.print("["+ String(arduino_mega_2560_digitl_pins_for_infrared[z]) +"]");
		 ////Serial.print("["+ String(SensorAnalog[z]) +"]");
		 ////Serial.print("["+ String(SensorDigitl[z]) +"]");
		 //Serial.println("");
	
	}
}

void set_sensor_reads_for_white_values()
{
	record_analog=true;
	write_sensor_to_array(dac_val_max_wav_ret);			
}

void set_sensor_reads_for_black_values()
{
	record_analog=true;
	write_sensor_to_array(dac_val_min_wav_ret);			
}



