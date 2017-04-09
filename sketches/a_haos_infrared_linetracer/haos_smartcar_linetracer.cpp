#include "Arduino.h"
#include "haos_smartcar_movement.h"
#include "haos_smartcar_infrared.h"

int main_loop_delay = 5; // = ms between chechs in the IR.

boolean move_wheels_for_reals = false;
boolean line_tracing;
boolean smartcar_is_still_learning_and_has_lost_his_way;


void move_car_for_line_tracing(int left_speed, int right_speed)
{
	if (move_wheels_for_reals)
	{
		Serial.println("" + String(left_speed) + "/"+ "" +  String(right_speed) + "");
		move_car_analogue(left_speed, right_speed);
	}
	else
	{
		// Fake it. See notes on various ways to simulate all types of robotics.
		//Serial.println("LT : " + String(left_speed) + " RT : " +  String(right_speed) + "");
		Serial.println("" + String(left_speed) + "/"+ "" +  String(right_speed) + "");
	}
}

void turn_on_some_line_tracing_please()
{
	line_tracing = true;
	//smartcar_is_still_learning_and_has_lost_his_way = false;
}

void turn_off_line_tracer()
{
	line_tracing = false;
	
	//smartcar_is_dumb_and_lost_his_way = false;
}

void line_trace_loop()
{


    if (line_tracing == true)
    {
        unsigned char sensor_data = 0;
        
       
       int the_readings[8]={0, 0, 0, 0, 0, 0, 0, 0};
       
       infrared_sensor_read(the_readings);
       
        for(int z = 0; z < 8; z++)
        {
            unsigned int val = the_readings[z];
            sensor_data |= (val << z);
        }
        /*
        for(int z = 0; z < 8; z++)
        {
            unsigned int val = digitalRead(SensorD[z]);
            sensor_data |= (val << z);
        }
        */
        
        sensor_data = ~sensor_data;
        
        //Serial.print(sensor_data, HEX);
        
        //Serial.write(" ");
        
        switch (sensor_data)
        {
            //our move forward situations.
            case 0x18:
            case 0x10:
            case 0x08:
            case 0x38:
            case 0x1c:
            case 0x3c:
            move_car_for_line_tracing(150,150);
            //status = "moving Forward";
            break;
            
            //our turn right situations.
            case 0x0c:
            case 0x04:
            case 0x06:
            case 0x0e:
            case 0x1e:
            case 0x0f:
            move_car_for_line_tracing(75, 150);
            //new_text_to_send = "Turn Right";
            break;
            
            //our turn left situations.
            case 0x30:
            case 0x20:
            case 0x60:
            case 0x70:
            case 0x78:
            case 0xf0:
            //turn_left_speed(30, 75);
            move_car_for_line_tracing(150, 75);
            //new_text_to_send = "Turn Left";
            break;
            
             //our spin right situations.
           	case 0x07:
            case 0x03:
            case 0x02:
            case 0x01:
            move_car_for_line_tracing(-150, 150);
            //new_text_to_send = "Pivot Right";
            break;
            
             //our spin left situations.
           	case 0xc0:
            case 0x40:
            case 0x80:
            case 0xe0:
            move_car_for_line_tracing(150, -150);
            //new_text_to_send = "Pivot Left";
            break;
            
            case 0x00:
            case 0xff:
            //new_text_to_send = "No line to follow";
            move_car_for_line_tracing(0, 0);
            break;
            default:
            //new_text_to_send = "Hit Default";
            move_car_for_line_tracing(0, 0);
            break;
        }
        delay(main_loop_delay);
    }
    
    /*
    if (new_text_to_send != stored_text_to_send)
    {
    	stored_text_to_send = new_text_to_send;
    	Serial.print(stored_text_to_send);
    }
    */
    
}



