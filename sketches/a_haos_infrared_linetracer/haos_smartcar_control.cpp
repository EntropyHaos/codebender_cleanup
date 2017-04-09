#include "a_haos_linetracer.h"

// ToDo : put some instructions here?

unsigned char text[] = "\n Welcome!\n Arduino SmartCAR  Test Program.\n";


void setup_smart_car_control_logic()
{
	Serial.begin(115200);

	int i = 0;

	while (text[i] != '\0')
	{
		Serial.write(text[i++]);
	}
	//Serial.write("Received cmds: ");
}

void softReset()
{
	asm volatile ("  jmp 0");
}

void serialEvent()
{
	int command = Serial.read();

	Serial.print("cmd Rx: " + String(command));


	//  01 - 10 : Motion
	//  11 - 20 : LED
	//  21 - 30 : Sound
	//  31 - 40 : UltraSonic
	//  41 - 50 : InfraRed


	/*__  __                   _____ __  __            _    _____          _____
	 /  \/  |                 / ____|  \/  |          | |  / ____|   /\   |  __ \
	 | \  / | _____   _____  | (___ | \  / | __ _ _ __| |_| |       /  \  | |__) |
	 | |\/| |/ _ \ \ / / _ \  \___ \| |\/| |/ _` | '__| __| |      / /\ \ |  _  /
	 | |  | | (_) \ V /  __/  ____) | |  | | (_| | |  | |_| |____ / ____ \| | \ \
	 |_|  |_|\___/ \_/ \___| |_____/|_|  |_|\__,_|_|   \__|\_____/_/    \_\_|  \*/

	switch (command)
	{
		case 0:
		
			//Serial.println("ZERO SIGNAL RECIEVED?");
			// Maybe useful for signal of connection?
			// need further looks.
			
			break;
		case 1:
			//move_forward();
			move_car_analogue(100, 100);
			break;
		case 2:
			//turn_left();
			move_car_analogue(-100, 100);
			break;
		case 3:
			//turn_right();
			move_car_analogue(100, -100);
			break;
		case 4:
			//move_backward();
			move_car_analogue(-100, -100);
			break;
		case 5:
			// Stops Motion
			move_car_analogue(0, 0);
			break;
		case 6:

			break;
		case 7:
			break;
		case 8:
			break;
		case 9:
			break;
		case 10:
			break;

			//

			/*_      ______ _____     _____                      _    _____          _____
			 / |    |  ____|  __ \   / ____|                    | |  / ____|   /\   |  __ \
			 | |    | |__  | |  | | | (___  _ __ ___   __ _ _ __| |_| |       /  \  | |__) |
			 | |    |  __| | |  | |  \___ \| '_ ` _ \ / _` | '__| __| |      / /\ \ |  _  /
			 | |____| |____| |__| |  ____) | | | | | | (_| | |  | |_| |____ / ____ \| | \ \
			 |______|______|_____/  |_____/|_| |_| |_|\__,_|_|   \__|\_____/_/    \_\_|  \*/

		case 11:
			//;
			front_led_control(true);
			break;
		case 12:
			//;
			front_led_control(false);
			break;
		case 13:
			//;
			rear_led_control(true);
			break;
		case 14:
			//;
			rear_led_control(false);
			break;
		case 15:
			//;
			break;
		case 16:
			//;
			break;
		case 17:
			//;
			break;
		case 18:
			//;
			break;
		case 19:
			//;
			break;
		case 20:
			//;
			break;

			/* _____                       _    _____ __  __          _____ _______
			 /  ____|                     | |  / ____|  \/  |   /\   |  __ \__   __|
			 | (___   ___  _   _ _ __   __| | | (___ | \  / |  /  \  | |__) | | |
			  \___ \ / _ \| | | | '_ \ / _` |  \___ \| |\/| | / /\ \ |  _  /  | |
			  ____) | (_) | |_| | | | | (_| |  ____) | |  | |/ ____ \| | \ \  | |
			 |_____/ \___/ \__,_|_| |_|\__,_| |_____/|_|  |_/_/    \_\_|  \_\ |*/


		case 21:
			//;
			turn_buzzer_on();
			break;
		case 22:
			//;
			turn_buzzer_off();
			break;
		case 23:
			//;
			break;
		case 24:
			//;
			break;
		case 25:
			//;
			break;
		case 26:
			//;
			break;
		case 27:
			//;
			break;
		case 28:
			//;
			break;
		case 29:
			//;
			break;
		case 30:
			//;
			break;

			/*_    _ _   _______ _____             _____             _
			 | |  | | | |__   __|  __ \     /\    / ____|           (_)
			 | |  | | |    | |  | |__) |   /  \  | (___   ___  _ __  _  ___
			 | |  | | |    | |  |  _  /   / /\ \  \___ \ / _ \| '_ \| |/ __|
			 | |__| | |____| |  | | \ \  / ____ \ ____) | (_) | | | | | (__
			  \____/|______|_|  |_|  \_\/_/    \_\_____/ \___/|_| |_|_|\__*/



		case 31:
			//;
			break;
		case 32:
			//;
			break;
		case 33:
			//;
			break;
		case 34:
			//;
			break;
		case 35:
			//;
			break;
		case 36:
			//;
			break;
		case 37:
			//;
			break;
		case 38:
			//;
			break;
		case 39:
			//;
			break;
		case 40:
			//;
			ultrasonic_sensor_read();
			break;

			/*_____        __           _____  ______ _____
			 |_   _|      / _|         |  __ \|  ____|  __ \
			   | |  _ __ | |_ _ __ __ _| |__) | |__  | |  | |
			   | | | '_ \|  _| '__/ _` |  _  /|  __| | |  | |
			  _| |_| | | | | | | | (_| | | \ \| |____| |__| |
			 |_____|_| |_|_| |_|  \__,_|_|  \_\______|_____*/



		case 41:
			set_sensor_reads_for_white_values();
			break;
		case 42:
			set_sensor_reads_for_black_values();
			break;
		case 43:
			config_digital_infrared_sensors();
			break;
		case 44:
			// Line Tracing Mode On
			
			break;
		case 45:
			// LINE ON TRACEER FROM APP
			//Serial.println("LT off!");
			turn_on_some_line_tracing_please();

			break;
		case 46:
			//Serial.println("LT off!");
			turn_off_line_tracer();

			break;
		case 47:
			//;
			break;
		case 48:
			//;
			break;
		case 49:
			//;
			break;
		case 50:
			//;
			infrared_sensor_read();
			break;

		case 99:
			//;
			softReset();
			break;


		default:
			move_car_analogue(0, 0);
			signal_error_with_led();
			//buzz_error();
			Serial.println(" * ERROR! (" + String(command) + ")");
			front_led_control(false);
			rear_led_control(false);
			break;
	}

	Serial.println("");
	/*

	ToDo : Output Command recieved.

	*/

}


