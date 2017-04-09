#include "a_haos_ultrasonic_smartcar.h"

void setup() {
	
	setup_smart_car_control_logic();
	setup_buzzer_pin();
	setup_led_pins();
	initialize_infrared_sensor_pack();
	setup_ultrasonic();
	startup_movement();
	buzz_ready();
}
void loop() {
    
}
