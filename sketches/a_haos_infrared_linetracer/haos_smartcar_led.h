
#ifndef smart_car_led_control
#define smart_car_led_control


void setup_led_pins();

void front_led_control(bool value);
void rear_led_control(bool value);
void signal_error_with_led();
#endif
