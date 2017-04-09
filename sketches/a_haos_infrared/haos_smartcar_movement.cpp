#include "Arduino.h"

#define LEFT_MD_A 22
#define LEFT_MD_B 23
#define RIGHT_MD_A 24
#define RIGHT_MD_B 25
#define LEFT_MOTOR_EN 4
#define RIGHT_MOTOR_EN 5


// This function needs to be called during setup of IC.
void setup_ports_for_movement()
{
    pinMode(LEFT_MD_A, OUTPUT);
    pinMode(LEFT_MD_B, OUTPUT);
    
    pinMode(RIGHT_MD_A, OUTPUT);
    pinMode(RIGHT_MD_B, OUTPUT);
    
    pinMode(LEFT_MOTOR_EN, OUTPUT);
    pinMode(RIGHT_MOTOR_EN, OUTPUT);
    
    
    // Set everything to be off.
    digitalWrite(LEFT_MD_A, LOW);
    digitalWrite(LEFT_MD_B, LOW);
    
    digitalWrite(RIGHT_MD_A, LOW);
    digitalWrite(RIGHT_MD_B, LOW);
    
    digitalWrite(LEFT_MOTOR_EN, LOW);
    digitalWrite(RIGHT_MOTOR_EN, LOW);
}

void startup_movement(){
	setup_ports_for_movement();	
}
void move_stop() {
    analogWrite(LEFT_MOTOR_EN, 0);
    analogWrite(RIGHT_MOTOR_EN, 0);
}

boolean left_wheels_are_moving_forward = true;
boolean right_wheels_are_moving_forward = true;
boolean need_delay_for_direction_change = false;

void switch_wheel_direction_on_left_side_instead_of_right(boolean left_side_switch)
{
	if (left_side_switch)
	{
	    digitalWrite(LEFT_MD_A, (left_wheels_are_moving_forward ? HIGH : LOW ));
	    digitalWrite(LEFT_MD_B, (left_wheels_are_moving_forward ? LOW : HIGH ));
	}
	else
	{
	    digitalWrite(RIGHT_MD_A, (right_wheels_are_moving_forward ? LOW : HIGH ));
	    digitalWrite(RIGHT_MD_B, (right_wheels_are_moving_forward ? HIGH : LOW ));
	}
}

boolean manage_direction_change_concern(int new_left_motor_speed, int new_right_motor_speed)
{
	boolean direction_change_delay_needed = false;
	
	if (left_wheels_are_moving_forward != (new_left_motor_speed < 0))
	{
		switch_wheel_direction_on_left_side_instead_of_right(true);
		left_wheels_are_moving_forward = !left_wheels_are_moving_forward;
		direction_change_delay_needed = true;
	}
	if (right_wheels_are_moving_forward != (new_right_motor_speed < 0))
	{
		switch_wheel_direction_on_left_side_instead_of_right(false);
		right_wheels_are_moving_forward = !right_wheels_are_moving_forward;
		direction_change_delay_needed = true;
	}
	return direction_change_delay_needed;
}


void move_car_analogue(int left_motor_speed, int right_motor_speed)
{
	if (manage_direction_change_concern(left_motor_speed, right_motor_speed))
	{
		move_stop();
		delay(500);
	}

    // Turn motors on if motion is requested
    if (left_motor_speed != 0 || right_motor_speed != 0)
    {
	    analogWrite(LEFT_MOTOR_EN, abs(left_motor_speed));
	    analogWrite(RIGHT_MOTOR_EN, abs(right_motor_speed));
    }
    else
    {
    	move_stop();
    }
}
