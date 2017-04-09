#ifndef HAOS_HAOS_SMARTCAR_INFRARED_H
#define HAOS_HAOS_SMARTCAR_INFRARED_H

void initialize_infrared_sensor_pack();

void config_analoge_infrared_sensors();
void config_digital_infrared_sensors();


void serial_print_array_values();

void set_sensor_reads_for_white_values();
void set_sensor_reads_for_black_values();

void config_digital_infrared_sensors();

void infrared_sensor_read();

void infrared_sensor_read(int array_to_read_values_into[8]);

#endif
