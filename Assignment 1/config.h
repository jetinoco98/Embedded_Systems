#ifndef CONFIG_H
#define	CONFIG_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "timer.h"

#define BAUDRATE 9600
#define ARRAY_SIZE 5

void spi_setup();
void uart_setup();
int spi_write(int addr);

void magnetometer_config();
void acquire_magnetometer_data();

int combine_xy_register_bytes(uint8_t LSB_byte, uint8_t MSB_byte);
int combine_z_register_bytes(uint8_t LSB_byte, uint8_t MSB_byte);
double calculate_average(int values[], int size);


#endif