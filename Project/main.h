#ifndef MAIN_H
#define	MAIN_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "timer.h"
#include "buffer.h"
#include "pwm.h"

#define BAUDRATE 9600
#define ARRAY_SIZE 5

#define LED1 LATAbits.LATA0
#define LED2 LATGbits.LATG9
#define LED_LEFT LATBbits.LATB8
#define LED_RIGHT LATFbits.LATF1
#define LED_BRAKES LATFbits.LATF0

void io_setup();
void adc_setup();
void uart_setup();

void parse_acknowledgment(int result_parse);
void initialize_uart_tx();
void set_action(int command);

void turn_off_all_leds();
void toggle_led_group();

void get_adc_values(double *voltage, double *distance);
void write_distance_on_tx(double distance);
void write_voltage_on_tx(double voltage);


#endif