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

/**
 * Writes into the UART TX Buffer a custom message. 
 * - `$MACK,1*` when a match was found by the parser.
 * - `$MACK,0*` when the commands FIFO was full and the new command was rejected.
 */
void parse_acknowledgment(int result_parse);

/**
 * Inserts one character from the TX Circular Buffer into the UART FIFO TX Register.
 */
void initialize_uart_tx();


void turn_off_all_leds();
void toggle_led_group();

void get_adc_values(double *voltage, double *distance);

/**
 * Converts the float value of `distance` into a formatted string with `sprintf`, then uses
 * the `write_text_buffer()` function to update the UART TX Buffer char by char.
 */
void write_distance_on_tx(double distance);

/**
 * Converts the float value of `voltage` into a formatted string with `sprintf`, then uses
 * the `write_text_buffer()` function to update the UART TX Buffer char by char.
 */
void write_voltage_on_tx(double voltage);


#endif