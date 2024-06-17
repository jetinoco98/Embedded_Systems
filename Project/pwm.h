#ifndef PWM_H
#define	PWM_H

#include "xc.h"                        // Include XC header for PIC microcontrollers
#include "timer.h"
#include <stdio.h>                     // Standard I/O library
#include <stdbool.h>                   // Boolean type definitions

// Define macros for PWM output compare registers
#define RIGHTB OC1R 
#define RIGHTF OC2R 

#define LEFTB OC3R 
#define LEFTF OC4R

// Define macros for movement directions
#define STOP 0
#define FORWARD 1 
#define LEFT 2
#define RIGHT 3 
#define BACKWARDS 4

// Define macro for PWM frequency
#define PWMFREQUENCY 10 //KHz
#define PWMPRESCALER 1 

int move(int dir);
void pwm_setup();

#endif