#ifndef PWM_H
#define	PWM_H

#include "xc.h"                        // Include XC header for PIC microcontrollers
#include <stdio.h>                     // Standard I/O library
#include <stdbool.h>                   // Boolean type definitions

// Define macros for PWM output compare registers
#define RIGHTB OC1R 
#define RIGHTF OC2R 

#define LEFTB OC3R 
#define LEFTF OC4R

// Define macros for movement directions
#define FORWARD 1 
#define BACKWARDS 2 
#define RIGHT 3 
#define LEFT 4 
#define STOP 5

// Define macro for PWM frequency
#define PWMFREQUENCY 7200

int move(int dir);
void pwm_setup();

#endif