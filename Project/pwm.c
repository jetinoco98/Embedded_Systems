#include "pwm.h"

volatile int pwmPeriod;

// Function to move the robot in the specified direction with a given speed
int move(int dir){
    int dutyCycle, speed;
    int num = 80;   // Can be an input parameter of the function

    // Check if num is within the valid range
    if (num < 0 || num > 100){
        // Error handling: invalid speed
        return -1;  // Return an error code
    } else {
        dutyCycle = 40 + (int)(num * 0.6); // Calculate duty cycle from speed
        speed = dutyCycle * pwmPeriod / 100;
    }   
    
    switch(dir){
        case LEFT: 
            LEFTF  = 0;         // Set left forward to 0
            LEFTB  = speed;     // Set left backward to speed
            RIGHTF = speed;     // Set right forward to speed
            RIGHTB = 0;         // Set right backward to 0
            break;
        case RIGHT: 
            LEFTF  = speed;     // Set left forward to speed
            LEFTB  = 0;         // Set left backward to 0
            RIGHTF = 0;         // Set right forward to 0
            RIGHTB = speed;     // Set right backward to speed
            break;
        case BACKWARDS: 
            LEFTF  = speed;     // Set left forward to speed
            LEFTB  = 0;         // Set left backward to 0
            RIGHTF = speed;     // Set right forward to speed
            RIGHTB = 0;         // Set right backward to 0
            break;
        case FORWARD: 
            LEFTF  = 0;         // Set left forward to 0
            LEFTB  = speed;     // Set left backward to speed
            RIGHTF = 0;         // Set right forward to 0
            RIGHTB = speed;     // Set right backward to speed
            break;
        case STOP:
            LEFTF  = 0;         // Set left forward to 0
            LEFTB  = 0;     // Set left backward to 0
            RIGHTF = 0;         // Set right forward to 0
            RIGHTB = 0;     // Set right backward to 0
    }
}

void pwm_setup(){
    
    // Reset PWM control registers
    OC1CON1 = OC1CON2 = OC2CON1 = OC2CON2 = OC3CON1 = OC3CON2 = OC4CON1 = OC4CON2 = 0;

    // Configure PWM modules
    OC1CON1bits.OCTSEL = 7;     // Select input clock for OC1
    OC1CON1bits.OCM = 0b110;    // Set OC1 to edge-aligned PWM mode
    OC1CON2bits.SYNCSEL = 0x1F; // No synchronization for OC1
    RPOR0bits.RP65R = 0b010000; // Map RD1 to OC1

    OC2CON1bits.OCTSEL = 7;     // Select input clock for OC2
    OC2CON1bits.OCM = 0b110;    // Set OC2 to edge-aligned PWM mode
    OC2CON2bits.SYNCSEL = 0x1F; // No synchronization for OC2
    RPOR1bits.RP66R = 0b010001; // Map RD2 to OC2

    OC3CON1bits.OCTSEL = 7;     // Select input clock for OC3
    OC3CON1bits.OCM = 0b110;    // Set OC3 to edge-aligned PWM mode
    OC3CON2bits.SYNCSEL = 0x1F; // No synchronization for OC3
    RPOR1bits.RP67R = 0b010010; // Map RD3 to OC3

    OC4CON1bits.OCTSEL = 7;     // Select input clock for OC4
    OC4CON1bits.OCM = 0b110;    // Set OC4 to edge-aligned PWM mode
    OC4CON2bits.SYNCSEL = 0x1F; // No synchronization for OC4
    RPOR2bits.RP68R = 0b010011; // Map RD4 to OC4
    
    // Set PWM frequency
    pwmPeriod = FCY_PROP/(PWMFREQUENCY*PWMPRESCALER) -1;

    OC1RS = OC2RS = OC3RS = OC4RS = pwmPeriod;

}
