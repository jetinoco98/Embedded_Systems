/*
 * File:   main.c
 * Author: jetin
 *
 * Created on March 5, 2024, 10:46 AM
 */


#include "xc.h"

void turn_on_when_pressed(){
    int pin_value;
    pin_value = PORTEbits.RE8;
    
    // When button is not pressed, its value is high
    if(pin_value) {LATAbits.LATA0 = 0;}
    // When button is pressed, its value is low
    else{LATAbits.LATA0 = 1;}
}

void toggle_when_pressed(int *previous_pin_value, int *previous_output_value){
    int pin_value;
    pin_value = PORTEbits.RE8;

    // Occurs only on first iteration of main while loop
    if (*previous_pin_value == -1){
        *previous_pin_value = pin_value;
        return;
    }
    
    // Occurs only when button is pushed, not released.
    if (*previous_pin_value == 1 && pin_value == 0){
        
        if (*previous_output_value == 0){
            LATAbits.LATA0 = 1;     // set pin A0 to high (Turn on LED)
            *previous_output_value = 1;
        }
        else{
            LATAbits.LATA0 = 0;     // set pin A0 to low (Turn off LED)
            *previous_output_value = 0;
        }
        
    }

    *previous_pin_value = pin_value;
}


int main(void) {
    // Initialize pins
    TRISAbits.TRISA0 = 0;   // pin A0 set as output
    TRISEbits.TRISE8 = 1;   // pin E8 set as input
    
    // Disable analog pins
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELF = ANSELG = 0x0000;
    
    int previous_pin_value = -1;
    int previous_output_value = 0;
    
    while(1){
        // Turn ON the LED LD1
        // LATAbits.LATA0 = 1;     // set pin A0 to high
        
        // Read the button T2, while it is pressed turn on LD1, else turn it off.
        // turn_on_when_pressed();
        
        // Toggle the LED status with each button press
        toggle_when_pressed(&previous_pin_value, &previous_output_value);
    }
    
    return 0;
}

