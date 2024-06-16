/*
 * File:   timer.c
 * Author: jetin
 *
 * Created on March 7, 2024, 4:39 PM
 */

#include "config.h"


// GLOBAL VARIABLES
volatile int led2 = 0;
long int tmr1_pr_value;
long int tmr1_temp_pr_value;


void __attribute__ ((__interrupt__ , __auto_psv__)) _T1Interrupt(void) {
    
    IFS0bits.T1IF = 0;  // Reset interrupt flag
    tmr1_temp_pr_value = tmr1_temp_pr_value - MAX_VALUE_16_BITS;
    
    // WHEN PR IS WHITHIN THE 16-BIT LIMIT

    if(tmr1_temp_pr_value < 0){
        // Blink LED2
        led2 = 1 - led2;
        if(led2) {LATGbits.LATG9 = 1;}   
        else {LATGbits.LATG9 = 0;}
        // Return to the original PR value
        tmr1_temp_pr_value = tmr1_pr_value;
    }
    
    // WHEN PR VALUE EXCEEDS THE 16-BITS LIMIT

    if(tmr1_temp_pr_value >= MAX_VALUE_16_BITS) {
        PR1 = MAX_VALUE_16_BITS;
        return;
    }
    else {
        PR1 = tmr1_temp_pr_value;
        return;
    }
    
}


int main(void) {

    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)
    int delay1 = 1000;
    int delay2 = 200;
    int led1 = 0;
    
    // Setup the timer with interrupt
    tmr1_setup_period(delay1);
    
    while(1){
        // Blink LED1
        led1 = 1 - led1;                  // toggle between 1 and 0
        if(led1) {LATAbits.LATA0 = 1;}   // set pin A0 to high
        else {LATAbits.LATA0 = 0;}      // set pin A0 to low
        
        // Wait for delay
        tmr2_wait_ms(delay2);
    }
  
    return 0;
}


void tmr1_setup_period(int ms){
    
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };

    // CALCULATE THE PRESCALER & PR VALUES
    int prescaler_bits;
    int pr_value;
   
    // OBTAINING THE PR VALUE
    for (int i = 0; i < 4; i++) {
        tmr1_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (tmr1_pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    // WHEN PR VALUE EXCEEDS THE 16-BITS LIMIT
    if (tmr1_pr_value >= MAX_VALUE_16_BITS) {pr_value = MAX_VALUE_16_BITS;}
    else {pr_value = tmr1_pr_value;}
    
    // Duplicate the PR value into temporal/original variables
    tmr1_temp_pr_value = tmr1_pr_value;
    
    // SET THE TIMER REGISTER VALUES
    TMR1 = 0;                   // Reset timer counter
    PR1 = pr_value;             // Timer will count up to this value
    T1CONbits.TCKPS = prescaler_bits;   // Prescaler bits (0,1,2,3)
    IFS0bits.T1IF = 0;          // Interrupt flag
    IEC0bits.T1IE = 1;          // Enable Timer1 interrupt
    T1CONbits.TON = 1;          // Start the timer
    
}


void tmr2_wait_ms(int ms){

    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // CONSTANTS FOR PRESCALER AND TIMER PR2 VALUE
    int prescaler_bits;
    int pr_value;
    long int temp_pr_value;

    // OBTAINING THE PR VALUE
    for (int i = 0; i < 4; i++) {
        temp_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (temp_pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    // SETUP & WAITING
    while (temp_pr_value > 0){
        
        if(temp_pr_value >= MAX_VALUE_16_BITS) {pr_value = MAX_VALUE_16_BITS;}
        else {pr_value = temp_pr_value;}
        
        temp_pr_value = temp_pr_value - MAX_VALUE_16_BITS;
            
        // SETUP THE TIMER 2
        TMR2 = 0;                   // Reset timer counter
        PR2 = pr_value;             // Timer will count up to this value
        T2CONbits.TCKPS = prescaler_bits;   // Prescaler bits (0,1,2,3)
        IFS0bits.T2IF = 0;          // Clear the interrupt flag
        T2CONbits.TON = 1;          // Start the timer

        // WAIT UNTIL FLAG
        while(!IFS0bits.T2IF);    // busy waiting
        IFS0bits.T2IF = 0;            // Clear the interrupt flag
    }
}