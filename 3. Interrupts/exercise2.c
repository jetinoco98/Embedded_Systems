/*
 * File:   timer.c
 * Author: jetin
 *
 * Created on March 7, 2024, 4:39 PM
 */

#include "config.h"


// GLOBAL VARIABLES
volatile int led2 = 0;


void __attribute__ ((__interrupt__ , __auto_psv__)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;  // Reset interrupt flag
    IEC0bits.T1IE = 0;  // Disable timer interrupt
    T1CONbits.TON = 0;  // Stop the timer
    
    // Toggle LED2
    led2 = 1 - led2;
    if(led2) {LATGbits.LATG9 = 1;}   
    else {LATGbits.LATG9 = 0;}
}


void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;    // Reset interrupt flag
    IEC0bits.T1IE = 1;      // Enable timer 1 interrupt
    tmr1_setup_period(10);  // Activate the timer1 for 10ms (for debouncing)
}


int main(void) {

    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)
    int delay2 = 200;
    int led1 = 0;
    
    // Analog pins disabled
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = 0x0000;
    
    // CONFIG: Button 1 with interrupt
    TRISEbits.TRISE9 = 1;       // pin E9 set as input
    RPINR0bits.INT1R = 0x59;    // 0x59 is 89 in hex. Refers to PIN RE9/RPI89

    INTCON2bits.GIE = 1;        // Enable global interrupt
    IFS1bits.INT1IF = 0;        // Clear INT1 interrupt flag
    IEC1bits.INT1IE = 1;        // Enable INT1 interrupt
    
    
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
        pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
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