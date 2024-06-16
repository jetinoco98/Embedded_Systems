/*
 * File:   timer.c
 * Author: jetin
 *
 * Created on March 7, 2024, 4:39 PM
 */

#include "timer.h"


int main(void) {
    
    TRISAbits.TRISA0 = 0;           // pin A0 set as output
    int led = 0;                    // LED on/off status
    tmr_setup_period(TIMER1, 200);  // Start the timer
    
    while(1){
        // Blink LED
        led = 1 - led;                  // toggle between 1 and 0
        if(led) {LATAbits.LATA0 = 1;}   // set pin A0 to high
        else {LATAbits.LATA0 = 0;}      // set pin A0 to low

        // Wait until timer expires
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}

void tmr_setup_period(int timer, int ms){
    
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // CALCULATE THE PRESCALER ADECUATE VALUES
    int prescaler_bits;
    long int pr_value;
    
    for (int i = 0; i < 4; i++){
        pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    // SET THE TIMER REGISTER VALUES
    if (timer == 1){
        TMR1 = 0;                               // Reset timer counter
        PR1 = pr_value;                         // Timer will count up to this value
        T1CONbits.TCKPS = prescaler_bits;       // Prescaler bits (0,1,2,3)
        IFS0bits.T1IF = 0;                      // Clear Interrupt flag
        T1CONbits.TON = 1;                      // Start the timer
    }
    else if (timer == 2){
        TMR2 = 0; 
        PR2 = pr_value;  
        T2CONbits.TCKPS = prescaler_bits;
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 1;
    }
    
}

int tmr_wait_period(int timer){

    if (timer == 1){
        while(!IFS0bits.T1IF);  // busy waiting
        IFS0bits.T1IF = 0;
    }
    else if (timer == 2){
        while(!IFS0bits.T2IF);
        IFS0bits.T2IF = 0;
    }

}