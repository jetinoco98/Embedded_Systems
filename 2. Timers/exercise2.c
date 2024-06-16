/*
 * File:   timer.c
 * Author: jetin
 *
 * Created on March 7, 2024, 4:39 PM
 */

#include "timer.h"


int main(void) {

    TRISAbits.TRISA0 = 0;   // pin A0 set as output

    while(1){
        // Turn on LED
        LATAbits.LATA0 = 1;     // set pin A0 to high
        // Wait for 20ms
        tmr_wait_ms(TIMER1, 20);

        // Turn off LED
        LATAbits.LATA0 = 0;     // set pin A0 to low
        // Wait for 200ms
        tmr_wait_ms(TIMER1, 200); 
    }
  
    return 0;
}


void tmr_wait_ms(int timer, int ms){

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

    // SETUP AND WAITING FOR TIMER
    if (timer == 1){
        // SETUP THE TIMER 1
        TMR1 = 0;                    
        PR1 = pr_value;              
        T1CONbits.TCKPS = prescaler_bits;
        IFS0bits.T1IF = 0;        
        T1CONbits.TON = 1;          

        // WAIT UNTIL FLAG
        while(!IFS0bits.T1IF);     // busy waiting
        IFS0bits.T1IF = 0;         // Clear the interrupt flag
    }
    
    else if (timer == 2){
        // SETUP THE TIMER 2
        TMR2 = 0; 
        PR2 = pr_value;  
        T2CONbits.TCKPS = prescaler_bits;
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 1;

        // WAIT UNTIL FLAG
        while(!IFS0bits.T2IF);
        IFS0bits.T2IF = 0;
    }
    
}