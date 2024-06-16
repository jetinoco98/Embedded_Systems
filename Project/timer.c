#include "timer.h"


void tmr_wait_ms(int timer, int ms){
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // CONSTANTS FOR PRESCALER AND TIMER PR2 VALUE
    int prescaler_bits;
    long int pr_value;
    long int temp_pr_value;

    // OBTAINING THE PR VALUE
    for (int i = 0; i < 4; i++) {
        temp_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (temp_pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    // SETUP & WAITING: 
    // * When the amount of [ms] surpasses the limitation of the timer due to
    // a max value of 16 bits, we will use the maximum PR value until a number
    // below 65536 is found.
    
    while (temp_pr_value > 0){
        
        if(temp_pr_value >= MAX_VALUE_16_BITS) {pr_value = MAX_VALUE_16_BITS;}
        else {pr_value = temp_pr_value;}

        // The temporal PR value is decreased similar to "division" by iteration
        temp_pr_value = temp_pr_value - MAX_VALUE_16_BITS;
            
        // SETUP THE APPROPRIATE TIMER REGISTER VALUES
        tmr_set_register_values(timer, pr_value, prescaler_bits, 0);
        
        // BUSY WAITING FOR FLAG
        if (timer == 1){
            while(!IFS0bits.T1IF);      // busy waiting
            IFS0bits.T1IF = 0;          // Clear the interrupt flag
        }
        else if (timer == 2){
            while(!IFS0bits.T2IF);   
            IFS0bits.T2IF = 0;      
        }
        else if (timer == 3){
            while(!IFS0bits.T3IF);  
            IFS0bits.T3IF = 0;        
        }
        else if (timer == 4){
            while(!IFS1bits.T4IF);    
            IFS1bits.T4IF = 0;          
        }
    }
}


void tmr_set_register_values(int timer, int pr_value, int prescaler_bits, int has_interrupt){
    if (timer == 1){
        TMR1 = 0;                                   // Reset timer counter
        PR1 = pr_value;                             // Timer will count up to this value
        T1CONbits.TCKPS = prescaler_bits;           // Prescaler bits (0,1,2,3)
        IFS0bits.T1IF = 0;                          // Clear the interrupt flag
        if (has_interrupt) {IEC0bits.T1IE = 1;}     // Enable timer interrupt
        T1CONbits.TON = 1;                          // Start the timer
    }
    else if (timer == 2){
        TMR2 = 0;                   
        PR2 = pr_value;                   
        T2CONbits.TCKPS = prescaler_bits;   
        IFS0bits.T2IF = 0;
        if (has_interrupt) {IEC0bits.T2IE = 1;}     
        T2CONbits.TON = 1;          
    }
    else if (timer == 3){
        TMR3 = 0;                   
        PR3 = pr_value;                   
        T3CONbits.TCKPS = prescaler_bits;   
        IFS0bits.T3IF = 0;
        if (has_interrupt) {IEC0bits.T3IE = 1;} 
        T3CONbits.TON = 1;          
    }
    else if (timer == 4){
        TMR4 = 0;                   
        PR4 = pr_value;                   
        T4CONbits.TCKPS = prescaler_bits;   
        IFS1bits.T4IF = 0;      
        if (has_interrupt) {IEC1bits.T4IE = 1;}  
        T4CONbits.TON = 1;          
    }
}


void tmr_setup_period(int timer, int ms, int has_interrupt){
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // CALCULATE THE PRESCALER & PR VALUES
    int prescaler_bits;
    long int pr_value;
    
    for (int i = 0; i < 4; i++) {
        pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    tmr_set_register_values(timer, pr_value, prescaler_bits, has_interrupt);
}


int tmr_wait_period(int timer){
    // Return inmediately or wait depending on current flag status
    if (timer == 1){
        // If the flag was already up, we zero it out and return 1 inmediately
        if (IFS0bits.T1IF){
            IFS0bits.T1IF = 0; 
            return 1;
        }
        // Else we busy wait until the flag goes up, then we zero it out and 
        // finally return 0
        else{
            while(!IFS0bits.T1IF); 
            IFS0bits.T1IF = 0; 
            return 0;
        }
    }

    if (timer == 2){
        if (IFS0bits.T2IF) {
            IFS0bits.T2IF = 0; 
            return 1;
        }
        else{
            while(!IFS0bits.T2IF);   
            IFS0bits.T2IF = 0;          
            return 0;
        }
    }

    if (timer == 3){
        if (IFS0bits.T3IF){
            IFS0bits.T3IF = 0;
            return 1;
        }
        else{
            while(!IFS0bits.T3IF) {};  
            IFS0bits.T3IF = 0;         
            return 0;
        }
    }

    if (timer == 4){
        if (IFS1bits.T4IF){
            IFS1bits.T4IF = 0;
            return 1;
        }
        else{
            while(!IFS1bits.T4IF) {};     
            IFS1bits.T4IF = 0;         
            return 0;
        }
    }  
    return -1;  // This would not happen in a working program
}
