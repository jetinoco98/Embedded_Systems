/*
 * File:   main.c
 * Author: jetin
 *
 * Created on April 4, 2024, 11:45 AM
 */

#include "config.h"


////// GLOBAL VARIABLES

// For LED control
volatile int led1 = 0;
volatile int led2 = 0;
volatile int enable_blink = 1;
// For UART TX
int deadlines_missed = 0;
int characters_received = 0;
int send_deadlines = 0;
int send_characters = 0;
// Buffer to store received characters
char buffer_rx[10];
int buffer_rx_index = 0;
char* target_words[] = {"LD1", "LD2"};


void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;    // Reset interrupt flag
    IEC1bits.T4IE = 1;      // Enable timer 1 interrupt
    send_characters = 1;
}

void __attribute__ ((__interrupt__ , __auto_psv__)) _INT2Interrupt(void) {
    IFS1bits.INT2IF = 0;    // Reset interrupt flag
    IEC1bits.T4IE = 1;      // Enable timer 1 interrupt
    send_deadlines = 1;
}

void __attribute__ ((__interrupt__ , __auto_psv__)) _T3Interrupt(void) {
    
    IFS0bits.T3IF = 0;  // Reset interrupt flag

    if(enable_blink){
        led2 = 1 - led2;
        if(led2) {LATGbits.LATG9 = 1;}   
        else {LATGbits.LATG9 = 0;}
    }
    else{
        LATGbits.LATG9 = 0;
    }
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void){
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag
    while (U1STAbits.URXDA){
        buffer_rx[buffer_rx_index++] = U1RXREG;
        characters_received++;
    }

}


void algorithm(){
    tmr_wait_ms(TIMER2, 7);
}


int main(void) {
    
    // Setup LEDS and buttons
    setup_io();
    int blink_delay = 200;
    // Configure the UART1 to use the MikroBUS 2.
    setup_uart();
    // For the use on the frequency of the algorithm function
    tmr_setup_period(TIMER1, 10, 0);
    // For the use on the LED blink functionality
    tmr_setup_period(TIMER3, blink_delay, 1);
    
    while(1){
        
        algorithm();    // Takes 7[ms]
        
        // ASSIGNMENT CODE 1: SEND DATA TO UART

        if (send_characters){
            char buffer_tx[10]; // Buffer to hold the formatted string
            sprintf(buffer_tx, "C=%d", characters_received);
            UART_write_string(buffer_tx);
            send_characters = 0;
        }

        if (send_deadlines){
            char buffer_tx[10]; // Buffer to hold the formatted string
            sprintf(buffer_tx, "D=%d", deadlines_missed);
            UART_write_string(buffer_tx);
            send_deadlines = 0;
        }
        
        // ASSIGNMENT CODE 2: ANALYZE DATA READ FROM UART
        
        int match = 0;
        
        for (int i = 0; i < sizeof(target_words) / sizeof(target_words[0]); i++) {
            if (strcmp(buffer_rx, target_words[i]) == 0) {
                handle_action(buffer_rx);
                memset(buffer_rx, 0, sizeof(buffer_rx)); // Clear buffer
                buffer_rx_index = 0;
                match = 1;
                break;
            } 
            else if (strstr(target_words[i], buffer_rx) != NULL) {
                match = 1;
                break;
            }
        }

        if (!match) {
            memset(buffer_rx, 0, sizeof(buffer_rx)); // Clear buffer
            buffer_rx_index = 0;
        }


        // END OF ASSIGNMENT CODE
        
        int ret = tmr_wait_period(TIMER1);  // (1) if the deadline was missed
        deadlines_missed += ret;
        
    }
    
    return 0;
}


void setup_io(){
    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)
    
    // Disable all analog pins
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    // CONFIG: PIN REMAPPING
    TRISEbits.TRISE8 = 1;       // pin E8 set as input
    RPINR0bits.INT1R = 0x58;    // 0x58 = 88. Refers to PIN RE8/RPI88

    TRISEbits.TRISE9 = 1;       // pin E9 set as input
    RPINR1bits.INT2R = 0x59;    // 0x59 = 89. Refers to PIN RE9/RPI89 

    // If GIE is enabled before remapping, crash occurs.
    
    // Enabling interrupts
    INTCON2bits.GIE = 1;        // Global interrupt enable
    IFS1bits.INT1IF = 0;        // Clear INT1 interrupt flag
    IEC1bits.INT1IE = 1;        // Enable INT1 interrupt
    IFS1bits.INT2IF = 0;        // Clear INT2 interrupt flag
    IEC1bits.INT2IE = 1;        // Enable INT2 interrupt
}


void setup_uart(){
    U1BRG = 468;                // Obtained from (FCY/(16*baud_rate) - 1)
    U1MODEbits.UARTEN = 1;      // Enable UART
    U1STAbits.UTXEN = 1;        // Enable U1TX
    // Remapping of UART1 to pins of MicroBus 2
    RPINR18bits.U1RXR = 0x4B;
    RPOR0bits.RP64R = 1;
    IEC0bits.U1RXIE = 1;
}


void UART_write_character(char c) {
    while (U1STAbits.UTXBF) {}; // Wait while buffer is full
    U1TXREG = c; // Transmit character
}


void UART_write_string(const char* str) {
    while (*str != '\0') {
        while (U1STAbits.UTXBF) {}; // Wait while buffer is full
        U1TXREG = *str; // Transmit character
        str++;
    }
}


void handle_action(const char* received_string) {
    if (strcmp(received_string, "LD2") == 0) {
        enable_blink = 1 - enable_blink;
    }
    if (strcmp(received_string, "LD1") == 0) {
        led1 = 1 - led1;
        if(led1) {LATAbits.LATA0 = 1;}   
        else {LATAbits.LATA0 = 0;}
    } 
}


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
    
    // SETUP & WAITING
    while (temp_pr_value > 0){
        
        if(temp_pr_value >= MAX_VALUE_16_BITS) {pr_value = MAX_VALUE_16_BITS;}
        else {pr_value = temp_pr_value;}

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
    
    if (timer == 1){
        if (IFS0bits.T1IF){IFS0bits.T1IF = 0; return 1;}
        else{while(!IFS0bits.T1IF); IFS0bits.T1IF = 0; return 0;}
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
}


void tmr_set_register_values(int timer, int pr_value, int prescaler_bits,
                             int has_interrupt){
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