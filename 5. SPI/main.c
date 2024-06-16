#include "config.h"


int spi_write(int addr){
    // Transmit segment
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr;
    // Receive segment
    while (SPI1STATbits.SPIRBF == 0);
    int response = SPI1BUF;
    return response;
}


int main(void) {
    
    // Variables
    int chip_addr = 0x40;
    int x_mag_LSB_addr = 0x42;

    // Setup and configuration
    uart_setup();
    spi_setup();
    magnetometer_config();

    // Acquire the Chip ID (CS3)
    LATDbits.LATD6 = 0;
    spi_write(chip_addr | 0x80);
    int value_from_chip = spi_write(0x00);
    LATDbits.LATD6 = 1;
    
    // Send it to the UART once
    tmr_wait_ms(TIMER1, 2000);  // Give time to connect on PC
    char buffer_tx[11];
    sprintf(buffer_tx, "N=%d", value_from_chip);
    UART_write_string(buffer_tx);

    // Set-up: Frequency of loop
    tmr_setup_period(TIMER2, 100, 0);
    
    while(1){
        
        ///////////////////////////////////
        // Acquire x-axis value from magnetometer
        ///////////////////////////////////
        
        LATDbits.LATD6 = 0;
        spi_write(x_mag_LSB_addr | 0x80);
        uint8_t LSB_byte = spi_write(0x00);
        uint8_t MSB_byte = spi_write(0x00);
        LATDbits.LATD6 = 1;
        
        // Mask to clear three least significant bits
        uint8_t masked_LSB = LSB_byte & 0b11111000;
        // Left-shift MSB_byte by 8 and OR with masked LSB_byte
        int combined_byte = ((MSB_byte << 8) | masked_LSB);
        // Divide by 8 to get correct scale
        int x_mag = combined_byte / 8;
        
        
        ///////////////////////////////////
        // Send it to UART at 10Hz
        ///////////////////////////////////
        
        // At 10Hz using protocol "$MAGX=xxx", where xxx is the value
        memset(buffer_tx, 0, sizeof(buffer_tx)); // Clear buffer
        sprintf(buffer_tx, "$MAG=%d*", x_mag);
        UART_write_string(buffer_tx);
        
        // Wait for timer
        tmr_wait_period(TIMER2);
        
    }

    return 0;
    
}


void uart_setup(){
    
    // Remapping of UART1 to pins of MicroBus 2
    RPINR18bits.U1RXR = 0x4B;
    RPOR0bits.RP64R = 1;
    IEC0bits.U1RXIE = 1;
    
    // UART Configuration
    U1BRG = 468;                // Obtained from (FCY/(16*baud_rate) - 1)
    U1MODEbits.UARTEN = 1;      // Enable UART
    U1STAbits.UTXEN = 1;        // Enable U1TX
    
}


void spi_setup(){
    // Port configuration: CS Lines
    TRISBbits.TRISB3 = 0;   // CS1: Accelerometer
    TRISBbits.TRISB4 = 0;   // CS2: Gyroscope
    TRISDbits.TRISD6 = 0;   // CS3: Magnetometer
    LATBbits.LATB3 = 1;
    LATBbits.LATB4 = 1;
    LATDbits.LATD6 = 1;
    
    // Port configuration: SPI Lines
    TRISAbits.TRISA1 = 1; // RA1?RPI17 MISO
    TRISFbits.TRISF12 = 0; //RF12?RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13?RP109 MOSI
    
    // Remapping of SPI1 to pins of Microbus1
    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) ? RPI17
    RPOR12bits.RP109R = 0b000101; // MOSI (SDO1) ? RF13;
    RPOR11bits.RP108R = 0b000110; // SCK1;
    
    // SPI Configutation
    SPI1CON1bits.MSTEN = 1;      // Master mode
    SPI1CON1bits.MODE16 = 0;     // 8-bit mode
    SPI1CON1bits.CKP = 1;        // Idle value of clock
    // [Prescaler] FSCK = (FCY)/(PPR * SPR) = 72M/(4*3) = 6 MHz
    SPI1CON1bits.PPRE = 0b10;    // Primary prescaler 4:1
    SPI1CON1bits.SPRE = 0b101;   // Secondary prescaler 3:1
    SPI1STATbits.SPIEN = 1;     // Enable SPI
    SPI1STATbits.SPIROV = 0;    // Clear overflow flag
    
}


void magnetometer_config(){

    int sleep_mode_addr = 0x4B;
    int sleep_mode_value = 0x01;
    int active_mode_addr = 0x4C;
    int active_mode_value = 0x00;

    // Make magnetometer switch to Sleep Mode
    LATDbits.LATD6 = 0;
    spi_write(sleep_mode_addr);
    spi_write(sleep_mode_value);
    LATDbits.LATD6 = 1;
    // Wait 2 ms
    tmr_wait_ms(TIMER1, 2);

    // Make magnetometer go to active mode
    LATDbits.LATD6 = 0;
    spi_write(active_mode_addr);
    spi_write(active_mode_value);
    LATDbits.LATD6 = 1;
    // Wait 2 ms
    tmr_wait_ms(TIMER1, 2);

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
        if (IFS0bits.T1IF){
            IFS0bits.T1IF = 0; 
            return 1;
        }
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
}

void UART_write_string(const char* str) {
    while (*str != '\0') {
        while (U1STAbits.UTXBF) {}; // Wait while buffer is full
        U1TXREG = *str; // Transmit character
        str++;
    }
}