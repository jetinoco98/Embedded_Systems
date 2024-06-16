#include "config.h"

// GLOBAL VARIABLES
char buffer_tx[47];
int buffer_index = 0;

int x_values[ARRAY_SIZE];
int y_values[ARRAY_SIZE];
int z_values[ARRAY_SIZE];
int array_index = 0;


// Interrupt for UART TRANSMISSION
void __attribute__ ((__interrupt__ , __auto_psv__)) _U1TXInterrupt(void) {
    // Clear TX interrupt flag 
    IFS0bits.U1TXIF = 0;
    
    // Loop to write multiple characters into the transmit register
    while (buffer_tx[buffer_index] != '\0' && !U1STAbits.UTXBF) {
        // Write current character into transmit register  
        U1TXREG = buffer_tx[buffer_index];
        // Shift the buffer index
        buffer_index++;
    }
    
    // If reached end of buffer, disable further interrupts
    if (buffer_tx[buffer_index] == '\0') {
        IEC0bits.U1TXIE = 0;    // Disable further interrupts
        buffer_index = 0;
    }
}

void algorithm(){
    tmr_wait_ms(TIMER4, 7);  // Busy waiting for 7ms
}


int main(void) {
    
    // For testing deadline
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISGbits.TRISG9 = 0;   // G9 pin set as output (LED 2)
    
    // Setup and configuration
    uart_setup();
    spi_setup();
    magnetometer_config();

    // Loop frequency (100Hz)
    tmr_setup_period(TIMER3, 10, 0);
    int tmr_counter = 0;
    
    while(1){
        // Main algorithm (takes 7 ms)
        algorithm();

        // Acquire SPI data (25Hz -> every 40ms)
        if (tmr_counter % 40 == 0){
            acquire_magnetometer_data();
        }

        // Initialize UART Transmission (5Hz -> every 200ms)
        if (tmr_counter % 200 == 0){
            // Calculate the averages
            double x_mag = calculate_average(x_values, ARRAY_SIZE);
            double y_mag = calculate_average(y_values, ARRAY_SIZE);
            double z_mag = calculate_average(z_values, ARRAY_SIZE);
            // Calculate the magnetic north
            double angle = atan2(y_mag, x_mag);
            
            // Insert the new string into the buffer
            memset(buffer_tx, 0, sizeof(buffer_tx)); // Clear buffer
            sprintf(buffer_tx, "$MAG,%.2f,%.2f,%.2f*$YAW,%.2f*", x_mag, y_mag, z_mag, angle);

            // Begin the transmission of the first character
            IEC0bits.U1TXIE = 1;                // Enable UART Interrupt
            while (U1STAbits.UTXBF);            // Wait in case buffer is full
            U1TXREG = buffer_tx[buffer_index];  // Write into buffer
            buffer_index++;                     // Shift the buffer index

            // Zero-out timer counter variable
            tmr_counter = 0;
        }

        // Wait for timer to reach desired frequency (100Hz)
        int ret = tmr_wait_period(TIMER3);
        if (ret) {LATGbits.LATG9 = 1;}  // Turn on LED if flag was already up.
        tmr_counter += 10;

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
    U1STAbits.UTXISEL0 = 0;     // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IEC0bits.U1TXIE = 1;        // Enable UART TX Interrupt
    IFS0bits.U1TXIF = 0;        // clear TX interrupt flag
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
    TRISAbits.TRISA1 = 1; // RA1-RPI17 MISO
    TRISFbits.TRISF12 = 0; //RF12-RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13-RP109 MOSI
    
    // Remapping of SPI1 to pins of Microbus1
    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101; // MOSI (SDO1) - RF13;
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


int spi_write(int addr){
    // Transmit segment
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr;
    // Receive segment
    while (SPI1STATbits.SPIRBF == 0);
    int response = SPI1BUF;
    return response;
}


void magnetometer_config(){
    // 1. Make magnetometer switch to Sleep Mode
    LATDbits.LATD6 = 0;
    spi_write(0x4B);
    spi_write(0x01);
    LATDbits.LATD6 = 1;
    tmr_wait_ms(TIMER1, 2);  // Wait 2ms

    // 2. Make magnetometer go to active mode
    // 3. Configure data rate to 25Hz
    LATDbits.LATD6 = 0;
    spi_write(0x4C);
    spi_write(0b00110000);  // Byte set according to datasheet of BMX055
    LATDbits.LATD6 = 1;
    tmr_wait_ms(TIMER1, 2);  // Wait 2ms
}


void acquire_magnetometer_data(){
    // Begin the reading from the first address that holds magnetic data
    // * We will read all registers until z-axis MSB in one go
    LATDbits.LATD6 = 0;
    int first_addr = 0x42;  
    spi_write(first_addr | 0x80);

    // x-axis magnetic data
    uint8_t x_LSB_byte = spi_write(0x00);
    uint8_t x_MSB_byte = spi_write(0x00);
    int x_value = combine_xy_register_bytes(x_LSB_byte, x_MSB_byte);
    x_values[array_index] = x_value;
    // y-axis magnetic data
    uint8_t y_LSB_byte = spi_write(0x00);
    uint8_t y_MSB_byte = spi_write(0x00);
    int y_value = combine_xy_register_bytes(y_LSB_byte, y_MSB_byte);
    y_values[array_index] = y_value;
    // z-axis magnetic data
    uint8_t z_LSB_byte = spi_write(0x00);
    uint8_t z_MSB_byte = spi_write(0x00);
    // The z-axis data is a 15-bit signed integer, so we use a different function
    int z_value = combine_z_register_bytes(z_LSB_byte, z_MSB_byte);
    z_values[array_index] = z_value;
    
    // Finalize reading
    LATDbits.LATD6 = 1;
    // Change index dynamically
    array_index = (array_index + 1) % ARRAY_SIZE;
}


double calculate_average(int values[], int size){
    double sum = 0.0;
    // Get the sum of all the values in the list
    for(int i=0; i<size; i++){
        sum += values[i];
    }
    // Return the average
    return sum/size;
}


int combine_xy_register_bytes(uint8_t LSB_byte, uint8_t MSB_byte){
    // Mask to clear three least significant bits
    uint8_t mask = 0b11111000;
    uint8_t masked_LSB = LSB_byte & mask;
    // Left-shift MSB_byte by 8 and OR with masked LSB_byte
    int combined_byte = (MSB_byte << 8) | masked_LSB;
    // Divide by 8 to get correct scale
    int final_byte = combined_byte / 8;
    return final_byte;
}


int combine_z_register_bytes(uint8_t LSB_byte, uint8_t MSB_byte){
    // Mask to clear the least significant bit
    uint8_t mask = 0b11111110;
    uint8_t masked_LSB = LSB_byte & mask;
    // Left-shift MSB_byte by 8 and OR with masked LSB_byte
    int combined_byte = (MSB_byte << 8) | masked_LSB;
    // Divide by 2 to get correct scale
    int final_byte = combined_byte / 2;
    return final_byte;
}
