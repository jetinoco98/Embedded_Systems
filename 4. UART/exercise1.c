/*
 * File:   main.c
 * Author: jetin
 *
 * Created on April 4, 2024, 11:45 AM
 */

#include "config.h"

char UART_read() {
    while (!U1STAbits.URXDA); // Wait until data is available
    return U1RXREG; // Read character
}

void UART_write(char data) {
    while (U1STAbits.UTXBF); // Wait while buffer is full
    U1TXREG = data; // Transmit character
}

int main(void) {
    
    // Configure the UART1 to use the MikroBUS 2.
    U1BRG = 468;                // Obtained from (FCY/(16*baud_rate) - 1)
    U1MODEbits.UARTEN = 1;      // Enable UART
    U1STAbits.UTXEN = 1;        // Enable U1TX
    
    // Remapping of UART1 to pins of MicroBus 2
    RPINR18bits.U1RXR = 0x4B;
    RPOR0bits.RP64R = 1;
    
    while(1){
        char received_char = UART_read();   // Read character from UART1
        UART_write(received_char);          // Send character back to UART1
    }
    
    return 0;
}