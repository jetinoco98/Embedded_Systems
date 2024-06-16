#include "main.h"

// Buffers
TextCircularBuffer uart_rx;
TextCircularBuffer uart_tx;
NumberCircularBuffer commands;
NumberCircularBuffer times;

// Global Control Variables
int state = 0;
int ongoing_tx = 0;
int ongoing_action = 0;


// TIMER FOR INTERRUPT-1 DEBOUNCING
void __attribute__ ((__interrupt__ , __auto_psv__)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;  // Reset interrupt flag
    IEC0bits.T1IE = 0;  // Disable timer interrupt
    T1CONbits.TON = 0;  // Stop the timer

    state = 1 - state;      // Toggle STATE
    turn_off_all_leds();    // LEDs are turned off
}

// Interrupt for STATE BUTTON (RE8)
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;    // Reset interrupt flag
    IEC0bits.T1IE = 1;      // Enable timer 1 interrupt
    tmr_setup_period(TIMER1, 20, 1);  // For debouncing: 20ms timer
}

// Interrupt for UART TRANSMISSION on Circular Buffers
void __attribute__ ((__interrupt__ , __auto_psv__)) _U1TXInterrupt(void) {
    // Clear TX interrupt flag 
    IFS0bits.U1TXIF = 0;
    ongoing_tx = 1;
    // Loop to write multiple characters into the transmit register
    while (!U1STAbits.UTXBF) {
        // Read the TX Circular Buffer
        char current_char = read_text_buffer(&uart_tx);
        // If buffer is empty: read_text_buffer returns '\0'
        if (current_char == '\0') {
            IEC0bits.U1TXIE = 0;    // Disable further interrupts
            ongoing_tx = 0;
            return;
        }  
        // Write current character into transmit register  
        U1TXREG = current_char;
    }
}

// Interrupt for UART RECEPTION
void __attribute__((__interrupt__ , __auto_psv__)) _U1RXInterrupt(void){
    // Clear RX interrupt flag
    IFS0bits.U1RXIF = 0; 
    // Obtain all possible characters from the register
    while (U1STAbits.URXDA){
        write_text_buffer(&uart_rx, U1RXREG);
    }
}


int main(void) {
    // Inputs and Outputs: LEDs, IR, Vsense, Buttons
    io_setup();
    uart_setup();
    adc_setup();
    pwm_setup();

    // Buffers initialization
    init_text_buffer(&uart_rx);
    init_text_buffer(&uart_tx);
    init_number_buffer(&commands);
    init_number_buffer(&times);
    
    // Local data variables
    double voltage = 0;
    double distance = 100;
    int command = 0;
    int time = 0;
    
    // Local control variables
    int obstacle_detected = 0;
    int deadline_led_timer = 0;

    // Setup: Loop frequency (1KHz -> 1ms)
    tmr_setup_period(TIMER2, 1, 0);
    int tmr_counter = 0;

    // CONTROL LOOP
    while(1){

        //////////////////////////////////////
        // STATE-LESS ACTIONS
        /////////////////////////////////////

        // Check and parse the UART RX buffer
        int result_parse = parse_uart_rx(&uart_rx, &commands, &times);
        parse_acknowledgment(result_parse);

        // Obtain the values from ADC auto converter
        if(AD1CON1bits.DONE){get_adc_values(&voltage, &distance);}
        // Write IR distance on TX buffer (10Hz -> 100ms)
        if(tmr_counter % 100 == 0){write_distance_on_tx(distance);}
        // Write Battery voltage in TX buffer (1 Hz -> 1000ms)
        if (tmr_counter % 1000 == 0){write_voltage_on_tx(voltage);}
        
        // Check for obstacles
        if (distance < 0.2){obstacle_detected = 1; LED_BRAKES = 1;}
        else {obstacle_detected = 0; LED_BRAKES = 0;}
        
        // Initialize the UART TX if no transmission is in progress.
        if (!ongoing_tx){initialize_uart_tx();}
        
        //////////////////////////////////////
        // STATE 0: WAIT FOR START
        /////////////////////////////////////

        if(state == 0){
            // Toggle LEDs at 1Hz (every 1000ms)
            if (tmr_counter % 1000 == 0){toggle_led_group();}
            // Robot is stationary
            move(STOP);
            // Any current action is cancelled
            ongoing_action = 0;     
        }

        //////////////////////////////////////
        // STATE 1: EXECUTE
        /////////////////////////////////////

        if (state == 1){
            // Blink LED A0
            if(tmr_counter % 1000 == 0){LED1 = 1 - LED1;}
            
            // Check for pending commands to perform an action
            if(!ongoing_action){
                command = dequeue_number_buffer(&commands);
                time = dequeue_number_buffer(&times);
                // Only a valid command can be obtained, otherwise is 0.
                if(command > 0){ongoing_action = 1;}
            }
            
            // Obstacle influences execution of action
            if(ongoing_action){
                // Set the PWMs depending on the presence of obstacle
                if(!obstacle_detected){set_action(command); }
                else{move(STOP);}
                // When time runs out, reset variables and STOP car
                if (time <= 0){ongoing_action = 0; move(STOP);}
                // Decreases time by 1ms on each iteration
                time--;
            }  
        }

        //////////////////////////////////////
        // END OF CONTROL LOOP
        /////////////////////////////////////

        // Wait for timer to reach desired frequency (1000Hz)
        int missed_deadline = tmr_wait_period(TIMER2);
        tmr_counter++;  // Increase counter by 1ms
        if(tmr_counter >= 1000){tmr_counter = 0;}   // Reset every second
        
        // TESTING purposes: LED(G9)turns for 100ms on each missed deadline
        if (missed_deadline) {
            deadline_led_timer = 100;
        }
        if (deadline_led_timer > 0){
            LED2 = 1;
            deadline_led_timer--;
        } else {LED2 = 0;}
        
    }
    
}


void io_setup(){
    // Analog pins disabled
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = 0x0000;  

    // Set pins as outputs  
    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)
    TRISBbits.TRISB8 = 0;   // pin B8 set as output (LED LEFT INDICATOR)
    TRISFbits.TRISF1 = 0;   // pin F1 set as output (LED LEFT INDICATOR)
    TRISFbits.TRISF0 = 0;   // pin F0 set as output (LED BRAKES)

    // Initialize values of outputs
    turn_off_all_leds();

    // Battery VSENSE
    ANSELBbits.ANSB11 = 1;  // Set B11 pin as Analog
    TRISBbits.TRISB11 = 1;  // Set B11 pin as input

    // IR Sensor
    ANSELBbits.ANSB15 = 1;
    TRISBbits.TRISB15 = 1;
    // Enable pin on the sensor
    TRISAbits.TRISA3 = 0;   // pin B4 set as output (Enable sensor)
    LATAbits.LATA3 = 1;     // set pin as high

    // Button Interrupts
    TRISEbits.TRISE8 = 1;       // pin E9 set as input (Button 1)
    RPINR0bits.INT1R = 0x58;    // 0x59 is 89 in hex. Refers to PIN RE9/RPI89

    INTCON2bits.GIE = 1;        // Enable global interrupt
    IFS1bits.INT1IF = 0;        // Clear INT1 interrupt flag
    IEC1bits.INT1IE = 1;        // Enable INT1 interrupt
}


void uart_setup(){
    // Remapping Configuration
    RPINR18bits.U1RXR = 0x4B; 
    RPOR0bits.RP64R = 1;
    U1BRG = 468;                // Obtained from (FCY/(16*baud_rate) - 1)
    
    // Interrupts
    IEC0bits.U1RXIE = 1;        // Enable RX interrupt
    
    U1STAbits.UTXISEL0 = 0;     // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;

    IEC0bits.U1TXIE = 1;        // Enable UART TX interrupt
    IFS0bits.U1TXIF = 0;        // clear TX interrupt flag
    U1MODEbits.UARTEN = 1;      // Enable UART
    U1STAbits.UTXEN = 1;        // Enable U1TX
}


void adc_setup(){
    // Setup ADC: Automatic sampling & conversion
    AD1CON3bits.ADCS = 8;       // ADC Conversion Clock Select bits
    AD1CON1bits.ASAM = 1;       // Sampling begins when SAMP bit is set -> 1: Automatic
    AD1CON3bits.SAMC = 16;      // Sample time 16 Tad
    AD1CON1bits.SSRC = 7;       // Conversion starts automatically -> 7: Automatic
    AD1CON2bits.VCFG = 0;       // Reference Voltage
    
    AD1CON2bits.CHPS = 0;       // Channel selection -> CH0
    AD1CON1bits.SIMSAM = 0;     // Sequential sampling
    AD1CON2bits.SMPI = 1;       // Interrupt after (n+1) number of sample/conversion operations
    
    // Automatic Scanning: Select Analog pins
    AD1CON2bits.CSCNA = 1;
    AD1CSSLbits.CSS11 = 1;  // Vsense
    AD1CSSLbits.CSS15 = 1;  // IR
            
    AD1CON1bits.ADON = 1;   // Turn ON ADC  
}


void turn_off_all_leds(){
    LED1 = 0;
    LED2 = 0;
    LED_LEFT = 0;
    LED_RIGHT = 0;
    LED_BRAKES = 0;
}


void toggle_led_group(){
    LED1 = 1 - LED1;
    LED_LEFT = 1 - LED_LEFT;
    LED_RIGHT = 1 - LED_RIGHT;
}


void parse_acknowledgment(int result_parse){
    // Acknowledgment of parsed command
    char buffer_ack_1[] = "$MACK,1*";
    char buffer_ack_0[] = "$MACK,0*";

    // Write to UART TX Circular Buffer: Match found
    if (result_parse == 1){
        int i;
        for(i=0; i<9; i++){
            write_text_buffer(&uart_tx, buffer_ack_1[i]);
        }
    }

    // Write to UART TX Circular Buffer: Full FIFO
    if (result_parse == 2){
        int i;
        for(i=0; i<9; i++){
            write_text_buffer(&uart_tx, buffer_ack_0[i]);
        }
    }
}


void initialize_uart_tx(){
    char current_tx_char = read_text_buffer(&uart_tx);
    if (current_tx_char != '\0'){
        IEC0bits.U1TXIE = 1;            // Enable UART Interrupt
        while (U1STAbits.UTXBF);        // Wait in case buffer is full
        U1TXREG = current_tx_char;      // Write into buffer
    }
}


void get_adc_values(double *voltage, double *distance){
    // Read ADC: Automatic sampling & conversion
    int ADC_value_sensor = ADC1BUF1;    // Read the conversion result
    int ADC_value_bat = ADC1BUF0;       // Read the conversion result

    // Convert the result to battery voltage
    double vsense = (double) ADC_value_bat * 3.3 / 1023;
    *voltage = vsense * 3;    // Due to voltage divider

    // Convert the result distance
    double dist_voltage = (double) ADC_value_sensor * 3.3 / 1023;
    *distance = 2.34 - 
        4.74 * dist_voltage + 
        4.06 * pow(dist_voltage, 2) - 
        1.60 * pow(dist_voltage, 3) + 
        0.24 * pow(dist_voltage, 4);
}


void write_distance_on_tx(double distance){
    char buffer_dist[11];
    sprintf(buffer_dist, "$MDIST,%.2f*", distance);
    // Write to UART TX Circular Buffer
    int i;
    for(i=0; i<12; i++){
        write_text_buffer(&uart_tx, buffer_dist[i]);
    }
}


void write_voltage_on_tx(double voltage){
    char buffer_batt[11];
    sprintf(buffer_batt, "$MBATT,%.2f*", voltage);
    // Write to UART TX Circular Buffer
    int i;
    for(i=0; i<12; i++){
        write_text_buffer(&uart_tx, buffer_batt[i]);
    }
}


void set_action(int command){
    if (command == 1){move(FORWARD);}
    else if (command == 2){move(LEFT);}
    else if (command == 3){move(RIGHT);}
    else if (command == 4){move(BACKWARDS);}
    else {move(STOP);}
}
