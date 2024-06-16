// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.

#define TIMER1 1
#define TIMER2 2

#define MAX_VALUE_16_BITS 65535
#define FCY_PROP 72000  // Proportional value (FCY/1000) to avoid overflows
#define MAX_ARRAY_SIZE 100

void tmr_wait_ms(int timer, int ms);
void tmr_setup_period(int timer, int ms);
int tmr_wait_period(int timer);

#endif