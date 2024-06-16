// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.

#define TIMER1 1
#define TIMER2 2

#define MAX_VALUE_16_BITS 65535
#define FCY_PROP 72000

void tmr1_setup_period(int ms);
void tmr2_wait_ms(int ms);

#endif