#ifndef TIMER_H
#define	TIMER_H

#include <xc.h> // include processor files - each processor file is guarded.

#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4

#define MAX_VALUE_16_BITS 65535
#define FCY_PROP 72000  // Proportional value: FCY/1000

void tmr_wait_ms(int timer, int ms);
void tmr_setup_period(int timer, int ms, int has_interrupt);
int tmr_wait_period(int timer);
void tmr_set_register_values(int timer, int pr_value, int prescaler_bits, int has_interrupt);

#endif