//
//  File: millis.c
//  Project: yafocuser
//
//  Created by Sven Kreiensen on 06.11.19.
//  Copyright © 2019 Sven Kreiensen. All rights reserved.
//
#include <avr/io.h>
#include <util/atomic.h>
#include <avr/interrupt.h>

volatile unsigned long timer1_millis;

ISR(TIMER1_COMPA_vect) {
    timer1_millis++;  
}

void init_millis(unsigned long f_cpu) {
    uint32_t ctc_match_overflow;

    ctc_match_overflow = ((f_cpu / 1000) / 8); //when timer1 is this value, 1ms has passed

    // (Set timer to clear when matching ctc_match_overflow) | (Set clock divisor to 8)
    TCCR1B |= (1 << WGM12) | (1 << CS11);

    // high byte first, then low byte
    OCR1AH = (ctc_match_overflow >> 8);
    OCR1AL = ctc_match_overflow;

    // Enable the compare match interrupt
    TIMSK1 |= (1 << OCIE1A);

    //REMEMBER TO ENABLE GLOBAL INTERRUPTS AFTER THIS WITH sei(); !!!
}

uint32_t millis(void) {
    uint32_t millis_return;

    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        millis_return = timer1_millis;
    }

    return millis_return;
} 

uint32_t secs(void) {
    uint32_t secs_return;

    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        secs_return = timer1_millis / 1000;
    }

    return secs_return;
} 

