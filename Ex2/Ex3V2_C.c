#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

long int flag; // consecutive interrupt counter

ISR (INT1_vect, ISR_NAKED) {
    // ISR_NAKED is passed as parameter to ISR macro in order the compiler
    // not to save the register state to the stack and not to return to 
    // the stored Program Counter
    asm volatile("pop __tmp_reg__"  "\n\t"
    "pop __tmp_reg__"  "\n\t" ::); // pop the stored program counter
    // PC is consisted of 13 bits, thus two 8bit spots at the stack
    // Note: asm volatile() adds inline assembly to the program
    
    do {    // bouncing effect
        EIFR = (1<<INTF1); // clear interrupt pin
        _delay_ms(100);    // delay a small amount of time
    } while((EIFR & 0x02) == 0x02); // check if interrupt pin
                                    // is changed
    // flag keeps count of the consecutive interrupts happened in the past
    if (flag++ != 0) {              // check the interrupt counter and increase
        sei();                      // enable interrupts inside the interrupt routine
        PORTB = 0xFF;               // light up all the leds   
        _delay_ms(500);             // delay for 0.5 sec
        PORTB = 0x01;               // light up the lamp
        _delay_ms(350);             // delay for the remaining 3.5 secs
        // changed to 350 from 3500
    } else {                        // that's the first interrupt
        sei();                      // enable interrupts
        PORTB = 0x01;               // light up the lamp
        _delay_ms(400);             // delay 4 secs
        // changed to 400 from 4000
    }
    PORTB = 0x00;                   // lights off
    flag = 0;                       // set the interrupt counter to zero    
    // sei again needed?
    asm volatile("rjmp 1f;"::);     // jump to main
}

int main() {
    // Interrupt on rising edge of INT1 pin
    EICRA = (1 << ISC11) | (1 << ISC10);
    // Enable the INT1 interrupt (PD3)
    EIMSK = (1 << INT1);
    sei();  // Enable global interrupts
    DDRB = 0xFF;    // Set PORTB as output
    flag = 0;
    
    asm volatile("1:"::);           // set the main label 
    while(1) ;                      // loop for ever waiting for
                                    // an interrupt
}


