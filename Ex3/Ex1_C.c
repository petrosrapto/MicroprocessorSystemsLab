#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TIMER_DELAY_S  4		    // we want a delay of 4 sec   
#define OVERFLOW_POINT 65535        // overflow point of timer (16 bits)  
#define TIMER_FREQ F_CPU/1024	    // cause of the value of TCCR1B set below
#define DEL_NU_TIMER OVERFLOW_POINT - TIMER_FREQ*TIMER_DELAY_S // find the total cycles of timer's delay
// when an overflow takes place an timer interrupt is fired

long int flag; // consecutive interrupt counter

void leds() {
    
    TCCR1B = (1<<CS12) | (0<<CS11) | (1<<CS10);
    // Set frequency of timer's increase to fclock/1024
    
    // set timer to 4 sec
    TCNT1 = DEL_NU_TIMER;
    if (flag++ != 0) {              // check the interrupt counter and increase
        PORTB = 0xFF;               // light up all the leds, that's not the first interrupt
        _delay_ms(500);             // delay for 0.5 sec
    } 
    PORTB = 0x01;               // light up the lamp 
}

ISR (INT1_vect) {
    do {    // bouncing effect
        EIFR = (1<<INTF1); // clear interrupt pin
        _delay_ms(100);    // delay a small amount of time
    } while((EIFR & 0x02) == 0x02); // check if interrupt pin
                                    // is changed
    leds();
}

ISR (TIMER1_OVF_vect) {
    // Stop the time counter
    TCCR1B = (0<<CS12) | (0<<CS11) | (0<<CS10);
    PORTB = 0x00;   // lights off
    flag = 0;       // interrupts are handled
}

int main() {
    // Interrupt on rising edge of INT1 pin
    EICRA = (1 << ISC11) | (1 << ISC10);
    // Enable the INT1 interrupt (PD3)
    EIMSK = (1 << INT1);
    DDRB = 0xFF;    // Set PORTB as output
    DDRC = 0x00;    // Set PORTC as input
    DDRD = 0x00;    // Set PORTD as input
    TIMSK1 = (1<<TOIE1); // Enable TCNT1 overflow timer interrupt
    TCCR1B = (0<<CS12) | (0<<CS11) | (0<<CS10); // Stop the time counter
    flag = 0;
    sei();  // Enable global interrupts
    
    while(1) {                      // loop for ever 
        if((PINC & 0x20) == 0x00) { // check if button pressed (logical 0)
            do { _delay_ms(100); }  // bouncing or button unpressed
            while((PINC & 0x20) == 0x00);
            leds();
        }
    }
}
