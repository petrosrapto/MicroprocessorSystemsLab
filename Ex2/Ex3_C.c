#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

long int flag;

ISR (INT1_vect) {
    do {
        EIFR = (1<<INTF1);
        _delay_ms(100);
    } while((EIFR & 0x02) == 0x02);
        
    if (flag++ != 0) {
        sei();
        PORTB = 0xFF;
        _delay_ms(500); 
        PORTB = 0x01;
        _delay_ms(3500); 
        PORTB = 0x00;
        flag = 0;
    }
    else {
        sei();
        PORTB = 0x01;
        _delay_ms(4000); 
        PORTB = 0x00;
        flag = 0;
    }
}

int main() {
    // Interrupt on riging edge of INT1 pin
    EICRA = (1 << ISC11) | (1 << ISC10);
    // Enable the INT1 interrupt (PD3)
    EIMSK = (1 << INT1);
    sei();  // Enable global interrupts
    DDRB = 0xFF;    // Set PORTB as output
    flag = 0;
    while(1);
}

