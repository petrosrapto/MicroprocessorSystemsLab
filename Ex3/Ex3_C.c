#define F_CPU 16000000UL
#include <avr/io.h>

int main() {
    DDRB = 0xFF;    // Set PORTB as output
    DDRD = 0x00;    // Set PORTD as input
    TCCR1A = (0<<WGM10) | (1<<WGM11) | (1<<COM1A1); // Init control register A of Timer 1
    TCCR1B = (1<<WGM12) | (1<<WGM13);               // Init control register B of Timer 1
    // with the above values we have fast PWM mode with TOP = ICR1
    
    while(1) {                                       // loop forever 
        TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10); // when no buttons are pressed
        // we stop PWM setting timer's frequency to zero
        // we want to change only the CS12, CS11, CS10 bits of the register
        
        // formula: top(fpwm, N) = fclk/(N*fpwm) - 1
        if ((PIND & 0x01) == 0x00) { // check if PD0 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS12) | (0<<CS11) | (1<<CS10);    // N = 1024
            ICR1H = 0;				//ICR=TOP
            ICR1L = 124;
            OCR1AH = 0x00;                    // values from 0 to 255
            OCR1AL = 62;                     // set OCR1AL = ICR/2, DC=50%
            while((PIND & 0x01) == 0x00);     // while being pressed PWM with 125Hz
        }
        else if ((PIND & 0x02) == 0x00) {          // check if PD1 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS12) | (0<<CS11) | (0<<CS10);    // N = 256
            ICR1H = 0;
            ICR1L = 249; 
            OCR1AH = 0x00;                    // values from 0 to 255
            OCR1AL = 124;                     // set OCR1AL = TOP   round of 124.5
            while((PIND & 0x02) == 0x00);     // while being pressed PWM with 250Hz
        }
        else if ((PIND & 0x04) == 0x00) {          // check if PD2 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS12) | (0<<CS11) | (0<<CS10);    // N = 256
            ICR1H = 0;				//ICR=TOP
            ICR1L = 124;            
            OCR1AH = 0x00;                    // values from 0 to 255
            OCR1AL = 62;                     // set OCR1AL = TOP   round of 62.5
            while((PIND & 0x04) == 0x00);     // while being pressed PWM with 500Hz
        }
        else if ((PIND & 0x08) == 0x00) {          // check if PD3 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (1<<CS11) | (1<<CS10);    // N = 64
            ICR1H = 0;
            ICR1L = 249;
            OCR1AH = 0x00;                    // values from 0 to 255
            OCR1AL = 124;                     // set OCR1AL = TOP   
            while((PIND & 0x08) == 0x00);     // while being pressed PWM with 1000Hz
        }
    }
}