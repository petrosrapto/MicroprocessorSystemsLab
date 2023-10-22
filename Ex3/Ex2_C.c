#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

#define maxValue 255
#define step 0.08
#define start 6 // 50% PWM

int main() {
    DDRB = 0xFF;    // Set PORTB as output
    DDRD = 0x00;    // Set PORTD as input
    TCCR1A = (1<<WGM10) | (1<<COM1A1); // Init control register A of Timer 1
    TCCR1B = (1<<WGM12) | (1<<CS11); // Init control register B of Timer 1
    // with the above values we have fast PWM mode
    
    char table[13] = {0xFA, 0xE5, 0xD1, 0xBC, 0xA8, 0x94, 0x7F, 0x6B, 0x57, 0x42, 0x2E, 0x19, 0x05}; 
    // example OCR1A value for 98% is table[0]
    // keeps the values of OCR1A for desired DCs
    // type == char cause we want 1 byte values
    
    int counter = start;             // keeps the index of table
    OCR1AH = 0x00;                  // values from 0 to 255
    OCR1AL = table[counter];       // init PWM 
    
    while(1) {                      // loop forever 
        if((PIND & 0x02) == 0x00) { // check if button pressed (logical 0)
            do {
                _delay_ms(5);
            } while((PIND & 0x02) == 0x00); // while being pressed wait
            if(counter == 0) continue;   // dont decrease counter if DC = 98%
            OCR1AH = 0x00;                // values from 0 to 255
            OCR1AL = table[--counter];    // decrease counter and fetch OCR1AL value  
        }
        if((PIND & 0x04) == 0x00) { // check if button pressed (logical 0)
            do {
                _delay_ms(5);
            } while((PIND & 0x04) == 0x00); // while being pressed wait
            if(counter == 12) continue;   // dont increase counter if DC = 2%
            OCR1AH = 0x00;               // values from 0 to 255
            OCR1AL = table[++counter];  // increase counter and fetch OCR1AL value
        }
    }
}
