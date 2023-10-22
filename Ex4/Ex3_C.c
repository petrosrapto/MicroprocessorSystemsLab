#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

// send one byte divided into 2 (4 bit) parts
void write_2_nibbles(char data) {
    char pinState = PIND; // read 4 LSB and resend them
			  // in order not to alter any previous state
    PORTD = (pinState & 0x0F) | (data & 0xF0) | (1<<PD3); // send 4MSB
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
    PORTD = (pinState & 0x0F) | ((data<<4) & 0xF0) | (1<<PD3); // send 4LSB
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
}

// send one byte of data to the lcd display
void lcd_data(char data) {
    PORTD |= (1<<PD2);  // select data register
    write_2_nibbles(data);
    _delay_us(100);
}

// send one byte of instruction to the lcd display
void lcd_command(char data) {
    PORTD &= (0xFF) & (0<<PD2);  // select command register
    write_2_nibbles(data);
    _delay_us(100);
}

void lcd_init() {
    _delay_ms(40);  // lcd init procedure
    PORTD = 0x30 | (1<<PD3); // 8 bit mode
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
    _delay_us(100);
    PORTD = 0x30 | (1<<PD3); // 8 bit mode
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
    _delay_us(100);
    PORTD = 0x20 | (1<<PD3); // change to 4 bit mode
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
    _delay_us(100);
    lcd_command(0x28);	// select character size 5x8 dots and two line display
    lcd_command(0x28);	
    lcd_command(0x0c);	// enable lcd, hide cursor
    lcd_command(0x01);	// clear display
    _delay_us(5000);
    lcd_command(0x06);	// enable auto increment of address, disable shift of the display
}

ISR (ADC_vect) {
    int adc = ADC;
    double vin = adc*5.0/1024; 
    
    lcd_command(0b11000000); //write to second line ADC result (DDRAM Address 0x40)
    _delay_us(5000);
    lcd_command(0b11000000); //write to second line ADC result (DDRAM Address 0x40)
    _delay_us(5000);
    
    char temp = vin;	// integer part of vin
    temp += 0x30;
    lcd_data(temp);
    _delay_ms(100); 
    
    lcd_data(0x2E); // dot
    _delay_ms(100); 
    
    temp = ((int)(vin*10)%10);		// 1st decimal
    temp += 0x30;
    lcd_data(temp);
    _delay_ms(100); 
    
    temp = ((int)(vin*100)%10);		// 2nd decimal
    temp += 0x30;
    lcd_data(temp);
    _delay_ms(100); 
    
}

int main() {
    DDRB = 0b00000010;    // set PORTB0,2-7 as input, PORTB1 output
    DDRD = 0xFF;    // Set PORTD as output
    TCCR1A = (0<<WGM10) | (1<<WGM11) | (1<<COM1A1); // Init control register A of Timer 1
    TCCR1B = (1<<WGM12) | (1<<WGM13);               // Init control register B of Timer 1
    // with the above values we have fast PWM mode with TOP = ICR1
    
    // REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC1(pin PC1),
    // ADLAR=0 => Right adjust the ADC result
    ADMUX = 0b01000001;
    
    // ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
    // ADIE=1 => enable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10001111;
    sei();  // enable interrupts
    lcd_init(); 	// init lcd
    _delay_ms(2);	// wait for lcd init
    
    while(1) {                                       // loop forever 
        TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10); // when no buttons are pressed
        // we stop PWM setting timer's frequency to zero
        // we want to change only the CS12, CS11, CS10 bits of the register
        
        lcd_command(0x01);      //clear screen
        _delay_us(5000);
        
        // formula: top(fpwm, N) = fclk/(N*fpwm) - 1, fpwm = 5k, N = 64, top = 49
        if ((PINB & 0x04) == 0x00) { // check if PB2 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (1<<CS11) | (1<<CS10);    // N = 64
            ICR1H = 0;				//ICR=TOP
            ICR1L = 49;
            OCR1AH = 0x00;                    
            OCR1AL = 10;                     // DC=20%
            
            lcd_command(0b10000000); //write to first line DC%
            _delay_us(5000);
            lcd_data(0x32);		// 2
            _delay_ms(100); 
            lcd_data(0x30);		// 0
            _delay_ms(100);
            lcd_data(0b00100101);	// %
            _delay_ms(100);
            
            while((PINB & 0x04) == 0x00) {
                ADCSRA |= (1<<ADSC); // Set ADSC flag of ADCSRA
                _delay_ms(10);
            }
                
        }
        else if ((PINB & 0x08) == 0x00) {          // check if PB3 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (1<<CS11) | (1<<CS10);
            ICR1H = 0;
            ICR1L = 49; 
            OCR1AH = 0x00;                   
            OCR1AL = 20;                     // DC=40%
            
            lcd_command(0b10000000); //write to first line DC%
            _delay_us(5000);
            lcd_data(0x34);
            _delay_ms(100); 
            lcd_data(0x30);
            _delay_ms(100);
            lcd_data(0b00100101);
            _delay_ms(100);
            
            while((PINB & 0x08) == 0x00) {
                ADCSRA |= (1<<ADSC); // Set ADSC flag of ADCSRA
                _delay_ms(10);
            }
        }
        else if ((PINB & 0x10) == 0x00) {          // check if PB4 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (1<<CS11) | (1<<CS10);
            ICR1H = 0;				//ICR=TOP
            ICR1L = 49;            
            OCR1AH = 0x00;                    
            OCR1AL = 30;                     // DC=60%
            
            lcd_command(0b10000000); //write to first line DC%
            _delay_us(5000);
            lcd_data(0x36);
            _delay_ms(100); 
            lcd_data(0x30);
            _delay_ms(100);
            lcd_data(0b00100101);
            _delay_ms(100);
            
            while((PINB & 0x10) == 0x00) {
                ADCSRA |= (1<<ADSC); // Set ADSC flag of ADCSRA 
                _delay_ms(10);
            }
        }
        else if ((PINB & 0x20) == 0x00) {          // check if PB5 pressed (logical 0)
            TCCR1B = (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (1<<CS11) | (1<<CS10);
            ICR1H = 0;
            ICR1L = 49;
            OCR1AH = 0x00;                    // values from 0 to 255
            OCR1AL = 40;                     // DC=80%   
            
            lcd_command(0b10000000); //write to first line DC%
            _delay_us(5000);
            lcd_data(0x38);
            _delay_ms(100); 
            lcd_data(0x30);
            _delay_ms(100);
            lcd_data(0b00100101);
            _delay_ms(100);
            
            while((PINB & 0x20) == 0x00) {
                ADCSRA |= (1<<ADSC); // Set ADSC flag of ADCSRA 
                _delay_ms(10);
            }
        }
    }
}
