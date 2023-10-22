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

// send one byte of instuction to the lcd display
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
    lcd_command(0x28); // select character size 5x8 dots and two line display
    lcd_command(0x0c); // enable lcd, hide cursor
    lcd_command(0x01); // clear display
    _delay_us(5000);
    lcd_command(0x06); // enable auto increment of address, disable shift of the display
}

void read_adc () {
    int adc = ADC;
    double vin = adc*5.0/1024; 
    
    // Cx: gas concentration
    // Cx = (Vgas - Vgas0)/M, Vgas0 = 0.1V
    // M = Sensitivity code*TIA Gain*10^(-9)*10^(3)
    // Sensitivity code = 129nA/ppm
    // TIA Gain = 100(kV/A)
    // So for Cx = 70ppm we have approx. Vgas = 1V
    // 1st Gas Level: Vgas in [0.1...1)
    // 2nd Gas Level: Vgas in [1...2)
    // 3rd Gas Level: Vgas in [2...3)
    // 4th Gas Level: Vgas in [3...4)
    // 5th Gas Level: Vgas in [4...4.5)
    // 6th Gas Level: Vgas in [4.9...5]
    
    if (vin < 1) {
        PORTB = 0x01; // first led on
        lcd_data('C');
        _delay_ms(10); // delay time decreased from 100 to 10
        lcd_data('L');
        _delay_ms(10);
        lcd_data('E');
        _delay_ms(10);
        lcd_data('A');
        _delay_ms(10);
        lcd_data('R');
        _delay_ms(10);
    } else {
        // light up the leds
        if (vin < 2) PORTB = 0x02;
        else if (vin < 3) PORTB = 0x04;
        else if (vin < 4) PORTB = 0x08;
        else if (vin < 4.9) PORTB = 0x10;
        else PORTB = 0x20;
        lcd_data('G');
        _delay_ms(10); // delay time decreased from 100 to 10
        lcd_data('A');
        _delay_ms(10);
        lcd_data('S');
        _delay_ms(10);
        lcd_data(' ');
        _delay_ms(10);
        lcd_data('D');
        _delay_ms(10);
        lcd_data('E');
        _delay_ms(10);
        lcd_data('T');
        _delay_ms(10);
        lcd_data('E');
        _delay_ms(10);
        lcd_data('C');
        _delay_ms(10);
        lcd_data('T');
        _delay_ms(10);
        lcd_data('E');
        _delay_ms(10);
        lcd_data('D');
        _delay_ms(10);
        PORTB = 0x00; // lights off
    }
}

int main() {
    DDRD = 0xFF;    // set portD as output
    DDRB = 0xFF;    // set portB as output
    DDRC = 0x00;    // set portB as input
    
    // REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0011 => select ADC3(pin PC3),
    // ADLAR=0 => Right adjust the ADC result
    ADMUX = 0b01000011; 
    // ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
    // ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10000111;
    lcd_init(); // init lcd
    _delay_ms(2);   // wait for lcd init
    
    while(1) {
        ADCSRA |= (1<<ADSC); // Set ADSC flag of ADCSRA
                             // enable conversion
        while((ADCSRA & (1<<ADSC)) == (1<<ADSC)); 
        // wait until flags become zero
        // that means that the conversion is complete
        read_adc();
        
        _delay_ms(100);
        lcd_command(0x01); // clear display
        _delay_us(5000);
    }
     
}
