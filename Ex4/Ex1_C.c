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
    _delay_ms(40);              // lcd init procedure
    PORTD = 0x30 | (1<<PD3);    // 8 bit mode
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
    _delay_us(100);
    PORTD = 0x30 | (1<<PD3);    // 8 bit mode
    PORTD &= (0xFF) & (0<<PD3); // set PD3 to zero, lcd enable pulse 
    _delay_us(100);
    PORTD = 0x20 | (1<<PD3);    // change to 4 bit mode
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
    
    char temp = vin; // integer part of vin
    temp += 0x30;
    lcd_data(temp);
    _delay_ms(100); 
    
    lcd_data(0x2E); // dot
    _delay_ms(100); 
    
    temp = ((int)(vin*10)%10);  // 1st decimal
    temp += 0x30;
    lcd_data(temp);
    _delay_ms(100); 
    
    temp = ((int)(vin*100)%10); // 2nd decimal
    temp += 0x30;
    lcd_data(temp);
    _delay_ms(100);  
}

int main() {
    int counter = 0;
    DDRD = 0xFF;    // set portD as output
    DDRB = 0xFF;    // set portB as output
    DDRC = 0x00;    // set portB as input
    
    // REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0010 => select ADC2(pin PC2),
    // ADLAR=0 => Right adjust the ADC result
    ADMUX = 0b01000010; 
    // ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
    // ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10000111;
    lcd_init();     // init lcd
    _delay_ms(2);   // wait for lcd init
    
    while(1) {
        ADCSRA |= (1<<ADSC); // Set ADSC flag of ADCSRA
                             // enable conversion
        counter++;          // increase counter
        PORTB = counter;    // output counter
        while((ADCSRA & (1<<ADSC)) == (1<<ADSC)); 
        // wait until flags become zero
        // that means that the conversion is complete
        read_adc(); 
        
        _delay_ms(1000);    
        lcd_command(0x01);  // clear display
        _delay_us(5000);
    }
     
}
