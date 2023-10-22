.include "m328PBdef.inc"
    
.equ FOSC_MHZ = 16	  ; Microcontroller operating frequency in MHz
 
.EQU PD3 = 3
.EQU PD2 = 2
 
.DEF temp2=r18
.DEF counter=r21	
.DEF temp=r22	
.DEF argL=r24		  ; used as argument 
.DEF argH=r25		  ; used as argument     
    
.def ADC_L = r19
.def ADC_H = r20
    
.org 0x0
rjmp reset

.org 0x2A		    ;ADC Conversion Complete Interrupt
rjmp ISR_ADC
    
reset:
    ldi temp, LOW(RAMEND) ;Initialize stack pointer
    out SPL, temp
    ldi temp, HIGH(RAMEND)
    out SPH, temp
    
    ser temp
    out DDRD, temp ; init PORTD (connected to lcd) as output
    out DDRB, temp ; set PORTB as output
 
    clr temp
    out DDRC, temp ;Set PORTC as input
    
    clr counter	   ; set counter to zero
    
    ; REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0010 => select ADC2(pin PC2),
    ; ADLAR=1 => Left adjust the ADC result
    ldi temp, 0b01100010 ; 
    sts ADMUX, temp
    
    ; ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
    ; ADIE=1 => enable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ldi temp, 0b10001111
    sts ADCSRA, temp
    sei ; enable global interrupts
    
    rcall lcd_init ; init lcd display
    ldi argL, low(2*FOSC_MHZ)
    ldi argH, high(2*FOSC_MHZ) ; wait 2 msec
    rcall wait_msec
    
main:
    
    lds temp, ADCSRA 
    ori temp, (1<<ADSC) ; Set ADSC flag of ADCSRA
    sts ADCSRA, temp	; in order to start conversion
    inc counter		; increase counter
    out PORTB, counter	; counter to PORTB
    
    ldi r24, low(1000*FOSC_MHZ)	; wait 1 sec
    ldi r25, high(1000*FOSC_MHZ)
    rcall wait_msec
    
    ldi temp ,0x01	; clear display
    rcall lcd_command
    ldi r24 ,low(5000*FOSC_MHZ)
    ldi r25 ,high(5000*FOSC_MHZ)
    rcall wait_usec 
    
    rjmp main		; loop forever

; -------------------- DELAY FUNCTIONS ------------------------------------
	
; delay of 4*F1 + 6 cycles (almost equal to F1 cycles)
wait_usec:
    sbiw r24, 1     ; 2 cycles
    brne wait_usec  ; 1 or 2 cycles
    ret		    ; 4 cycles
    
; delay of 1000*F1 + 6 cycles (almost equal to 1000*F1 cycles)
wait_msec:
; total delay of next 4 instruction group = 1+(249*4-1) = 996 cycles
    ldi r23, 249
loop_inn:
    dec r23         ; 1 cycle
    nop	      ; 1 cycle
    brne loop_inn   ; 1 or 2 cycles
    sbiw r24, 1     ; 2 cycles
    brne wait_msec   ; 1 or 2 cycles
    ret	      ; 4 cycles
    
; ----------------------------------------------------------------------    

; send one byte divided into 2 (4 bit) parts
write_2_nibbles:	; uses register r24 as argument
    push r24		; send the 4 MSB
    in r25 ,PIND	; read 4 LSB and resend them
    andi r25 ,0x0f	; in order not to alter any previous state
    andi r24 ,0xf0	; isolate the 4 MSB
    add r24 ,r25	; combine them with the preexisting 4 LSB
    out PORTD ,r24	; send them to output
    sbi PORTD ,PD3	; create enable pulse at pin PD3
    cbi PORTD ,PD3	; PD3=1 and after PD3=0
    nop			; do nothing (for delay purposes)   
    nop			; do nothing
    pop r24		; send the 4 LSB. restore saved byte
    swap r24		; swap 4 MSB with 4 LSB
    andi r24 ,0xf0	; send them
    add r24 ,r25
    out PORTD ,r24
    sbi PORTD ,PD3	; new Enable pulse
    cbi PORTD ,PD3
    nop			; do nothing
    nop			; do nothing
    ret
    
; send one byte of data to the lcd display. uses register r24 as argument
lcd_data:
    sbi PORTD ,PD2	    ; select data register (PD2=1)
    rcall write_2_nibbles   ; send byte
    ldi r24, low(100*FOSC_MHZ) ; wait 100?s in order the lcd to receive 
    ldi r25 ,high(100*FOSC_MHZ) ; the data successfully
    rcall wait_usec
    ret

; send one byte of instuction to the lcd display. uses register r24 as argument
lcd_command:
    cbi PORTD ,PD2	    ; select instuction register (PD2=0)
    rcall write_2_nibbles   ; send byte
    ldi r24 ,low(100*FOSC_MHZ) ;  wait 100?s in order the lcd to implement
    ldi r25 ,high(100*FOSC_MHZ) ; the instruction successfully 
    rcall wait_usec ; Note: "clear display" and "return home" instructions
    ret		    ; need more time
    
lcd_init: 
    ldi r24 ,low(40*FOSC_MHZ)  ; when we power on the lcd, it begins 
    ldi r25 ,high(40*FOSC_MHZ) ; its own initialization sequence
    rcall wait_msec ; wait 40 msec until the initialization is complete
    
    ldi r24 ,0x30  ; instruction to transition to 8 bit mode
    out PORTD ,r24 ; send the instruction twice
    sbi PORTD ,PD3 ; because we can not be sure whether at first the
    cbi PORTD ,PD3 ; input mode of the lcd is 4 bit or 8 bit.
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ) ; if lcd controller was at 8-bit mode
    rcall wait_usec ; nothing happens, but if it had input mode 4-bit
     ; it will transition to 8 bit
     
    ldi r24 ,0x30 
    out PORTD ,r24
    sbi PORTD ,PD3
    cbi PORTD ,PD3
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ)
    rcall wait_usec 
    
    ldi r24 ,0x20 ; transition to 4-bit mode
    out PORTD ,r24
    sbi PORTD ,PD3
    cbi PORTD ,PD3
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ)
    rcall wait_usec 
    
    ldi r24 ,0x28     ; select character size 5x8 dots
    rcall lcd_command ; and two line display 
    
    ldi r24 ,0x0c     ; enable lcd, hide cursor
    rcall lcd_command 
    
    ldi r24 ,0x01     ; clear display
    rcall lcd_command
    ldi r24 ,low(5000*FOSC_MHZ)
    ldi r25 ,high(5000*FOSC_MHZ)
    rcall wait_usec 
    
    ldi r24 ,0x06     ; enable auto increment of address 
    rcall lcd_command ; disable shift of the display
    ret
    
ISR_ADC:
    ; interrupt routine for ADC
    push r24	    ; save r24
    push r25	    ; save r25
    push temp	    ; save temp
    in temp, SREG ; save SREG
    push temp
    
    lds ADC_L,ADCL ; Read ADC result(Left adjusted)
    lds ADC_H,ADCH
    
    mov temp, ADC_H ; hold copy of ADC data
    mov temp2, ADC_L
    
    ; we want to multiply ADC data with 5
    ; in order to do this shift ADC 2 times to the right
    ; and add that to the original ADC data
    ; keep the 3 MSB (the carry and the 2MSB of temp)
    
    ror temp	   ; rotate ADC_H, ADC_H(0)->C
    ror temp2	   ; rotate ADC_L, C->ADC_L(7)
    ror temp	   ; rotate ADC_H, ADC_H(0)->C
    ror temp2	   ; rotate ADC_L, C->ADC_L(7)
    andi temp, 0x3F  ; isolate desired bits
    andi temp2, 0xF0
    add temp2, ADC_L 
    adc temp, ADC_H
    ; now the carry and the 2 MSB of temp holds
    ; the integer part of the Vin
    mov ADC_L, temp2 ; hold copy of the values
    mov ADC_H, temp
    rol temp
    rol temp
    rol temp
    andi temp, 0x07 ; now the 3LSB of temp hold
		    ; the integer part of the division
    ; lcd code for 0: 0x30, for 1:0x31...
    subi temp, -(0x30) ; addi temp, 0x30
    mov r24, temp   ; argument to lcd_data
    rcall lcd_data  ; send one byte of data to the lcd's controller
    
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ)
    rcall wait_msec 
    
    ldi r24, 0x2E  ; display dot
    rcall lcd_data
    
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ)
    rcall wait_msec 
    
     ; ---------------- 1st decimal -----------------------
    
    andi ADC_H, 0x3F ; remove the integer part of Vin
    mov temp2, ADC_L ; fetch copy of the values
    mov temp, ADC_H
  
    ror temp	   ; rotate ADC_H, ADC_H(0)->C
    ror temp2	   ; rotate ADC_L, C->ADC_L(7)
    ror temp	   ; rotate ADC_H, ADC_H(0)->C
    ror temp2	   ; rotate ADC_L, C->ADC_L(7)
    andi temp, 0x0F  ; isolate desired bits
    andi temp2, 0xFC
    add temp2, ADC_L 
    adc temp, ADC_H
    mov ADC_L, temp2 ; hold copy of the values
    mov ADC_H, temp
    
    ror temp
    ror temp 
    ror temp
    andi temp, 0x0F
    subi temp, -(0x30) ; addi temp, 0x30
    mov r24, temp   ; argument to lcd_data
    rcall lcd_data ; send one byte of data to the lcd's controller
    
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ)
    rcall wait_msec 
    
    ; ---------------- 2nd decimal -----------------------
    andi ADC_H, 0x07 ; remove the integer part of Vin
    mov temp2, ADC_L ; fetch copy of the values
    mov temp, ADC_H
  
    ror temp	   ; rotate ADC_H, ADC_H(0)->C
    ror temp2	   ; rotate ADC_L, C->ADC_L(7)
    ror temp	   ; rotate ADC_H, ADC_H(0)->C
    ror temp2	   ; rotate ADC_L, C->ADC_L(7)
    andi temp, 0x0F  ; isolate desired bits
    andi temp2, 0xFF
    add temp2, ADC_L 
    adc temp, ADC_H
    mov ADC_L, temp2 ; hold copy of the values
    mov ADC_H, temp
   
    andi temp, 0x0F
    subi temp, -(0x30) ; addi temp, 0x30
    mov r24, temp   ; argument to lcd_data
    rcall lcd_data ; send one byte of data to the lcd's controller
    
    ldi r24 ,low(100*FOSC_MHZ)
    ldi r25 ,high(100*FOSC_MHZ)
    rcall wait_msec 
    
    ; -----------------------------------------------
    
    pop temp	    
    out SREG, temp  ; restore SREG
    pop temp	    ; restore temp
    pop r25	    ; restore r25
    pop r24	    ; restore r24
    reti	    ; return to callee