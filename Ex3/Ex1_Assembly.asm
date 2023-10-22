.include "m328PBdef.inc"    ; ATmega328P microcontroller definitions
  
.org 0x0
rjmp reset		    ; reset
.org 0x4
rjmp ISR1		    ; jump to INT1's interrupt routine 
.org 0x1A
rjmp ISR_TIMER1_OVF	    ; jump to timer's interrupt routine 
    
.equ FOSC_MHZ = 16	    ; Microcontroller operating frequency in MHz
.equ DEL_BOUNCE_mS = 5	    ; Delay in mS for bouncing effect
.equ DEL_mS_ALL_LEDS = 500
.equ DEL_NU_BOUNCE=FOSC_MHZ*DEL_BOUNCE_mS ; delay_mS routine: (1000*DEL_NU+6) cycles
.equ DEL_NU_ALL_LEDS=FOSC_MHZ*DEL_mS_ALL_LEDS   
    
.equ TIMER_DELAY_S = 4			  ; we want a delay of 4 sec   
.equ OVERFLOW_POINT = 65535		  ; overflow point of timer (16 bits)  
.equ FOSC_HZ = 16000000			  ; Microcontroller operating frequency in Hz
.equ TIMER_FREQ = FOSC_HZ/1024		  ; cause of the value of TCCR1B set below
.equ DEL_NU_TIMER=OVERFLOW_POINT - TIMER_FREQ*TIMER_DELAY_S ; find the total cycles of timer's delay
; when an overflow takes place an timer interrupt is fired
    
.DEF temp=r22			  ; define temporary register 
.DEF flag=r21			  ; flag activated if first interrupt detected

.MACRO Leds				  ; define leds macro
    
    ; Set frequency of timer's increase to fclock/1024
    ldi temp, (1<<CS12) | (0<<CS11) | (1<<CS10) 
    sts TCCR1B, temp
    
    ; set timer to 4 sec
    ldi temp, HIGH(DEL_NU_TIMER) 
    sts TCNT1H, temp 
    ldi temp, LOW(DEL_NU_TIMER)
    sts TCNT1L, temp
    
    cpi flag, 0x00
    breq first_Interrupt
    
    ldi temp, 0xFF 
    out PORTB, temp			  ;light up all leds
    ldi r24, low(DEL_NU_ALL_LEDS)
    ldi r25, high(DEL_NU_ALL_LEDS)  
    rcall delay_mS			  ; delay 0.5 sec
    
first_Interrupt:
    inc flag
    ldi temp, 0x01  
    out PORTB, temp			  ;let there be light
    
.ENDMACRO
    
reset:
    ldi temp, low(RAMEND)    ;Initialize stack pointer
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp
    
    ; Interrupt on rising edge of INT1 pin
    ldi temp, (1 << ISC11) | (1 << ISC10)
    sts EICRA, temp
    
    ; Enable the INT1 interrupt (PD3)
    ldi temp, (1 << INT1)
    out EIMSK, temp
    
    ; Init PORTD as input
    clr temp
    out DDRD, temp 
    
    ; Init PORTC as input
    clr temp
    out DDRC, temp 
    
    ; Init PORTB as output
    ser temp
    out DDRB, temp
    
    ; Enable TCNT1 overflow timer interrupt
    ldi temp, (1<<TOIE1) 
    sts TIMSK1, temp 
    
    ; Stop the time counter
    ldi temp, (0<<CS12) | (0<<CS11) | (0<<CS10) 
    sts TCCR1B, temp
    
    ldi flag, 0x00  ; init flag 
    sei		    ; enable interrupts
    
main:
    in temp, PINC   ; keep the state of PINC
    andi temp, 0x20 ; isolate PC5
    cpi temp, 0x00  ; compare with zero
    ; if pushed temp == 0 (reversed logic)
    brne main ; if temp != 0 then continue looping
	      ; else the button is pressed
BTN_PRESSED:  ; while the button is pressed or bouncing loop
    ldi r24, low(DEL_NU_BOUNCE)
    ldi r25, high(DEL_NU_BOUNCE)  
    rcall delay_mS  ; delay to overcome bouncing
    in temp, PINC   ; fetch the state of PINC
    andi temp, 0x20 ; isolate PC5
    cpi temp, 0x00  ; compare with zero
    breq BTN_PRESSED ; if still pressed loop
    Leds      ; otherwise call Leds macro
    rjmp main ; jump again to main
    
; delay of 1000*F1 + 6 cycles (almost equal to 1000*F1 cycles)
delay_mS:
; total delay of next 4 instruction group = 1+(249*4-1) = 996 cycles
    ldi r23, 249
loop_inn:
    dec r23 ; 1 cycle
    nop	    ; 1 cycle
    brne loop_inn   ; 1 or 2 cycles
    sbiw r24, 1	    ; 2 cycles
    brne delay_mS   ; 1 or 2 cycles
    ret		    ; 4 cycles
    
    
ISR1:		    ; interrupt routine
		    ; for INT1
    push r25	    ; keep delay registers
    push r24        
    push temp	    ; save temp
    in temp, SREG   ; save SREG
    push temp
    
bouncing:
    ldi temp, (1<<INTF1) ; set interrupt
    out EIFR, temp	 ; flag to 0
    ldi r24, low(DEL_NU_BOUNCE)
    ldi r25, high(DEL_NU_BOUNCE)  
    rcall delay_mS
    in temp, EIFR
    andi temp, 0x02
    cpi temp, 0x02
    breq bouncing
    
    Leds	    ; call macro
    
    pop temp	    
    out SREG, temp  ; restore SREG
    pop temp	    ; restore temp
    pop r24 
    pop r25	    ; restore delay registers     
    reti	    ; return from interrupt
    
ISR_TIMER1_OVF:
    push r25	    ; keep delay registers
    push r24        
    push temp	    ; save temp
    in temp, SREG   ; save SREG
    push temp
    
    ; Stop the time counter
    ldi temp, (0<<CS12) | (0<<CS11) | (0<<CS10) 
    sts TCCR1B, temp
    
    ldi temp, 0x00  
    out PORTB, temp ;lights off
    
    ldi flag, 0x00  ; set flag to zero
    
    pop temp	    
    out SREG, temp  ; restore SREG
    pop temp	    ; restore temp
    pop r24 
    pop r25	    ; restore delay registers   
    reti	    ; return from interrupt
    