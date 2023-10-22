.include "m328PBdef.inc"    ; ATmega328P microcontroller definitions
  
.org 0x0
rjmp reset
.org 0x4
rjmp ISR1
    
.equ FOSC_MHZ = 16	    ; Microcontroller operating frequency in MHz
.equ DEL_BOUNCE_mS = 5	    ; Delay in mS for bouncing effect
.equ DEL_mS = 500	    ; Delay in mS (valid number from 1 to 4095) for main
.equ DEL_NU=FOSC_MHZ*DEL_mS ; delay_mS routine: (1000*DEL_NU+6) cycles
.equ DEL_NU_BOUNCE=FOSC_MHZ*DEL_BOUNCE_mS
.DEF temp=r20		    ; define temporary register 
; watch out delay function uses r23
.DEF counter=r22	    ; define the interrupt counter register	
.DEF mainCounter=r21	    ; define the main's counter register
    
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
    sei	    ; enable interrupts
    
    ; Init PORTD as input
    clr temp
    out DDRD, temp 
    
    ; Init PORTB as output
    ser temp
    out DDRB, temp
    ; Init PORTC as output
    out DDRC, temp
    
    ; Init Count Register
    clr counter
    out PORTC, counter ; show counter
    
loop1:
    clr mainCounter
loop2:
    out PORTB, mainCounter
    ldi r24, low(DEL_NU)
    ldi r25, high(DEL_NU)   ; Set delay (number of cycles)
    rcall delay_mS
    inc mainCounter
    cpi mainCounter, 16		    ; compare r26 with 16
    breq loop1
    rjmp loop2
    
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
    push r24	    ; save r24
    push r25	    ; save r25
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
    
    in temp, PIND   
    ; check if PD7 is pressed
    andi temp, 0x80
    cpi temp, 0x00
    breq skip
    ; if pressed  dont increase
dont_freeze:
    inc counter
    cpi counter, 32
    brne skip
    clr counter
skip:
    out PORTC, counter ; show counter
    pop temp	    
    out SREG, temp  ; restore SREG
    pop temp	    ; restore temp
    pop r25	    ; restore r25
    pop r24	    ; restore r24
    reti	    ; return to callee