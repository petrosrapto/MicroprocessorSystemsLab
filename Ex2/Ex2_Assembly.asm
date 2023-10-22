.include "m328PBdef.inc"    ; ATmega328P microcontroller definitions
  
.org 0x0
rjmp reset
.org 0x2
rjmp ISR0
    
.equ FOSC_MHZ = 16	    ; Microcontroller operating frequency in MHz
.equ DEL_BOUNCE_mS = 5	    ; Delay in mS for bouncing effect
.equ DEL_mS = 600	    ; Delay in mS (valid number from 1 to 4095) for main   
.equ DEL_NU=FOSC_MHZ*DEL_mS ; delay_mS routine: (1000*DEL_NU+6) cycles
.equ DEL_NU_BOUNCE=FOSC_MHZ*DEL_BOUNCE_mS
.DEF temp=r23		    ; define temporary register 
; watch out delay function uses r23
.DEF counter=r22	    ; define the interrupt counter register	
.DEF mainCounter=r21	    ; define the main's counter register
.DEF temp2=r20		    ; define temporary register 
.DEF temp3=r19		    ; define temporary register 
    
reset:
    ldi temp, low(RAMEND)    ;Initialize stack pointer
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp
    
    ; Interrupt on rising edge of INT1 pin
    ldi temp, (1 << ISC01) | (1 << ISC00)
    sts EICRA, temp
    
    ; Enable the INT1 interrupt (PD3)
    ldi temp, (1 << INT0)
    out EIMSK, temp
    sei	    ; enable interrupts
    
    ; Init PORTD as input
    clr temp
    out DDRD, temp 
    ; Init PORTB as input
    out DDRB, temp
    ; Init PORTC as output
    ser temp
    out DDRC, temp

loop1:
    clr mainCounter
loop2:
    out PORTC, mainCounter
    ldi r24, low(DEL_NU)
    ldi r25, high(DEL_NU)   ; Set delay (number of cycles)
    rcall delay_mS
    inc mainCounter
    cpi mainCounter, 32		    ; compare r26 with 32
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
    
    
ISR0:		    ; interrupt routine
		    ; for INT1
    push r24	    ; save r24
    push r25	    ; save r25
    push temp	    ; save temp
    in temp, SREG   ; save SREG
    push temp
    
bouncing:
    ldi temp, (1<<INTF0) ; set interrupt
    out EIFR, temp	 ; flag to 0
    ldi r24, low(DEL_NU_BOUNCE)
    ldi r25, high(DEL_NU_BOUNCE)  
    rcall delay_mS	; delay
    in temp, EIFR	; see if the interrupt
    andi temp, 0x01	; bit is activated
    cpi temp, 0x01	; if it is then bouncing
    breq bouncing	; is underway
    
    in temp, PINB   ; keep the port's state
    ldi temp2, 6    ; loop counter
    ldi temp3, 0    ; keep how many buttons 
		    ; are pressed
countLoop:
    ror temp	    ; rotate right and carry
		    ; the last bit
    brcs skip	    ; if carry == 1 dont increase (reversed logic)
		    ; if a button is pressed we have 0
    inc temp3	    ; increase button counter
  skip:
    dec temp2	    ; loopCounter-- 
    brne countLoop
    
    ldi temp, 0x00  ; temp will store the output state
    ldi temp2, 0x01 ; temp2 will alter the bits of temp
outputLoop:
    cpi temp3, 0x00 ; check if the wanted number of leds 
    breq cont	    ; became on
    dec temp3
    or temp, temp2  ; turn current bit at logical 1
    lsl temp	    ; rotate left 
    rjmp outputLoop
    
  cont:
    lsr temp	    ; last bit from outputLoop remained
		    ; zero, a shift to the right is needed
    out PORTC, temp
    ldi r24, low(500)
    ldi r25, high(500)  
    rcall delay_mS  ; delay to see results clearly
    pop temp	    
    out SREG, temp  ; restore SREG
    pop temp	    ; restore temp
    pop r25	    ; restore r25
    pop r24	    ; restore r24
    reti	    ; return to callee


