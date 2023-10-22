.include "m328PBdef.inc"    ; ATmega328P microcontroller definitions
  
.org 0x0
rjmp reset
.org 0x4
rjmp ISR1
    
.equ FOSC_MHZ = 16	    ; Microcontroller operating frequency in MHz
.equ DEL_BOUNCE_mS = 5	    ; Delay in mS for bouncing effect
.equ DEL_mS = 500	    ; Delay in mS (valid number from 1 to 4095) for main
.equ DEL_mS_LAMP = 4000	    ; Delay in mS (valid number from 1 to 4095) for main
.equ DEL_mS_ALL_LEDS = 500
.equ DEL_NU=FOSC_MHZ*DEL_mS ; delay_mS routine: (1000*DEL_NU+6) cycles
.equ DEL_NU_BOUNCE=FOSC_MHZ*DEL_BOUNCE_mS
.equ DEL_NU_LAMP=FOSC_MHZ*DEL_mS_LAMP
.equ DEL_NU_ALL_LEDS=FOSC_MHZ*DEL_mS_ALL_LEDS
.equ DEL_NU_LIGHT2=FOSC_MHZ*3500	    
.DEF temp=r23		    ; define temporary register 
; watch out delay function uses r23
.DEF counter=r22	    ; define the interrupt counter register	
.DEF mainCounter=r21	    ; define the main's counter register
.DEF flag=r20
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
    
    ; Init interrupt flag 
    ldi flag, 0x00
    
main:
    rjmp main
    
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
    pop temp	    ; pop the program counter
    pop temp	    ; saved in two stack places
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
    
    ; if flag == 0 then first interrupt
    cpi flag, 0x00
    brne light2	    

    inc flag
    sei	    ; enable interrupts while in interrupt handler
    ldi temp, 0x01  
    out PORTB, temp ;let there be light
    ldi r24, low(DEL_NU_LAMP)
    ldi r25, high(DEL_NU_LAMP)  
    rcall delay_mS  ; delay 4 secs
    ; during that delay interrupts could take place.
    ; As a result the delay time will reset
    ldi temp, 0x00
    out PORTB, temp ; lights off
    rjmp cont

light2:
    inc flag
    sei	; enable interrupts while in interrupt handler
    ldi temp, 0xFF
    out PORTB, temp ;light up all lamps
    ldi r24, low(DEL_NU_ALL_LEDS)
    ldi r25, high(DEL_NU_ALL_LEDS)  
    rcall delay_mS  ; delay 0.5 secs
    ldi temp, 0x01  
    out PORTB, temp ;let there be light
    ldi r24, low(DEL_NU_LIGHT2)
    ldi r25, high(DEL_NU_LIGHT2)  
    rcall delay_mS  ; delay 3.5 secs
    ; during that delay interrupts could take place.
    ; As a result the delay time will reset
    ldi temp, 0x00
    out PORTB, temp ; lights off
    
popLoop:
    cpi flag, 0x01  ; compare flag with zero
    breq cont	    ; continue
    dec flag	    ; decrease flag
    pop temp	    
    pop temp	    ; restore temp
    rjmp popLoop
cont:
    pop temp	    
    out SREG, temp  ; restore SREG
    		    ; enable interrupts for main
		    ; reti did that by default
    pop temp	    ; restore temp
    sei
    ldi flag, 0x00  ; restore flag to 0 (first interrupt)
    rjmp main	    ; jump to main and not return



