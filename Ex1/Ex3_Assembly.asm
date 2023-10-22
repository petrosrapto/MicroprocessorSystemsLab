.include "m328PBdef.inc"
; define which register is used with each name
.DEF temp = r16
.DEF leds = r17

reset:
    ldi r24, low(RAMEND)    ;Initialize stack pointer
    out SPL, r24
    ldi r24, high(RAMEND)
    out SPH, r24

main:
    ldi leds,0x01	    ; initialize led state
    ser temp		    
    out DDRD,temp	    ; PORTD output
    bset 6		    ; set 6th bit (T flag) of state 
			    ; register to 1
; 1 -> going left, 0 -> going right
left:
    out PORTD, leds	    ; output led state
    ldi r25, HIGH(500)
    ldi r24, LOW(500)  
    rcall wait_x_msec	    ; wait 0.5sec
    lsl leds		    ; move 1 to the left
    sbrs leds, 7	    ; if last-left led skip jump 
    rjmp left		    ; otherwise repeat left movement
last_left:
    out PORTD, leds	    ; output led state
    ldi r25, HIGH(1000)
    ldi r24, LOW(1000)   
    rcall wait_x_msec	    ; wait 1sec and go right
			    ; attention! the program will wait
			    ; another 0.5sec after right: label 
			    ; for last_left state
    bclr 6		    ; set 6th bit (T flag) of state
			    ; register to 0
; 1 -> going left, 0 -> going right
right:
    out PORTD, leds	    ; output led state
    ldi r25, HIGH(500)
    ldi r24, LOW(500)  
    rcall wait_x_msec	    ; wait 0.5sec 
    lsr leds		    ; move 1 right
    sbrs leds, 0	    ; if last-right led skip jump
    rjmp right		    ; otherwise repeat right movement
last_right: 
    out PORTD, leds	    ; output led state
    ldi r25, HIGH(1000)
    ldi r24, LOW(1000)   
    rcall wait_x_msec	    ; wait 1sec and go left
			    ; attention! the program will wait
			    ; another 0.5sec after left: label 
			    ; for last_right state
    bset 6		    ; set 6th bit (T flag) of state
			    ; register to 0
; 1 -> going left, 0 -> going right
    rjmp left		    ; repeat 

wait4:		   
	ret			   ; just return to the caller (4 cycles)
	
wait1m:
	ldi r26, 98		   ; (1 cycle)
loop1:	
	rcall wait4 ; 7 cycles total
	dec r26	    ; 1 cycle total
	brne loop1  ; 2 cycles if taken 1 if not
	rcall wait4 ; 7 cycles total
	nop	    ; do nothing (1 cycle)
	nop	    ; do nothing (1 cycle)
	ret	    ; return to the caller (4 cycles)
	
; Functionality of the wait_x_msec explained:
; x is passed as parameter to "double" register r25,r24
; wait_x_msec waits (x-1)msec - 1 cycle with the help of loop2
; loop waits the rest of the cycles needed
; before loop2 a check if x was 1 is need
wait_x_msec:
	ldi r26, 98
loop:	; loop repeated 98 times 
	rcall wait4 ; call (3 cycles) and wait4 (4 cycles)
	dec r26	    ; r26-- (1 cycle)
	brne loop   ; if r26 == 0 then exit loop
	; brne takes 2 cycles for eact jump to loop
	; 1 cycle for last check
	; So, loop takes 980-1 cycles in total
	rcall wait4 ; 7 cycles total
	nop	    ; do nothing (1 cycle)
	nop	    ; do nothing (1 cycle)
	sbiw r24, 1 ; subtract 1 from word (16 bit) register
		    ; formed by r25,r24 (2 cycles)
	breq retu   ; if x was 1 then return (2 cycles) 
	nop	    ; do nothing (1 cycle)
	nop	    ; do nothing (1 cycle)
loop2:  ; if loop2 is executed y times then it takes 1000*y-1 cycles
	rcall wait1m  ; 996 cycles total (y)
	sbiw r24, 1   ; 2 cycles
	brne loop2    ; 2 or 1 cycles
retu:	ret	      ; return to the caller (4 cycles)