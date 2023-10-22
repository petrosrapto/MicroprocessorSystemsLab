.include "m328PBdef.inc"

reset:    
	ldi r24 , low(RAMEND)	   ; initialize stack pointer
	out SPL , r24
	ldi r24 , high(RAMEND)
	out SPH , r24                	

main:   
    	ldi r24 , low(273)         ; load r25:r24 with how many milliseconds 
	ldi r25 , high(273)        ; the function wait_x_msec must wait
	rcall wait_x_msec	   ; call function wait_x_msec (3 cycles) 
	rjmp main		   ; loop for ever

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


