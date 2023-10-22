.include "m328PBdef.inc"
; define which register is used with each name
.DEF A=r16 
.DEF Ainc=r25 
.DEF B=r17 
.DEF Binc=r26 
.DEF C=r18
.DEF Cinc=r27 
.DEF D=r19
.DEF Dinc=r28 
.DEF A=r16 
.DEF DN=r20
.DEF temp=r21
.DEF F0=r22
.DEF F1=r23
.DEF counter=r24
    
reset:    
       ldi r24 , low(RAMEND)   	; initialize stack pointer
       out SPL , r24
       ldi r24 , high(RAMEND)
       out SPH , r24                	

main:
    ldi counter, 6		; repeat six times
    ldi A, 0x55			; initial values
    ldi B, 0x43
    ldi C, 0x22
    ldi D, 0x02

    ldi Ainc, 0x02		; how much each variable
    ldi Binc, 0x03		; increases 
    ldi Cinc, 0x04
    ldi Dinc, 0x05
    loop:
	mov DN, D
	com DN			; DN = D'

	mov F0, A
	or F0, B		; F0 = A + B (temporarily)

	mov temp, B
	or temp, DN		; temp = B + D'

	and F0, temp		; F0 = (A+B)(B+D') (final)
				; (A'B'+B'D)'=(A+B)(B+D')
				
	mov F1, A		
	or F1, C		; F1 = A + C (temporarily)

	and F1, temp		; F1 = (A+C)(B+D') (final)
	

	add A, Ainc		; increase the variables
	add B, Binc
	add C, Cinc
	add D, Dinc
	dec counter		; decrease counter
	brne loop		; if counter > 0 repeat
				; exit otherwise