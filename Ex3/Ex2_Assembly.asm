.include "m328PBdef.inc"

.org 0x0
rjmp reset

.equ FOSC_MHZ = 16	    ; Microcontroller operating frequency in MHz
.equ DEL_BOUNCE_mS = 5	    ; Delay in mS for bouncing effect
.equ DEL_NU_BOUNCE=FOSC_MHZ*DEL_BOUNCE_mS ; delay_mS routine: (1000*DEL_NU+6) cycles
    
.DEF temp=r22	
.DEF counter=r21
.DEF lpmReg=r0	
    
reset:
    ldi temp, LOW(RAMEND) ;Initialize stack pointer
    out SPL, temp
    ldi temp, HIGH(RAMEND)
    out SPH, temp

; Init PORTB as output
    ser temp
    out DDRB, temp
    
; Init PORTD as input
    clr temp
    out DDRD, temp

PWM:
    ldi temp, (1<<WGM10) | (1<<COM1A1)
    sts TCCR1A, temp
    
    ldi temp, (1<<WGM12) | (1<<CS11)
    sts TCCR1B, temp
    ; The above values are for fast PWM
    
    ldi zl, low(Table*2+6)
    ldi zh, high(Table*2+6)
    ; the double register Z keeps table's 
    ; address stored in program memory
    ; each byte represent the DC that corresponds
    ; to each counter value
    ; Z initialized to Table*2+6 in order to point 
    ; to 50% DC (PWM)
    
    ; for 8-bit PWM we have max value = 0xFF (255)
    ldi counter, 0x06	; init counter to 50% DC
    ldi temp, high(127) ; for 50% DC 255/2
    sts OCR1AH, temp
    ldi temp, low(127) ; for 50% DC 255/2
    sts OCR1AL, temp

main:
    in temp, PIND	; keep the state of PIND
    andi temp, 0x02	; isolate PD1
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD1 ; if temp != 0 then continue looping

    in temp, PIND	; keep the state of PIND
    andi temp, 0x04	; isolate PD2
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD2 ; if temp != 0 then continue loopin	
    
    rjmp main  

PressD1:
    ldi r24, low(DEL_NU_BOUNCE)
    ldi r25, high(DEL_NU_BOUNCE)  
    rcall delay_mS  ; delay to overcome bouncing
    in temp, PIND	; keep the state of PIND
    andi temp, 0x02	; isolate PD1
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD1	; if the button is still pressed wait
    
    ; button unpressed, proceed
    cpi counter, 0      ; DC=50+(6*8)=98%, #table = 13 elements 
    breq main		; if at 98% dont decrease more
    
    dec counter		; otherwise decrease counter
    
 
    sbiw zl, 1		; decrease Z register one byte
			; to access next DC value
    lpm			; r0 <-- memory
    
    clr temp
    sts OCR1AH, temp	; values are from 0 to 255
    sts OCR1AL, lpmReg	; set the compare level 
    rjmp main		; continue looping

PressD2:
    ldi r24, low(DEL_NU_BOUNCE)
    ldi r25, high(DEL_NU_BOUNCE)  
    rcall delay_mS  ; delay to overcome bouncing
    in temp, PIND	; keep the state of PIND
    andi temp, 0x04	; isolate PD2
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD2 ; if the button is still pressed wait
    
    ; button unpressed, proceed
    cpi counter, 12	; if at 2% DC dont increase more
    breq main		
    
    inc counter		; otherwise increase counter

    adiw zl, 1		; increase Z register one byte
			; to access previous DC value
    lpm			;r0 <-- memory

    clr temp
    sts OCR1AH, temp	; values are from 0 to 255
    sts OCR1AL, lpmReg	; set the compare level 
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
   
Table:
; Non-Inverted PWM, formula for OCR1AL: hex(round(maxValue-maxValue*DC))
; DCs start from 2% to 98% increasing by 8%
.DW 0xE5FA, 0xBCD1, 0x94A8, 0x6B7F, 0x4257, 0x192E, 0x0005
