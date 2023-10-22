.include "m328PBdef.inc"

.org 0x0
rjmp reset

.DEF temp=r22	
    
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
    ldi temp, (0<<WGM10) | (1<<WGM11) | (1<<COM1A1)
    sts TCCR1A, temp
    ; The above values are for fast PWM with TOP = ICR1    
    
main:
    
    ldi temp, (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (0<<CS11) | (0<<CS10)
    ; we stop PWM setting timer's frequency to zero
    sts TCCR1B, temp
    
    in temp, PIND	; keep the state of PIND
    andi temp, 0x01	; isolate PD0
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD0 ; if temp != 0 then continue looping
    
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
    
    in temp, PIND	; keep the state of PIND
    andi temp, 0x08	; isolate PD2
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD3 ; if temp != 0 then continue loopin	
    
    rjmp main  

; formula: top(fpwm, N) = fclk/(N*fpwm) - 1
    
PressD0: ; PWM with 125Hz
    ldi temp, (1<<WGM12) | (1<<WGM13) | (1<<CS12) | (0<<CS11) | (1<<CS10)
    ; N = 1024
    sts TCCR1B, temp
    
    clr temp
    sts OCR1AH, temp	; values are from 0 to 255
    ldi temp, 62
    sts OCR1AL, temp	; set the compare level 
    
    clr temp
    sts ICR1H, temp	; values are from 0 to 255
    ldi temp, 124
    sts ICR1L, temp	; set the top level 
    
    in temp, PIND	; keep the state of PIND
    andi temp, 0x01	; isolate PD0
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD0	; if the button is still pressed wait
    rjmp main		; continue looping
    
    
PressD1: ; PWM with 250Hz
    ldi temp, (1<<WGM12) | (1<<WGM13) | (1<<CS12) | (0<<CS11) | (0<<CS10)
    ; N = 256
    sts TCCR1B, temp
    
    clr temp
    sts OCR1AH, temp	; values are from 0 to 255
    ldi temp, 124
    sts OCR1AL, temp	; set the compare level 
    
    clr temp
    sts ICR1H, temp	; values are from 0 to 255
    ldi temp, 249
    sts ICR1L, temp	; set the top level 
    
    in temp, PIND	; keep the state of PIND
    andi temp, 0x02	; isolate PD1
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD1	; if the button is still pressed wait
    rjmp main		; continue looping

PressD2: ; PWM with 500Hz
    ldi temp, (1<<WGM12) | (1<<WGM13) | (1<<CS12) | (0<<CS11) | (0<<CS10)
    ; N = 256
    sts TCCR1B, temp
    
    clr temp
    sts OCR1AH, temp	; values are from 0 to 255
    ldi temp, 62
    sts OCR1AL, temp	; set the compare level 
    
    clr temp
    sts ICR1H, temp	; values are from 0 to 255
    ldi temp, 124
    sts ICR1L, temp	; set the top level 
    
    in temp, PIND	; keep the state of PIND
    andi temp, 0x04	; isolate PD2
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD2	; if the button is still pressed wait
    rjmp main		; continue looping
    
PressD3: ; PWM with 1000Hz
    ldi temp, (1<<WGM12) | (1<<WGM13) | (0<<CS12) | (1<<CS11) | (1<<CS10)
    ; N = 64
    sts TCCR1B, temp
    
    clr temp
    sts OCR1AH, temp	; values are from 0 to 255
    ldi temp, 124
    sts OCR1AL, temp	; set the compare level 
    
    clr temp
    sts ICR1H, temp	; values are from 0 to 255
    ldi temp, 249
    sts ICR1L, temp	; set the top level 
    
    in temp, PIND	; keep the state of PIND
    andi temp, 0x08	; isolate PD3
    cpi temp, 0x00	; compare with zero
    ; if pushed temp == 0 (reversed logic)
    breq PressD3	; if the button is still pressed wait
    rjmp main		; continue looping