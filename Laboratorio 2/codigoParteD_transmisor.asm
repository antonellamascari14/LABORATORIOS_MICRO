.include "m328pdef.inc"


.equ F_CPU = 16000000
.equ BAUD = 115200
.equ BPS = 8    ; (16MHz / (16 * 115200)) - 1 = 8

; registros

.def dato_pulsadores = r16
.def estado_usart = r17
.def contador_1 = r18
.def contador_2 = r19

.cseg
.org 0x0000

    rjmp INICIO

INICIO:

; inicializo stack pointer

ldi dato_pulsadores, LOW(RAMEND)
out SPL, dato_pulsadores
ldi dato_pulsadores, HIGH(RAMEND)
out SPH, dato_pulsadores

; configuro las entradas para el dip switch

cbi DDRC, DDC0 
cbi DDRC, DDC1
cbi DDRC, DDC2 

; activo resistencias pull-up internas 

sbi PORTC, PORTC0 
sbi PORTC, PORTC1
sbi PORTC, PORTC2

; configuro la velocidad usart 11520

    ldi dato_pulsadores, HIGH(BPS)
    sts UBRR0H, dato_pulsadores
    ldi dato_pulsadores, LOW(BPS)
    sts UBRR0L, dato_pulsadores

; Habilito transmisor TX

	ldi dato_pulsadores, (1<<TXEN0) ; genera una máscara con un uno en la posición del bit TXEN0
	sts UCSR0B, dato_pulsadores      ; habilita la funcion de transmision fisicamente en el pin tx
    ldi dato_pulsadores, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, dato_pulsadores

MAIN_LOOP:

; leer el puerto C

	in dato_pulsadores, PINC
	andi dato_pulsadores, 0b00000111  ; conserva solo los pines PC0, PC1 y PC2

; invertimos los bit pull-up (arriba 1)

	com dato_pulsadores
    andi dato_pulsadores, 0b00000111  ; consreva de nuevo los pines que el com invirtió

; transmitir el dato

	rcall transmitir_usart ; llama a la subrutina que envía el byte procesado por tx
	rcall retardo_antirrebote ; llama al retrdo para evitar transmisiones erroneas

	rjmp MAIN_LOOP

; subrutinas

transmitir_usart: 
	lds estado_usart, UCSR0A ; lee el registro de estado A del periférico del usart
	sbrs estado_usart, UDRE0   ; se fija que el buffer de envio este libre    
    rjmp transmitir_usart   ; si no está libre, espera
    sts UDR0, dato_pulsadores      ; carga el dato en el registro de transmisión
    ret

	; Subrutina de retardo corto para antirrebote
retardo_antirrebote:
	ldi contador_1, 100 ; carga 100 para el ciclo externo

bucle_externo:
    ldi contador_2, 250 ; 250 para el interno
bucle_interno:
    
    dec contador_2
    brne bucle_interno
    dec contador_1
    brne bucle_externo
    ret


