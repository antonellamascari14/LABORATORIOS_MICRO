.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD = 115200
.equ BPS = 8    ; (16MHz / (16 * 115200)) - 1 = 8

; registros

.def dato_recibido = r16
.def estado_usart = r17
.def mascara_leds = r18
.def controlador_desplazamiento =r19
.def salida_temp =r20

.cseg 
.org  0x0000

	rjmp INICIO

INICIO: 

;inicializo stack pointer

ldi dato_recibido, LOW(RAMEND)
out SPL, dato_recibido
ldi dato_recibido, HIGH(RAMEND)
out SPH, dato_recibido

; salidas de los ledss

ldi dato_recibido, 0b00111111 ; PB0 A PB5 salidas
out DDRB, dato_recibido
ldi dato_recibido, 0b00000011 ; PC0 a PC1 como salidas
out SSRC, dato_recibido

; apagar las led al principio

clr dato_recibido
out PORTB, dato_recibido
out PORTC, dato_recibido

; configuro la velocidad USART

ldi dato_recibido, HIGH(BPS)
sts UBRR0H, dato_recibido
ldi dato_recibido, LOW(BPS)
sts UBRR0L, dato_recibido

; Habilitar Receptor (RX)

    ldi dato_recibido, (1<<RXEN0)
    sts UCSR0B, dato_recibido

; Formato: 8 bits de datos, 1 bit de parada

    ldi dato_recibido, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, dato_recibido

MAIN_LOOP:

rcall recibir_usart
andi dato_recibido, 0b00000111 ; garatniza el rango de 0 a 7
rcall encender_led
rjmp MAIN_LOOP

; subrutina para recibir 

recibir_usart:

lds estado_usart, 


