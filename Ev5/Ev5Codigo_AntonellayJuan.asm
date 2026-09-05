.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD = 9600
.equ UBRR_VAL = 16  ; (16MHz / (16 * 57600)) - 1

.cseg
.org 0x0000
    rjmp RESET

RESET:
    ; stack pointer

    ldi r16, LOW(RAMEND)
    out SPL, r16
    ldi r16, HIGH(RAMEND)
    out SPH, r16

    ; salidas del display PD2..PD7 como Salidas (Segmentos a, b, c, d, e, f)
  
    sbi DDRD, DDD2
    sbi DDRD, DDD3
    sbi DDRD, DDD4
    sbi DDRD, DDD5
    sbi DDRD, DDD6
    sbi DDRD, DDD7

    ; PB0 como Salida (Segmento g)
    sbi DDRB, DDB0

    ; inicualizo modulo USART
    rcall USART_INIT

LOOP_PRINCIPAL:

    ; espera el dato desde el monitor serie / terminal

    rcall USART_RECEIVE       ; retorna el byte en r16
    
    ; convierte el caracter recibido a un índice hexadecimal (0 a 15)

    rcall ASCII_A_INDEX       ; retorna el índice en r16 (0xFF si es inválido)

    cpi r16, 0xFF
    breq LOOP_PRINCIPAL       ; ignora los  caracteres que estan fuera del rango '0'-'F'

    ; lee el patrón desde la LUT en la memoria FLASH usando lpm

    rcall OBTENER_PATRON_LUT  ; entrada: r16 (índice), salida: r18 (patrón)

    ; envia patrón a los pines del Display

    rcall MOSTRAR_DISPLAY

    rjmp LOOP_PRINCIPAL

; subrutina de inicializacion de USART

USART_INIT:
    ldi r16, HIGH(UBRR_VAL)
    sts UBRR0H, r16
    ldi r16, LOW(UBRR_VAL)
    sts UBRR0L, r16

    ; habilitar Receptor (RX) y Transmisor (TX) del arduino

    ldi r16, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, r16

    ; Formato de marco: 8 bits de datos, 1 bit de parada
    ldi r16, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, r16
    ret

; subrutina de recepcion de datos por USART
USART_RECEIVE:
    lds r17, UCSR0A
    sbrs r17, RXC0            ; se fija si recibio el dato
    rjmp USART_RECEIVE        
    lds r16, UDR0             ; carga dato recibido
    ret

; subrutina de conversion ascii a indice de 0 a 15 

ASCII_A_INDEX:

    ; si se envio un valor numérico directo (0x00 a 0x0F)
    cpi r16, 16
    brlo ES_VALIDO

    ; rango 0 - 9 (ASCII 0x30 - 0x39)
    cpi r16, '0'
    brlo ERROR_ASCII
    cpi r16, '9' + 1
    brlo CONVERTIR_NUMERO

    ; rango A - F (ASCII 0x41 - 0x46)
    cpi r16, 'A'
    brlo ERROR_ASCII
    cpi r16, 'F' + 1
    brlo CONVERTIR_MAYUS

    ; Rango 'a' - 'f' (ASCII 0x61 - 0x66)
    cpi r16, 'a'
    brlo ERROR_ASCII
    cpi r16, 'f' + 1
    brlo CONVERTIR_MINUS

ERROR_ASCII:
    ldi r16, 0xFF             ; marcador de error
    ret

CONVERTIR_NUMERO:
    subi r16, '0'
    ret

CONVERTIR_MAYUS:
    subi r16, 'A' - 10
    ret

CONVERTIR_MINUS:
    subi r16, 'a' - 10
    ret

ES_VALIDO:
    ret

; subrutina consulta a la lut con lpm
OBTENER_PATRON_LUT:
    ; carga dirección base de la lut en el punntero Z multiplicado por 2

    ldi ZL, LOW(LUT_7SEG * 2)
    ldi ZH, HIGH(LUT_7SEG * 2)

    ; suma el indice del carácter
    clr r17
    add ZL, r16
    adc ZH, r17

    ; lee el byte desde la Memoria flash
    lpm r18, Z
    ret

;subrutina para enviar mascara de bits al display
MOSTRAR_DISPLAY:
    ; r18 = 0b0gfedcba
    
    ; actualiza segmentos a-f en PD2..PD7
    mov r19, r18
    andi r19, 0b00111111      ; toma bits a,b,c,d,e,f
    lsl r19
    lsl r19                  ; desplaza a PD2..PD7

    in r20, PORTD
    andi r20, 0b00000011      ; reserva PD0 (RX) y PD1 (TX)
    or r20, r19
    out PORTD, r20

    ; actualizar Segmento g en PB0 

    mov r19, r18
    andi r19, 0b01000000      ; tomar bit g
    lsr r19
    lsr r19
    lsr r19
    lsr r19
    lsr r19
    lsr r19                  ; mover a PB0

    in r20, PORTB
    andi r20, 0b11111110      ; reservar PB1..PB7
    or r20, r19
    out PORTB, r20
    ret

; LUT en flash (catodo comun)
; formato del Byte: 0b0gfedcba

LUT_7SEG:
    .db 0b00111111, 0b00000110 ; '0' (0x3F), '1' (0x06)
    .db 0b01011011, 0b01001111 ; '2' (0x5B), '3' (0x4F)
    .db 0b01100110, 0b01101101 ; '4' (0x66), '5' (0x6D)
    .db 0b01111101, 0b00000111 ; '6' (0x7D), '7' (0x07)
    .db 0b01111111, 0b01101111 ; '8' (0x7F), '9' (0x6F)
    .db 0b01110111, 0b01111100 ; 'A' (0x77), 'b' (0x7C)
    .db 0b00111001, 0b01011110 ; 'C' (0x39), 'd' (0x5E)
    .db 0b01111001, 0b01110001 ; 'E' (0x79), 'F' (0x71)