.include "m328pdef.inc"  ; Para los registros 

.def  TEMP = r16  ; registro temporal
.def ESTADO = r17 ; Guarda la secuencia  actual (1, 2, 3...)
.def DELAY1 = r18 ; Para el retardo y ver la led prendida :)
.def DELAY2 = r19 
.def DELAY3 = r20


.ORG 0x0000

	RJMP MAIN

MAIN: 

	; Primero inicializa el Stack Pointer
	ldi TEMP, HIGH(RAMEND)
	out SPH, TEMP
	ldi TEMP, LOW(RAMEND)
	out SPL, TEMP


	; Configuramos los puertos 
	; PORTD como salidas
	ldi TEMP, 0xFF
	out DDRD, TEMP
	out PORTD, TEMP ; Al inicio los LED esan apagados

	; PORTC como entradas

	clr TEMP
	out DDRC, TEMP
	ldi TEMP, (1<<PORTC0) | (1<<PORTC1) | (1<<PORTC2)
	out PORTC, TEMP  ;aCTIVAR pull-ups internas en A0, A1 Y A2

	; ESTADO INICIAL

	ldi ESTADO, 1 ; Comienza el estadp en 1

LOOP_PRINCIPAL: 
	; Verificar Botones 
	rcall LEER_BOTONES

	; Comparación de estado con saltos condicionales BRNE
	cpi ESTADO, 1
	brne TEST_E2
	rjmp EJECUTAR_ESTADO_1

TEST_E2:
	cpi ESTADO, 2
	brne TEST_E3
	rjmp EJECUTAR_ESTADO_2

TEST_E3:
	cpi ESTADO, 3
	brne TEST_E4
	rjmp EJECUTAR_ESTADO_3

TEST_E4:
	cpi ESTADO, 4
	brne TEST_E5
	rjmp EJECUTAR_ESTADO_4

TEST_E5:
	cpi ESTADO, 5
	brne TEST_E6
	rjmp EJECUTAR_ESTADO_5

TEST_E6:
	cpi ESTADO, 6
	brne TEST_E7
	rjmp EJECUTAR_ESTADO_6

TEST_E7:
	cpi ESTADO, 7
	brne TEST_E8
	rjmp EJECUTAR_ESTADO_7

TEST_E8:
	cpi ESTADO, 8
	brne ESTADO_INVALIDO
	rjmp EJECUTAR_ESTADO_8

ESTADO_INVALIDO:
	ldi ESTADO, 1
	rjmp LOOP_PRINCIPAL

	; SECUENCIAS / ESTADOS

	EJECUTAR_ESTADO_1:

	; eSTADO 1: leds pares / impares

	ldi TEMP, 0b01010101
	out PORTD, TEMP
	rcall RETARDO_Y_LECTURA

	ldi TEMP, 0b10101010
	out PORTD, TEMP
	rcall RETARDO_Y_LECTURA
	rjmp LOOP_PRINCIPAL

	EJECUTAR_ESTADO_2:

	; ESTADO: de 2 en 2

	ldi TEMP, 0b00000011
	out PORTD, TEMP
	rcall RETARDO_Y_LECTURA

	ldi TEMP, 0b00001100
	out PORTD, TEMP
	rcall RETARDO_Y_LECTURA

	ldi TEMP, 0b00110000
	out PORTD, TEMP
	rcall RETARDO_Y_LECTURA

	ldi TEMP, 0b11000000
	out PORTD, TEMP
	rcall RETARDO_Y_LECTURA
	rjmp LOOP_PRINCIPAL

	EJECUTAR_ESTADO_3:

	