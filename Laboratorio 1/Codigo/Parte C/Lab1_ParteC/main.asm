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
; ESTADO 3: De punta a punta

ldi TEMP, 0b10000001
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b01000010
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00100100
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00011000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA
rjmp LOOP_PRINCIPAL

EJECUTAR_ESTADO_4:

;ESTADO 4: Efecto Kitt

ldi TEMP, 0b10000000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b01000000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00100000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00010000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00001000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00000100
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00000010
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00000001
out PORTD, TEMP
rcall RETARDO_Y_LECTURA
rjmp LOOP_PRINCIPAL

EJECUTAR_ESTADO_5:

; ESTADO 5: Llenado

ldi TEMP, 0b10000000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11000000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11100000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11110000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11111000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11111100
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11111110
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11111111
out PORTD, TEMP
rcall RETARDO_Y_LECTURA
rjmp LOOP_PRINCIPAL

EJECUTAR_ESTADO_6:

; ESTADO 6: de 4 en 4

ldi TEMP, 0b11110000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00001111
out PORTD, TEMP
rcall RETARDO_Y_LECTURA
rjmp LOOP_PRINCIPAL

EJECUTAR_ESTADO_7:

; ESTADO 7: Choque en el medio

ldi TEMP, 0b10000001
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11000011
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11100111
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b11111111
out PORTD, TEMP
rcall RETARDO_Y_LECTURA
rjmp LOOP_PRINCIPAL


EJECUTAR_ESTADO_8:

; ESTADO 8: de a 3

ldi TEMP, 0b11100000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b01110000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00111000
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00011100
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00001110
out PORTD, TEMP
rcall RETARDO_Y_LECTURA

ldi TEMP, 0b00000111
out PORTD, TEMP
rcall RETARDO_Y_LECTURA
rjmp LOOP_PRINCIPAL

; lectura de los botones y el antirebote

LEER_BOTONES:

in TEMP, PINC

; Boton 3 (reset) 2 (PINC2)
sbrs TEMP, PINC2 ; sI PINC2 es 0 (presionado), se ejecuta la siguiente linea
rjmp ACCION_RESET

; Boton 1: SIGUIENTE  A0 (PINC0)

sbrs TEMP, PINC0
rjmp ACCION_SIG

;Boton 2: Anterior A1 (PINC1)

sbrs TEMP, PINC1
rjmp ACCION_ANT

ret

ACCION_RESET:
ldi ESTADO, 1
rcall ANTIRREBOTE
pop TEMP ; Limpia la dirección de retorno en la pila
pop TEMP
rjmp LOOP_PRINCIPAL

ACCION_SIG:
inc ESTADO
cpi ESTADO, 9
brne FIN_SIG
ldi ESTADO, 1   ; Vuelve a 1 si pasa de 8

FIN_SIG:
rcall ANTIRREBOTE
pop TEMP ; Limpia la dirección de retorno en la pila
pop TEMP
rjmp LOOP_PRINCIPAL

ACCION_ANT:
dec ESTADO
cpi ESTADO, 0
brne FIN_ANT
ldi ESTADO, 8  ; Vuelve a 8 si se pasa de 1

FIN_ANT:
rcall ANTIRREBOTE
pop TEMP ; Limpia la dirección de retorno en la pila
pop TEMP
rjmp LOOP_PRINCIPAL


;tiempos

RETARDO_y_LECTURA:
ldi DELAY1, 70
D_LOOP:
rcall LEER_BOTONES
rcall DELAY_CORTO
dec DELAY1
brne D_LOOP
ret

DELAY_CORTO:
ldi DELAY2, 200
D2: ldi DELAY3, 250
D3: dec DELAY3
brne D3
dec DELAY2
brne D2
ret

ANTIRREBOTE: ; Esperar a que el usuario suelte el botón
rcall DELAY_CORTO
ESPERAR_SOLTAR:
in TEMP, PINC
andi TEMP, 0b00000111
cpi TEMP, 0b00000111 ; Verifica si están todos sin presionar
brne ESPERAR_SOLTAR  ; Si sigue presionado, espera
ret 


	
