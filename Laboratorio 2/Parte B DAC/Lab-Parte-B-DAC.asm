/*
 Lab-Parte-B-DAC.asm

 USART:		PD0 = RX
			PD1 = TX

 DAC R-2R:		PD2 - PD7 = bits 2 - 7
				PB0 - PB1 = bits 0 - 1

Por UART:		1 = Senal 6
				2 = Senal 12
*/

.include "m328pdef.inc"

.equ F_CPU = 16000000                    ; Freq de trabajo = 16 MHz
.equ BAUD = 9600                         ; Velocidad de comunicacion UART
.equ UBRR_VALUE = (F_CPU/(16*BAUD))-1   ; Calculamos valor para el Baud Rate

.equ OCR1A_VALUE = 77          ; Valor de comparacion del Timer1, 77 es para que la
								; onda tenga mas o menos 100Hz (cada onda son 255 valores)

;			 Nombres de Registros

.def temp      = r16		; Reg temporal
.def seleccion = r17		; Guarda la seleccion de senal
.def indice    = r18		; Posicion actual dentro de la LUT
.def dato      = r19		; Guarda el valor leido de la LUT
.def aux       = r20		; Registro auxiliar

;			VECTOR DE RESET, INTERRUPCIONES

.cseg
.org 0x0000
	rjmp Inicio                          

.org OC1Aaddr                            ; Vector de Compare Match A del Timer1
    rjmp TIMER1_COMPA_ISR                ; Salta a la interrupcion del Timer1

.org 0x0034                              ; Para iniciar desp. de la tabla de vectores
;------------------------------------------------------------------
Inicio:

    ldi temp,HIGH(RAMEND)
    out SPH,temp                         ; Inicializamos el Stack Pointer
    ldi temp,LOW(RAMEND)
    out SPL,temp

    clr r1                               ; r1 siempre en cero

; -------- Config. Puertos del DAC --------------

    ldi temp, 0xFC                  ; PD(2-7) como salidas
    out DDRD,temp

    ldi temp,0x03                  ; PB(0-1) como salidas
    out DDRB,temp

;------------------Estado Inicial ---------------
	clr temp
	out PORTD, temp			; Inicializamos el DAC
	out PORTB, temp

    clr seleccion          ; Ninguna senal seleccionada
    clr indice             ; Comenzamos en la muestra 0

;oooooooooooo Iniciar la UART oooooooooooooo

    rcall initUART       ; Llamamos a la configuracion UART

; ------- Mostrar el Menu de UART --------------

    ldi ZH, HIGH(MENU<<1)       ; Direccion del menu
    ldi ZL, LOW(MENU<<1)

    rcall UART_PRINT           ; Mandamos el menu por el puerto serial

;------ Timer 0 ----------------

    rcall TIMER1_INIT         ; Configuramos Timer1
    sei                       ; Habilitamos interrupciones globales

;ooooooooooooooo BUCLE PRINCIPAL oooooooooooooooo

MAIN:
    lds temp, UCSR0A                 ; Leemos estado del USART
    sbrs temp, RXC0                  ; Verificamos si recibimos un dato
    rjmp MAIN

    lds temp, UDR0                   ; Si llega algo lo guardams en temp

;-------- Opcion 1: Senal 6 ------------- Onda con un pico

    cpi temp,'1'                         ; Comparamos el dato recibido con 1
    breq SELECCIONAR_6                   ; Si es =1 se eligio la senal 6

;---------- Opcion 2: Senal 12 ----------- Rampa / Sierra

    cpi temp,'2'                         ; Comparamos el dato recibido con "2"
    breq SELECCIONAR_12                  ; Si es 2 elegimos la senal 12

;------------ R : RESET ------------

    cpi temp,'R'
    breq RESET_UART

    cpi temp,'r'
    breq RESET_UART


;oooooooooooo M : MOSTRAR MENU oooooooooooooo

    cpi temp,'M'                 ; Compara con M
    breq MOSTRAR_MENU
    cpi temp,'m'                 ; Compara con m
	breq MOSTRAR_MENU			 ; Muestra el Menu sin importar si es o no Mayuscula

    rjmp MAIN                   ; Si no tocamos ninguna, queda esperndo

;-----------------------------------------------
RESET_UART:
    cli
    rjmp Inicio

;ooooooooooooo SELECCIONAR SENAL 6 ooooooooooooooo
SELECCIONAR_6:
    cli                      ; Deshab. las interrup. momentaneamente 
							; (Es para que el Timer0 no meta la pata si justo se da)
    ldi seleccion,6          ; Guardamos la senal elegida
    clr indice               ; iniciamos desde la muestra 0
    sei						; Volvemos a habilitar interrup

    ldi ZH, HIGH(MSG_SENAL6<<1)           ; Cargamos direccion del mensaje
    ldi ZL, LOW(MSG_SENAL6<<1)

    rcall UART_PRINT                     ; Mostramos mensaje por UART
    rjmp MAIN                            ; Volvemos al main

;ooooooooooooo SELECCIONAR SENAL 12 ooooooooooooooo
SELECCIONAR_12:
    cli                      

    ldi seleccion,12          
    clr indice						; Todo lo mismo que en la Senal 6
    sei						

    ldi ZH, HIGH(MSG_SENAL12<<1)           ; Cargamos direccion del mensaje
    ldi ZL, LOW(MSG_SENAL12<<1)

    rcall UART_PRINT                   
    rjmp MAIN    

;ooooooooooooo MOSTRAR MENU oooooooooooooooooo

MOSTRAR_MENU:
    ldi ZH, HIGH(MENU<<1)                 ; Cargamos la direccion del menu
    ldi ZL, LOW(MENU<<1)               

    rcall UART_PRINT                     ; Mandamos menu por USART
    rjmp MAIN                            ; Volvemos al bucle principal

;oooooooooooo CONFIG. UART oooooooooooooooooooooooo

initUART:

    ldi temp, LOW(UBRR_VALUE)             ; Parte baja del valor del Baud Rate
    sts UBRR0L, temp                      ; Cargamos velocidad de comunicacion
    ldi temp, HIGH(UBRR_VALUE)            ; Parte alta del Baud Rate
    sts UBRR0H, temp

    ldi temp, (1<<RXEN0)|(1<<TXEN0)       ; Habilitamos recepcion y transmision
    sts UCSR0B, temp                      ; Configuramos USART

    ldi temp,(1<<UCSZ01)|(1<<UCSZ00)     ; Configuramos datos de 8 bits
    sts UCSR0C, temp                      ; (8 bits, sin paridad, 1 de stop)
    ret 

;oooooooooooo ENVIAR UN CARACTER UART ooooooooooooo

; Lo que queremos transmitir tiene que estar guardado en 'temp'

UART_TX:

	UART_TX_ESPERAR:

				lds aux, UCSR0A             ; Checkea el estado del USART
				sbrs aux, UDRE0             ; Si no esta recibiendo, queda esperando
				rjmp UART_TX_ESPERAR
				sts UDR0, temp              ; Envia
	ret

/*oooooooooooooo Print el texto del UART ooooooooooooooooo

 Z apunta al comienzo del texto guardado en memoria
 Los mensajes terminan con 0
 */

UART_PRINT:

	UART_PRINT_LOOP:
			lpm temp, Z+                          ; Leemos un caracter y avanzamos Z
			tst temp                             ; Verificamos si el caracter = 0
			breq UART_PRINT_FIN                  ; Si es 0 terminamos de imprimir
			rcall UART_TX                        ; Enviamos caracter por UART
			rjmp UART_PRINT_LOOP                 ; Continuamos con el sig.
	UART_PRINT_FIN:
 ret     

/* ooooooooooooo Config. el Timer1 oooooooooooooooooooooooooooooooo

Timer1 en modo CTC, se limpia el solo cuando llega al OCR1A
Prescaler = 8
OCR1A = 77
Freq. de interrup.:			16 MHz : ( 8 x (77 + 1))	mas o menos unos 25.641 Hz

Como son 255 muestras:
							25641 : 255 = aprox. 100 Hz
-------------------------------------------------------------- */

TIMER1_INIT:

    clr temp	
    sts TCCR1A, temp                     ; Limpiamos la config. A del Timer1
    sts TCCR1B, temp                     ; Paramos y limpiamos el Timer1

    sts TCNT1H, temp                     ; Inicializamos el contador
    sts TCNT1L, temp

;----------- Comparacion --------------------

    ldi temp, HIGH(OCR1A_VALUE)           ; Parte alta de OCR1A
    sts OCR1AH, temp
    ldi temp, LOW(OCR1A_VALUE)            ; Parte baja de OCR1A
    sts OCR1AL, temp

    ldi temp, (1<<OCIE1A)                 ; Habilitamos el Compare Match A
    sts TIMSK1, temp

    ldi temp, (1<<WGM12)|(1<<CS11)        ; CTC + Prescaler en 8
    sts TCCR1B, temp                      ; Comienza a contar el Timer1

    ret

/* ------ Interrup. del Timer-------------

 Cada vez que TCNT1 = OCR1A:
	1 - Se verifica cual senal se selecciono
	2 - Lee una muestra de esa LUT
	3 - Envia esa muestra al DAC
	4 - Pasa al indice sig.
*/

TIMER1_COMPA_ISR:
    push temp			; Guarda r16

    in temp,SREG		; Guarda el registro de estado
    push temp
    push dato			; Guarda r19
    push aux			; Guarda r20

    push r30			; Guarda ZL
    push r31			; Guarda ZH

;----------- Checkear la Selecc. --------------
    cpi seleccion,6                     ; Compara select con 6
    breq LEER_SENAL6                    ; Si es la 6, lee esa LUT
    cpi seleccion,12                    ; Compara con 12
    breq LEER_SENAL12                   ; Lee LUt de 12

    rjmp FIN_ISR                        ; Si no se selecciono nada sale

;oooooooooooooooooooooooo LEER LUT - SENAL 6 oooooooooooooooooooooooooooooooooooo

LEER_SENAL6:
    ldi ZH, HIGH(LUT_SENAL6<<1)          ; Carga la direccion de la LUT
    ldi ZL, LOW(LUT_SENAL6<<1)

    add ZL, indice                       ; Suma el numero de muestra
    adc ZH, r1                           ; Suma el acarreo a parte alta

    lpm dato, Z                          ; Lee la LUT_SENAL6[x]
    rjmp ENVIAR_DAC

;oooooooooooooooooooooooo LEER LUT - SENAL 12 oooooooooooooooooooooooooooooooooooo

LEER_SENAL12:
    ldi ZH, HIGH(LUT_SENAL12<<1)
    ldi ZL, LOW(LUT_SENAL12<<1)

    add ZL, indice                       ; Lo mismo que con la Senl 6
    adc ZH, r1                           

    lpm dato, Z       
	                  
;-----------------------------------------
ENVIAR_DAC:
    rcall DAC_OUTPUT          ; Manda el valor al DAC
    inc indice                ; Pasa a la sig. muestra

; Como el registro del indice es de 8 bit, pasa a 0 por si solo, no hay que 
; setearlo cuando rermina la LUT

FIN_ISR:

    pop r31                             ; Pop la ZH
    pop r30                             ; ZL
    pop aux                             ; Pop r20
    pop dato                            ; Pop r19
    pop temp                            ; Recupera SREG guardado
    out SREG, temp
    pop temp                            ; Recupera el r16
reti

;oooooooooooooooooo Salidas ooooooooooooooooooooooo

DAC_OUTPUT:
    mov temp, dato
    andi temp, 0xFC          ; Guarda en 'temp' solo los bits 2-7 de la muestra

    in aux, PORTD
    andi aux, 0x03			; Lee y guarda solo los bits de la UART (de PORTD)
    or temp, aux					; Guarda los bit del UART
    out PORTD, temp					; Out los bit del 2 al 7
;---------------------------
    mov temp, dato
    andi temp, 0x03			; Guarda los Bits 0-1 de la muestra
						; No es necesario leer PORTB xq no hay nada mas conectado
    or temp, aux		; Agregamos PB1 y PB0 del DAC
    out PORTB, temp		; Mandamos los bits al DAC
  ret

;oooooooooooooooooooooooo Mensajes del MENU en USART ooooooooooooooooooooooooooooo
MENU:
    .db 13,10		; '13' es dar HOME y '10' es Salto de Linea, "13,10" es como un ENTER
    .db "============================", 13,10
    .db "GENERADOR DE ONDAS - DAC R-2R ", 13,10
    .db "============================", 13,10
    .db " 1 = Senal 6 (Onda) ", 13, 10
    .db " 2 = Senal 12 (Rampa) ", 13, 10
    .db " M/m = Mostrar menu ", 13, 10
	.db " R/r = Reset sistema", 13, 10
    .db " Seleccione una opcion:", 13, 10, 0		; El 0 es como un "End Print"

MSG_SENAL6:
    .db 13,10
    .db " o Senal 6 seleccionada",	13, 10, 0

MSG_SENAL12:
    .db 13,10
    .db " o Senal 12 seleccionada ", 13, 10, 0

;oooooooooooooooooooooooo LUT SENAL 6 ooooooooooooooooooooooooooooooooooooooooo
; En las dos LUT hay 256 muestras

LUT_SENAL6:

    .db 73,74,75,75,74,73,73,73
    .db 73,72,71,69,68,67,67,67
    .db 68,68,67,65,62,61,59,57
    .db 56,55,55,54,54,54,55,55
    .db 55,55,55,55,54,53,51,50
    .db 49,49,52,61,77,101,132,169
    .db 207,238,255,254,234,198,154,109
    .db 68,37,17,5,0,1,6,13
    .db 20,28,36,45,52,57,61,64
    .db 65,66,67,68,68,69,70,71
    .db 71,71,71,71,71,71,71,71
    .db 72,72,72,73,73,74,75,75
    .db 76,77,78,79,80,81,82,83
    .db 84,86,88,91,93,96,98,100
    .db 102,104,107,109,112,115,118,121
    .db 123,125,126,127,127,127,127,127
    .db 126,125,124,121,119,116,113,109
    .db 105,102,98,95,92,89,87,84
    .db 81,79,77,76,75,74,73,72
    .db 70,69,68,67,67,67,68,68
    .db 68,69,69,69,69,69,69,69
    .db 70,71,72,73,73,74,74,75
    .db 75,75,75,75,75,74,74,73
    .db 73,73,73,72,72,72,71,71
    .db 71,71,71,71,71,70,70,70
    .db 69,69,69,69,69,70,70,70
    .db 69,68,68,67,67,67,67,66
    .db 66,66,65,65,65,65,65,65
    .db 65,65,64,64,63,63,64,64
    .db 65,65,65,65,65,65,65,64
    .db 64,64,64,64,64,64,64,65
    .db 65,65,66,67,68,69,71,72

;oooooooooooooooooooooooo LUT SENAL 12 oooooooooooooooooooooooooooooooooooooooo
LUT_SENAL12:

    .db 0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06
    .db 0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E
    .db 0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16
    .db 0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E
    .db 0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26
    .db 0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E
    .db 0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36
    .db 0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E
    .db 0x3F,0x40,0x41,0x42,0x43,0x44,0x45,0x46
    .db 0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E
    .db 0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56
    .db 0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E
    .db 0x5F,0x60,0x61,0x62,0x63,0x64,0x65,0x66
    .db 0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E
    .db 0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76
    .db 0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E
    .db 0x7F,0x80,0x81,0x82,0x83,0x84,0x85,0x86
    .db 0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E
    .db 0x8F,0x90,0x91,0x92,0x93,0x94,0x95,0x96
    .db 0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E
    .db 0x9F,0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6
    .db 0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE
    .db 0xAF,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6
    .db 0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE
    .db 0xBF,0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6
    .db 0xC7,0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE
    .db 0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,0xD5,0xD6
    .db 0xD7,0xD8,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE
    .db 0xDF,0xE0,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6
    .db 0xE7,0xE8,0xE9,0xEA,0xEB,0xEC,0xED,0xEE
    .db 0xEF,0xF0,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6
    .db 0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE

	; Colorin Colorado, el cuento se ha acabado 