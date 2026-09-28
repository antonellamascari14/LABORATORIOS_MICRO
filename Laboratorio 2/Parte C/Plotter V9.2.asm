; LAB 2 C - Plotter 
; Arquitectura basada en LUTs

.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD = 9600
.equ UBRR_VAL = 103


; ============== Pines ====================
.equ pin_bajar_sol    = PD2
.equ pin_subir_sol    = PD3
.equ pin_mover_abajo  = PD4
.equ pin_mover_arriba = PD5
.equ pin_mover_izq    = PD7
.equ pin_mover_der    = PD6

; ============= Registos ========================
.def aux1     = r16
.def aux2     = r17
.def char_rx  = r18
.def contador = r19

; ================== Reset ==============
.cseg
.org 0x0000
    rjmp RESET

RESET:
    ldi aux1, HIGH(RAMEND)    ; Configuracin del Stack Pointer
    out SPH, aux1
    ldi aux1, LOW(RAMEND)
    out SPL, aux1

; -----------------------------------------------------
    ; Config. D2-D7 como salidas
    in aux1, DDRD					; las | son como un OR y las \ para indicar que sigue en la sig. linea
    ori aux1, (1<<pin_bajar_sol) | \
              (1<<pin_subir_sol) | \
              (1<<pin_mover_abajo) | \
              (1<<pin_mover_arriba) | \
              (1<<pin_mover_izq) | \
              (1<<pin_mover_der)

    out DDRD, aux1

; -------------Estado inicial ------------------------
 
    clr aux1
    out PORTD, aux1
    rcall lapiz_arriba

    ldi aux1, HIGH(UBRR_VAL)
    sts UBRR0H, aux1
    ldi aux1, LOW(UBRR_VAL)
    sts UBRR0L, aux1

    ldi aux1, (1<<RXEN0) | (1<<TXEN0)		; Habilitar transmision y recepcion
    sts UCSR0B, aux1

    ldi aux1, (1<<UCSZ01) | (1<<UCSZ00)		; 8 bits de datos
    sts UCSR0C, aux1

    rcall mostrar_menu

; ========================================================
main_loop:
    rcall recibir_caracter

	; Como el codigo es muy extenso, no sirve solo usar BRNE/BREQ, el alcance 
	; no es sufciciente, por lo que se usan para saltar wntre las etiqetas de
	; los RJMP, los cuales tienen un alcance mayor.

    cpi char_rx, '1'		; Seria Check 1
    brne check_2
    rjmp _triangulo

check_2:
    cpi char_rx, '2'
    brne check_3
    rjmp _circulo

check_3:
    cpi char_rx, '3'
    brne check_4
    rjmp _pentagrama

check_4:
    cpi char_rx, '4'
    brne check_p
    rjmp _casita

check_p:
    cpi char_rx, 'p'
    breq salto_pokemon
    cpi char_rx, 'P'
    brne check_t
	salto_pokemon:
		rjmp _pokemon

check_t:
    cpi char_rx, 't'
    breq salto_todas
    cpi char_rx, 'T'
    brne check_m
	salto_todas:
		rjmp _todas

check_m:
    cpi char_rx, 'm'
    breq salto_menu
    cpi char_rx, 'M'
    brne check_c
	salto_menu:
		rjmp _mostrar_menu

check_c:
    cpi char_rx, 'c'
    breq salto_centro
    cpi char_rx, 'C'
    brne check_w
	salto_centro:
		rjmp _centro_uart

check_w:
    cpi char_rx, 'w'
    breq salto_arriba
    cpi char_rx, 'W'
    brne check_s
	salto_arriba:
		rjmp _manual_arriba

check_s:
    cpi char_rx, 's'
    breq salto_abajo
    cpi char_rx, 'S'
    brne check_a
	salto_abajo:
		rjmp _manual_abajo

check_a:
    cpi char_rx, 'a'
    breq salto_izq
    cpi char_rx, 'A'
    brne check_d
	salto_izq:
		rjmp _manual_izq

check_d:
    cpi char_rx, 'd'
    breq salto_der
    cpi char_rx, 'D'
    brne comando_invalido
	salto_der:
		rjmp _manual_der

comando_invalido:		; Si se ingresa otra cosa, queda esperando
    rjmp main_loop

;=======================================

_triangulo:
    ldi ZL, low(MSG_TRIANGULO*2)
    ldi ZH, high(MSG_TRIANGULO*2)
    rcall mostrar_mensaje
    rcall dibujar_triangulo
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_circulo:
    ldi ZL, low(MSG_CIRCULO*2)
    ldi ZH, high(MSG_CIRCULO*2)
    rcall mostrar_mensaje
    rcall dibujar_circulo
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_pentagrama:
    ldi ZL, low(MSG_PENTAGRAMA*2)
    ldi ZH, high(MSG_PENTAGRAMA*2)
    rcall mostrar_mensaje
    rcall dibujar_pentagrama
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_casita:
    ldi ZL, low(MSG_CASITA*2)
    ldi ZH, high(MSG_CASITA*2)
    rcall mostrar_mensaje
    rcall dibujar_casita
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_pokemon:
    ldi ZL, low(MSG_POKEMON*2)
    ldi ZH, high(MSG_POKEMON*2)
    rcall mostrar_mensaje
    rcall dibujar_pokemon
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_todas:					; llama una separacion entre cada figura
    ldi ZL, low(MSG_TODAS*2)
    ldi ZH, high(MSG_TODAS*2)
    rcall mostrar_mensaje
    rcall dibujar_triangulo
    rcall separacion
    rcall dibujar_circulo
    rcall separacion
    rcall dibujar_pentagrama
    rcall separacion
    rcall dibujar_casita
    rcall separacion
    rcall dibujar_pokemon
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_mostrar_menu:
    ldi ZL, low(MSG_MENU*2)
    ldi ZH, high(MSG_MENU*2)
    rcall mostrar_mensaje
    rcall mostrar_menu
    rjmp main_loop

_centro_uart:
    ldi ZL, low(MSG_CENTRO*2)
    ldi ZH, high(MSG_CENTRO*2)
    rcall mostrar_mensaje
    rcall ir_al_centro
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_manual_arriba:
    ldi ZL, low(MSG_ARRIBA*2)
    ldi ZH, high(MSG_ARRIBA*2)
    rcall mostrar_mensaje
    rcall mover_arriba
    rcall mover_arriba
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_manual_abajo:
    ldi ZL, low(MSG_ABAJO*2)
    ldi ZH, high(MSG_ABAJO*2)
    rcall mostrar_mensaje
    rcall mover_abajo
    rcall mover_abajo
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_manual_izq:
    ldi ZL, low(MSG_IZQUIERDA*2)
    ldi ZH, high(MSG_IZQUIERDA*2)
    rcall mostrar_mensaje
    rcall mover_izq
    rcall mover_izq
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

_manual_der:
    ldi ZL, low(MSG_DERECHA*2)
    ldi ZH, high(MSG_DERECHA*2)
    rcall mostrar_mensaje
    rcall mover_der
	rcall mover_der
	    ldi ZL, low(MSG_FIN*2)
		ldi ZH, high(MSG_FIN*2)
		rcall mostrar_mensaje
    rjmp main_loop

;------------------------------------------
ir_al_centro:

    rcall lapiz_arriba  
    ldi contador, 40		; Mover a la izquierda

	centro_x:
		rcall mover_izq
		dec contador
		brne centro_x

		ldi contador, 60		; Mover abajo

	centro_y:
		rcall mover_abajo
		dec contador
		brne centro_y

		ret

; ========== Movimiento entre figuras (opcion Todas) ===============

desplazar_izquierda:
    rcall lapiz_arriba
    ldi contador,80

_l_desp_izq:
    rcall mover_izq
    dec contador
    brne _l_desp_izq
    ret

/* =========================================================
		Cada figura tiene su LUT correspondiente, esto nos
		da la posibilidad de modificar facil y rapidamente
		las mismas, sin necesidad de mapear el movimiento 
		haciendo uso de RCALLs para cada movimiento
*/

dibujar_triangulo:

    rcall lapiz_abajo
    ldi ZL, low(tabla_triangulo*2)
    ldi ZH, high(tabla_triangulo*2)
    rcall ejecutar_LUT
    ret

dibujar_circulo:
    rcall lapiz_abajo
    ldi ZL, low(tabla_circulo*2)
    ldi ZH, high(tabla_circulo*2)
    rcall ejecutar_LUT
    ret

dibujar_pentagrama:

    rcall lapiz_abajo
    ldi ZL, low(tabla_pentagrama*2)
    ldi ZH, high(tabla_pentagrama*2)
    rcall ejecutar_LUT
    ret

dibujar_casita:

    rcall lapiz_abajo
    ldi ZL, low(tabla_casita*2)
    ldi ZH, high(tabla_casita*2)
    rcall ejecutar_LUT
    ret

dibujar_pokemon:

    rcall lapiz_abajo
    ldi ZL, low(tabla_diglett*2)
    ldi ZH, high(tabla_diglett*2)
    rcall ejecutar_LUT
    ret

;======== Mover el Lapiz =============================
lapiz_abajo:

	cbi PORTD, pin_subir_sol		; Desactivar subir
    sbi PORTD, pin_bajar_sol		; Activar bajar

    rcall retardo_conmutacion		; Tiempo para el solenoide
    ret

lapiz_arriba:

    cbi PORTD, pin_bajar_sol	 ; Desactivar bajar
    sbi PORTD, pin_subir_sol	 ; Activar subir
    rcall retardo_conmutacion    ; Tiempo para el solenoide
    ret

; ================ Movimientos ===================
mover_arriba:

    sbi PORTD, pin_mover_arriba
    rcall retardo_pulso
    cbi PORTD, pin_mover_arriba
    rcall retardo_paso
    ret

mover_abajo:

    sbi PORTD, pin_mover_abajo
    rcall retardo_pulso
    cbi PORTD, pin_mover_abajo
    rcall retardo_paso
    ret

mover_izq:

    sbi PORTD, pin_mover_izq
    rcall retardo_pulso
    cbi PORTD, pin_mover_izq
    rcall retardo_paso
    ret

mover_der:

    sbi PORTD, pin_mover_der
    rcall retardo_pulso
    cbi PORTD, pin_mover_der
    rcall retardo_paso
    ret

mover_arriba_izq:

    sbi PORTD, pin_mover_arriba
    sbi PORTD, pin_mover_izq
    rcall retardo_pulso
    cbi PORTD, pin_mover_arriba
    cbi PORTD, pin_mover_izq
    rcall retardo_paso
    ret

mover_arriba_der:

    sbi PORTD, pin_mover_arriba
    sbi PORTD, pin_mover_der
    rcall retardo_pulso
    cbi PORTD, pin_mover_arriba
    cbi PORTD, pin_mover_der
    rcall retardo_paso
    ret

mover_abajo_izq:

    sbi PORTD, pin_mover_abajo
    sbi PORTD, pin_mover_izq
    rcall retardo_pulso
    cbi PORTD, pin_mover_abajo
    cbi PORTD, pin_mover_izq
    rcall retardo_paso
    ret

mover_abajo_der:

    sbi PORTD, pin_mover_abajo
    sbi PORTD, pin_mover_der
    rcall retardo_pulso
    cbi PORTD, pin_mover_abajo
    cbi PORTD, pin_mover_der
    rcall retardo_paso
    ret

; ====================== MENU UART ===========================

mostrar_menu:

    ldi ZL, low(texto_menu*2)
    ldi ZH, high(texto_menu*2)

menu_loop:
    lpm aux2, Z+
    cpi aux2, 0x00
    breq menu_fin
    rcall enviar_caracter
    rjmp menu_loop

menu_fin:
    ret

; ====================== MENSAJE DE OPERACION ======================
mostrar_mensaje:
		mensaje_loop:
			lpm aux2, Z+
			cpi aux2, 0x00
			breq mensaje_fin
			rcall enviar_caracter
			rjmp mensaje_loop

		mensaje_fin:
			ret

;======================  USART ===================
enviar_caracter:

	esperar_tx:
		lds aux1, UCSR0A
		sbrs aux1, UDRE0
		rjmp esperar_tx
		sts UDR0, aux2
		ret

	recibir_caracter:
		lds aux1, UCSR0A
		sbrs aux1, RXC0
		rjmp recibir_caracter
		lds char_rx, UDR0
		ret

; ============ Retardo por la Conmutacion del Solenoide ==============

retardo_conmutacion:
    push r23
    push r24
    push r25
    ldi r23, 25

d_c0:
    ldi r25, 200

d_c1:
    ldi r24, 250

d_c2:
    dec r24
    brne d_c2

    dec r25
    brne d_c1

    dec r23
    brne d_c0

    pop r25
    pop r24
    pop r23
    ret

; ==========  Retardo del PULSO  ================
retardo_pulso:

    push r23
    push r24
    push r25

    ldi r23, 10

d_p0:
    ldi r25, 200

d_p1:
    ldi r24, 250

d_p2:
    dec r24
    brne d_p2

    dec r25
    brne d_p1

    dec r23
    brne d_p0

    pop r25
    pop r24
    pop r23
    ret

; ================Retardo entre Pasos =======================
retardo_paso:
    push r23
    push r24
    push r25
    ldi r23, 1

d_s0:
    ldi r25, 200

d_s1:
    ldi r24, 250

d_s2:
    dec r24
    brne d_s2
    dec r25
    brne d_s1

    dec r23
    brne d_s0

    pop r25
    pop r24
    pop r23
    ret

; ================ Retraso entre las figuras =====================
retardo_entrefig:
    push r23
    push r24
    push r25
    ldi r23, 100

d_e0:
    ldi r25, 200

d_e1:
    ldi r24, 250

d_e2:
    dec r24
    brne d_e2

    dec r25
    brne d_e1

    dec r23
    brne d_e0

    pop r25
    pop r24
    pop r23
    ret
; ----------- Leer y ejecutar LUTs --------------

ejecutar_LUT:			; Las acciones segun el valor en la LUT

	leer_LUT:

		lpm aux2,Z+

		cpi aux2,0xFF
		breq fin_LUT

		cpi aux2,1
		breq lut_arriba

		cpi aux2,2
		breq lut_abajo

		cpi aux2,3
		breq lut_izq

		cpi aux2,4
		breq lut_der

		cpi aux2,5
		breq lut_arr_izq

		cpi aux2,6
		breq lut_arr_der

		cpi aux2,7
		breq lut_aba_izq

		cpi aux2,8
		breq lut_aba_der

		cpi aux2,0x09
		breq lut_lapiz_arriba

		cpi aux2,0x0A
		breq lut_lapiz_abajo

		cpi aux2,0x0B
		breq lut_separacion

		rjmp leer_LUT
;------------------------------------------
	lut_arriba:
		rcall mover_arriba
		rjmp leer_LUT

	lut_abajo:
		rcall mover_abajo
		rjmp leer_LUT

	lut_izq:
		rcall mover_izq
		rjmp leer_LUT

	lut_der:
		rcall mover_der
		rjmp leer_LUT

	lut_arr_izq:
		rcall mover_arriba_izq
		rjmp leer_LUT

	lut_arr_der:
		rcall mover_arriba_der
		rjmp leer_LUT

	lut_aba_izq:
		rcall mover_abajo_izq
		rjmp leer_LUT

	lut_aba_der:
		rcall mover_abajo_der
		rjmp leer_LUT

	lut_lapiz_arriba:
		rcall lapiz_arriba
		rjmp leer_LUT

	lut_lapiz_abajo:
		rcall lapiz_abajo
		rjmp leer_LUT

	lut_separacion:
		rcall separacion
		rjmp leer_LUT

	lut_retardo_entrefig:
		rcall retardo_entrefig
		rjmp leer_LUT

	fin_LUT:
		rcall lapiz_arriba
		ret

; ---------- Separacion entre figuras --------------
separacion:
    rcall lapiz_arriba
    ldi contador,30

sep_loop:
    rcall mover_izq
    dec contador
    brne sep_loop
    ret

; =============  Tablas LUT ================
	/* -------------------------------------------
	 Codigos de movimiento:		(dentro de las LUT)
	 1 arriba
	 2 abajo
	 3 izquierda
	 4 derecha
	 5 arriba izquierda
	 6 arriba derecha
	 7 abajo izquierda
	 8 abajo derecha
	 0x09 Levantar Lapiz
	 0x0A Bajar lapiz
	 0x0B Separacion
	 -------------------------------------------*/
	
tabla_triangulo:
.db 0x0A,0
.db 7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,0
.db 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,0
.db 4,4,4,4,4,4,4,4,4,4,4,4,4,4
.db 5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,0
.db 0x09,2,2,2,2,2
.db 0xFF,0

;--------------------------------------
tabla_circulo:
.db 0x09,4,4,4,4,4,4,4,4,4,0x0A,0	;Inicia en el centro del circulo 
.db 1,1,1,5,5,1,5,5
.db 5,5,5,5,3,5,5,3
.db 3,3,3,7,7,3,7,7
.db 7,7,7,7,2,7,7,2
.db 2,2,2,8,8,2,8,8
.db 8,8,8,8,4,8,8,4
.db 4,4,4,6,6,4,6,6
.db 6,6,6,6,1,6,6,1,1,0
.db 0x09,3,3,3,3,3,3,3				; Finaliza en el centro 
.db 3,3,0xFF,0

;--------------------------------
tabla_pentagrama:
.db 0x09,1,1,1,1,1,1,0x0A				; Inicia en el centro de la estrella
.db 8,2,8,2,8,2,8,2,8,2,8,2,8,2,8,5
.db 3,5,3,5,3,5,3,5,3,5,3,5,3,5,3,5
.db 3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4
.db 4,4,4,7,3,7,3,7,3,7,3,7,3,7,3,7
.db 3,7,3,7,3,4,4,6,1,6,1,6,1,6,1,6
.db 1,6,1,6,1,4,1,6
.db 0x09,2,2,2,2,2,2,0xFF				; Finaliza en el centro

;--------------------------------
tabla_casita:
.db 0x09,4,4,4,4,4,0x0A,0			;Inicia en el centro
.db 3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2
.db 2,2,2,2,2,2,2,2,2,2,4,4,4,4,4,4
.db 4,4,4,4,4,4,4,4,1,1,1,1,1,1,1,1
.db 1,1,1,1,5,5,5,5,5,5,5,7,7,7,7,7
.db 7,7,0x09,4,4,4,4,4,4,4,4,4,4,4,4,4
.db 4,0x0A,1,1,1,1,1,1,1,1,1,3,3,3,3,2
.db 2,2,2,2
.db 0x09,2,2,2,2,2,3,3,3,3,0xFF,0		;Finaliza en el centro
;---------------------------------
tabla_diglett:
.db 0x09,2,2,2,2,2,2,2,2,0			;Inicia mas abajo 
.db 2,2,2,2,2,2,0x0A,0
.db 4,4,4,4,4,4,4,4
.db 4,4,4,4,4,4
.db 1,1,1,1,1,1,1,1
.db 1,1,1,1,1,1,1,1
.db 1,1,1,1,1,5,5,1,5,0
.db 5,3,5,5,5,3,5,5
.db 3,3,3,3,3,0
.db 7,7,3,7,7,7,3,7
.db 7,2,7,7,2,2,2,2
.db 2,2,2,2,2,2,2,2
.db 2,2,2,2,2,2,2,2
.db 2,4,4,4,4,4,4,0
.db 4,4,4,4,4,4,4,4,0x09,0				; Termina el cuerpo  y levanta el lapiz
;------------------------------- Nariz-----------------------------
.db 1,1,1,1,1,1,1,1,1,1,1,1,0x0A,0			; Va al centro (Borde de la nariz) y baja el lapiz
.db 3,3,3,3,3,5,1,1,6,4,4,4,4,4			
.db 4,4,4,4,4,8,2,2,7,3,3,3,3,3,0x09,0		; Termina Nariz y sube lapiz
;-------------------------------- Ojos ---------------------------
.db 1,1,1,1,1,3,3,3						; Esq inf. der. del ojo izq
.db 0x0A,1,1,1,1,1,1,1
.db 3,3,3,3,2,2,2,2,2,2,2,4,4,4,4,1,1,1,1,1
.db 3,3,3,2,2,2,4,4,4,0x09							; Fin del ojo izq.
.db 2,2,4,4,4,4,0x0A,0								; Esq inf. izq. del ojo der. y bajo lapiz
.db 1,1,1,1,1,1,1,0
.db 4,4,4,2,2,2,2,2,2,2,3,3,3,3,1,1,1,1,1,0				; le saque un 4 para compensar la lapicera
.db 4,4,4,2,2,2,3,3,3,0x09								; Fin del ojo der y levanta el lapiz
;-------------------------------- Piedras--------------------------		
.db 3,3,3,3,3,3,3,2,2,2						 
.db 2,2,2,2
.db 2,2,2,2,2,3,3,3,3,3,3,3
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09		; Piedra
.db 4,4,4,4,4,2,2,2,2,0											; Deriva hasta el pto. de inicio de
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09		; la piedra siguiente
.db 4,4,4,4,4,4,4,2,2,0
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09
.db 4,4,4,4,4,4,2,0
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09
.db 4,4,4,4,4,4,1,1
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09
.db 4,4,4,4,4,4,1,0
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09
.db 4,4,4,4,1,5,5,0
.db 0x0A,3,7,3,2,8,2,8,2,8,4,8,1,6,1,4,1,5,5,3,5,1,3,0x09
.db 0xFF,0
; ==============  TEXTO DEL MENU UART  ==============

texto_menu:
.db 13,10,"=== MENU DE ACCIONES ===", 13, 10
.db " 1 : Triangulo", 13, 10
.db " 2 : Circulo", 13, 10
.db " 3 : Estrella ", 13, 10
.db " 4 : Casita ", 13, 10
.db " P : Pokemon", 13, 10
.db " T : Todas", 13, 10
.db " M : Menu ", 13, 10
.db " C : Centrado ", 13, 10
.db " WASD : Movimiento manual ", 13,10
.db " ---------------",13,10
.db " Opcion seleccionada :",13, 10
.db " --------------- ",13,10,0

; ============== MEensajes de seleccion ==============
MSG_TRIANGULO:
	.db 13,10,"Triangulo seleccionado"
	.db 13,10, "Ejecutando... Por favor espere.",0x00

MSG_CIRCULO:
	.db 13,10,"Circulo seleccionado"
	.db 13,10, "Ejecutando... Por favor espere.",0x00

MSG_PENTAGRAMA:
	.db 13,10,"Estrella seleccionada "
	.db 13,10, "Ejecutando... Por favor espere.",0x00

MSG_CASITA:
	.db 13,10,"Casita seleccionada "
	.db 13,10, "Ejecutando... Por favor espere.",0x00

MSG_POKEMON:
	.db 13,10,"Pokemon seleccionado"
	.db 13,10, "Ejecutando... Por favor espere.",0x00

MSG_TODAS:
	.db 13,10,"Dibujar todas las figuras seleccionado"
	.db 13,10, "Ejecutando... Por favor espere.",0x00

MSG_MENU:
	.db 13,10,"Abriendo Menu",13,10,0x00

MSG_CENTRO:
	.db 13,10,"Centrado seleccionado "
	.db 13,10, "Centrando... Por favor espere. ",0x00

MSG_ARRIBA:
	.db 13,10,"Movimiento arriba seleccionado ",0x00

MSG_ABAJO:
	.db 13,10,"Movimiento abajo seleccionado",0x00

MSG_IZQUIERDA:
	.db 13,10,"Movimiento izquierda seleccionado",0x00

MSG_DERECHA:
	.db 13,10,"Movimiento derecha sele1ccionado ",0x00
	
MSG_FIN:
	.db 13,10, "      Operacion Finalizada",13,10,0x00,0