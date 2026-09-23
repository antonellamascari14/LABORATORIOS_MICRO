.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD = 115200
.equ BPS = 8    ; (16MHz / (16 * 115200)) - 1 = 8

.equ total_letras = 17    ; "jefe maestroo!!"
.equ cuadros_espera = 25  ; 25 cuadros * 8ms = 200ms por paso de píxel

; Registros
.def aux1                 = r16    
.def aux2                 = r17    
.def modo_activo         = r18   ; 0 es el mensaje, 1-3 figuras fijas
.def posicion_letra       = r19   ; letra actual del mensaje
.def desplazamiento_bits  = r20   ; avance de los bits
.def contador_filas       = r21    
.def contador_cuadros     = r22    
.def tecla_recibida       = r25    

.cseg
.org 0x0000
    rjmp INICIO

INICIO:
    ; Inicializamos el stack pointer 
    ldi aux1, HIGH(RAMEND)
    out SPH, aux1
    ldi aux1, LOW(RAMEND)
    out SPL, aux1

    ; Configurar pines de salida 
    ; PORTD como salidas  para filas 0..5
    in aux1, ddrd
    ori aux1, 0b11111100
    out ddrd, aux1

    ; PORTB como salidas 
    ldi aux1, 0b00111111
    out ddrb, aux1

    ; PORTC como salidas
    in aux1, ddrc
    ori aux1, 0b00001111
    out ddrc, aux1

    ; Inicializo usart y apago leds
    rcall inicializar_usart
    rcall apagar_filas
    rcall mostrar_menu

    clr modo_activo
    clr posicion_letra
    clr desplazamiento_bits
    ldi contador_cuadros, cuadros_espera

MAIN_LOOP:
    ; Compruebo si la computadora mandó un nuevo comando usart
    rcall recepcion_usart
    brcc siguiente_paso

    ; Procesar comando recibido
    mov aux1, tecla_recibida
    cpi aux1, 'm'
    breq menu
    cpi aux1, 'M' 
    breq menu

    cpi aux1, '0'
    brlo siguiente_paso
    cpi aux1, '4'             ; Opciones válidas: '0', '1', '2', '3'
    brsh siguiente_paso

    subi aux1, '0'
    mov modo_activo, aux1
    clr posicion_letra
    clr desplazamiento_bits

    mov aux1, tecla_recibida
    rcall enviar_caracter_usart
    ldi aux1, '\r'
    rcall enviar_caracter_usart 
    ldi aux1, '\n'
    rcall enviar_caracter_usart 
    rjmp siguiente_paso

menu: 
    rcall mostrar_menu_usart

siguiente_paso: 
    ; Dibuja 1 cuadro completo de barrido (8 ms)
    rcall refrescar_pantalla

    ; En modo figuras fijas, no actualiza la animación
    tst modo_activo
    brne MAIN_LOOP

    ; cuando muestra el mensaje, controla la velocidad del desplazamiento
    dec contador_cuadros 
    brne MAIN_LOOP

    ; Reiniciar contador de cuadros y mover el texto 1 píxel
    ldi contador_cuadros, cuadros_espera
    inc desplazamiento_bits
    cpi desplazamiento_bits, 8 
    brne MAIN_LOOP

    clr desplazamiento_bits
    inc posicion_letra
    cpi posicion_letra, total_letras
    brne MAIN_LOOP
    clr posicion_letra

    rjmp MAIN_LOOP
; Rutina de multiplexado de 8 filas
refrescar_pantalla:
    clr contador_filas

loop_escaneo:
    rcall apagar_filas 
    rcall obtener_patron_fila   ; Entrega el patrón de la fila en aux2
    rcall cargar_columnas       ; Asigna las columnas a los pines
    rcall encender_fila_actual ; Activa el ánodo de la fila correspondiente 
    rcall retardo_fila          ; Pausa de 1 ms por fila

    inc contador_filas
    cpi contador_filas, 8
    brne loop_escaneo

    rcall apagar_filas
    ret

; Obtener dato de la fila según el modo
obtener_patron_fila: 
    tst modo_activo
    breq patron_texto

    ; Figuras fijas 1-3
    mov aux1, modo_activo
    dec aux1

    ldi zl, low(PATRONES_FIGURAS * 2)
    ldi zh, high(PATRONES_FIGURAS * 2)
    
    lsl aux1
    lsl aux1
    lsl aux1
    add aux1, contador_filas
    
    clr r23                  ; Preserva la bandera carry
    add zl, aux1
    adc zh, r23
    lpm aux2, z
    ret

patron_texto:
    ; Letra K (actual)
    ldi zl, low(patrones_mensaje * 2)
    ldi zh, high(patrones_mensaje * 2)

    mov aux1, posicion_letra
    lsl aux1
    lsl aux1
    lsl aux1
    add aux1, contador_filas
    
    clr r23                  ; Preserva la bandera carry
    add zl, aux1
    adc zh, r23
    lpm aux2, z

    ; Letra K+1 (siguiente)

    ldi zl, low(patrones_mensaje * 2)
    ldi zh, high(patrones_mensaje * 2)

    mov aux1, posicion_letra
    inc aux1
    cpi aux1, total_letras
    brne siguiente_letra
    clr aux1

siguiente_letra:
    lsl aux1
    lsl aux1
    lsl aux1
    add aux1, contador_filas
    
    clr r23                  ; Preserva la bandera carry
    add zl, aux1
    adc zh, r23
    lpm aux1, z

    ; Combinar bits de ambas letras para desplazamiento suave
    mov r23, desplazamiento_bits
    tst r23
    breq fin_rotacion

rotar:
    lsl aux1
    rol aux2
    dec r23
    brne rotar

fin_rotacion:
    ret

; Control físico de pines 
apagar_filas:
    in aux1, portd
    andi aux1, 0b00000011   
    out portd, aux1

    in aux1, portb
    andi aux1, 0b11111000   
    out portb, aux1
    ret

encender_fila_actual:
    cpi contador_filas, 0
    breq _f0
    cpi contador_filas, 1
    breq _f1
    cpi contador_filas, 2
    breq _f2
    cpi contador_filas, 3
    breq _f3
    cpi contador_filas, 4
    breq _f4
    cpi contador_filas, 5
    breq _f5
    cpi contador_filas, 6
    breq _f6
    cpi contador_filas, 7
    breq _f7
    ret

_f0: sbi portd, pd2
     ret
_f1: sbi portd, pd3
     ret
_f2: sbi portd, pd4
     ret
_f3: sbi portd, pd5
     ret
_f4: sbi portd, pd6
     ret
_f5: sbi portd, pd7
     ret
_f6: sbi portb, pb0
     ret
_f7: sbi portb, pb1
     ret

; columnas 
cargar_columnas:
    mov r24, aux2

    ; inversión de bits para evitar efecto espejo
    ldi r23, 8
    clr r25
inv_bits:
    rol r24
    ror r25
    dec r23
    brne inv_bits
    mov r24, r25

    com r24                 ; inversión para cátodo en low

    ; bits 0-3 a pb2-pb5
    in aux1, portb
    andi aux1, 0b11000011
    mov r23, r24
    andi r23, 0x0f
    lsl r23
    lsl r23
    or aux1, r23
    out portb, aux1

    ; bits 4-7 a pc0-pc3
    in aux1, portc
    andi aux1, 0xf0
    mov r23, r24
    swap r23
    andi r23, 0x0f
    or aux1, r23
    out portc, aux1
    ret

; Retardo de 1 ms por fila a 16 MHz
retardo_fila:
    push r24
    push r25
    ldi r25, 40

bucle_f2:
    ldi r24, 133

bucle_f1:
    dec r24
    brne bucle_f1
    dec r25
    brne bucle_f2
    pop r25
    pop r24
    ret

inicializar_usart:
    ldi aux1, high(BPS)
    sts ubrr0h, aux1
    ldi aux1, low(BPS)
    sts ubrr0l, aux1

    ldi aux1, (1<<rxen0) | (1<<txen0)
    sts ucsr0b, aux1

    ldi aux1, (1<<ucsz01) | (1<<ucsz00)
    sts ucsr0c, aux1
    ret

enviar_caracter_usart:
    lds aux2, ucsr0a
    sbrs aux2, udre0
    rjmp enviar_caracter_usart
    sts udr0, aux1
    ret

mostrar_menu:
mostrar_menu_usart:
    ldi zl, low(texto_menu * 2)
    ldi zh, high(texto_menu * 2)
    rcall enviar_texto_usart
    ret

enviar_texto_usart:
    lpm aux1, z+
    tst aux1
    breq _fin_texto
    rcall enviar_caracter_usart
    rjmp enviar_texto_usart
_fin_texto:
    ret

recepcion_usart:
    lds aux1, ucsr0a
    sbrs aux1, rxc0
    clc
    sbrc aux1, rxc0
    sec
    brcc _fin_rx
    lds tecla_recibida, udr0
_fin_rx:
    ret

; memoria flash: menú
texto_menu:
    .db "   MATRIZ MULTIPLEXADA DIRECTA    ", 13, 10  
    .db "0: Mensaje (jefe maestroo!!)  ", 13, 10  
    .db "1: Corazon                    ", 13, 10  
    .db "2: Carita :3                  ", 13, 10  
    .db "3: Carita XD                  ", 13, 10  
    .db "Elija una opcion (0-3): ", 0, 0
           

patrones_mensaje:
    ; 0: ' '
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ; 1: 'J'
    .db 0x1e, 0x04, 0x04, 0x04, 0x04, 0x84, 0x78, 0x00
    ; 2: 'E'
    .db 0xfe, 0x80, 0x80, 0xf8, 0x80, 0x80, 0xfe, 0x00
    ; 3: 'F'
    .db 0xfe, 0x80, 0x80, 0xf8, 0x80, 0x80, 0x80, 0x00
    ; 4: 'E'
    .db 0xfe, 0x80, 0x80, 0xf8, 0x80, 0x80, 0xfe, 0x00
    ; 5: ' '
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ; 6: 'M'
    .db 0x81, 0xc3, 0xa5, 0x99, 0x81, 0x81, 0x81, 0x00
    ; 7: 'A'
    .db 0x3c, 0x42, 0x81, 0xff, 0x81, 0x81, 0x81, 0x00
    ; 8: 'E'
    .db 0xfe, 0x80, 0x80, 0xf8, 0x80, 0x80, 0xfe, 0x00
    ; 9: 'S'
    .db 0x3c, 0x42, 0x80, 0x3c, 0x02, 0x42, 0x3c, 0x00
    ; 10: 'T'
    .db 0xfe, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00
    ; 11: 'R'
    .db 0xfc, 0x82, 0x82, 0xfc, 0x88, 0x84, 0x82, 0x00
    ; 12: 'O'
    .db 0x3c, 0x42, 0x81, 0x81, 0x81, 0x42, 0x3c, 0x00
    ; 13: 'O'
    .db 0x3c, 0x42, 0x81, 0x81, 0x81, 0x42, 0x3c, 0x00
    ; 14: '!'
    .db 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x10, 0x00
    ; 15: '!'
    .db 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x10, 0x00
    ; 16: ' '
    .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

PATRONES_FIGURAS:
    ; FIGURA 1: Corazón
    .db 0x00, 0x66, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00

    ; FIGURA 2: Carita :3
    .db 0x00, 0x0E, 0x62, 0x66, 0x02, 0x62, 0x6E, 0x00

    ; FIGURA 3: Carita XD
    .db 0x00, 0xAE, 0xA9, 0x49, 0x49, 0xA9, 0xAE, 0x00
