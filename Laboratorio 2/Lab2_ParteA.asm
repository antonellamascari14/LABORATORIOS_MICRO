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