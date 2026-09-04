;
; EV 4.asm
;
.include "m328pdef.inc"

; registros

.def r_aux1        = r16   
.def r_aux2        = r17    
.def reg_grupo     = r18    ; grupo actual de figuras (0, 1, 2)
.def reg_subfig    = r19    
.def reg_fig_act   = r20    ; indice lineal de la figura activa (0 a 5)
.def reg_tx_addr   = r21    ; byte de dirección/registro para MAX7219
.def reg_tx_data   = r22    ; byte de datos para MAX7219
.def reg_bit_cnt   = r23    ; contador de bits en transmisión serie
.def reg_btn_flags = r24    ; banderas de memorización de botones


.equ MAX_CS        = PB2    ; chip select 
.equ MAX_DIN       = PB3    ; data In 
.equ MAX_CLK       = PB5    ; clock 

.equ SW1_PIN       = PC4    ; boton 1
.equ SW2_PIN       = PC5    ; boton 2 

; registros internos 

.equ MAX_REG_DIG0   = 0x01
.equ MAX_REG_DECODE = 0x09
.equ MAX_REG_INTENS = 0x0A
.equ MAX_REG_SCAN   = 0x0B
.equ MAX_REG_SHUTDN = 0x0C
.equ MAX_REG_TEST   = 0x0F


.cseg
.org 0x0000
    rjmp PROGRAM_START


PROGRAM_START:
    ; iniciamos el stack pointer 

    ldi r_aux1, HIGH(RAMEND)
    out SPH, r_aux1
    ldi r_aux1, LOW(RAMEND)
    out SPL, r_aux1

    ; configurar pines de salida 

    ldi r_aux1, (1<<MAX_CS) | (1<<MAX_DIN) | (1<<MAX_CLK)
    out DDRB, r_aux1
    ldi r_aux1, (1<<MAX_CS)    ; CS en alto por defecto
    out PORTB, r_aux1

    ; configurar entradas para botones 

    ldi r_aux1, 0x00
    out DDRC, r_aux1
    ldi r_aux1, (1<<SW1_PIN) | (1<<SW2_PIN)
    out PORTC, r_aux1

    ; iniciar el controlador MAX7219

    rcall DRIVER_MAX7219_INIT

  
    clr reg_grupo
    clr reg_subfig
    clr reg_fig_act
    clr reg_btn_flags

    ; cargar la primera figura

    rcall DIBUJAR_FIGURA


MAIN_LOOP:
    rcall PROCESAR_PULSADORES

    ; calcular índice de figura - (Grupo * 2) + SubFigura

    mov r_aux1, reg_grupo
    lsl r_aux1
    add r_aux1, reg_subfig

    ; evalua si cambió el estado de la figura

    cp r_aux1, reg_fig_act
    breq SIN_CAMBIOS

    ; si cambió se actualiza el display

    mov reg_fig_act, r_aux1
    rcall DIBUJAR_FIGURA

SIN_CAMBIOS:
    rcall DEBOUNCE_DELAY
    rjmp MAIN_LOOP

; lectura y control de los botones

PROCESAR_PULSADORES:
    in r_aux1, PINC

    ; boton 1
    sbrc r_aux1, SW1_PIN
    rjmp _sw1_liberado

    sbrc reg_btn_flags, 0      
    rjmp _evaluar_sw2          ; lo ignora si se mantiene presionado

    sbr reg_btn_flags, (1<<0)  ; marca como presionado
    inc reg_grupo
    cpi reg_grupo, 3
    brne _evaluar_sw2
    clr reg_grupo              ; reinicia a 0 tras llegar a 2
    rjmp _evaluar_sw2

_sw1_liberado:
    cbr reg_btn_flags, (1<<0)  ; reestablece la flag al soltar

_evaluar_sw2:
    ; boton 2
    sbrc r_aux1, SW2_PIN
    rjmp _sw2_liberado

    sbrc reg_btn_flags, 1      
    rjmp _fin_sw

    sbr reg_btn_flags, (1<<1)  ;  presionado
    ldi r_aux2, 1
    eor reg_subfig, r_aux2     ; alterna entre 0 y 1 usando XOR
    rjmp _fin_sw

_sw2_liberado:
    cbr reg_btn_flags, (1<<1)  ; reestablece la flag

_fin_sw:
    ret

; dibuja la figura complet

DIBUJAR_FIGURA:
    ; calcular dirección base en la Flash: (PATRONES_FIGURAS * 2) + (reg_fig_act * 8)

    ldi ZL, LOW(PATRONES_FIGURAS * 2)
    ldi ZH, HIGH(PATRONES_FIGURAS * 2)

    mov r_aux1, reg_fig_act
    lsl r_aux1
    lsl r_aux1
    lsl r_aux1                 ; multiplicar por 8

    add ZL, r_aux1
    clr r_aux2
    adc ZH, r_aux2

    ; envia las 8 filas a los registros 0x01..0x08 del MAX7219

    ldi reg_tx_addr, MAX_REG_DIG0

_loop_filas:
    lpm reg_tx_data, Z+
    rcall SPI_SEND_FRAME

    inc reg_tx_addr
    cpi reg_tx_addr, 0x09
    brne _loop_filas

    ret

;inicializa el contador 

DRIVER_MAX7219_INIT:

    ; test de matriz desactivado
    ldi reg_tx_addr, MAX_REG_TEST
    ldi reg_tx_data, 0x00
    rcall SPI_SEND_FRAME

    ; sin decodificación BCD 
    ldi reg_tx_addr, MAX_REG_DECODE
    ldi reg_tx_data, 0x00
    rcall SPI_SEND_FRAME

    ; limite de escaneo: 8 filas (0 a 7)
    ldi reg_tx_addr, MAX_REG_SCAN
    ldi reg_tx_data, 0x07
    rcall SPI_SEND_FRAME

    ; ajusta la intensidad luminosa
    ldi reg_tx_addr, MAX_REG_INTENS
    ldi reg_tx_data, 0x05
    rcall SPI_SEND_FRAME

    ; habilita la operación normal
    ldi reg_tx_addr, MAX_REG_SHUTDN
    ldi reg_tx_data, 0x01
    rcall SPI_SEND_FRAME

    ; apaga todos los LEDs de la matriz
    ldi reg_tx_addr, MAX_REG_DIG0
    ldi reg_tx_data, 0x00
_clear_matrix:
    rcall SPI_SEND_FRAME
    inc reg_tx_addr
    cpi reg_tx_addr, 0x09
    brne _clear_matrix

    ret


; transimision spi-bit banging
; Transmite reg_tx_addr (byte alto) seguido de reg_tx_data (byte bajo)

SPI_SEND_FRAME:
    cbi PORTB, MAX_CS          ; activar CS (Nivel bajo)

    ; transmisión byte 1: registro
    mov r_aux2, reg_tx_addr
    rcall _TX_BYTE_BITBANG

    ; transmisión byte 2: dato
    mov r_aux2, reg_tx_data
    rcall _TX_BYTE_BITBANG

    sbi PORTB, MAX_CS          ; latch de datos (CS en alto)
    ret

_TX_BYTE_BITBANG:
    ldi reg_bit_cnt, 8
_bit_loop:
    cbi PORTB, MAX_DIN         ; DIN en bajo por defecto
    sbrc r_aux2, 7             ; Verificar Bit 7 
    sbi PORTB, MAX_DIN         ; DIN en alto si Bit 7 = 1

    sbi PORTB, MAX_CLK         ; flanco de bajada de reloj
    cbi PORTB, MAX_CLK         ; flanco de subdia
    lsl r_aux2                 ; desplazo los bits a la izquierda
    dec reg_bit_cnt
    brne _bit_loop
    ret


;retardo antirebote

DEBOUNCE_DELAY:
    push r25
    push r26
    push r27
    ldi r27, 5
_delay_l3:
    ldi r25, 80
_delay_l2:
    ldi r26, 200
_delay_l1:
    dec r26
    brne _delay_l1
    dec r25
    brne _delay_l2
    dec r27
    brne _delay_l3
    pop r27
    pop r26
    pop r25
    ret


PATRONES_FIGURAS:
    ; FIGURA 0: Carita Sonriendo
    .db 0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C

    ; FIGURA 1: Carita Guiñando
    .db 0x3C, 0x42, 0xA5, 0x81, 0xA1, 0x99, 0x42, 0x3C

    ; FIGURA 2: Corazón
    .db 0x00, 0x66, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00

    ; FIGURA 3: Carita :3
    .db 0x00, 0x0E, 0x62, 0x66, 0x02, 0x62, 0x6E, 0x00

    ; FIGURA 4: Asterisco
    .db 0x18, 0x99, 0x5A, 0xFF, 0xFF, 0x5A, 0x99, 0x18

    ; FIGURA 5: Carita XD
    .db 0x00, 0xAE, 0xA9, 0x49, 0x49, 0xA9, 0xAE, 0x00