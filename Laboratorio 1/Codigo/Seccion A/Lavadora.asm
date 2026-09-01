;
; TMI LAB 1 A.asm
.include "m328pdef.inc"

; DEFINICIÓN DE REGISTROS
.def TEMP          = r16
.def MODO_CARGA    = r17  ; 0: Ligera, 1: Media, 2: Pesada
.def CONTADOR      = r18  ; Registro para repeticiones
.def TIEMPO_GIRO   = r19  ; Parámetro de tiempo de giro
.def TIEMPO_PAUSA  = r20  ; Parámetro de tiempo de pausa
.def R_DEL1        = r21  ; Registros para subrutinas de retardo
.def R_DEL2        = r22
.def R_DEL3        = r23

.org 0x0000
    rjmp RESET

RESET:
    ; 1. Inicialización del Stack Pointer
    ldi TEMP, LOW(RAMEND)
    out SPL, TEMP
    ldi TEMP, HIGH(RAMEND)
    out SPH, TEMP

    ; 2. Configuración de Puertos I/O
    ; PORTB: Salidas para LEDs de estado (PB0..PB4)
    ldi TEMP, 0b00011111
    out DDRB, TEMP
    clr TEMP
    out PORTB, TEMP

    ; PORTC: Salidas para LEDs de carga (PC0..PC2) y Motores/LEDs (PC3..PC4)
    ldi TEMP, 0b00011111
    out DDRC, TEMP
    clr TEMP
    out PORTC, TEMP

    ; PORTD: Entradas con Pull-up habilitados en PD0..PD3
    ldi TEMP, 0b00000000
    out DDRD, TEMP
    ldi TEMP, 0b00001111
    out PORTD, TEMP

    ; Estado Inicial: Carga Ligera por defecto
    clr MODO_CARGA
    rcall ACTUALIZAR_LEDS_CARGA

; MÁQUINA DE ESTADOS: ESTADO DE ESPERA (IDLE)
ESTADO_IDLE:
    ; Encender LED "Listo" (PB0) y apagar los demás en PORTB
    ldi TEMP, (1 << PB0)
    out PORTB, TEMP

    ; Verificar pulsador de cambio de carga (PD1)
    sbis PIND, PD1
    rcall CAMBIAR_CARGA

    ; Verificar pulsador de inicio (PD0)
    sbis PIND, PD0
    rjmp VERIFICAR_SENSORES

    rjmp ESTADO_IDLE

CAMBIAR_CARGA:
    rcall DELAY_DEBOUNCE
    inc MODO_CARGA
    cpi MODO_CARGA, 3
    brne ACTUALIZAR_LEDS_CARGA
    clr MODO_CARGA           ; Reiniciar a Carga Ligera

ACTUALIZAR_LEDS_CARGA:
    ; Conservar estado de las salidas de motor (PC3..PC4)
    in TEMP, PORTC
    andi TEMP, 0b00011000
    cpi MODO_CARGA, 0
    breq CHK_LIGERA
    cpi MODO_CARGA, 1
    breq CHK_MEDIA

CHK_PESADA:
    ori TEMP, (1 << PC2)
    out PORTC, TEMP
    ret
CHK_LIGERA:
    ori TEMP, (1 << PC0)
    out PORTC, TEMP
    ret
CHK_MEDIA:
    ori TEMP, (1 << PC1)
    out PORTC, TEMP
    ret

VERIFICAR_SENSORES:
    rcall DELAY_DEBOUNCE
    ; La puerta debe estar cerrada (PD2 = 0) y el agua llena (PD3 = 0)
    sbic PIND, PD2
    rjmp ESTADO_IDLE        ; Puerta abierta: no inicia
    sbic PIND, PD3
    rjmp ESTADO_IDLE        ; Tanque sin agua: no inicia

    rjmp PROCESO_LAVADO

; PROCESO 1: LAVADO
PROCESO_LAVADO:
    ; Encender LED Lavado (PB1)
    ldi TEMP, (1 << PB1)
    out PORTB, TEMP

    ; Configurar tiempos según tipo de carga
    cpi MODO_CARGA, 0
    breq CARGA_LAVADO_LIGERA
    cpi MODO_CARGA, 1
    breq CARGA_LAVADO_MEDIA

CARGA_LAVADO_PESADA: ; Giro: 4s | Pausa: 3s
    ldi TIEMPO_GIRO, 4
    ldi TIEMPO_PAUSA, 3
    rjmp INICIAR_CICLO_LAVADO

CARGA_LAVADO_LIGERA: ; Giro: 2s | Pausa: 1s
    ldi TIEMPO_GIRO, 2
    ldi TIEMPO_PAUSA, 1
    rjmp INICIAR_CICLO_LAVADO

CARGA_LAVADO_MEDIA:  ; Giro: 3s | Pausa: 2s
    ldi TIEMPO_GIRO, 3
    ldi TIEMPO_PAUSA, 2

INICIAR_CICLO_LAVADO:
    ldi CONTADOR, 5          ; Repetir el proceso 5 veces

LOOP_LAVADO:
    ; Activar Motor Derecha (PC3)
    sbi PORTC, PC3
    mov r16, TIEMPO_GIRO
    rcall DELAY_SEGUNDOS
    cbi PORTC, PC3

    ; Pausa
    mov r16, TIEMPO_PAUSA
    rcall DELAY_SEGUNDOS

    dec CONTADOR
    brne LOOP_LAVADO

    rjmp PROCESO_CENTRIFUGADO

; PROCESO 2: CENTRIFUGADO
PROCESO_CENTRIFUGADO:
    ; Encender LED Centrifugado (PB2)
    ldi TEMP, (1 << PB2)
    out PORTB, TEMP

    ; Configurar tiempos según tipo de carga
    cpi MODO_CARGA, 0
    breq CARGA_CENT_LIGERA
    cpi MODO_CARGA, 1
    breq CARGA_CENT_MEDIA

CARGA_CENT_PESADA:  ; 21 segundos
    ldi TIEMPO_GIRO, 21
    rjmp INICIAR_CENTRIFUGADO

CARGA_CENT_LIGERA:  ; 15 segundos
    ldi TIEMPO_GIRO, 15
    rjmp INICIAR_CENTRIFUGADO

CARGA_CENT_MEDIA:   ; 18 segundos
    ldi TIEMPO_GIRO, 18

INICIAR_CENTRIFUGADO:
    ; Activar Motor Derecha (PC3)
    sbi PORTC, PC3
    mov r16, TIEMPO_GIRO
    rcall DELAY_SEGUNDOS
    cbi PORTC, PC3

    rjmp PROCESO_SECADO

; PROCESO 3: SECADO
PROCESO_SECADO:
    ; Encender LED Secado (PB3)
    ldi TEMP, (1 << PB3)
    out PORTB, TEMP

    ; Configurar tiempos según tipo de carga
    cpi MODO_CARGA, 0
    breq CARGA_SEC_LIGERA
    cpi MODO_CARGA, 1
    breq CARGA_SEC_MEDIA

CARGA_SEC_PESADA:   ; Giro: 9s | Pausa: 7s
    ldi TIEMPO_GIRO, 9
    ldi TIEMPO_PAUSA, 7
    rjmp INICIAR_SECADO

CARGA_SEC_LIGERA:   ; Giro: 5s | Pausa: 3s
    ldi TIEMPO_GIRO, 5
    ldi TIEMPO_PAUSA, 3
    rjmp INICIAR_SECADO

CARGA_SEC_MEDIA:    ; Giro: 7s | Pausa: 5s
    ldi TIEMPO_GIRO, 7
    ldi TIEMPO_PAUSA, 5

INICIAR_SECADO:
    ; 1. Giro a la Derecha (PC3)
    sbi PORTC, PC3
    mov r16, TIEMPO_GIRO
    rcall DELAY_SEGUNDOS
    cbi PORTC, PC3

    ; 2. Pausa
    mov r16, TIEMPO_PAUSA
    rcall DELAY_SEGUNDOS

    ; 3. Giro a la Izquierda (PC4)
    sbi PORTC, PC4
    mov r16, TIEMPO_GIRO
    rcall DELAY_SEGUNDOS
    cbi PORTC, PC4

    rjmp FIN_PROCESO

; ESTADO FINAL
FIN_PROCESO:
    ; Encender LED Fin (PB4) y apagar los demás
    ldi TEMP, (1 << PB4)
    out PORTB, TEMP

BUCLE_FIN:
    rjmp BUCLE_FIN  ; Mantener el sistema detenido hasta reinicio

; SUBRUTINAS DE RETARDO (Ajustadas a 16 MHz)

; Retardo en segundos. Recibe la cantidad de segundos en R16
DELAY_SEGUNDOS:
    tst r16
    breq FIN_DELAY_SEG
LOOP_SEG:
    rcall DELAY_1S
    ; Verificación de seguridad de la puerta durante ejecución
    sbic PIND, PD2
    rcall SEGURIDAD_PUERTA
    dec r16
    brne LOOP_SEG
FIN_DELAY_SEG:
    ret

; Detiene inmediatamente las salidas de motor si se abre la puerta
SEGURIDAD_PUERTA:
    cbi PORTC, PC3
    cbi PORTC, PC4
ESPERAR_CIERRE_PUERTA:
    sbic PIND, PD2
    rjmp ESPERAR_CIERRE_PUERTA
    ret

; Genera 1 Segundo exacto
DELAY_1S:
    ldi  R_DEL1, 82
    ldi  R_DEL2, 43
    ldi  R_DEL3, 0
L1: dec  R_DEL3
    brne L1
    dec  R_DEL2
    brne L1
    dec  R_DEL1
    brne L1
    ret

; Antirebote (~20ms)
DELAY_DEBOUNCE:
    ldi  R_DEL1, 130
    ldi  R_DEL2, 221
L2: dec  R_DEL2
    brne L2
    dec  R_DEL1
    brne L2
    ret