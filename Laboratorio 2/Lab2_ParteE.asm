.include "m328pdef.inc"

.equ F_CPU = 16000000
.equ BAUD  = 115200
.equ BPS   = 8    ; (16MHz / (16 * 115200)) - 1 = 8

; estados de la maquina
.equ ESTADO_CERRADA  = 0
.equ ESTADO_ABRIENDO = 1
.equ ESTADO_ABIERTA  = 2
.equ ESTADO_CERRANDO = 3
.equ ESTADO_DETENIDO = 4

; registros
.def aux1             = r16
.def aux2             = r17
.def estado_actual    = r18   ; mantiene el estado del sistema 0-4
.def flag_obstaculo   = r19   ; 1 = se detecto un obstaculo y requiere reporte
.def sreg_temp        = r20   ; resguardo de SREG en ISR

; interrupciones
.cseg
.org 0x0000
    rjmp RESET
.org PCI0addr                 ; vector de interrupción pin Change 0 PCINT0_vect
    rjmp ISR_PCINT0

; inicialización
RESET:
    ; stack pointer
    ldi aux1, HIGH(RAMEND)
    out SPH, aux1
    ldi aux1, LOW(RAMEND)
    out SPL, aux1

    ; configuro entradas (PORTC: PC0..PC3 con Pull-ups)
    clr aux1
    out DDRC, aux1
    ldi aux1, 0b00001111
    out PORTC, aux1

    ; configuro entrada de interrupción PORTB: PB0 con Pull-up
    cbi DDRB, PB0
    sbi PORTB, PB0

    ; configuro salidas PORTD: PD4 = MotorAbrir, PD5 = MotorCerrar, PD6 = Alarma
    in aux1, DDRD
    ori aux1, (1<<PD4) | (1<<PD5) | (1<<PD6)
    out DDRD, aux1
    
    ; apagar salidas inicialmente (CORREGIDO: lleva ~)
    in aux1, PORTD
    andi aux1, ~((1<<PD4) | (1<<PD5) | (1<<PD6))
    out PORTD, aux1

    ; inicializamos USART
    rcall INICIALIZAR_USART

    ; configurar Interrupción por cambio de pin PCINT0 en PB0
    lds aux1, PCICR
    ori aux1, (1<<PCIE0)       ; habilita grupo PCIE0 en portb
    sts PCICR, aux1

    lds aux1, PCMSK0
    ori aux1, (1<<PCINT0)      ; habilita máscara para pin PB0
    sts PCMSK0, aux1

    ; reset 
    clr flag_obstaculo
    ldi estado_actual, ESTADO_CERRADA

    ; habilitar interrupciones globales 
    sei

    ; mensaje inicial al encender
    ldi zl, LOW(msg_cerrada * 2)
    ldi zh, HIGH(msg_cerrada * 2)
    rcall ENVIAR_TEXTO_USART

MAIN_LOOP:
    ; verificar si la ISR registró una detección de obstáculo
    tst flag_obstaculo
    breq verificar_maquina

    ; si hubo obstaculo, transmitir los mensajes por usart
    clr flag_obstaculo
    ldi zl, LOW(msg_obstaculo * 2)
    ldi zh, HIGH(msg_obstaculo * 2)
    rcall ENVIAR_TEXTO_USART

    ldi zl, LOW(msg_detenido * 2)
    ldi zh, HIGH(msg_detenido * 2)
    rcall ENVIAR_TEXTO_USART

verificar_maquina:
    rcall EVALUAR_MAQUINA_ESTADOS
    rcall RETARDO_DEBOUNCE
    rjmp MAIN_LOOP

; rutina de interrupcion ISR para sensor S3 (PB0)
ISR_PCINT0:
    in sreg_temp, SREG         ; guarda el registro de estado

    ; Comprobar si S3 (PB0) está en nivel bajo, obstáculo detectado
    sbic PINB, PB0
    rjmp FIN_ISR               ; si está en alto, no hay obstáculo

    ; solo actuar si la puerta se está moviendo 
    cpi estado_actual, ESTADO_ABRIENDO
    breq APAGADO_EMERGENCIA
    cpi estado_actual, ESTADO_CERRANDO
    breq APAGADO_EMERGENCIA
    rjmp FIN_ISR

APAGADO_EMERGENCIA:
    ; detiene inmediatamente motor abrir, Motor cerrar y alarma (CORREGIDO)
    in aux1, PORTD
    andi aux1, ~((1<<PD4) | (1<<PD5) | (1<<PD6))
    out PORTD, aux1

    ldi estado_actual, ESTADO_DETENIDO
    ldi flag_obstaculo, 1      ; indicar al main loop que envíe la alerta usart

FIN_ISR:
    out SREG, sreg_temp        ; restaura registro de estado
    reti

; maquina de Estados
EVALUAR_MAQUINA_ESTADOS:
    cpi estado_actual, ESTADO_CERRADA
    breq M_ESTADO_CERRADA

    cpi estado_actual, ESTADO_ABRIENDO
    breq M_ESTADO_ABRIENDO

    cpi estado_actual, ESTADO_ABIERTA
    breq M_ESTADO_ABIERTA

    cpi estado_actual, ESTADO_CERRANDO
    breq M_ESTADO_CERRANDO

    cpi estado_actual, ESTADO_DETENIDO
    breq M_ESTADO_DETENIDO
    ret

; estado 0: puerta cerrada
M_ESTADO_CERRADA:
    sbis PINC, PC0              ; se fija si se presiono BT_ABRIR
    rjmp INICIAR_APERTURA
    ret

INICIAR_APERTURA:
    sbis PINC, PC2              ; si ya está en S1, no abrir
    ret

    ; activa Motor Abrir y Alarma 
    in aux1, PORTD
    ori aux1, (1<<PD4) | (1<<PD6)
    out PORTD, aux1

    ldi estado_actual, ESTADO_ABRIENDO

    ldi zl, LOW(msg_abriendo * 2)
    ldi zh, HIGH(msg_abriendo * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; estado 1: puerta abriendo
M_ESTADO_ABRIENDO:
    ; comprueba si llego al final de carrera superior S1
    sbis PINC, PC2
    rjmp FIN_APERTURA

    ; comprueba si se presiono BT_CERRAR para invertir marcha
    sbis PINC, PC1
    rjmp INICIAR_CIERRE
    ret

FIN_APERTURA:
    ; Apaga Motor Abrir y Alarma (CORREGIDO: lleva ~)
    in aux1, PORTD
    andi aux1, ~((1<<PD4) | (1<<PD6))
    out PORTD, aux1

    ldi estado_actual, ESTADO_ABIERTA

    ldi zl, LOW(msg_abierta * 2)
    ldi zh, HIGH(msg_abierta * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; estado 2: puerta abierta
M_ESTADO_ABIERTA:
    sbis PINC, PC1              ; se fija si se presiono BT_CERRAR
    rjmp INICIAR_CIERRE
    ret

INICIAR_CIERRE:
    sbis PINC, PC3              ; si ya esta en S2, no cerrar
    ret

    ; Apaga Motor Abrir por seguridad (CORREGIDO: lleva ~) y activa Motor Cerrar y Alarma 
    in aux1, PORTD
    andi aux1, ~(1<<PD4)
    ori aux1, (1<<PD5) | (1<<PD6)
    out PORTD, aux1

    ldi estado_actual, ESTADO_CERRANDO

    ldi zl, LOW(msg_cerrando * 2)
    ldi zh, HIGH(msg_cerrando * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; estado 3: puerta cerrando
M_ESTADO_CERRANDO:
    ; comprueba si llego al final de carrera inferior S2
    sbis PINC, PC3
    rjmp FIN_CIERRE

    ; comprueba si se presiono BT_ABRIR para invertir marcha
    sbis PINC, PC0
    rjmp INICIAR_APERTURA
    ret

FIN_CIERRE:
    ; apaga Motor Cerrar y Alarma (CORREGIDO: lleva ~)
    in aux1, PORTD
    andi aux1, ~((1<<PD5) | (1<<PD6))
    out PORTD, aux1

    ldi estado_actual, ESTADO_CERRADA

    ldi zl, LOW(msg_cerrada * 2)
    ldi zh, HIGH(msg_cerrada * 2)
    rcall ENVIAR_TEXTO_USART
    ret

; estado 4: detenido por seguridad
M_ESTADO_DETENIDO:
    sbis PINC, PC0
    rjmp INICIAR_APERTURA

    sbis PINC, PC1
    rjmp INICIAR_CIERRE
    ret

; rutinas de comunicacion USART
INICIALIZAR_USART:
    ldi aux1, HIGH(BPS)
    sts UBRR0H, aux1
    ldi aux1, LOW(BPS)
    sts UBRR0L, aux1

    ldi aux1, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, aux1

    ldi aux1, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, aux1
    ret

ENVIAR_CARACTER:
    lds aux2, UCSR0A
    sbrs aux2, UDRE0
    rjmp ENVIAR_CARACTER
    sts UDR0, aux1
    ret

ENVIAR_TEXTO_USART:
    lpm aux1, z+
    tst aux1
    breq FIN_TEXTO
    rcall ENVIAR_CARACTER
    rjmp ENVIAR_TEXTO_USART
FIN_TEXTO:
    ret

; retardo anti rebote
RETARDO_DEBOUNCE:
    push r24
    push r25
    ldi r25, 100
d_loop1:
    ldi r24, 250
d_loop2:
    dec r24
    brne d_loop2
    dec r25
    brne d_loop1
    pop r25
    pop r24
    ret

; mensajes en la memoria flash (Alineados a 2 bytes)
msg_abriendo:  .db "Puerta abriendo.", 13, 10, 0, 0     
msg_abierta:   .db "Puerta abierta.", 13, 10, 0        
msg_cerrando:  .db "Puerta cerrando.", 13, 10, 0, 0     
msg_cerrada:   .db "Puerta cerrada.", 13, 10, 0         
msg_obstaculo: .db "Obstaculo detectado.", 13, 10, 0, 0    
msg_detenido:  .db "Movimiento detenido por seguridad.", 13, 10, 0, 0
   
