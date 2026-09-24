.include "m328pdef.inc"

.equ f_cpu = 16000000
.equ baud  = 115200
.equ bps   = 8    ; (16MHz / (16 * 115200)) - 1 = 8

; estados de la maquina
.equ estado_cerrada  = 0
.equ estado_abriendo = 1
.equ estado_abierta  = 2
.equ estado_cerrando = 3
.equ estado_detenido = 4

; asignacion de registros
.def aux1            = r16
.def aux2            = r17
.def estado_actual    = r18   ; guarda el estado actual (0 al 4)
.def flag_obstaculo   = r19   ; 1 si salto el sensor de obstaculo
.def sreg_temp        = r20   ; guarda sreg durante la interrupcion

; vectores de interrupcion
.cseg
.org 0x0000
    rjmp reset
.org PCI0addr                 ; interrupcion por cambio de pin en portb
    rjmp isr_pcint0

; inicializacion del sistema
reset:
    ; configuramos del stack pointer
    ldi aux1, HIGH(RAMEND)
    out SPH, aux1
    ldi aux1, LOW(RAMEND)
    out SPL, aux1

    ; entradas en portc pc0 - pc3 con pull-up
    clr aux1
    out DDRC, aux1
    ldi aux1, 0b00001111
    out PORTC, aux1

    ; entrada del sensor s3 en pb0 con pull-up
    cbi DDRB, PB0
    sbi PORTB, PB0

    ; salidas en portd: pd4 (abrir), pd5 (cerrar), pd6 (alarma)
    in aux1, DDRD
    ori aux1, (1<<PD4) | (1<<PD5) | (1<<PD6)
    out DDRD, aux1
    
    ; apago todas las salidas al arrancar
    in aux1, PORTD
    andi aux1, ~((1<<PD4) | (1<<PD5) | (1<<PD6))
    out PORTD, aux1

    ; inicializo la comunicacion serie
    rcall inicializar_usart

    ; configuro la interrupcion pcint0 para pb0
    lds aux1, PCICR
    ori aux1, (1<<PCIE0)
    sts PCICR, aux1

    lds aux1, PCMSK0
    ori aux1, (1<<PCINT0)
    sts PCMSK0, aux1

    ; estado inicial
    clr flag_obstaculo
    ldi estado_actual, estado_cerrada

    ; habilito interrupciones globales
    sei

    ; mensaje inicial
    ldi zl, LOW(msg_cerrada * 2)
    ldi zh, HIGH(msg_cerrada * 2)
    rcall enviar_texto_usart

main_loop:
    ; me fijo si el sensor detecto un obstaculo
    tst flag_obstaculo
    breq verificar_maquina

    ; si hubo obstaculo mando los avisos por usart
    clr flag_obstaculo
    ldi zl, LOW(msg_obstaculo * 2)
    ldi zh, HIGH(msg_obstaculo * 2)
    rcall enviar_texto_usart

    ldi zl, LOW(msg_detenido * 2)
    ldi zh, HIGH(msg_detenido * 2)
    rcall enviar_texto_usart

verificar_maquina:
    rcall evaluar_maquina_estados
    rcall retardo_debounce
    rjmp main_loop

; interrupcion para el sensor de obstaculo s3 pb0
isr_pcint0:
    in sreg_temp, SREG

    ; si pb0 esta en alto no hay obstaculo
    sbic PINB, PB0
    rjmp fin_isr

    ; solo actuo si la puerta se estaba moviendo
    cpi estado_actual, estado_abriendo
    breq apagado_emergencia
    cpi estado_actual, estado_cerrando
    breq apagado_emergencia
    rjmp fin_isr

apagado_emergencia:
    ; apago motores y alarma 
    in aux1, PORTD
    andi aux1, ~((1<<PD4) | (1<<PD5) | (1<<PD6))
    out PORTD, aux1

    ldi estado_actual, estado_detenido
    ldi flag_obstaculo, 1

fin_isr:
    out SREG, sreg_temp
    reti

; maquina de estados principal
evaluar_maquina_estados:
    ; si estan apretados bt_abrir y bt_cerrar al mismo tiempo, no hago nada
    in aux1, PINC
    com aux1                  ; invierto los bits para evaluar en 1 los presionados
    andi aux1, (1<<PC0) | (1<<PC1)
    cpi aux1, (1<<PC0) | (1<<PC1)
    breq fin_evaluar          ; si los dos estan apretados, ignoro la orden

    cpi estado_actual, estado_cerrada
    breq m_estado_cerrada

    cpi estado_actual, estado_abriendo
    breq m_estado_abriendo

    cpi estado_actual, estado_abierta
    breq m_estado_abierta

    cpi estado_actual, estado_cerrando
    breq m_estado_cerrando

    cpi estado_actual, estado_detenido
    breq m_estado_detenido

fin_evaluar:
    ret

; estado 0: puerta cerrada
m_estado_cerrada:
    sbis PINC, PC0             ; se pulso bt_abrir
    rjmp iniciar_apertura
    ret

iniciar_apertura:
    sbis PINC, PC2             ; si ya esta en el final de carrera s1, no abro
    ret

    ; apago motor cerrar (por seguridad) y prendo motor abrir + alarma
    in aux1, PORTD
    andi aux1, ~(1<<PD5)
    ori aux1, (1<<PD4) | (1<<PD6)
    out PORTD, aux1

    ldi estado_actual, estado_abriendo

    ldi zl, LOW(msg_abriendo * 2)
    ldi zh, HIGH(msg_abriendo * 2)
    rcall enviar_texto_usart
    ret

; estado 1: puerta abriendo
m_estado_abriendo:
    sbis PINC, PC2             ; llego al final de carrera s1
    rjmp fin_apertura

    sbis PINC, PC1             ; apretaron bt_cerrar para invertir
    rjmp iniciar_cierre
    ret

fin_apertura:
    ; apago motor abrir y alarma
    in aux1, PORTD
    andi aux1, ~((1<<PD4) | (1<<PD6))
    out PORTD, aux1

    ldi estado_actual, estado_abierta

    ldi zl, LOW(msg_abierta * 2)
    ldi zh, HIGH(msg_abierta * 2)
    rcall enviar_texto_usart
    ret

; estado 2: puerta abierta
m_estado_abierta:
    sbis PINC, PC1             ; apretaron bt_cerrar
    rjmp iniciar_cierre
    ret

iniciar_cierre:
    sbis PINC, PC3             ; si ya esta en el final de carrera s2, no cierro
    ret

    ; apago motor abrir y prendo motor cerrar + alarma
    in aux1, PORTD
    andi aux1, ~(1<<PD4)
    ori aux1, (1<<PD5) | (1<<PD6)
    out PORTD, aux1

    ldi estado_actual, estado_cerrando

    ldi zl, LOW(msg_cerrando * 2)
    ldi zh, HIGH(msg_cerrando * 2)
    rcall enviar_texto_usart
    ret

; estado 3: puerta cerrando
m_estado_cerrando:
    sbis PINC, PC3             ; llego al final de carrera s2
    rjmp fin_cierre

    sbis PINC, PC0             ; apretaron bt_abrir para invertir
    rjmp iniciar_apertura
    ret

fin_cierre:
    ; apago motor cerrar y alarma
    in aux1, PORTD
    andi aux1, ~((1<<PD5) | (1<<PD6))
    out PORTD, aux1

    ldi estado_actual, estado_cerrada

    ldi zl, LOW(msg_cerrada * 2)
    ldi zh, HIGH(msg_cerrada * 2)
    rcall enviar_texto_usart
    ret

; estado 4: detenido por obstaculo
m_estado_detenido:
    sbis PINC, PC0
    rjmp iniciar_apertura

    sbis PINC, PC1
    rjmp iniciar_cierre
    ret

; comunicacion usart
inicializar_usart:
    ldi aux1, HIGH(bps)
    sts UBRR0H, aux1
    ldi aux1, LOW(bps)
    sts UBRR0L, aux1

    ldi aux1, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, aux1

    ldi aux1, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, aux1
    ret

enviar_caracter:
    lds aux2, UCSR0A
    sbrs aux2, UDRE0
    rjmp enviar_caracter
    sts UDR0, aux1
    ret

enviar_texto_usart:
    lpm aux1, z+
    tst aux1
    breq fin_texto
    rcall enviar_caracter
    rjmp enviar_texto_usart
fin_texto:
    ret

; retardo antirebote
retardo_debounce:
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

; mensajes guardados en flash
msg_abriendo:  .db "Puerta abriendo.", 13, 10, 0, 0     
msg_abierta:   .db "Puerta abierta.", 13, 10, 0        
msg_cerrando:  .db "Puerta cerrando.", 13, 10, 0, 0     
msg_cerrada:   .db "Puerta cerrada.", 13, 10, 0         
msg_obstaculo: .db "Obstaculo detectado.", 13, 10, 0, 0    
msg_detenido:  .db "Movimiento detenido por seguridad.", 13, 10, 0, 0
