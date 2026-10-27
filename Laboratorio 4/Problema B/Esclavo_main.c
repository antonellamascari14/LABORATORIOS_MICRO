#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>

#define PIN_SS              PB2
#define PIN_MOSI            PB3
#define PIN_MISO            PB4
#define PIN_SCK             PB5

#define PIN_MOTOR_PWM       PD5
#define PIN_MOTOR_IN1       PD7
#define PIN_MOTOR_IN2       PB0

#define PIN_SERVO           PB1

#define PIN_LED_VERDE       PD2
#define PIN_LED_ROJO        PD4
#define PIN_BUZZER          PD3

#define CMD_NOP             0x00
#define CMD_START           0x01
#define CMD_STOP            0x02
#define CMD_SET_SPEED       0x03
#define CMD_PISTON          0x04
#define CMD_EMERGENCY       0x05
#define CMD_RESET_ALARM     0x06

#define STATUS_IDLE         0x10
#define STATUS_RUN          0x20
#define STATUS_PISTON       0x30
#define STATUS_EMERGENCY    0x40
#define SPI_ACK             0x06

#define SERVO_TICKS_REPOSO  2000
#define SERVO_TICKS_ACTIVO  3000

typedef enum {
	EST_IDLE = 0,
	EST_RUN,
	EST_PISTON,
	EST_EMERGENCIA
} EstadoEsclavo;

static volatile EstadoEsclavo estado_esclavo = EST_IDLE;
static volatile uint32_t milisegundos = 0;

ISR(TIMER0_OVF_vect) {
	milisegundos++;
}

static inline uint32_t obtener_millis(void) {
	uint32_t ms;
	uint8_t sreg = SREG;
	cli();
	ms = milisegundos;
	SREG = sreg;
	return ms;
}

static volatile uint8_t byte_indice = 0;
static volatile uint8_t comando_recibido = CMD_NOP;
static volatile uint8_t valor_recibido = 0;
static volatile bool    nueva_trama_disponible = false;

static uint8_t consigna_pwm = 0;
static uint8_t velocidad_actual = 0;
static uint32_t t_inicio_piston = 0;
static uint32_t t_ultimo_parpadeo = 0;
static bool flag_parpadeo = false;

void hardware_pwm_iniciar(void) {
	TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01) | (1 << CS00);
	TIMSK0 |= (1 << TOIE0);
	OCR0B = 0;

	DDRD |= (1 << PIN_MOTOR_PWM) | (1 << PIN_MOTOR_IN1);
	DDRB |= (1 << PIN_MOTOR_IN2);
	PORTD &= ~((1 << PIN_MOTOR_PWM) | (1 << PIN_MOTOR_IN1));
	PORTB &= ~(1 << PIN_MOTOR_IN2);

	DDRB |= (1 << PIN_SERVO);
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	ICR1 = 39999;
	OCR1A = SERVO_TICKS_REPOSO;
}

void motor_fijar_velocidad(uint8_t pwm) {
	velocidad_actual = pwm;
	if (pwm > 0) {
		PORTD |= (1 << PIN_MOTOR_IN1);
		PORTB &= ~(1 << PIN_MOTOR_IN2);
		OCR0B = pwm;
		} else {
		OCR0B = 0;
		PORTD &= ~(1 << PIN_MOTOR_IN1);
		PORTB &= ~(1 << PIN_MOTOR_IN2);
	}
}

void servo_posicionar(uint16_t ticks) {
	OCR1A = ticks;
}

void spi_esclavo_iniciar(void) {
	DDRB |= (1 << PIN_MISO);
	DDRB &= ~((1 << PIN_SS) | (1 << PIN_MOSI) | (1 << PIN_SCK));
	SPCR = (1 << SPE) | (1 << SPIE);
	SPDR = STATUS_IDLE;
}

ISR(SPI_STC_vect) {
	uint8_t dato = SPDR;

	if (byte_indice == 0) {
		comando_recibido = dato;
		byte_indice = 1;
		switch (estado_esclavo) {
			case EST_IDLE:        SPDR = STATUS_IDLE;      break;
			case EST_RUN:         SPDR = STATUS_RUN;       break;
			case EST_PISTON:      SPDR = STATUS_PISTON;    break;
			case EST_EMERGENCIA:  SPDR = STATUS_EMERGENCY; break;
			default:              SPDR = STATUS_IDLE;      break;
		}
		} else {
		valor_recibido = dato;
		byte_indice = 0;
		nueva_trama_disponible = true;
		SPDR = SPI_ACK;
	}
}

void procesar_trama_recibida(void) {
	uint8_t cmd = comando_recibido;
	uint8_t val = valor_recibido;

	switch (cmd) {
		case CMD_START:
		if (estado_esclavo != EST_EMERGENCIA) {
			estado_esclavo = EST_RUN;
			consigna_pwm = (val > 0) ? val : consigna_pwm;
			if (consigna_pwm == 0) consigna_pwm = 180;
			motor_fijar_velocidad(consigna_pwm);
			PORTD |= (1 << PIN_LED_VERDE);
			PORTD &= ~((1 << PIN_LED_ROJO) | (1 << PIN_BUZZER));
		}
		break;

		case CMD_STOP:
		if (estado_esclavo != EST_EMERGENCIA) {
			estado_esclavo = EST_IDLE;
			motor_fijar_velocidad(0);
			PORTD &= ~((1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER));
		}
		break;

		case CMD_SET_SPEED:
		consigna_pwm = val;
		if (estado_esclavo == EST_RUN) {
			motor_fijar_velocidad(consigna_pwm);
		}
		break;

		case CMD_PISTON:
		if (estado_esclavo == EST_RUN) {
			estado_esclavo = EST_PISTON;
			t_inicio_piston = obtener_millis();
			servo_posicionar(SERVO_TICKS_ACTIVO);
			PORTD |= (1 << PIN_BUZZER);
			_delay_ms(30);
			PORTD &= ~(1 << PIN_BUZZER);
		}
		break;

		case CMD_EMERGENCY:
		estado_esclavo = EST_EMERGENCIA;
		motor_fijar_velocidad(0);
		servo_posicionar(SERVO_TICKS_REPOSO);
		PORTD &= ~(1 << PIN_LED_VERDE);
		break;

		case CMD_RESET_ALARM:
		if (estado_esclavo == EST_EMERGENCIA) {
			estado_esclavo = EST_IDLE;
			motor_fijar_velocidad(0);
			PORTD &= ~((1 << PIN_LED_ROJO) | (1 << PIN_BUZZER));
		}
		break;

		default:
		break;
	}
}

int main(void) {
	DDRD |= (1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER);
	PORTD &= ~((1 << PIN_LED_VERDE) | (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER));

	hardware_pwm_iniciar();
	spi_esclavo_iniciar();
	sei();

	while (1) {
		uint32_t ahora = obtener_millis();

		if (nueva_trama_disponible) {
			nueva_trama_disponible = false;
			procesar_trama_recibida();
		}

		if (estado_esclavo == EST_PISTON) {
			if (ahora - t_inicio_piston >= 600) {
				servo_posicionar(SERVO_TICKS_REPOSO);
				estado_esclavo = EST_RUN;
			}
		}

		if (estado_esclavo == EST_EMERGENCIA) {
			if (ahora - t_ultimo_parpadeo >= 150) {
				t_ultimo_parpadeo = ahora;
				flag_parpadeo = !flag_parpadeo;
				if (flag_parpadeo) {
					PORTD |= (1 << PIN_LED_ROJO) | (1 << PIN_BUZZER);
					} else {
					PORTD &= ~((1 << PIN_LED_ROJO) | (1 << PIN_BUZZER));
				}
			}
		}
	}

	return 0;
}
