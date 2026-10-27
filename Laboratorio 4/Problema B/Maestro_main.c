#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define LCD_ADDR        0x4E

#define CMD_START       0x01
#define CMD_STOP        0x02
#define CMD_SPEED       0x03
#define CMD_PISTON      0x04
#define CMD_EMERGENCY   0x05
#define CMD_RESET_ALARM 0x06

#define TEMP_LIMITE     50

#define ADC_CH_POT      0
#define ADC_CH_LDR      1

#define PIN_UART_TX     PD1
#define PIN_BOTON       PD2
#define PIN_DHT11       PD3

#define PIN_SS          PB2
#define PIN_MOSI        PB3
#define PIN_MISO        PB4
#define PIN_SCK         PB5

typedef enum {
	ESTADO_PARO = 0,
	ESTADO_MARCHA,
	ESTADO_DESVIO_PIEZA,
	ESTADO_EMERGENCIA
} estado_maestro_t;

static estado_maestro_t estado_actual = ESTADO_PARO;
static uint16_t contador_piezas = 0;
static uint8_t velocidad_pwm = 0;
static uint8_t velocidad_enviada = 255;
static uint8_t temp_motor = 25;
static uint16_t ldr_base = 500;
static uint8_t ldr_bloqueado = 0;
static volatile uint8_t evento_boton = 0;
static const char *ultimo_cmd_str = "STOP ";

void uart_init(void) {
	UBRR0H = 0;
	UBRR0L = 103;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_enviar_char(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void uart_imprimir(const char *str) {
	while (*str) {
		uart_enviar_char(*str++);
	}
}

ISR(INT0_vect) {
	evento_boton = 1;
}

void twi_iniciar(void) {
	TWSR = 0x00;
	TWBR = 72;
	TWCR = (1 << TWEN);
}

void twi_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void twi_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(50);
}

void twi_escribir(uint8_t dato) {
	TWDR = dato;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void lcd_enviar_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t dato = (nibble & 0xF0) | (rs ? 0x01 : 0x00) | 0x08;
	twi_start();
	twi_escribir(LCD_ADDR);
	twi_escribir(dato | 0x04);
	_delay_us(2);
	twi_escribir(dato & ~0x04);
	_delay_us(50);
	twi_stop();
}

void lcd_enviar_byte(uint8_t valor, uint8_t rs) {
	lcd_enviar_nibble(valor & 0xF0, rs);
	lcd_enviar_nibble((valor << 4) & 0xF0, rs);
}

void lcd_cmd(uint8_t cmd) {
	lcd_enviar_byte(cmd, 0);
	if (cmd <= 0x02) {
		_delay_ms(2);
	}
}

void lcd_data(char d) {
	lcd_enviar_byte((uint8_t)d, 1);
}

void lcd_init(void) {
	twi_iniciar();
	_delay_ms(50);
	lcd_enviar_nibble(0x30, 0);
	_delay_ms(5);
	lcd_enviar_nibble(0x30, 0);
	_delay_us(150);
	lcd_enviar_nibble(0x30, 0);
	_delay_ms(1);
	lcd_enviar_nibble(0x20, 0);
	lcd_cmd(0x28);
	lcd_cmd(0x0C);
	lcd_cmd(0x06);
	lcd_cmd(0x01);
	_delay_ms(2);
}

void lcd_texto(uint8_t fila, uint8_t col, const char *str) {
	lcd_cmd((fila == 0) ? (0x80 + col) : (0xC0 + col));
	while (*str) lcd_data(*str++);
}

void adc_init(void) {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_leer(uint8_t canal) {
	ADMUX = (ADMUX & 0xF0) | (canal & 0x07);
	_delay_us(10);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void spi_init(void) {
	DDRB |= (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS);
	DDRB &= ~(1 << PIN_MISO);
	PORTB |= (1 << PIN_SS);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1);
}

uint8_t spi_tx_byte(uint8_t b) {
	SPDR = b;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

void enviar_trama(uint8_t cmd, uint8_t val) {
	PORTB &= ~(1 << PIN_SS);
	_delay_us(200);
	spi_tx_byte(cmd);
	_delay_us(300);
	spi_tx_byte(val);
	_delay_us(200);
	PORTB |= (1 << PIN_SS);
	_delay_ms(2);
}

uint8_t dht11_leer(void) {
	uint8_t bits[5] = {0, 0, 0, 0, 0};
	uint8_t i, j;
	uint16_t timeout;

	DDRD |= (1 << PIN_DHT11);
	PORTD &= ~(1 << PIN_DHT11);
	_delay_ms(18);
	PORTD |= (1 << PIN_DHT11);
	_delay_us(30);

	DDRD &= ~(1 << PIN_DHT11);
	PORTD |= (1 << PIN_DHT11);

	timeout = 1000;
	while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }
	timeout = 1000;
	while (!(PIND & (1 << PIN_DHT11))) { if (--timeout == 0) return temp_motor; }
	timeout = 1000;
	while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) return temp_motor; }

	cli();
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 8; j++) {
			timeout = 1000;
			while (!(PIND & (1 << PIN_DHT11))) { if (--timeout == 0) { sei(); return temp_motor; } }
			_delay_us(35);
			if (PIND & (1 << PIN_DHT11)) {
				bits[i] |= (1 << (7 - j));
				timeout = 1000;
				while (PIND & (1 << PIN_DHT11)) { if (--timeout == 0) { sei(); return temp_motor; } }
			}
		}
	}
	sei();

	if (bits[4] == ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF) && bits[2] > 0) {
		return bits[2];
	}
	return temp_motor;
}

int main(void) {
	DDRD &= ~(1 << PIN_BOTON);
	PORTD |= (1 << PIN_BOTON);

	EICRA = (1 << ISC01);
	EIMSK = (1 << INT0);

	uart_init();
	adc_init();
	spi_init();
	lcd_init();
	sei();

	char buf[20];
	uint8_t btn_prev = 1;
	uint8_t ciclos_dht = 0;
	uint8_t ciclos_lcd = 0;

	lcd_texto(0, 0, "SISTEMA LISTO   ");
	_delay_ms(400);
	lcd_cmd(0x01);

	ldr_base = adc_leer(ADC_CH_LDR);

	estado_actual = ESTADO_PARO;
	enviar_trama(CMD_STOP, 0);
	ultimo_cmd_str = "STOP ";

	while (1) {
		uint8_t btn_actual = (PIND & (1 << PIN_BOTON)) ? 1 : 0;
		uint8_t boton_presionado = 0;

		if (evento_boton || (btn_prev == 1 && btn_actual == 0)) {
			evento_boton = 0;
			EIFR = (1 << INTF0);
			boton_presionado = 1;
			_delay_ms(180);
			btn_actual = (PIND & (1 << PIN_BOTON)) ? 1 : 0;
		}
		btn_prev = btn_actual;

		velocidad_pwm = (uint8_t)(adc_leer(ADC_CH_POT) >> 2);
		uint16_t ldr_actual = adc_leer(ADC_CH_LDR);

		ciclos_dht++;
		if (ciclos_dht >= 30) {
			ciclos_dht = 0;
			uint8_t t = dht11_leer();
			if (t > 0 && t < 95) {
				temp_motor = t;
			}
		}

		switch (estado_actual) {
			case ESTADO_PARO:
			if (temp_motor >= TEMP_LIMITE) {
				enviar_trama(CMD_EMERGENCY, temp_motor);
				ultimo_cmd_str = "EMERG";
				estado_actual = ESTADO_EMERGENCIA;
			}
			else if (boton_presionado) {
				ldr_base = adc_leer(ADC_CH_LDR);
				ldr_bloqueado = 0;
				enviar_trama(CMD_START, velocidad_pwm);
				ultimo_cmd_str = "START";
				velocidad_enviada = velocidad_pwm;
				estado_actual = ESTADO_MARCHA;
			}
			break;

			case ESTADO_MARCHA:
			if (temp_motor >= TEMP_LIMITE) {
				enviar_trama(CMD_EMERGENCY, temp_motor);
				ultimo_cmd_str = "EMERG";
				estado_actual = ESTADO_EMERGENCIA;
			}
			else if (boton_presionado) {
				enviar_trama(CMD_STOP, 0);
				ultimo_cmd_str = "STOP ";
				estado_actual = ESTADO_PARO;
			}
			else {
				int16_t diff_ldr = (int16_t)ldr_actual - (int16_t)ldr_base;
				if (abs(diff_ldr) > 120) {
					if (!ldr_bloqueado) {
						estado_actual = ESTADO_DESVIO_PIEZA;
					}
					} else {
					ldr_bloqueado = 0;
				}

				int16_t diff = (int16_t)velocidad_pwm - (int16_t)velocidad_enviada;
				if (diff >= 3 || diff <= -3) {
					velocidad_enviada = velocidad_pwm;
					enviar_trama(CMD_SPEED, velocidad_pwm);
					ultimo_cmd_str = "SPEED";
				}
			}
			break;

			case ESTADO_DESVIO_PIEZA:
			ldr_bloqueado = 1;
			contador_piezas++;
			enviar_trama(CMD_PISTON, 1);
			ultimo_cmd_str = "PISTN";
			estado_actual = ESTADO_MARCHA;
			break;

			case ESTADO_EMERGENCIA:
			if (boton_presionado && temp_motor < TEMP_LIMITE) {
				enviar_trama(CMD_RESET_ALARM, 0);
				enviar_trama(CMD_STOP, 0);
				ultimo_cmd_str = "STOP ";
				estado_actual = ESTADO_PARO;
			}
			break;

			default:
			estado_actual = ESTADO_PARO;
			break;
		}

		ciclos_lcd++;
		if (ciclos_lcd >= 6) {
			ciclos_lcd = 0;
			if (estado_actual == ESTADO_MARCHA || estado_actual == ESTADO_DESVIO_PIEZA) {
				sprintf(buf, "RUN  V:%3u T:%2uC", velocidad_pwm, temp_motor);
				} else if (estado_actual == ESTADO_PARO) {
				sprintf(buf, "STOP V:%3u T:%2uC", velocidad_pwm, temp_motor);
				} else {
				sprintf(buf, "EMERGENCIA T:%2uC", temp_motor);
			}
			lcd_texto(0, 0, buf);

			sprintf(buf, "P:%-3u CMD:%s ", contador_piezas, ultimo_cmd_str);
			lcd_texto(1, 0, buf);
		}

		_delay_ms(30);
	}
}
