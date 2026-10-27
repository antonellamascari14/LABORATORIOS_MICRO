#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>        


// pines
#define PIN_MATRIZ     PD6
#define PIN_HOLD       PD2
#define PIN_CALIBRAR   PD3


// configuracion de la matriz
#define MATRIZ_SERPENTINA   0   // 1 = las filas van en zigzag, 0 = todas en el mismo sentido

// configuracion del sensor
#define MPU_DIR             0x68   // direccion i2c (AD0 a GND)
#define MPU_REG_PWR         0x6B
#define MPU_REG_CONFIG      0x1A   // filtro pasa bajos interno
#define MPU_REG_DATOS       0x3B   // primer registro de aceleracion
#define SENS_ACEL           16384.0f   // sensibilidad del Acelerometro en ±2g (16384 LSB/g)
#define SENS_GIRO           131.0f     //  Sensibilidad del Giroscopio en ±250 deg/s (131 LSB/(deg/s))
#define RAD_A_GRAD          57.29578f

// el sentido de la burbuja depende de como este montado el sensor, si se mueve al reves se cambia aca
#define INVERTIR_X          0
#define INVERTIR_Y          0
#define INTERCAMBIAR_EJES   0   // 1 = roll mueve filas y pitch mueve columnas

// filtro y mapeo
#define ALFA_FILTRO         0.96f   // filtro complementario: 96% Giro, 4% Acel
#define DT                  0.015f  // tiempo de muestreo estimado por iteracion (15 ms)
#define LIMITE_GRADOS       30.0f   //  gradosque la burbuja llega al borde
#define ZONA_MUERTA         3.0f    // menos de 3 grados = centro
#define CENTRO_MATRIZ       3       // posicion de reposo fila 3, columna 3
#define RATIO_COMBINADA     0.5f    // si el eje menor es >= 50% del mayor se considera inclinacion combinada

// indicador de limite
#define FRAMES_PITIDO       8       // duraion del pitido limite 100ms
#define FRAMES_PARPADEO     24      // duracion del parpadeo visual 3 parpadeos
#define FRAMES_UART         20      // cada cuantas iteraciones se manda el angulo (250 ms)

#define NUM_LEDS            64

// notas en hz para el buzzer

#define DO4      262
#define MI4      330
#define SOL4     392
#define DO5      523
#define LA5      880
#define SILENCIO 0

#define FREQ_TO_ICR1(f) ((f > 0) ? ((F_CPU / (8UL * (f))) - 1) : 0) // calcula el valor del registro icr1 del Timer 1 en modo Fast PWM

// direcciones de la eeprom
#define DIR_FIRMA        ((uint8_t *)0)     // marca para saber si la eeprom ya fue inicializada
#define DIR_OFFSET_ROLL  ((uint16_t *)2)    // offset de roll en decimas de grado
#define DIR_OFFSET_PITCH ((uint16_t *)4)    // offset de pitch en decimas de grado
#define FIRMA_EEPROM     0xB6               // para validar si la EEPROM esta escrita

// colores (indices de la paleta)
#define COLOR_ROJO      0   // predomina x
#define COLOR_VERDE     1   // predomina y
#define COLOR_AZUL      2   // nivelado
#define COLOR_AMARILLO  3   // inclinacion combinada
#define COLOR_BLANCO    4   // confirmacion de calibracion

// matriz r, g, b 
const uint8_t paleta[5][3] = {
	{60,  0,  0},   // rojo
	{ 0, 60,  0},   // verde
	{ 0,  0, 60},   // azul
	{60, 40,  0},   // amarillo
	{40, 40, 40}    // blanco
};

// estados de la maquina de estados
typedef enum {
	ESTADO_INICIO,      // configura el sensor y arranca el filtro
	ESTADO_ACTIVO,      // mide, filtra y mueve la burbuja
	ESTADO_HOLD,        // posicion y color congelados
	ESTADO_CALIBRAR     // toma la posicion actual como nuevo cero
} estado_t;

estado_t estado_actual = ESTADO_INICIO;

// variables del sistema

float roll_f = 0;     // angulos filtrados sin calibrar
float pitch_f = 0;
int16_t offset_roll = 0;    // en decimas de grado, vienen de la eeprom
int16_t offset_pitch = 0;

float roll_rel = 0;   // angulos ya con el offset aplicado realtivos a 0
float pitch_rel = 0;
float roll_mostrado = 0;   // angulos que se mandan por uart (en hold quedan congelados)
float pitch_mostrado = 0;

uint8_t burbuja_fila = CENTRO_MATRIZ; // posicion actual en filas (0..7)
uint8_t burbuja_col = CENTRO_MATRIZ;
uint8_t burbuja_color = COLOR_AZUL; // color actual de la burbuja


uint8_t en_borde_previo = 0; // bandera de flanco para no repetir pitido en el borde
uint8_t frames_pitido = 0;    // contador de tiempo de sonido activo
uint8_t frames_parpadeo = 0;
uint8_t contador_uart = 0; // divisor de frecuencia para envios por UART

uint8_t pantalla[NUM_LEDS][3];

// guardan si el boton estaba apretado en la lectura anterior
uint8_t previo_hold = 0;
uint8_t previo_calibrar = 0;


// parte del uart (solo transmision)

void init_uart(unsigned int baud) {
	unsigned int ubrr = F_CPU / 16 / baud - 1;
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_enviar_char(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void uart_enviar_texto(const char *texto) {
	while (*texto) {
		uart_enviar_char(*texto++);
	}
}

void uart_enviar_numero(uint16_t numero) {
	char buffer[6];
	int i = 0;

	if (numero == 0) {
		uart_enviar_char('0');
		return;
	}

	while (numero > 0) { //allgoritmo de conversion de entero a texto (Base 10)
		buffer[i++] = (numero % 10) + '0';
		numero /= 10;
	}

	while (i > 0) {
		uart_enviar_char(buffer[--i]);
	}
}

// los angulos pueden ser negativos, asi que se manda el signo aparte
void uart_enviar_entero(int16_t numero) {
	if (numero < 0) {
		uart_enviar_char('-');
		numero = -numero;
	}
	uart_enviar_numero((uint16_t)numero);
}


// i2c (modulo twi del atmega, 400 khz)

void init_twi(void) {
	TWSR = 0;                                // preescaler 1
	TWBR = ((F_CPU / 400000UL) - 16) / 2;    // calculo para 400 kHz: TWBR = 12
	TWCR = (1 << TWEN);
}

// espera a que el twi termine, devuelve 0 si se colgo (sensor desconectado)
static uint8_t twi_esperar(void) {
	uint16_t timeout = 0;
	while (!(TWCR & (1 << TWINT))) {
		if (++timeout > 30000) return 0;
	}
	return 1;
}

//envia start o start repetido
static uint8_t twi_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	if (!twi_esperar()) return 0;
	uint8_t estado = TWSR & 0xF8;
	return (estado == 0x08 || estado == 0x10); // 0x08: START transmitido, 0x10: repite START
}

static void twi_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

// manda un byte y verifica que el estado sea el esperado 
static uint8_t twi_escribir(uint8_t dato, uint8_t esperado) {
	TWDR = dato; // carga el byte
	TWCR = (1 << TWINT) | (1 << TWEN);
	if (!twi_esperar()) return 0;
	return ((TWSR & 0xF8) == esperado); // 0x18: SLA+W ACK, 0x28: Data ACK, 0x40: SLA+R ACK
}

// Lee un byte del bus I2C enviando ACK o NACK al esclavo
static uint8_t twi_leer(uint8_t con_ack) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (con_ack ? (1 << TWEA) : 0);
	twi_esperar();
	return TWDR;  // dato recibido
}


// mpu6050

uint8_t mpu_escribir_reg(uint8_t reg, uint8_t valor) { // Escribe un valor en un registro especifico del sensor
	if (!twi_start()) { twi_stop(); return 0; }
	if (!twi_escribir((MPU_DIR << 1), 0x18)) { twi_stop(); return 0; }
	if (!twi_escribir(reg, 0x28)) { twi_stop(); return 0; }
	if (!twi_escribir(valor, 0x28)) { twi_stop(); return 0; }
	twi_stop();
	return 1;
}

// lee 14 bytes seguidos: acel x,y,z - temperatura - giro x,y,z
uint8_t mpu_leer_datos(uint8_t *buf) {
	if (!twi_start()) { twi_stop(); return 0; }
	if (!twi_escribir((MPU_DIR << 1), 0x18)) { twi_stop(); return 0; }
	if (!twi_escribir(MPU_REG_DATOS, 0x28)) { twi_stop(); return 0; }
	if (!twi_start()) { twi_stop(); return 0; }                    // start repetido
	if (!twi_escribir((MPU_DIR << 1) | 1, 0x40)) { twi_stop(); return 0; }

	for (uint8_t i = 0; i < 14; i++) {
		buf[i] = twi_leer(i < 13);    // el ultimo byte va con nack
	}
	twi_stop();
	return 1;
}

uint8_t mpu_inicializar(void) {
	if (!mpu_escribir_reg(MPU_REG_PWR, 0x00)) return 0;     // saca al sensor del modo sleep
	if (!mpu_escribir_reg(MPU_REG_CONFIG, 0x03)) return 0;  // pasa bajos de 44 hz
	return 1;
}

// lee el sensor y actualiza roll_f y pitch_f con el filtro complementario
// si es la primera lectura (inicializar = 1) parte directo del angulo del acelerometro
uint8_t mpu_actualizar(uint8_t inicializar) {
	uint8_t buf[14];
	if (!mpu_leer_datos(buf)) return 0;
        // reconstruccion de enteros de 16 bits signados desde el buffer i2c
	float ax = (int16_t)((buf[0] << 8) | buf[1]);
	float ay = (int16_t)((buf[2] << 8) | buf[3]);
	float az = (int16_t)((buf[4] << 8) | buf[5]);
	float gx = (int16_t)((buf[8] << 8) | buf[9]) / SENS_GIRO;    // grados/s
	float gy = (int16_t)((buf[10] << 8) | buf[11]) / SENS_GIRO;

	// angulos que da solo el acelerometro (con la gravedad como referencia)
	float roll_acel = atan2f(ay, az) * RAD_A_GRAD;
	float pitch_acel = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_A_GRAD;

	if (inicializar) {
		roll_f = roll_acel;
		pitch_f = pitch_acel;
		} else {
		// el giroscopio responde rapido pero deriva, el acelerometro es lento pero no deriva
		roll_f = ALFA_FILTRO * (roll_f + gx * DT) + (1.0f - ALFA_FILTRO) * roll_acel;
		pitch_f = ALFA_FILTRO * (pitch_f + gy * DT) + (1.0f - ALFA_FILTRO) * pitch_acel;
	}

	// se le resta el cero guardado en la calibracion
	roll_rel = roll_f - (offset_roll / 10.0f);
	pitch_rel = pitch_f - (offset_pitch / 10.0f);
	return 1;
}


// sonido timer 1, pwm en pb1

void init_buzzer(void) {
	TCCR1A = (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	DDRB |= (1 << PB1);
	PORTB &= ~(1 << PB1);
}

void sonar(uint16_t freq) {
	if (freq == SILENCIO) {
		TCCR1A &= ~(1 << COM1A1); // desconecta la salida del Timer en pb1
		PORTB &= ~(1 << PB1);
		} else {
		TCCR1A |= (1 << COM1A1);  // conecta la salida pwm no invertida al pin OC1A
		ICR1 = FREQ_TO_ICR1(freq); // establece el periodo para la frecuencia deseada
		OCR1A = ICR1 / 2;
	}
}

void detener_sonido(void) {
	sonar(SILENCIO);
}


// matriz ws2812b

// manda un byte a la matriz con el tiempo justo para 16 mhz (20 ciclos por bit)
static void ws_enviar_byte(uint8_t dato) {
	uint8_t alto = PORTD | (1 << PIN_MATRIZ);
	uint8_t bajo = PORTD & ~(1 << PIN_MATRIZ);
	uint8_t bits = 8;

	asm volatile( 
	"bucle_%=:                           \n\t"
	"out  %[puerto], %[alto]             \n\t"   // ciclo0: sube la linea inicio de bit
	"nop \n\t nop \n\t nop \n\t nop      \n\t"   // c1-c4 mantiene en alto
	"sbrs %[dato], 7                     \n\t"   // c5: si el bit es 1 salta la siguiente
	"out  %[puerto], %[bajo]             \n\t"   // c6: (solo bit 0) baja la linea
	"nop \n\t nop \n\t nop               \n\t"   // c7-c9
	"nop \n\t nop \n\t nop               \n\t"   // c10-c12
	"out  %[puerto], %[bajo]             \n\t"   // c13: (bit 1) baja la linea
	"lsl  %[dato]                        \n\t"   // c14 desplaza izq
	"nop \n\t nop                        \n\t"   // c15-c16
	"dec  %[bits]                        \n\t"   // c17 decrementa
	"brne bucle_%=                       \n\t"   // c18-c19 repite mientras haya bits
	: [dato] "+r" (dato), [bits] "+r" (bits)
	: [puerto] "I" (_SFR_IO_ADDR(PORTD)), [alto] "r" (alto), [bajo] "r" (bajo)
	);
}

// manda toda la memoria de pantalla a la matriz el orden es g, r, b
void matriz_mostrar(void) {
	uint8_t sreg = SREG;
	cli();
	for (uint8_t i = 0; i < NUM_LEDS; i++) {
		ws_enviar_byte(pantalla[i][1]); // green
		ws_enviar_byte(pantalla[i][0]); // red
		ws_enviar_byte(pantalla[i][2]); // blue
	}
	SREG = sreg;
	_delay_us(80);   // reset
}

// pasa de (fila, columna) al numero de led en la tira
uint8_t indice_led(uint8_t fila, uint8_t col) {
	if (MATRIZ_SERPENTINA && (fila & 1)) {
		return fila * 8 + (7 - col);
	}
	return fila * 8 + col;
}

void matriz_limpiar(void) {
	for (uint8_t i = 0; i < NUM_LEDS; i++) {
		pantalla[i][0] = 0;
		pantalla[i][1] = 0;
		pantalla[i][2] = 0;
	}
}

void matriz_pixel(uint8_t fila, uint8_t col, uint8_t color) {
	uint8_t i = indice_led(fila, col);
	pantalla[i][0] = paleta[color][0]; // carga la intensidad d elos colores r, g, b
	pantalla[i][1] = paleta[color][1];
	pantalla[i][2] = paleta[color][2];
}

void matriz_llenar(uint8_t color) {
	for (uint8_t f = 0; f < 8; f++) {
		for (uint8_t c = 0; c < 8; c++) {
			matriz_pixel(f, c, color);
		}
	}
}

// dibuja la burbuja (si visible = 0 deja la matriz apagada, se usa para el parpadeo)
void dibujar_burbuja(uint8_t visible) {
	matriz_limpiar();
	if (visible) {
		matriz_pixel(burbuja_fila, burbuja_col, burbuja_color);
	}
	matriz_mostrar();
}


// botones con polling y antirrebote 

uint8_t leer_hold(void) {
	return !(PIND & (1 << PIN_HOLD)); // devuelve 1 si el boton esta pulsado (activa en bajo)
}

uint8_t leer_calibrar(void) {
	return !(PIND & (1 << PIN_CALIBRAR));
}

// devuelve 1 solo en el instante en que se aprieta el boton
uint8_t hay_flanco(uint8_t actual, uint8_t *previo) {
	if (actual && !(*previo)) {
		*previo = 1;
		_delay_ms(20);
		return 1;
	}
	if (!actual && *previo) {
		*previo = 0;
		_delay_ms(20);
	}
	return 0;
}


// eeprom

void eeprom_inicializar(void) {
	if (eeprom_read_byte(DIR_FIRMA) != FIRMA_EEPROM) {
		eeprom_update_byte(DIR_FIRMA, FIRMA_EEPROM);
		eeprom_update_word(DIR_OFFSET_ROLL, 0);    // sin calibrar = offset cero
		eeprom_update_word(DIR_OFFSET_PITCH, 0);
	}
     // lee los que estaban almacenados de antes
	offset_roll = (int16_t)eeprom_read_word(DIR_OFFSET_ROLL);
	offset_pitch = (int16_t)eeprom_read_word(DIR_OFFSET_PITCH);

	// si los grados son de mas de 45 se descartan
	if (offset_roll > 450 || offset_roll < -450) offset_roll = 0;
	if (offset_pitch > 450 || offset_pitch < -450) offset_pitch = 0;
}


// logica de la burbuja

// convierte un angulo en una posicion 0..7 de la matriz
uint8_t angulo_a_indice(float angulo) {
	if (fabsf(angulo) < ZONA_MUERTA) {
		return CENTRO_MATRIZ;    // zona muerta: se queda en el centro
	}

	// saturacion: aunque el sensor siga inclinandose, la burbuja no sale de la matriz
	if (angulo > LIMITE_GRADOS) angulo = LIMITE_GRADOS;
	if (angulo < -LIMITE_GRADOS) angulo = -LIMITE_GRADOS;

	// -30 grados -> 0 y +30 grados -> 7
	uint8_t indice = (uint8_t)((angulo + LIMITE_GRADOS) * 7.0f / (2.0f * LIMITE_GRADOS) + 0.5f);
	if (indice > 7) indice = 7;
	return indice;
}

// calcula posicion y color de la burbuja a partir de roll_rel y pitch_rel
void calcular_burbuja(void) {
	float ang_x = INTERCAMBIAR_EJES ? pitch_rel : roll_rel;   // mueve columnas
	float ang_y = INTERCAMBIAR_EJES ? roll_rel : pitch_rel;   // mueve filas
	if (INVERTIR_X) ang_x = -ang_x;
	if (INVERTIR_Y) ang_y = -ang_y;

	burbuja_col = angulo_a_indice(ang_x); // asigna roll a mov horizontal
	burbuja_fila = angulo_a_indice(ang_y); // asigna pitch a vertival

	float abs_x = fabsf(ang_x);
	float abs_y = fabsf(ang_y);
	float mayor = (abs_x > abs_y) ? abs_x : abs_y;
	float menor = (abs_x > abs_y) ? abs_y : abs_x;

	if (abs_x < ZONA_MUERTA && abs_y < ZONA_MUERTA) { // evaluacion de la paleta de colres
		burbuja_color = COLOR_AZUL;       // nivelado
		} else if (menor >= ZONA_MUERTA && menor >= RATIO_COMBINADA * mayor) {
		burbuja_color = COLOR_AMARILLO;   // los dos ejes inclinados parecido
		} else if (abs_x > abs_y) {
		burbuja_color = COLOR_ROJO;       // predomina X
		} else {
		burbuja_color = COLOR_VERDE;      // predomina Y
	}
}

// avisa (pitido y parpadeo) solo en el momento en que la burbuja llega al borde
void revisar_limite(void) {
	uint8_t en_borde = (burbuja_fila == 0 || burbuja_fila == 7 || burbuja_col == 0 || burbuja_col == 7);

	if (en_borde && !en_borde_previo) {
		frames_pitido = FRAMES_PITIDO;
		frames_parpadeo = FRAMES_PARPADEO;
		sonar(LA5);
	}
	en_borde_previo = en_borde;

	// el pitido se apaga solo despues de unas iteraciones
	if (frames_pitido > 0) {
		frames_pitido--;
		if (frames_pitido == 0) detener_sonido();
	}
}

// manda los angulos por uart cada cierto numero de iteraciones
void enviar_angulos(uint8_t hold) {
	contador_uart++;
	if (contador_uart < FRAMES_UART) return;
	contador_uart = 0;

	uart_enviar_texto("Roll: ");
	uart_enviar_entero((int16_t)roll_mostrado);
	uart_enviar_texto("  Pitch: ");
	uart_enviar_entero((int16_t)pitch_mostrado);
	uart_enviar_texto(hold ? "  [HOLD ON]\r\n" : "  [HOLD OFF]\r\n");
}


// estados

void estado_inicio(void) {
	uart_enviar_texto("\r\n BURBUJA DE NIVEL \r\n");

	// si el sensor no responde se muestra todo rojo y se reintenta
	while (!mpu_inicializar()) {
		uart_enviar_texto("Error: MPU6050 no responde\r\n");
		matriz_llenar(COLOR_ROJO);
		matriz_mostrar();
		_delay_ms(500);
		matriz_limpiar();
		matriz_mostrar();
		_delay_ms(500);
	}

	_delay_ms(100);              // el sensor tarda un poco en estabilizar
	mpu_actualizar(1);           // primera lectura: el filtro parte del acelerometro

	uart_enviar_texto("Offset roll (decimas): ");
	uart_enviar_entero(offset_roll);
	uart_enviar_texto("  Offset pitch (decimas): ");
	uart_enviar_entero(offset_pitch);
	uart_enviar_texto("\r\n");

	// animacion de inicio: destello azul
	matriz_llenar(COLOR_AZUL);
	matriz_mostrar();
	sonar(DO4);
	_delay_ms(150);
	sonar(SOL4);
	_delay_ms(150);
	detener_sonido();

	en_borde_previo = 0;
	estado_actual = ESTADO_ACTIVO;
}

// lee, filtra y mueve la burbuja
void estado_activo(void) {
	while (1) {
		if (hay_flanco(leer_hold(), &previo_hold)) {
			uart_enviar_texto("HOLD activado\r\n");
			estado_actual = ESTADO_HOLD;
			return;
		}
		if (hay_flanco(leer_calibrar(), &previo_calibrar)) {
			estado_actual = ESTADO_CALIBRAR;
			return;
		}

		if (mpu_actualizar(0)) {
			calcular_burbuja();
			roll_mostrado = roll_rel;
			pitch_mostrado = pitch_rel;
			revisar_limite();
		}

		// parpadeo: durante unas iteraciones la burbuja se apaga de a ratos
		uint8_t visible = 1;
		if (frames_parpadeo > 0) {
			visible = ((frames_parpadeo >> 2) & 1) ? 0 : 1;
			frames_parpadeo--;
		}
		dibujar_burbuja(visible);

		enviar_angulos(0);
		_delay_ms(10);
	}
}

// la burbuja queda congelada, pero se sigue leyendo el sensor para que el filtro no pierda la referencia
void estado_hold(void) {
	frames_parpadeo = 0;
	frames_pitido = 0;
	detener_sonido();
	dibujar_burbuja(1);

	while (1) {
		if (hay_flanco(leer_hold(), &previo_hold)) {
			uart_enviar_texto("HOLD desactivado\r\n");
			estado_actual = ESTADO_ACTIVO;
			return;
		}

		mpu_actualizar(0);    // solo mantiene el filtro, no se toca burbuja_fila/col/color

		enviar_angulos(1);
		_delay_ms(10);
	}
}

// toma la inclinacion actual como nuevo cero y lo guarda en eeprom
void estado_calibrar(void) {
	offset_roll = (int16_t)(roll_f * 10.0f);
	offset_pitch = (int16_t)(pitch_f * 10.0f);
	eeprom_update_word(DIR_OFFSET_ROLL, (uint16_t)offset_roll);
	eeprom_update_word(DIR_OFFSET_PITCH, (uint16_t)offset_pitch);

	uart_enviar_texto("Calibrado. Offset roll: ");
	uart_enviar_entero(offset_roll);
	uart_enviar_texto("  Offset pitch: ");
	uart_enviar_entero(offset_pitch);
	uart_enviar_texto(" (decimas de grado)\r\n");

	// confirmacion: destello blanco con dos tonos
	matriz_llenar(COLOR_BLANCO);
	matriz_mostrar();
	sonar(DO5);
	_delay_ms(120);
	sonar(SOL4);
	_delay_ms(120);
	detener_sonido();

	en_borde_previo = 0;
	estado_actual = ESTADO_ACTIVO;
}


// main

int main(void) {
	// salida de la matriz
	DDRD |= (1 << PIN_MATRIZ);
	PORTD &= ~(1 << PIN_MATRIZ);

	// botones de hold y calibrar como entradas con pull-up
	DDRD &= ~((1 << PIN_HOLD) | (1 << PIN_CALIBRAR));
	PORTD |= (1 << PIN_HOLD) | (1 << PIN_CALIBRAR);

	init_uart(9600);
	init_twi();
	init_buzzer();
	eeprom_inicializar();

	// matriz apagada al arrancar
	matriz_limpiar();
	matriz_mostrar();

	while (1) {
		switch (estado_actual) {
			case ESTADO_INICIO:    estado_inicio();    break;
			case ESTADO_ACTIVO:    estado_activo();    break;
			case ESTADO_HOLD:      estado_hold();      break;
			case ESTADO_CALIBRAR:  estado_calibrar();  break;
		}
	}
}