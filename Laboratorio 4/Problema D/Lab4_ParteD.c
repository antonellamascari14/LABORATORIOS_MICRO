#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>


#define PIN_MATRIZ    PD6
#define PIN_JOY_SW    PD2
#define PIN_REINICIO  PD3

#define CANAL_X       0
#define CANAL_Y       1
#define CANAL_RUIDO   2



#define INVERTIR_X          1   
#define INVERTIR_Y          1   
#define MATRIZ_SERPENTINA   0   // 1 = las filas van en zigzag, 0 = todas en el mismo sentido

// joystick
#define CENTRO_JOY          512
#define ZONA_MUERTA         150    // fuera de este rango (+-) se considera movimiento
#define REPETICION_CURSOR   15    // iteraciones entre pasos si se mantiene el joystick (~200 ms)

// tiempos del juego
#define TIEMPO_ENCENDIDO    500    // ms que se ilumina cada elemento de la secuencia
#define TIEMPO_PAUSA        200    // ms entre elementos
#define ESPERA_LIBRE        250    // iteraciones de ~12 ms sin tocar nada = ~3 s en modo libre

#define MAX_NIVEL           32
#define NUM_LEDS            64
#define NUM_COLORES         6

// notas en hz (estoy reciclando las notas del piano electrónico)
#define DO3      131
#define DO4      262
#define RE4      294
#define MI4      330
#define FA4      349
#define SOL4     392
#define LA4      440
#define DO5      523
#define SILENCIO 0

#define FREQ_TO_ICR1(f) ((f > 0) ? ((F_CPU / (8UL * (f))) - 1) : 0)

// direcciones de la eeprom
#define DIR_FIRMA     ((uint8_t *)0)    // marca para saber si la eeprom ya fue inicializada
#define DIR_RECORD    ((uint8_t *)1)    // record historico (niveles completados)
#define DIR_ULTIMO    ((uint8_t *)2)    // niveles completados en la ultima partida
#define DIR_SEMILLA   ((uint16_t *)4)   // semilla para el generador aleatorio
#define FIRMA_EEPROM  0xA5

// colores (indices de la paleta)
#define COLOR_ROJO      0
#define COLOR_VERDE     1
#define COLOR_AZUL      2
#define COLOR_AMARILLO  3
#define COLOR_MAGENTA   4
#define COLOR_CELESTITO      5
#define COLOR_BLANCO    6   // es para ver el movimiento del cursor, no se usa en la secuencia

// r, g, b (intensidad baja)
const uint8_t paleta[7][3] = {
	{60,  0,  0},   // rojo
	{ 0, 60,  0},   // verde
	{ 0,  0, 60},   // azul
	{60, 40,  0},   // amarillo
	{60,  0, 40},   // magenta
	{ 0, 50, 50},   // celestito
	{40, 40, 40} // blanco
};

// un tono distinto para cada color
const uint16_t tonos[NUM_COLORES] = { DO4, RE4, MI4, FA4, SOL4, LA4 };

// estados de la maquina de estados
typedef enum {
	ESTADO_INICIO,      //  bienvenida y record
	ESTADO_ESPERA,      // modo libre: cursor movible y boton cambia el color
	ESTADO_MOSTRAR,     // reproduce la secuencia
	ESTADO_TURNO,       // el jugador repite la secuencia
	ESTADO_EXITO,       // si la secuencia se completa, sube de nivel
	ESTADO_ERROR        // fallo, animacion y reinicio
} estado_t;

estado_t estado_actual = ESTADO_INICIO;

// variables del juego

uint8_t sec_fila[MAX_NIVEL]; // guarda la fila asignada a cada elemento de la secuencia
uint8_t sec_col[MAX_NIVEL]; // este la columna
uint8_t sec_color[MAX_NIVEL]; // este el color
uint8_t nivel = 0;           // largo de la secuencia actual (0 = juego sin empezar)
uint8_t record = 0;
uint8_t ultimo = 0;

uint8_t cursor_fila = 3; // donde arranca el puntero
uint8_t cursor_col = 3;
uint8_t cursor_color = COLOR_BLANCO; // color del cursor


uint8_t pantalla[NUM_LEDS][3];

// guardan si el boton estaba apretado en la lectura anterior
uint8_t previo_sw = 0;
uint8_t previo_reinicio = 0;


// parte del uart (solo transmision)

void init_uart(unsigned int baud) {
	unsigned int ubrr = F_CPU / 16 / baud - 1; // calcula el valor del divisor para la velocidad requerida
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << TXEN0); // habilita solo la transmision
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); //  8 bits de datos, 1 bit de parada
}

void uart_enviar_char(char c) {
	while (!(UCSR0A & (1 << UDRE0))); // Espera que transmisión esté vacío
	UDR0 = c;
}

void uart_enviar_texto(const char *texto) {
	while (*texto) { //  mientras el apuntador no encuentre el 0 queda en el while
		uart_enviar_char(*texto++);
	}
}

// conversion manual a entero sin signo 

void uart_enviar_numero(uint16_t numero) {
	char buffer[6]; //  temporal para almacenar los dígitos
	int i = 0;

	if (numero == 0) {
		uart_enviar_char('0');
		return;
	}

	while (numero > 0) {
		buffer[i++] = (numero % 10) + '0'; // obtiene el residuo y lo convierte a código ASCII
		numero /= 10;
	}

	while (i > 0) {
		uart_enviar_char(buffer[--i]);
	}
}


//  joystick

void init_adc(void) {
	ADMUX = (1 << REFS0);   
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   // preescaler 128
}

uint16_t leer_adc(uint8_t canal) {
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}


// sonido (timer 1, pwm en pb1, igual que el piano)

void init_buzzer(void) {
	TCCR1A = (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	DDRB |= (1 << PB1);
	PORTB &= ~(1 << PB1);
}

void sonar(uint16_t freq) {
	if (freq == SILENCIO) {
		TCCR1A &= ~(1 << COM1A1); // desconecta la salida del timer 1 del pin pb1 para cortar la señal PWM
		PORTB &= ~(1 << PB1);
		} else {
		TCCR1A |= (1 << COM1A1); // la vuelve a activar
		ICR1 = FREQ_TO_ICR1(freq);
		OCR1A = ICR1 / 2; // deja el ciclo de trabajo exacto aa la mitad de ICR1 para maximizar la resonancia
	}
}

void detener_sonido(void) {
	sonar(SILENCIO);
}


// matriz ws2812b

// manda un byte a ka matriz con el tiempo justo para 16 mhz (20 ciclos por bit)

static void ws_enviar_byte(uint8_t dato) {
	uint8_t alto = PORTD | (1 << PIN_MATRIZ);
	uint8_t bajo = PORTD & ~(1 << PIN_MATRIZ);
	uint8_t bits = 8;

	asm volatile( // bloqeu de código en lenguaje ensamblador con tiempos críticos estritos
	"bucle_%=:                           \n\t"
	"out  %[puerto], %[alto]             \n\t"   // c0: sube la linea
	"nop \n\t nop \n\t nop \n\t nop      \n\t"   // c1-c4
	"sbrs %[dato], 7                     \n\t"   // c5: si el bit es 1 salta la siguiente
	"out  %[puerto], %[bajo]             \n\t"   // c6: (solo bit 0) baja la linea
	"nop \n\t nop \n\t nop               \n\t"   // c7-c9
	"nop \n\t nop \n\t nop               \n\t"   // c10-c12
	"out  %[puerto], %[bajo]             \n\t"   // c13: (bit 1) baja la linea
	"lsl  %[dato]                        \n\t"   // c14
	"nop \n\t nop                        \n\t"   // c15-c16
	"dec  %[bits]                        \n\t"   // c17
	"brne bucle_%=                       \n\t"   // c18-c19
	: [dato] "+r" (dato), [bits] "+r" (bits)
	: [puerto] "I" (_SFR_IO_ADDR(PORTD)), [alto] "r" (alto), [bajo] "r" (bajo)
	);
}

// manda toda la memoria de pantalla a la matriz (el orden del  es g, r, b)
void matriz_mostrar(void) {
	uint8_t sreg = SREG;
	cli();
	for (uint8_t i = 0; i < NUM_LEDS; i++) {
		ws_enviar_byte(pantalla[i][1]); // intensidades
		ws_enviar_byte(pantalla[i][0]);
		ws_enviar_byte(pantalla[i][2]);
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
	pantalla[i][0] = paleta[color][0]; // copia los colores a la tableta
	pantalla[i][1] = paleta[color][1];
	pantalla[i][2] = paleta[color][2];
}

void matriz_llenar(uint8_t color) { // le da el mismo color a todos los LEDs de la matriz de forma simultánea
	for (uint8_t f = 0; f < 8; f++) {
		for (uint8_t c = 0; c < 8; c++) {
			matriz_pixel(f, c, color); // recorre y asigna
		}
	}
}

void dibujar_cursor(void) {
	matriz_limpiar();
	matriz_pixel(cursor_fila, cursor_col, cursor_color);
	matriz_mostrar();
}


// entradas (joystick y botones)

uint8_t leer_sw(void) {
	return !(PIND & (1 << PIN_JOY_SW));
}

uint8_t leer_reinicio(void) {
	return !(PIND & (1 << PIN_REINICIO));
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


// devuelve 1 si el joystick estaba fuera de la zona muerta
uint8_t actualizar_cursor(void) {
	static uint8_t espera_repeticion = 0;

	int16_t dx = (int16_t)leer_adc(CANAL_X) - CENTRO_JOY; // le la variacion de los ejes respecto al punto medio
	int16_t dy = (int16_t)leer_adc(CANAL_Y) - CENTRO_JOY;
	if (INVERTIR_X) dx = -dx; 
	if (INVERTIR_Y) dy = -dy;

	int16_t abs_x = (dx < 0) ? -dx : dx;
	int16_t abs_y = (dy < 0) ? -dy : dy;

	int8_t mover_col = 0;
	int8_t mover_fila = 0;

	// solo se mueve en el eje que mas se inclino, asi no hay diagonales
	if (abs_x > ZONA_MUERTA || abs_y > ZONA_MUERTA) {
		if (abs_x >= abs_y) {
			mover_col = (dx > 0) ? 1 : -1; // va a la derecha si dx es positivo, de lo contrario a la izquierda
			} else {
			mover_fila = (dy > 0) ? 1 : -1; // lo mismo pero arriba o abajo
		}
	}

	if (mover_col == 0 && mover_fila == 0) { // se fija si el joystick está en reposo
		espera_repeticion = 0;
		return 0;
	}

	if (espera_repeticion == 0) {
		if (mover_col == 1 && cursor_col < 7) cursor_col++;
		if (mover_col == -1 && cursor_col > 0) cursor_col--;
		if (mover_fila == 1 && cursor_fila < 7) cursor_fila++;
		if (mover_fila == -1 && cursor_fila > 0) cursor_fila--;
		espera_repeticion = REPETICION_CURSOR;
		} else {
		espera_repeticion--;
	}
	return 1;
}


// eeprom

void eeprom_inicializar(void) {
	
	if (eeprom_read_byte(DIR_FIRMA) != FIRMA_EEPROM) {
		eeprom_update_byte(DIR_FIRMA, FIRMA_EEPROM);
		eeprom_update_byte(DIR_RECORD, 0); // pone un récord inicial por defecto de cero niveles
		eeprom_update_byte(DIR_ULTIMO, 0);
		eeprom_update_word(DIR_SEMILLA, 1234);
	}

	record = eeprom_read_byte(DIR_RECORD); // recupera lo que se grabo
	ultimo = eeprom_read_byte(DIR_ULTIMO);
	if (record > MAX_NIVEL) record = 0;
	if (ultimo > MAX_NIVEL) ultimo = 0;

	// la semilla se guarda en eeprom y cambia en cada arranque, asi no sale siempre la misma secuencia
	uint16_t semilla = eeprom_read_word(DIR_SEMILLA);
	srand(semilla ^ leer_adc(CANAL_RUIDO));
	eeprom_update_word(DIR_SEMILLA, (uint16_t)rand());
}


// logica del juego

// agrega un elemento nuevo (fila, columna, color) a la secuencia
void agregar_elemento(uint8_t posicion) {
	sec_fila[posicion] = rand() % 8;
	sec_col[posicion] = rand() % 8;
	sec_color[posicion] = rand() % NUM_COLORES;
}

// record abajo (Ccelestito) y ultima partida arriba (verde), un led por nivel
void mostrar_puntajes(void) {
	matriz_limpiar();
	for (uint8_t i = 0; i < record; i++) {
		matriz_pixel(i / 8, i % 8, COLOR_CELESTITO);
	}
	for (uint8_t i = 0; i < ultimo; i++) {
		matriz_pixel(4 + i / 8, i % 8, COLOR_VERDE);
	}
	matriz_mostrar();
}

void estado_inicio(void) {
	nivel = 0;
	cursor_fila = 3;
	cursor_col = 3;
	cursor_color = COLOR_BLANCO;

	uart_enviar_texto("\r\n SIMON DICE \r\n");
	uart_enviar_texto("Record: ");
	uart_enviar_numero(record);
	uart_enviar_texto("   Ultima partida: ");
	uart_enviar_numero(ultimo);
	uart_enviar_texto("\r\n");

	// animacion de bienvenida: una fila por color con su tono
	for (uint8_t c = 0; c < NUM_COLORES; c++) {
		matriz_limpiar();
		for (uint8_t col = 0; col < 8; col++) {
			matriz_pixel(c, col, c);
		}
		matriz_mostrar();
		sonar(tonos[c]);
		_delay_ms(200);
	}
	detener_sonido();

	mostrar_puntajes();
	_delay_ms(2000);

	estado_actual = ESTADO_ESPERA;
}

// modo libre: el joystick mueve el cursor y el boton le cambia el color al azar
// si pasan ~3 s sin tocar nada arranca la secuencia
void estado_espera(void) {
	uint16_t inactividad = 0;

	while (inactividad < ESPERA_LIBRE) {
		if (hay_flanco(leer_reinicio(), &previo_reinicio)) {
			estado_actual = ESTADO_INICIO;
			return;
		}

		uint8_t actividad = actualizar_cursor();

		if (hay_flanco(leer_sw(), &previo_sw)) {
			cursor_color = rand() % NUM_COLORES;
			sonar(tonos[cursor_color]);
			dibujar_cursor();
			_delay_ms(100);
			detener_sonido();
			actividad = 1;
		}

		dibujar_cursor();

		if (actividad) {
			inactividad = 0;
			} else {
			inactividad++;
		}
		_delay_ms(10);
	}

	// si el juego recien empieza se crea el primer elemento
	if (nivel == 0) {
		nivel = 1;
		agregar_elemento(0);
		uart_enviar_texto("Nivel 1\r\n");
	}

	estado_actual = ESTADO_MOSTRAR;
}

// muestra la secuencia: cada elemento ~500 ms, pausa de 200 ms y su tono
void estado_mostrar(void) {
	matriz_limpiar();
	matriz_mostrar();
	_delay_ms(500);

	for (uint8_t i = 0; i < nivel; i++) {
		matriz_limpiar();
		matriz_pixel(sec_fila[i], sec_col[i], sec_color[i]);
		matriz_mostrar();
		sonar(tonos[sec_color[i]]);
		_delay_ms(TIEMPO_ENCENDIDO);

		detener_sonido();
		matriz_limpiar();
		matriz_mostrar();
		_delay_ms(TIEMPO_PAUSA);
	}

	estado_actual = ESTADO_TURNO;
}

// el jugador mueve el cursor y confirma cada posicion con el boton del joystick
void estado_turno(void) {
	uint8_t indice = 0;

	while (1) {
		if (hay_flanco(leer_reinicio(), &previo_reinicio)) {
			estado_actual = ESTADO_INICIO;
			return;
		}

		actualizar_cursor();
		dibujar_cursor();

		if (hay_flanco(leer_sw(), &previo_sw)) {
			if (cursor_fila == sec_fila[indice] && cursor_col == sec_col[indice]) {
				// acierto: se ilumina con el color y tono de ese elemento
				matriz_limpiar();
				matriz_pixel(sec_fila[indice], sec_col[indice], sec_color[indice]);
				matriz_mostrar();
				sonar(tonos[sec_color[indice]]);
				_delay_ms(250);
				detener_sonido();

				indice++;
				if (indice >= nivel) {
					estado_actual = ESTADO_EXITO;
					return;
				}
				} else {
				estado_actual = ESTADO_ERROR;
				return;
			}
		}
		_delay_ms(10);
	}
}

void estado_exito(void) {
	// nivel completado: se guarda en eeprom y se actualiza el record si hace falta
	eeprom_update_byte(DIR_ULTIMO, nivel);
	ultimo = nivel;
	if (nivel > record) {
		record = nivel;
		eeprom_update_byte(DIR_RECORD, record);
		uart_enviar_texto("Nuevo record: ");
		uart_enviar_numero(record);
		uart_enviar_texto("\r\n");
	}

	uart_enviar_texto("Nivel ");
	uart_enviar_numero(nivel);
	uart_enviar_texto(" superado\r\n");

	// animacion de exito: destellos verdes 
	const uint16_t tonos_exito[4] = { DO4, MI4, SOL4, DO5 };
	for (uint8_t i = 0; i < 4; i++) {
		matriz_llenar(COLOR_VERDE);
		matriz_mostrar();
		sonar(tonos_exito[i]);
		_delay_ms(120);
		matriz_limpiar();
		matriz_mostrar();
		detener_sonido();
		_delay_ms(60);
	}

	// si llego al maximo de niveles se da por ganado el juego y arranca de nuevo
	if (nivel >= MAX_NIVEL) {
		uart_enviar_texto("Juego completado!\r\n");
		estado_actual = ESTADO_INICIO;
		return;
	}

	agregar_elemento(nivel);
	nivel++;
	uart_enviar_texto("Nivel ");
	uart_enviar_numero(nivel);
	uart_enviar_texto("\r\n");

	estado_actual = ESTADO_ESPERA;
}

void estado_error(void) {
	uint8_t completados = nivel - 1;
	eeprom_update_byte(DIR_ULTIMO, completados);
	ultimo = completados;

	uart_enviar_texto("Error! Niveles completados: ");
	uart_enviar_numero(completados);
	uart_enviar_texto("\r\n");

	// animacion de fallo: parpadeo rojo 
	for (uint8_t i = 0; i < 3; i++) {
		matriz_llenar(COLOR_ROJO);
		matriz_mostrar();
		sonar(DO3);
		_delay_ms(250);
		matriz_limpiar();
		matriz_mostrar();
		detener_sonido();
		_delay_ms(150);
	}

	// el juego se reinicia desde el nivel 1
	estado_actual = ESTADO_INICIO;
}


// main

int main(void) {
	// salida de la matriz
	DDRD |= (1 << PIN_MATRIZ);
	PORTD &= ~(1 << PIN_MATRIZ);

	// boton del joystick y de reinicio como entradas con pull-up
	DDRD &= ~((1 << PIN_JOY_SW) | (1 << PIN_REINICIO));
	PORTD |= (1 << PIN_JOY_SW) | (1 << PIN_REINICIO);

	init_uart(9600);
	init_adc();
	init_buzzer();
	eeprom_inicializar();

	// matriz apagada al arrancar
	matriz_limpiar();
	matriz_mostrar();

	while (1) {
		switch (estado_actual) {
			case ESTADO_INICIO:   estado_inicio();   break;
			case ESTADO_ESPERA:   estado_espera();   break;
			case ESTADO_MOSTRAR:  estado_mostrar();  break;
			case ESTADO_TURNO:    estado_turno();    break;
			case ESTADO_EXITO:    estado_exito();    break;
			case ESTADO_ERROR:    estado_error();    break;
		}
	}
}