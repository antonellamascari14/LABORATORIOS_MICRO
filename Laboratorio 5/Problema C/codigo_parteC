
#define F_CPU 16000000UL 

#include <avr/io.h>        
#include <avr/interrupt.h> // para manejo de rutinas de interrupcion (isr, sei, cli)
#include <avr/sleep.h>     // funciones para gestionar los modos de bajo consumo (idle, power-down)
#include <avr/wdt.h>       //  de control del watchdog timer (wdt_reset, wdt_disable)
#include <util/delay.h>    // retardos bloqueantes precisos en microsegundos y milisegundos
#include <util/atomic.h>   // atomic_block para leer variables compartidas sin corrupcion de datos
#include <stdint.h>        
#include <stdbool.h>       
#include <stdio.h>        



// periodos y tiempos
#define periodo_leds_ms 200UL  
#define periodo_lcd_ms 500UL   
#define periodo_temp_ms 2000UL 
#define dht_start_ms 20UL     
#define tiempo_inact_standby_ms 10000UL // 10 s sin actividad para pasar a standby
#define tiempo_inact_apagado_ms 30000UL // 30 s sin actividad acumulada para pasar a apagado

#define antirrebote_ms 50UL 
#define buzzer_ms 50UL      // duracion del pitido corto de confirmacion del buzzer

// pines
#define led_ddr DDRB   
#define led_port PORTB 
#define led_bit PB5    

#define test_bit PB4 // pin pb4 (d12) de prueba para medir el periodo del wdt en osciloscopio

#define buz_ddr DDRD    
#define buzz_port PORTD 
#define buzz_bit PD4    

#define btn_ddr DDRD   
#define btn_port PORTD // registro de salida para activar resistencias pull-up internas
#define btn_pinr PIND  
#define btn_bit PD2    // boton 1 para actividad y despertar
#define btn2_bit PD5   // boton 2 exclusivo para emergencia y recuperacion

#define dht_ddr DDRD  
#define dht_port PORTD 
#define dht_pinr PIND  
#define dht_bit PD3    

#define canal_adc 0 // canal analogico adc0

// lcd i2c 
#define lcd_addr 0x27      // direccion esclavo i2c de 7 bits del  pcf8574
#define lcd_command 0      // bit rs = 0 para enviar comandos de configuracion al lcd
#define lcd_data 1         // bit rs = 1 para escribir caracteres visibles en el lcd
#define lcd_backlight 0x08 // bit p3 del pcf8574 que controla la luz de fondo de la pantalla
#define en 0x04            // bit p2 del pcf8574 que genera el pulso enable de validacion

// prr: mascaras explicitas (solo bits definidos, bit 4 reservado)

#define prr_adc    0x01 // bit 0: apaga el reloj del conversor analogico digital (adc)
#define prr_usart0 0x02 // bit 1: apaga el reloj de la interfaz serie usart0
#define prr_spi    0x04 // bit 2: apaga el reloj del bus spi
#define prr_tim1   0x08 // bit 3: apaga el reloj del temporizador timer1
#define prr_tim0   0x20 // bit 5: apaga el reloj del temporizador timer0
#define prr_tim2   0x40 // bit 6: apaga el reloj del temporizador timer2
#define prr_twi    0x80 // bit 7: apaga el reloj de la interfaz i2c (twi)

// standby: solo se apaga el adc, spi y timer2 (uart, twi y timer0 se necesitan)
#define prr_standby (prr_adc | prr_spi | prr_tim2)
// apagado: se apaga todo salvo timer0 (que igual se detiene en power-down)
#define prr_apagado (prr_adc | prr_usart0 | prr_spi | prr_tim1 | prr_tim2 | prr_twi)

// uart / protocolo
#define uart_baud 9600UL                        
#define ubrr_val ((F_CPU / 16UL / uart_baud) - 1) // valor del registro UBRR0 (= 103 para 16 mhz)

#define rx_size 32            // tamaño del buffer circular de recepcion (potencia de 2)
#define tx_size 128           // tamaño del buffer circular de tranmsision (potencia de 2)
#define rx_mask (rx_size - 1) // mascara de envolvente rapida para indice rx (0x1f)
#define tx_mask (tx_size - 1) // mascara de envolvente rapida para indice tx (0x7f)

#define trama_ini '<'          // caracter delimitador de inicio de trama
#define trama_fin '>'          // caracter delimitador de fin de trama
#define trama_payload 9        // longitud fija de datos utiles: cc(2) + dddd(4) + kkk(3)
#define trama_timeout_ms 500UL // tiempo maximo para recibir una trama completa antes de descartarla

// comandos app / arduino
#define cmd_estado_id 1     // id 01: solicitar estado actual de la fsm
#define cmd_sensores_id 2   // id 02: solicitar valores de temperatura, humedad y adc
#define cmd_cambiar_id 3    // id 03: conmutar entre activo (0) y standby (1)
#define cmd_actuador_id 4   // id 04: deshabilitar (0) o habilitar (1) el parpadeo del led
#define cmd_emergencia_id 5 // id 05: forzar entrada a estado seguro desde la app
#define cmd_alarmas_id 6    // id 06: solicitar historial de ultimos eventos de alarma
#define cmd_test_wdt_id 7   // id 07: simular bloqueo infinito para probar reset del wdt (dato 1234)

// respuestas arduino / app

#define rsp_estado 81 // id 81: respuesta con codigo de estado actual
#define rsp_temp 82   // id 82: respuesta con lectura de temperatura en grados celsius
#define rsp_hum 83    // id 83: respuesta con lectura de humedad relativa en porcentaje
#define rsp_adc 84    // id 84: respuesta con valor crudo de 10 bits del adc
#define rsp_evento 90 // id 90: notificacion asincrona o historial de alarma
#define rsp_error 99  // id 99: trama de diagnostico ante error de protocolo

// codigos de error (rsp_error)
#define err_checksum 1     // codigo 0001: la suma de verificacion no coincide
#define err_desconocido 2  // codigo 0002: identificador de comando cc no existe en la tabla
#define err_incompleta 3   // codigo 0003: trama truncada, timeout o caracter invalido
#define err_no_permitido 4 // codigo 0004: comando rechazado porque el sistema esta en estado seguro
#define err_dato 5         // codigo 0005: argumento dddd fuera de rango permitido

// codigos de evento/alarma (rsp_evento)
#define evt_reset_wdt 1      // evento 1: el sistema se recupero de un reinicio por watchdog
#define evt_emerg_local 2    // evento 2: emergencia disparada por pulsador fisico pd5
#define evt_emerg_app 3      // evento 3: emergencia disparada remotamente via bluetooth
#define evt_error_checksum 4 // evento 4: se recibio una trama con checksum corrupto

// fsm
typedef enum {
	estado_activo = 0, 
	estado_standby,    
	estado_apagado,   
	estado_seguro,     
	n_estados          
} estado_t;

// variables compartidas con isr
static volatile uint32_t ms_ticks = 0;    // contador global de milisegundos gobernado por timer0
static volatile bool flag_boton = false;  // bandera activada por int0 al pulsar boton 1 (pd2)
static volatile bool flag_pc = false;     // bandera activada por pcint2 al pulsar boton 2 (pd5)
static volatile bool flag_wdt = false;    // bandera activada por wdt_vect cada 8 s en apagado
static volatile uint16_t adc_valor = 0;   // ultima conversion analogica almacenada
static volatile uint16_t wdt_eventos = 0; // contador de despertares periodicos por wdt

static volatile uint8_t rx_buf[rx_size]; // arreglo de memoria para buffer circular de recepcion uart
static volatile uint8_t rx_head = 0, rx_tail = 0; // punteros de escritura y lectura de rx_buf
static volatile uint8_t tx_buf[tx_size]; // arreglo de memoria para buffer circular de transmision uart
static volatile uint8_t tx_head = 0, tx_tail = 0; // punteros de escritura y lectura de tx_buf
static volatile bool rx_error = false;   // bandera de error de hardware uart (framing, overrun, paridad)

// variables de aplicacion
static uint32_t t_led, t_lcd, t_temp, t_dht; // marcas temporales de ejecucion de cada tarea periodica
static uint8_t dht_fase = 0;                 // fase actual de lectura del dht11 (0: espera, 1: pulso inicio)

static uint32_t t_ult_boton, t_buzzer, t_press; // tiempos de control para antirrebote y duracion del buzzer
static bool buzzer_activo = false, presionado = false; // estados internos del buzzer y boton 2

static bool evt_boton = false;      // evento  de pulsacion del boton 1 (pd2)
static bool evt_boton2 = false;     // evento  de pulsacion del boton 2 (pd5)
static bool evt_bt_emerg = false;   // evento  de emergencia recibida por bluetooth
static bool evt_bt_standby = false; // evento  de paso a standby por bluetooth
static bool evt_bt_wake = false;    // evento  de despertar a activo por bluetooth

static estado_t estado_actual = estado_activo; // estado vigente de la maquina de estados
static uint32_t t_actividad = 0;               // marca de tiempo de la ultima interaccion valida

static uint8_t dht_temp = 0, dht_hum = 0; // ultimas lecturas validas de temperatura y humedad
static uint8_t lcd_bl_mask = lcd_backlight; // mascara actual de retroiluminacion del lcd
static bool led_habilitado = true;          // permite habilitar o silenciar el parpadeo del led via app
static bool usart_on = false;               // indica si el periferico usart0 se encuentra encendido
static bool causa_wdt = false;              // indica si el ingreso a estado seguro fue por reset de wdt

static uint8_t alarmas[8];                     // historial circular de los ultimos 8 codigos de evento
static uint8_t alarma_head = 0, alarma_cant = 0; // indice de insercion y contador de alarmas guardadas

// parser de tramas
static char trama[trama_payload + 1]; // buffer temporal para ensamblar los 9 caracteres de carga util
static uint8_t rx_idx = 0;            // indice del caracter actual dentro de la trama en curso
static bool en_trama = false;         // indica si ya se detecto '<' y se esta esperando '>'
static uint32_t t_rx = 0;             // marca de tiempo del ultimo byte recibido para detectar timeout

static const char *const nombre_estado[n_estados] = { "act", "stb", "off", "seg" }; // textos cortos de estado

// causa de reinicio: se captura en .init3, antes de main,
// y se deshabilita el wdt para evitar reinicios en cadena.
uint8_t mcusr_copy __attribute__((section(".noinit"))); // variable en ram que conserva su valor tras reset
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3"))); // prototipo en seccion de arranque
void get_mcusr(void)
{
	mcusr_copy = MCUSR; // guarda el registro de causa de reinicio antes de que se borre
	MCUSR = 0;          // limpia las banderas de reinicio (porf, extrf, borf, wdrf)
	wdt_disable();      // apaga el watchdog inmediatamente para evitar bucles de reinicio
}

// prototipos
static uint32_t millis(void);
static void registrar_actividad(void);
static void resincronizar_tareas(void);
static void actualizar_pantalla_lcd(void);
static void lcd_i2c_write(uint8_t data, uint8_t mode);
static void bt_enviar(uint8_t cc, uint16_t dato);
static void registrar_evento(uint8_t codigo);

// interrupcioness ISR
ISR(TIMER0_COMPA_vect) { ms_ticks++; } // incrementa el contador de milisegundos cada 1 ms exacto

ISR(TIMER1_COMPA_vect)
{
	if (ADCSRA & (1 << ADEN)) ADCSRA |= (1 << ADSC); // dispara una conversion adc cada 100 ms si esta habilitado
}

ISR(ADC_vect) { adc_valor = ADC; } // guarda el resultado de 10 bits al finalizar la conversion analogica

ISR(INT0_vect)
{
	EICRA = (1 << ISC01); // restaura deteccion por flanco de bajada (por si venia de nivel bajo en power-down)
	flag_boton = true;    // avisa al bucle principal que se presiono el boton 1
}

ISR(PCINT2_vect)
{
	if (!(btn_pinr & (1 << btn2_bit))) flag_pc = true; // detecta flanco de bajada en pd5 (boton 2)
}

ISR(WDT_vect)
{
	flag_wdt = true;         // indica que el despertar desde power-down fue causado por el wdt
	wdt_eventos++;           // incrementa el contador de ciclos de despertar
	PINB = (1 << test_bit);  // conmuta el pin pb4 (d12) para medir el periodo de 8 s con osciloscopio
}

ISR(USART_RX_vect)
{
	uint8_t estado = UCSR0A; // lee el registro de estado antes de leer udr0 para capturar errores
	uint8_t dato = UDR0;     // extrae el byte recibido del registro de datos uart
	if (estado & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) { rx_error = true; return; } // descarta byte si hubo error de trama
	uint8_t sig = (rx_head + 1) & rx_mask; // calcula la siguiente posicion en el buffer circular
	if (sig != rx_tail) { rx_buf[rx_head] = dato; rx_head = sig; } // guarda el dato si el buffer no esta lleno
}

ISR(USART_UDRE_vect)
{
	if (tx_head == tx_tail) {
		UCSR0B &= ~(1 << UDRIE0); // deshabilita interrupcion si ya no quedan bytes por enviar
	} else {
		UDR0 = tx_buf[tx_tail];           // carga el siguiente byte pendiente en el registro de transmision
		tx_tail = (tx_tail + 1) & tx_mask; // avanza el puntero de lectura del buffer tx
	}
}

// utilidades
static uint32_t millis(void)
{
	uint32_t t; // variable local para copia segura
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = ms_ticks; } // lee los 32 bits sin interrupciones intermedias
	return t; // devuelve el tiempo transcurrido en milisegundos
}

static void registrar_actividad(void) { t_actividad = millis(); } // reinicia el temporizador de inactividad

static void resincronizar_tareas(void)
{
	uint32_t ahora = millis();       // toma la marca de tiempo actual
	t_led = t_lcd = t_temp = ahora;  // alinea todas las tareas periodicas al instante presente
	dht_ddr &= ~(1 << dht_bit);      // asegura que el pin del dht11 quede liberado como entrada
	dht_fase = 0;                    // reinicia la maquina de lectura del sensor a fase 0
}

// watchdog
// modo reset, ~1 s (supervision en activo / standby / seguro)
static void wdt_modo_reset_1s(void)
{
	uint8_t sreg = SREG; // respalda el estado actual de interrupciones globales
	cli();               // deshabilita interrupciones para cumplir secuencia temporizada de 4 ciclos
	wdt_reset();         // reinicia el contador interno del watchdog
	WDTCSR |= (1 << WDCE) | (1 << WDE); // habilita el permiso de escritura sobre wdtcsr
	WDTCSR = (1 << WDE) | (1 << WDP2) | (1 << WDP1); // configura modo reset (wde=1) con prescaler de 1 s
	SREG = sreg;         // restaura el registro de estado original
}

// modo interrupcion solamente, ~8 s (despertador en apagado). no reinicia el mcu.
static void wdt_modo_int_8s(void)
{
	uint8_t sreg = SREG; // respalda el registro sreg
	cli();               // deshabilita interrupciones globales
	wdt_reset();         // pone en cero el contador del wdt
	MCUSR &= ~(1 << WDRF); // limpia la bandera wdrf para permitir desactivar wde
	WDTCSR |= (1 << WDCE) | (1 << WDE); // habilita modificacion de configuracion
	WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // modo solo interrupcion (wdie=1, wde=0) a 8 segundos
	SREG = sreg;         // restaura interrupciones
}

// i2c y lcd
static void i2c_init(void)
{
	TWSR = 0;                              // prescaler del bus twi en 1
	TWBR = ((F_CPU / 100000UL) - 16) / 2;  // fija velocidad de reloj scl en 100 khz (estandar)
	TWCR = (1 << TWEN);                    // habilita el modulo hardware twi
}

static void i2c_start(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // envia condicion de inicio start en el bus i2c
	while (!(TWCR & (1 << TWINT)));                   // espera hasta que el hardware complete la accion
}

static void i2c_write(uint8_t data)
{
	TWDR = data;                       // carga el byte a transmitir en el registro de datos twi
	TWCR = (1 << TWINT) | (1 << TWEN); // inicia el envio del byte
	while (!(TWCR & (1 << TWINT)));    // aguarda confirmacion de transmision completa
}

static void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // genera condicion de parada stop y libera el bus
}

static void lcd_i2c_write(uint8_t data, uint8_t mode)
{
	uint8_t high = data & 0xf0;        // extrae los 4 bits mas significativos (alto)
	uint8_t low = (data << 4) & 0xf0;  // desplaza y extrae los 4 bits menos significativos (bajo)
	uint8_t d[4];                      // secuencia de 4 bytes para pulsar enable en modo 4 bits
	d[0] = high | mode | lcd_bl_mask | en; //  alto con pulso enable en alto
	d[1] = high | mode | lcd_bl_mask;      //  alto con flanco de bajada en enable
	d[2] = low | mode | lcd_bl_mask | en;  //  bajo con pulso enable en alto
	d[3] = low | mode | lcd_bl_mask;       //  bajo con flanco de bajada en enable

	i2c_start();                             // inicia comunicacion i2c
	i2c_write(lcd_addr << 1);                // envia direccion del lcd en modo escritura (bit 0 = 0)
	for (uint8_t i = 0; i < 4; i++) i2c_write(d[i]); // transmite los 4 bytes consecutivos
	i2c_stop();                              // finaliza la transferencia i2c
}

static void lcd_i2c_init(void)
{
	_delay_ms(50);                     // espera estabilizacion de tension tras encender el lcd
	lcd_i2c_write(0x03, lcd_command);  // primer comando de sincronizacion en modo 8 bits
	_delay_ms(5);                      // retardo requerido por hoja de datos hd44780
	lcd_i2c_write(0x03, lcd_command);  // segundo comando de sincronizacion
	_delay_us(100);                    // espera minima de 100 us
	lcd_i2c_write(0x03, lcd_command);  // tercer comando de sincronizacion
	_delay_us(100);                    // espera minima de 100 us
	lcd_i2c_write(0x02, lcd_command);  // conmuta la interfaz del controlador a bus de 4 bits

	lcd_i2c_write(0x28, lcd_command);  // configura 2 lineas de visualizacion y matriz de 5x8 puntos
	lcd_i2c_write(0x0c, lcd_command);  // enciende el display y apaga el cursor parpadeante
	lcd_i2c_write(0x06, lcd_command);  // configura incremento automatico del cursor hacia la derecha
	lcd_i2c_write(0x01, lcd_command);  // limpia toda la memoria ddram de la pantalla
	_delay_ms(2);                      // espera 2 ms a que termine el borrado interno
}

static void lcd_i2c_set_cursor(uint8_t fila, uint8_t col)
{
	lcd_i2c_write((fila == 0) ? (0x80 + col) : (0xc0 + col), lcd_command); // posiciona cursor en fila 0 o 1
}

static void lcd_i2c_write_string(const char *str)
{
	while (*str) lcd_i2c_write((uint8_t)(*str++), lcd_data); // envia caracter por caracter hasta el nulo final
}

static void actualizar_pantalla_lcd(void)
{
	if (estado_actual == estado_apagado) return; // no intenta escribir si el modulo twi esta apagado

	char tmp[17]; // buffer local para formatear lineas de hasta 16 caracteres
	uint16_t adc; // copia local del valor analogico
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { adc = adc_valor; } // lectura segura de la variable compartida

	if (estado_actual == estado_standby) {
		lcd_i2c_set_cursor(0, 0); lcd_i2c_write_string("esperando...    "); // mensaje fijo en reposo standby
		#ifdef debug_inactividad
		snprintf(tmp, sizeof(tmp), "inact:%3lus      ", (unsigned long)((millis() - t_actividad) / 1000UL));
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string(tmp); // muestra segundos de inactividad si esta habilitado
#else
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string("                "); // limpia la segunda linea con espacios
#endif
	} else if (estado_actual == estado_seguro) {
		lcd_i2c_set_cursor(0, 0);
		lcd_i2c_write_string(causa_wdt ? "reset por WDT   " : "emergencia      "); // indica motivo de bloqueo
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string("pulsa para salir"); // instruccion de recuperacion local
	} else {
		snprintf(tmp, sizeof(tmp), "t:%2uc h:%2u%%    ", dht_temp, dht_hum); // formatea temperatura y humedad
		lcd_i2c_set_cursor(0, 0); lcd_i2c_write_string(tmp); // escribe sensores en la fila superior
		snprintf(tmp, sizeof(tmp), "adc:%4u %s   ", adc, nombre_estado[estado_actual]); // formatea adc y estado
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string(tmp); // escribe lectura analogica y estado en fila inferior
	}
}

// usart0 (hc-05) con buffers circulares
static void usart_init(void)
{
	rx_head = rx_tail = tx_head = tx_tail = 0; // reinicia los indices de los buffers circulares
	rx_idx = 0;                                // reinicia el contador de caracteres de trama
	en_trama = false;                          // limpia estado del analizador de tramas
	UBRR0H = (uint8_t)(ubrr_val >> 8);         // carga byte alto del divisor de baudios
	UBRR0L = (uint8_t)ubrr_val;                // carga byte bajo del divisor de baudios (103 -> 9600 bps)
	UCSR0A = 0;                                // modo velocidad normal (u2x0 = 0)
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);    // trama asincrona de 8 bits, sin paridad, 1 bit de parada (8n1)
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // habilita receptor, transmisor e interrupcion rx
	usart_on = true;                           // marca el modulo uart como operativo
}

static void usart_apagar(void)
{
	// vaciar la transmision pendiente antes de cortar el periferico
	uint32_t t0 = millis(); // marca inicial para evitar espera infinita
	while ((tx_head != tx_tail) && (millis() - t0 < 100UL)) wdt_reset(); // espera que se vacie el buffer tx
	while (!(UCSR0A & (1 << UDRE0))) { } // aguarda a que el registro de datos udr0 quede vacio
	_delay_ms(2);                        // espera que salga fisicamente el ultimo byte por el pin tx
	UCSR0B = 0;                          // deshabilita receptor, transmisor e interrupciones uart
	usart_on = false;                    // registra que la uart quedo apagada
}

static void tx_put(uint8_t c)
{
	uint8_t sig = (tx_head + 1) & tx_mask; // calcula siguiente posicion libre en tx_buf
	if (sig == tx_tail) return;            // si el buffer circular esta lleno, descarta el caracter
	tx_buf[tx_head] = c;                   // almacena el caracter en la cola de transmision
	tx_head = sig;                         // actualiza el indice de cabeza
	UCSR0B |= (1 << UDRIE0);               // activa interrupcion de buffer vacio para iniciar envio
}

static void bt_enviar(uint8_t cc, uint16_t dato)
{
	if (!usart_on) return; // no intenta transmitir si la uart fue apagada en power-down
	char f[12];            // arreglo auxiliar para armar la trama de 11 caracteres + nulo
	uint8_t k = (uint8_t)((cc + dato) & 0xFF); // calcula checksum modulo 256
	snprintf(f, sizeof(f), "<%02u%04u%03u>", (unsigned)cc, (unsigned)dato, (unsigned)k); // formatea trama ascii
	for (uint8_t i = 0; f[i]; i++) tx_put((uint8_t)f[i]); // encola cada caracter en el buffer circular tx
}

static void bt_error(uint8_t codigo) { bt_enviar(rsp_error, codigo); } // envia trama de error id 99

static void registrar_evento(uint8_t codigo)
{
	alarmas[alarma_head & 7] = codigo;      // guarda el codigo de alarma en el historial circular de 8 lugares
	alarma_head = (alarma_head + 1) & 7;    // avanza el indice de escritura modulo 8
	if (alarma_cant < 8) alarma_cant++;     // incrementa la cantidad de eventos almacenados (maximo 8)
	bt_enviar(rsp_evento, codigo);          // notifica inmediatamente el evento por bluetooth (id 90)
}

// comandos (cada handler devuelve 0 = ok, o un codigo de error)
static uint8_t cmd_estado(uint16_t d)
{
	(void)d;                                          // ignora el campo dato porque no se utiliza
	bt_enviar(rsp_estado, (uint16_t)estado_actual);   // responde con el numero de estado vigente (0 a 3)
	return 0;                                         // ejecucion exitosa
}

static uint8_t cmd_sensores(uint16_t d)
{
	(void)d;                                               // ignora el parametro dato
	uint16_t adc;                                          // variable local para lectura atomica
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { adc = adc_valor; } // copia el valor del adc de forma segura
	bt_enviar(rsp_temp, dht_temp);                         // envia trama 82 con temperatura
	bt_enviar(rsp_hum, dht_hum);                           // envia trama 83 con humedad
	bt_enviar(rsp_adc, adc);                               // envia trama 84 con valor analogico
	return 0;                                              // ejecucion exitosa
}

static uint8_t cmd_cambiar(uint16_t d)
{
	if (estado_actual == estado_seguro) return err_no_permitido; // impide salir de emergencia desde la app
	if (d > 1) return err_dato;                                  // valida que el dato sea 0 o 1
	registrar_actividad();                                       // reinicia el contador de inactividad
	if (d == 1) { if (estado_actual == estado_activo) evt_bt_standby = true; } // solicita ir a standby
	else        { if (estado_actual == estado_standby) evt_bt_wake = true; }   // solicita despertar a activo
	return 0;                                                    // ejecucion exitosa
}

static uint8_t cmd_actuador(uint16_t d)
{
	if (estado_actual == estado_seguro) return err_no_permitido; // bloquea control de actuador en emergencia
	if (d > 1) return err_dato;                                  // valida que el dato sea 0 o 1
	led_habilitado = (d == 1);                                   // habilita o deshabilita el parpadeo del led
	if (!led_habilitado) led_port &= ~(1 << led_bit);            // apaga fisicamente el led si se deshabilito
	registrar_actividad();                                       // cuenta como interaccion valida de usuario
	if (estado_actual == estado_standby) evt_bt_wake = true;     // despierta el sistema si estaba en standby
	return 0;                                                    // ejecucion exitosa
}

static uint8_t cmd_emergencia(uint16_t d)
{
	(void)d;               // ignora el dato recibido
	evt_bt_emerg = true;   // activa la bandera para pasar a estado seguro en la fsm
	registrar_actividad(); // actualiza marca de actividad
	return 0;              // ejecucion exitosa
}

static uint8_t cmd_alarmas(uint16_t d)
{
	(void)d;                                                     // ignora el dato recibido
	if (alarma_cant == 0) { bt_enviar(rsp_evento, 0); return 0; } // responde con 0 si no hay alarmas registradas
	uint8_t n = (alarma_cant > 4) ? 4 : alarma_cant;             // limita el envio a los ultimos 4 eventos
	for (uint8_t i = 0; i < n; i++) {
		uint8_t idx = (uint8_t)(alarma_head - n + i) & 7;        // calcula el indice cronologico en el historial
		bt_enviar(rsp_evento, alarmas[idx]);                     // transmite cada evento almacenado
	}
	return 0;                                                    // ejecucion exitosa
}

// solo para la prueba b del wdt: simula un bloqueo
static uint8_t cmd_test_wdt(uint16_t d)
{
	if (d != 1234) return err_dato; // exige clave 1234 para evitar disparos accidentales
	for (;;) { }                    // bucle infinito sin wdt_reset() para forzar reinicio por watchdog
	return 0;
}

typedef struct {
	uint8_t id;                   // numero identificador del comando (cc)
	uint8_t (*handler)(uint16_t); // puntero a la funcion que ejecuta dicho comando
} comando_t;

static const comando_t tabla_comandos[] = {
	{ cmd_estado_id,     cmd_estado },     // entrada 0: comando 01
	{ cmd_sensores_id,   cmd_sensores },   // entrada 1: comando 02
	{ cmd_cambiar_id,    cmd_cambiar },    // entrada 2: comando 03
	{ cmd_actuador_id,   cmd_actuador },   // entrada 3: comando 04
	{ cmd_emergencia_id, cmd_emergencia }, // entrada 4: comando 05
	{ cmd_alarmas_id,    cmd_alarmas },    // entrada 5: comando 06
	{ cmd_test_wdt_id,   cmd_test_wdt },   // entrada 6: comando 07
};
#define n_comandos (sizeof(tabla_comandos) / sizeof(tabla_comandos[0])) // calcula cantidad total de comandos

static void procesar_trama(void)
{
	uint8_t cc = (uint8_t)((trama[0] - '0') * 10 + (trama[1] - '0')); // convierte los 2 digitos ascii de comando a entero
	uint16_t dato = (uint16_t)(trama[2] - '0') * 1000u + (uint16_t)(trama[3] - '0') * 100u
	              + (uint16_t)(trama[4] - '0') * 10u + (uint16_t)(trama[5] - '0'); // convierte 4 digitos de dato
	uint16_t chk = (uint16_t)(trama[6] - '0') * 100u + (uint16_t)(trama[7] - '0') * 10u
	             + (uint16_t)(trama[8] - '0'); // convierte los 3 digitos ascii del checksum recibido

	if (chk != (uint16_t)((cc + dato) & 0xFF)) { // verifica integridad sumando comando + dato modulo 256
		bt_error(err_checksum);                  // responde con codigo de error de checksum
		registrar_evento(evt_error_checksum);    // guarda el evento en el historial de alarmas
		return;                                  // aborta el procesamiento de la trama corrupta
	}

	for (uint8_t i = 0; i < n_comandos; i++) { // recorre la tabla de despacho buscando coincidencia de id
		if (tabla_comandos[i].id == cc) {
			uint8_t r = tabla_comandos[i].handler(dato); // ejecuta la funcion asociada mediante su puntero
			if (r) bt_error(r);                          // si el comando devolvio error, lo notifica por uart
			return;
		}
	}
	bt_error(err_desconocido); // informa que el comando cc no esta registrado
}

// parser de tramas fuera de isr. detecta tramas incompletas, largas y con basura.
static void tarea_bt(void)
{
	if (!usart_on) return; // sale inmediatamente si la interfaz serie esta apagada

	if (rx_error) { rx_error = false; if (en_trama) { en_trama = false; rx_idx = 0; bt_error(err_incompleta); } } // gestiona error de hardware

	uint32_t ahora = millis(); // obtiene tiempo actual
	if (en_trama && (ahora - t_rx >= trama_timeout_ms)) { // verifica si paso demasiado tiempo sin recibir '>'
		en_trama = false; rx_idx = 0; // aborta la trama truncada
		bt_error(err_incompleta);       // reporta error por tiempo agotado
	}

	while (rx_tail != rx_head) { // extrae todos los bytes disponibles en el buffer circular rx
		char c = (char)rx_buf[rx_tail];      // lee el caracter mas antiguo
		rx_tail = (rx_tail + 1) & rx_mask;   // avanza el indice de lectura circular
		t_rx = ahora;                        // actualiza marca de tiempo de actividad rx

		if (c == trama_ini) {
			if (en_trama) bt_error(err_incompleta); // llego un '<' nuevo sin haber cerrado la trama previa
			en_trama = true; rx_idx = 0;            // inicia captura de nueva trama
		} else if (!en_trama) {
			continue;                               // ignora caracteres sueltos fuera de delimitadores
		} else if (c == trama_fin) {
			en_trama = false;                       // marca cierre de trama al recibir '>'
			if (rx_idx == trama_payload) { trama[rx_idx] = 0; procesar_trama(); } // valida longitud exacta de 9 bytes
			else bt_error(err_incompleta);          // rechaza si llegaron menos de 9 caracteres
			rx_idx = 0;                             // reinicia indice para la proxima trama
		} else if ((c >= '0') && (c <= '9') && (rx_idx < trama_payload)) {
			trama[rx_idx++] = c;                    // almacena digito numerico valido en el arreglo
		} else {
			en_trama = false; rx_idx = 0;           // descarta si llega letra, simbolo o exceso de longitud
			bt_error(err_incompleta);               // notifica trama mal formada
		}
	}
}

// dht11
static bool dht_esperar(uint8_t nivel, uint8_t max_us)
{
	while (((dht_pinr >> dht_bit) & 1) != nivel) { // espera hasta que el pin alcance el nivel logico esperado
		if (max_us == 0) return false;             // retorna falso si se agota el tiempo maximo (evita cuelgues)
		max_us--;                                  // decrementa contador de microsegundos
		_delay_us(1);                              // espera 1 microsegundo
	}
	return true;                                   // el pin cambio dentro del tiempo permitido
}

static bool dht11_leer_respuesta(void)
{
	uint8_t d[5] = {0, 0, 0, 0, 0}; // arreglo para los 5 bytes del protocolo dht11

	dht_ddr &= ~(1 << dht_bit); // libera el bus configurando pd3 como entrada
	_delay_us(30);              // espera 30 us a que la resistencia pull-up suba la linea

	if (!dht_esperar(0, 100)) return false; // espera pulso de presencia en bajo del sensor (80 us)
	if (!dht_esperar(1, 100)) return false; // espera pulso de presencia en alto del sensor (80 us)
	if (!dht_esperar(0, 100)) return false; // espera inicio de transmision del primer bit

	for (uint8_t i = 0; i < 40; i++) { // lee secuencialmente los 40 bits (5 bytes)
		if (!dht_esperar(1, 100)) return false; // espera fin del pulso bajo previo a cada bit (50 us)
		_delay_us(40);                          // espera 40 us para distinguir bit '0' (~26 us) de '1' (~70 us)
		if ((dht_pinr >> dht_bit) & 1) d[i >> 3] |= (0x80 >> (i & 7)); // si sigue en alto, graba un 1
		if (!dht_esperar(0, 100)) return false; // espera a que termine el pulso alto del bit actual
	}

	if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return false; // comprueba checksum de los 4 primeros bytes
	dht_hum = d[0];  // actualiza variable global de humedad entera
	dht_temp = d[2]; // actualiza variable global de temperatura entera
	return true;     // lectura valida completada
}

static void tarea_dht(void)
{
	uint32_t ahora = millis(); // lee tiempo actual en milisegundos

	if (dht_fase == 0) { // fase 0: aguardar cumplimiento del periodo de muestreo (2 s)
		if (ahora - t_temp >= periodo_temp_ms) {
			t_temp = ahora;              // actualiza marca temporal de muestreo
			dht_ddr |= (1 << dht_bit);   // configura pd3 como salida
			dht_port &= ~(1 << dht_bit); // fuerza nivel bajo para iniciar comunicacion con dht11
			t_dht = ahora;               // guarda instante de inicio del pulso de 20 ms
			dht_fase = 1;                // pasa a fase 1 para esperar sin bloquear el cpu
		}
	} else { // fase 1: esperar 20 ms de pulso start de forma cooperativa
		if (ahora - t_dht >= dht_start_ms) {
			dht11_leer_respuesta();      // captura la trama de 40 bits del sensor
			dht_ddr &= ~(1 << dht_bit);  // devuelve pd3 a estado de alta impedancia (entrada)
			dht_fase = 0;                // vuelve a fase 0 para el proximo ciclo
		}
	}
}

// tareas periodicas
static void tarea_led(void)
{
	if (millis() - t_led >= periodo_leds_ms) { // verifica si transcurrieron 200 ms
		t_led += periodo_leds_ms;              // suma el periodo exacto para evitar deriva temporal
		if (led_habilitado) led_port ^= (1 << led_bit); // invierte el estado del led pb5 si esta habilitado
	}
}

static void tarea_lcd(void)
{
	if (millis() - t_lcd >= periodo_lcd_ms) { // verifica si transcurrieron 500 ms
		t_lcd += periodo_lcd_ms;              // actualiza marca de tiempo
		actualizar_pantalla_lcd();            // refresca el contenido de la pantalla
	}
}

static void tarea_buzzer(void)
{
	if (buzzer_activo && (millis() - t_buzzer >= buzzer_ms)) { // verifica si pasaron 50 ms de pitido
		buzz_port &= ~(1 << buzz_bit);                         // apaga el buzzer en pd4
		buzzer_activo = false;                                 // limpia bandera de buzzer activo
	}
}

static void tarea_pulsador(void)
{
	uint32_t ahora = millis(); // tiempo actual en milisegundos
	bool pc;                   // copia local de la interrupcion de cambio de pin

	// boton 1 (int0)
	if (flag_boton) {
		flag_boton = false; // consume la bandera levantada por int0_vect
		if (ahora - t_ult_boton >= antirrebote_ms) { // aplica filtro antirrebote de 50 ms
			t_ult_boton = ahora;           // guarda instante de pulsacion valida
			buzz_port |= (1 << buzz_bit);  // enciende buzzer como retroalimentacion sonora
			buzzer_activo = true;          // indica que debe apagarse tras 50 ms
			t_buzzer = ahora;              // registra inicio del sonido
			registrar_actividad();         // reinicia el contador de inactividad del sistema
			evt_boton = true;              // genera evento para la maquina de estados
		}
	}

	// boton 2 (nivel por polling + pcint para despertar de power-down)
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pc = flag_pc; flag_pc = false; } // lee y limpia bandera de pcint2

	if (!(btn_pinr & (1 << btn2_bit)) || pc) { // detecta boton 2 presionado (nivel 0 o evento pcint)
		if (!presionado && (ahora - t_press >= antirrebote_ms)) { // valida flanco nuevo tras antirrebote
			presionado = true;             // bloquea repeticiones mientras siga apretado
			t_press = ahora;               // guarda marca de tiempo
			buzz_port |= (1 << buzz_bit);  // activa pitido de confirmacion
			buzzer_activo = true;          // habilita temporizador de apagado del buzzer
			t_buzzer = ahora;              // registra inicio del pitido
			registrar_actividad();         // actualiza tiempo de ultima actividad
			evt_boton2 = true;             // genera evento de emergencia o recuperacion
		}
	} else {
		if (presionado && (ahora - t_press >= antirrebote_ms)) { // detecta liberacion estable del boton 2
			presionado = false;            // habilita futura pulsacion
			t_press = ahora;               // guarda instante de liberacion
		}
	}
}

// configuracion de perifericos
static void adc_init(void)
{
	ADMUX = (1 << REFS0) | (canal_adc & 0x0F); // selecciona referencia avcc (5v) y canal adc0
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // activa adc, isr y prescaler 128
}

// restaura todo lo necesario para activo / seguro tras standby o power-down
static void perifericos_activar(void)
{
	PRR = 0x00;                  // habilita los relojes de todos los perifericos del microcontrolador
	i2c_init();                  // reinicializa el bus twi/i2c
	if (!usart_on) usart_init(); // reactiva la interfaz serie si estaba apagada
	adc_init();                  // configura y enciende el conversor adc
	TIMSK1 |= (1 << OCIE1A);     // rehabilita el disparo periodico cada 100 ms del timer1
	wdt_modo_reset_1s();         // configura el watchdog en modo supervision (reset en 1 s)
}

// transiciones de estado
static void ir_activo(void)
{
	perifericos_activar();          // enciende todos los modulos necesarios
	estado_actual = estado_activo;  // cambia el estado de la fsm a activo
	causa_wdt = false;              // limpia bandera de fallo previo por watchdog
	EICRA = (1 << ISC01);           // configura int0 por flanco descendente
	lcd_bl_mask = lcd_backlight;    // activa bit de retroiluminacion del lcd
	lcd_i2c_write(0x0c, lcd_command); // enciende el display lcd
	registrar_actividad();          // reinicia temporizador de inactividad
	resincronizar_tareas();         // sincroniza tareas periodicas
	actualizar_pantalla_lcd();      // actualiza inmediatamente el texto en pantalla
}

static void ir_standby(void)
{
	estado_actual = estado_standby; // cambia estado de la fsm a standby
	led_port &= ~(1 << led_bit);    // apaga el led indicador para reducir consumo
	// menos despertares innecesarios: sin disparo periodico del adc (timer1) y adc apagado
	TIMSK1 &= ~(1 << OCIE1A);       // deshabilita interrupcion de comparacion del timer1
	ADCSRA &= ~(1 << ADEN);         // apaga el modulo adc antes de cortar su reloj
	PRR = prr_standby;              // apaga relojes de adc, spi y timer2
	resincronizar_tareas();         // alinea tiempos de tareas
	actualizar_pantalla_lcd();      // muestra "esperando..." en el lcd
}

static void ir_apagado(void)
{
	estado_actual = estado_apagado; // cambia estado de la fsm a apagado
	led_port &= ~(1 << led_bit);    // asegura que el led quede apagado
	buzz_port &= ~(1 << buzz_bit);  // asegura que el buzzer quede silenciado
	buzzer_activo = false;          // limpia estado interno del buzzer

	// 1) lcd (necesita twi): apagar retroiluminacion y display
	lcd_bl_mask = 0x00;               // pone en cero la mascara de luz de fondo (p3 = 0)
	lcd_i2c_write(0x08, lcd_command); // envia comando display off (d=0) mientras twi sigue activo

	// 2) cortar perifericos
	ADCSRA &= ~(1 << ADEN); // apaga analogica del adc
	usart_apagar();         // vacia buffer tx y deshabilita usart0
	PRR = prr_apagado;      // corta relojes de adc, usart0, spi, timer1, timer2 y twi

	// 3) wdt como despertador periodico (solo interrupcion, 8 s)
	wdt_modo_int_8s();      // reconfigura wdt para despertar cada 8 s sin reiniciar el sistema
}

static void ir_seguro(void)
{
	perifericos_activar();          // asegura que lcd, uart y wdt esten operativos para reportar alarma
	estado_actual = estado_seguro;  // cambia estado de la fsm a seguro
	buzz_port &= ~(1 << buzz_bit);  // silencia el buzzer
	buzzer_activo = false;          // limpia bandera de buzzer
	led_port |= (1 << led_bit);     // enciende el led de forma fija como senal visual de alerta
	lcd_bl_mask = lcd_backlight;    // mantiene encendida la luz de fondo de la pantalla
	lcd_i2c_write(0x0c, lcd_command); // asegura display encendido
	actualizar_pantalla_lcd();      // muestra mensaje de emergencia o reset por wdt
}

// maquina de estados 
static void actualizar_fsm(void)
{
	uint32_t inactivo = millis() - t_actividad; // calcula tiempo transcurrido desde ultima actividad
	bool boton    = evt_boton;                  // copia local de evento boton 1
	bool boton2   = evt_boton2;                 // copia local de evento boton 2
	bool emerg_bt = evt_bt_emerg;               // copia local de emergencia por bluetooth
	bool stb_bt   = evt_bt_standby;             // copia local de paso a standby por bluetooth
	bool wake_bt  = evt_bt_wake;                // copia local de despertar por bluetooth
	evt_boton = evt_boton2 = evt_bt_emerg = evt_bt_standby = evt_bt_wake = false; // consume todos los eventos

	if (estado_actual == estado_seguro) {
		
		
		(void)boton;            // descarta explicitamente pulsaciones del boton 1
		if (boton2) ir_activo(); // vuelve a operacion normal solo al presionar boton 2
		else        tarea_lcd(); // mantiene refresco de pantalla en estado seguro
	}
	else if (boton2 || emerg_bt) {
		ir_seguro();                                                // transicion prioritaria a emergencia
		registrar_evento(boton2 ? evt_emerg_local : evt_emerg_app); // registra si fue local o remota
	}
	else if (estado_actual == estado_activo) {
		if (stb_bt || inactivo >= tiempo_inact_standby_ms) {
			ir_standby(); // pasa a standby tras 10 s sin actividad o por comando remoto
		} else {
			tarea_led();  // ejecuta parpadeo del led
			tarea_dht();  // ejecuta lectura no bloqueante del dht11
			tarea_lcd();  // ejecuta actualizacion periodica del lcd
		}
	}
	else if (estado_actual == estado_standby) {
		if (boton || wake_bt)                         ir_activo();  // despierta con boton 1 o bluetooth
		else if (inactivo >= tiempo_inact_apagado_ms) ir_apagado(); // pasa a power-down tras 30 s acumulados
		else                                          tarea_lcd();  // mantiene mensaje "esperando..."
	}
	else { // estado_apagado
		if (boton) ir_activo(); // vuelve a activo al despertar mediante boton 1
	}
}

// modos de bajo consumo
static void dormir_idle(void)
{
	set_sleep_mode(SLEEP_MODE_IDLE); // selecciona modo idle (detiene cpu pero mantiene timer0 y uart)
	cli();                           // deshabilita interrupciones para verificacion atomica de condiciones
	// se evita perder un evento que llegue justo antes de dormir
	if (!flag_boton && !flag_pc && (rx_head == rx_tail)) {
		sleep_enable();  // habilita bit se en registro smcr
		sei();           // activa interrupciones justo antes de dormir (garantizado por arquitectura avr)
		sleep_cpu();     // suspende el nucleo hasta que ocurra una interrupcion
		sleep_disable(); // deshabilita bit se al despertar por seguridad
	}
	sei(); // asegura interrupciones habilitadas si no entro a dormir
}

// lectura rapida tras despertar por wdt: se habilita el adc, se lee y se vuelve a apagar
static void lectura_rapida(void)
{
	PRR &= ~prr_adc;                                                    // reactiva reloj del adc en prr
	ADMUX = (1 << REFS0) | (canal_adc & 0x0F);                          // configura referencia avcc y canal 0
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // habilita adc sin interrupcion
	ADCSRA |= (1 << ADSC);                                              // inicia conversion unica
	while (ADCSRA & (1 << ADSC));                                       // espera fin de conversion por polling
	adc_valor = ADC;                                                    // almacena nueva muestra analogica
	ADCSRA = 0;                                                         // apaga el conversor adc
	PRR |= prr_adc;                                                     // vuelve a cortar reloj del adc en prr
}

static void dormir_apagado(void)
{
	do {
		flag_wdt = false;                  // limpia bandera de despertar por watchdog
		EICRA = 0x00;                      // configura int0 por nivel bajo (unico modo activo sin reloj en power-down)
		PCIFR = (1 << PCIF2);              // limpia bandera pendiente de interrupcion por cambio de pin
		set_sleep_mode(SLEEP_MODE_PWR_DOWN); // selecciona modo power-down (detiene relojes y oscilador principal)
		cli();                             // deshabilita interrupciones para evaluar condicion sin carrera
		if (!flag_boton && !flag_pc) {
			sleep_enable();      // habilita modo sleep
			sleep_bod_disable(); // apaga detector brown-out durante el sueno para ahorrar mas corriente
			sei();               // habilita interrupciones globales
			sleep_cpu();         // entra en sueno profundo
			sleep_disable();     // desactiva permiso de sueno al despertar
		}
		sei();                // restaura interrupciones
		EICRA = (1 << ISC01); // devuelve int0 a deteccion por flanco de bajada

		// desperto solo por el wdt: lectura rapida y volver a dormir
		if (flag_wdt && !flag_boton && !flag_pc) lectura_rapida(); // muestrea adc y sigue durmiendo
	} while (!flag_boton && !flag_pc); // permanece en el ciclo mientras no se pulse ningun boton fisico
}

// inicializacion
static void gpio_init(void)
{
	led_ddr |= (1 << led_bit) | (1 << test_bit);    // configura pb5 (led) y pb4 (pin test) como salidas
	buz_ddr |= (1 << buzz_bit);                     // configura pd4 (buzzer) como salida
	btn_ddr &= ~((1 << btn_bit) | (1 << btn2_bit)); // configura pd2 y pd5 como entradas
	btn_port |= (1 << btn_bit) | (1 << btn2_bit);   // activa resistencias pull-up internas en ambos botones
	dht_ddr &= ~(1 << dht_bit);                     // configura pd3 (dht11) inicialmente como entrada
}

static void timer0_init(void)
{
	TCCR0A = (1 << WGM01);              // modo ctc (borrado al coincidir con ocr0a)
	TCCR0B = (1 << CS01) | (1 << CS00); // prescaler de 64 (250 khz a 16 mhz)
	OCR0A = 249;                        // tope de cuenta: (16 mhz / (64 * 1000 hz)) - 1 = 249 -> 1 ms
	TIMSK0 = (1 << OCIE0A);             // habilita interrupcion por comparacion a
}

static void timer1_init(void)
{
	TCCR1A = 0;                          // operacion normal en registro a
	TCCR1B = (1 << WGM12) | (1 << CS12); // modo ctc con prescaler de 256 (62.5 khz)
	OCR1A = 6249;                        // tope de cuenta: (16 mhz / (256 * 10 hz)) - 1 = 6249 -> 100 ms
	TIMSK1 = (1 << OCIE1A);              // habilita interrupcion cada 100 ms para disparar el adc
}

static void int0_init(void)
{
	EICRA = (1 << ISC01); // configura interrupcion externa int0 por flanco descendente
	EIFR = (1 << INTF0);  // limpia bandera previa de int0
	EIMSK = (1 << INT0);  // habilita mascara de interrupcion int0
}

static void pcint_init(void)
{
	PCMSK2 = (1 << PCINT21); // habilita deteccion de cambio de estado en el pin pd5
	PCIFR = (1 << PCIF2);    // limpia bandera previa del grupo pcint2
	PCICR = (1 << PCIE2);    // habilita interrupciones pin change para el puerto d
}

int main(void)
{
	gpio_init();         // inicializa direccion y pull-ups de pines digitales
	i2c_init();          // inicializa bus i2c a 100 khz
	lcd_i2c_init();      // ejecuta secuencia de arranque de la pantalla lcd
	adc_init();          // configura conversor analogico digital
	timer0_init();       // arranca base de tiempos de 1 ms
	timer1_init();       // arranca temporizador de muestreo adc de 100 ms
	int0_init();         // habilita interrupcion del boton 1 (pd2)
	pcint_init();        // habilita interrupcion del boton 2 (pd5)
	usart_init();        // inicializa puerto serie uart a 9600 bps
	wdt_modo_reset_1s(); // activa supervision watchdog de 1 segundo
	sei();               // habilita las interrupciones globales

	t_actividad = millis(); // fija marca inicial de actividad
	resincronizar_tareas(); // sincroniza temporizadores de tareas periodicas

	if (mcusr_copy & (1 << WDRF)) {
		// reinicio por wdt: se identifica la causa y se recupera un estado seguro
		causa_wdt = true;                // indica que el reinicio fue por watchdog
		ir_seguro();                     // bloquea el sistema en estado seguro
		registrar_evento(evt_reset_wdt); // guarda y envia alarma de reset por wdt
	} else {
		actualizar_pantalla_lcd();       // dibuja pantalla inicial en estado activo
	}

	while (1) {
		wdt_reset();        // refresca el watchdog en cada ciclo normal para evitar reinicio
		tarea_pulsador();   // atiende eventos y antirrebote de ambos botones
		tarea_buzzer();     // apaga el buzzer cumplido su tiempo
		tarea_bt();         // procesa tramas recibidas desde bluetooth
		actualizar_fsm();   // evalua transiciones y ejecuta tareas del estado activo

		if (estado_actual == estado_standby)      dormir_idle();    // suspende cpu en idle durante standby
		else if (estado_actual == estado_apagado) dormir_apagado(); // suspende sistema en power-down durante apagado
	}
}
