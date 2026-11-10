#define F_CPU 16000000UL 

#include <avr/io.h>        
#include <avr/interrupt.h> 
#include <util/delay.h>    
#include <util/atomic.h>   // ATOMIC_BLOCK: lectura segura de variables de 16/32 bits compartidas con ISR
#include <stdint.h>       
#include <stdbool.h>       
#include <stdio.h>        

// periodos de las tareas periodicas 

#define periodo_leds_ms 200UL // tiempo de parpadeo del led 
#define periodo_lcd_ms 500UL  // para refrescar el texto mostrado en la pantalla LCD
#define periodo_temp_ms 2000UL // tiempo en ms entre cada consulta al sensor DHT11
#define dht_start_ms 20UL      // tiempo en ms que debemos mantener en bajo el pin de datos para despertar al DHT11

// tiempos de inactividad, ambos contados desde la ultima actividad

#define tiempo_inact_standby_ms 10000UL 
#define tiempo_inact_apagado_ms 30000UL

// pulsador y buzzer

#define antirrebote_ms 50UL       
#define buzzer_ms 50UL             
#define pulsacion_larga_ms 3000UL  // tiempo sosteniendo el botón (3 s) para activar o salir de la emergencia

// pines

// de la led
#define led_ddr DDRB   
#define led_port PORTB 
#define led_bit PB5  
  
// del buzzer
#define buz_ddr DDRD   
#define buzz_port PORTD
#define buzz_bit PD4   

// del boton
#define btn_ddr DDRD   
#define btn_port PORTD 
#define btn_pinr PIND  
#define btn_bit PD2    // el pin PD2 asignado al botón (coincide con la interrupción externa INT0)


// sensor temp
#define dht_ddr DDRD   
#define dht_port PORTD 
#define dht_pinr PIND  // registro de lectura digital para leer los pulsos que envía el DHT11
#define dht_bit PD3    

#define canal_adc 0    // Canal del conversor analógico asignado 

// lcd por i2c
#define lcd_addr 0x27     
#define lcd_command 0     //  (RS = 0) para enviar configuraciones a la LCD
#define lcd_data 1        //  (RS = 1) para escribir letras o numeros en la LCD
#define lcd_backlight 0x08// para mantener encendida la luz de fondo de la LCD
#define en 0x04           //  Enable para mandar el pulso de reloj que valida el dato enviado


// estados de la fsm la variable estado_actual va guardando en cual estamos

typedef enum {
	estado_activo = 0, 
	estado_standby,    
	estado_apagado,    
	estado_seguro,     
	n_estados          // Guarda el número total de estados definidos
} estado_t;


// variables compartidas con ISR

static volatile uint32_t ms_ticks = 0;  // contador de milisegundos incrementado por el Timer 0
static volatile bool flag_boton = false;// bandera levantada por la interrupción INT0 al presionar el botón
static volatile uint16_t adc_valor = 0; // guarda la última lectura de 10 bits entregada por el ADC

// planificador cooperativo

static uint32_t t_led, t_lcd, t_temp, t_dht; // guardan la marca de tiempo (millis) de la última ejecución de cada tarea
static uint8_t dht_fase = 0;                  // controla la etapa de lectura del DHT11 (0 = esperar tiempo, 1 = enviando pulso de inicio)

// pulsador y buzzer
static uint32_t t_ult_boton, t_buzzer, t_press; // tiempos guardados para antirrebote, apagado de buzzer y tiempo presionado
static bool buzzer_activo = false, presionado = false, largo_hecho = false; // banderas de estado interno del botón y buzzer

// eventos generados por tarea_pulsador 
static bool evt_boton = false;   
static bool evt_largo = false;  

// fsm

static estado_t estado_actual = estado_activo; // incializa la maquina arrancando directamente en estado activo
static uint32_t t_actividad = 0;                // guarda la última interacción del usuario

// sensores

static uint8_t dht_temp = 0, dht_hum = 0; // guardan la temperatura y la humedad  leídas desde el sensor

static uint8_t lcd_bl_mask = lcd_backlight; // para apagaro  encender la retroalimentacion 
static const char *const nombre_estado[n_estados] = { "act", "stb", "off", "seg" };


// prototipos

static uint32_t millis(void);               // función para leer los milisegundos con la libreria atomic
static void registrar_actividad(void);      //  para reiniciar el tiempo de inactividad
static void resincronizar_tareas(void);     //  para acomodar las marcas de tiempo al cambiar de estado
static void actualizar_pantalla_lcd(void);  //  dibuja el texto según el estado actual


// interrupciones

ISR(TIMER0_COMPA_vect) { ms_ticks++; } // interrupcion del Timer 0, suma 1 ms al contador cada vez que ocurre la coincidencia

ISR(TIMER1_COMPA_vect)
{
	if (ADCSRA & (1 << ADEN)) ADCSRA |= (1 << ADSC); // interrupción del Timer 1, cada 100 ms dispara una nueva conversión del ADC si esta activo
}

ISR(ADC_vect) { adc_valor = ADC; } // interrupcion del ADC, se ejecuta sola al terminar la conversión analógica y guarda el valor en la variable

ISR(INT0_vect) { flag_boton = true; } // interrupcion del botón (INT0): detecta la pulsación fisica y avisa a la tarea del boton levantando la bandera


// configuracion de millis()

// devuevle los ms transcurridos protegiendo la lectura de interrupciones para evitar datos corruptos
static uint32_t millis(void)
{
	uint32_t t; //  guarda el valor leído
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = ms_ticks; } // bloquea momentáneamente las interrupciones mientras copia ms_ticks
	return t; 
}

// actualiza la marca de tiempo de la última vez que el usuario presionó el botón
static void registrar_actividad(void) { t_actividad = millis(); }

// reinicia los contadores de tiempo de las tareas cuando volvemos de un estado de reposo
static void resincronizar_tareas(void)
{
	uint32_t ahora = millis(); 
	t_led = t_lcd = t_temp = ahora; 
	dht_fase = 0; 
}

// i2c y lcd


static void i2c_init(void)
{
	TWSR = 0; 
	TWBR = ((F_CPU / 100000UL) - 16) / 2;
	TWCR = (1 << TWEN); // habilita  el  TWI
}

// envia la condicion de START para tomar el control del  I2C
static void i2c_start(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // manda el pulso de inicio por hardware
	while (!(TWCR & (1 << TWINT))); // espera ue este listo
}

// escribe un byte de datos 
static void i2c_write(uint8_t data)
{
	TWDR = data; // carga el dato a transmitir 
	TWCR = (1 << TWINT) | (1 << TWEN); // comienza la transmisión del byte
	while (!(TWCR & (1 << TWINT)));
}

// manda la condicion de STOP para liberar el bus I2C
static void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // manda la señal de parada
}

// Formatea un dato en dos nibbles (4 bits) para enviarlo al chip PCF8574 que controla la pantalla LCD
static void lcd_i2c_write(uint8_t data, uint8_t mode)
{
	uint8_t high = data & 0xf0;        // separa los 4 bits más significativos
	uint8_t low = (data << 4) & 0xf0;  // desplza y separa los 4 bits menos significativos
	uint8_t d[4];                      //arreglo que envia los pulsos enable
	d[0] = high | mode | lcd_bl_mask | en;  
	d[1] = high | mode | lcd_bl_mask;       
	d[2] = low | mode | lcd_bl_mask | en;  
	d[3] = low | mode | lcd_bl_mask;        

	i2c_start(); 
	i2c_write(lcd_addr << 1); 
	for (uint8_t i = 0; i < 4; i++) i2c_write(d[i]); 
	i2c_stop(); 
}

static void lcd_i2c_init(void)
{
	_delay_ms(50); 
	lcd_i2c_write(0x03, lcd_command); 
	_delay_ms(5); // retardo segun la datasheet
	lcd_i2c_write(0x03, lcd_command); 
	_delay_us(100);
	lcd_i2c_write(0x03, lcd_command);
	_delay_us(100); 
	lcd_i2c_write(0x02, lcd_command); 

	lcd_i2c_write(0x28, lcd_command); 
	lcd_i2c_write(0x0c, lcd_command); // enciende la pantalla y oculta el cursor parpadeante
	lcd_i2c_write(0x06, lcd_command); // configura el desplazamiento del cursor a la derecha
	lcd_i2c_write(0x01, lcd_command); // borra todo el contenido de la pantalla LCD
	_delay_ms(2); // retardo para completar la limpieza de pantalla
}

// mueve el cursor a la fila y columna deseada dentro de la LCD
static void lcd_i2c_set_cursor(uint8_t fila, uint8_t col)
{
	lcd_i2c_write((fila == 0) ? (0x80 + col) : (0xc0 + col), lcd_command); // calcula la dirección DDRAM correspondiente
}


static void lcd_i2c_write_string(const char *str)
{
	while (*str) lcd_i2c_write((uint8_t)(*str++), lcd_data); // recorre la cadena hasta encontrar el caracter nulo de fin
}


static void actualizar_pantalla_lcd(void)
{
	
	if (estado_actual == estado_apagado) return; 

	char tmp[17]; // buffer auxiliar para dar formato a los textos impresos
	uint16_t adc; // variable local para guardar el valor del ADC leido de forma atomica
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { adc = adc_valor; } // copia el valor del ADC 

	if (estado_actual == estado_standby) {
		lcd_i2c_set_cursor(0, 0); lcd_i2c_write_string("esperando...    ");
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string("                "); 
		} else if (estado_actual == estado_seguro) {
		lcd_i2c_set_cursor(0, 0); lcd_i2c_write_string("emergencia      "); 
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string("mant. btn 3s    "); 
		} else {
		snprintf(tmp, sizeof(tmp), "t:%2uc h:%2u%%    ", dht_temp, dht_hum); // formatea la lectura de temperatura y humedad
		lcd_i2c_set_cursor(0, 0); lcd_i2c_write_string(tmp); // imprime el texto en la fila 1
		snprintf(tmp, sizeof(tmp), "adc:%4u %s   ", adc, nombre_estado[estado_actual]); // formatea el ADC y el nombre del estado
		lcd_i2c_set_cursor(1, 0); lcd_i2c_write_string(tmp); // imprime el texto en la fila 2
	}
}

// configuracion del DHT11

//  aguarda a que  cambie al nivel deseado dentro de un tiempo máximo 
static bool dht_esperar(uint8_t nivel, uint8_t max_us)
{
	while (((dht_pinr >> dht_bit) & 1) != nivel) { // verifica el estado del pin
		if (max_us == 0) return false; 
		max_us--; 
		_delay_us(1); 
	}
	return true; // Si cambió al nivel esperado dentro del tiempo devuelve verdadero
}

// lee la trama de 40 bits enviada por el DHT11 y verifica el checksum
static bool dht11_leer_respuesta(void)
{
	uint8_t d[5] = {0, 0, 0, 0, 0}; //   guarda los 5 bytes recibidos del sensor

	dht_ddr &= ~(1 << dht_bit); // pone el pin en modo entrada liberando el bus
	_delay_us(30);

	if (!dht_esperar(0, 100)) return false; // espera la respuesta inicial en bajo del DHT11
	if (!dht_esperar(1, 100)) return false; // lo mismo pero en alto
	if (!dht_esperar(0, 100)) return false; // espera el inicio de la transmisión de datos

	for (uint8_t i = 0; i < 40; i++) { // bucle para capturar los 40 bits de datos uno a uno
		if (!dht_esperar(1, 100)) return false; 
		_delay_us(40); // espera 40 us para medir si el pulso en ALTO es un 0 o un 1
		if ((dht_pinr >> dht_bit) & 1) d[i >> 3] |= (0x80 >> (i & 7)); // si sigue alto es un 1, lo guarda en el byte
		if (!dht_esperar(0, 100)) return false; // Espera el fin del pulso en alto
	}

	if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return false; // revisa si la suma de los datos coincide con el byte de checksum
	dht_hum = d[0];  
	dht_temp = d[2]; 
	return true;    
}

// rutina temporal de 2 fases, los 20 ms en bajo se esperan sin bloquear

// gestiona el proceso de lectura del DHT11 dividido en fases para no congelar el micro durante los 20ms
static void tarea_dht(void)
{
	uint32_t ahora = millis(); // guarda el tiempo actual

	if (dht_fase == 0) { // FASE 0: Esperar a que se cumplan los 2 segundos entre mediciones
		if (ahora - t_temp >= periodo_temp_ms) { 
			t_temp = ahora; 
			dht_ddr |= (1 << dht_bit);   // deja el pin como SALIDA
			dht_port &= ~(1 << dht_bit); // pone la linea en BAJO para pedir lectura
			t_dht = ahora; // Marca la hora de inicio del pulso de activación
			dht_fase = 1;  // Avanza a la FASE 1
		}
		} else { // FASE 1: Esperar los 20 ms en bajo de la señal de inicio sin bloquear
		if (ahora - t_dht >= dht_start_ms) { // ¿Transcurrieron los 20 ms del pulso de inicio?
			dht11_leer_respuesta();     // Lee los datos recibidos del sensor
			dht_ddr &= ~(1 << dht_bit); // Cambia el pin de nuevo a ENTRADA
			dht_fase = 0;               // Reinicia a FASE 0 para la próxima lectura
		}
	}
}

// tareas periodicas



static void tarea_led(void)
{
	if (millis() - t_led >= periodo_leds_ms) { 
		t_led += periodo_leds_ms; // acumula el tiempo para evitar corrimientos
		led_port ^= (1 << led_bit); // (enciende/apaga)
	}
}

// actualiza los mensajes de la LCD cada 500 ms

static void tarea_lcd(void)
{
	if (millis() - t_lcd >= periodo_lcd_ms) { 
		t_lcd += periodo_lcd_ms;  
		actualizar_pantalla_lcd(); 
	}
}

// controla el tiempo que el buzzer se queda sonando para apagarlo de forma no bloqueante
static void tarea_buzzer(void)
{
	if (buzzer_activo && (millis() - t_buzzer >= buzzer_ms)) { 
		buzz_port &= ~(1 << buzz_bit);
		buzzer_activo = false;          
	}
}



// procesa el botón físico, aplica antirrebote y determina si fue una pulsación corta o larga
static void tarea_pulsador(void)
{
	uint32_t ahora = millis(); // guarda el tiempo actual

	if (flag_boton) { // se fija si currió la interrupción externa del botón
		flag_boton = false; 
		if (ahora - t_ult_boton >= antirrebote_ms) { 
			t_ult_boton = ahora; 
			buzz_port |= (1 << buzz_bit); // prende el buzzer 
			buzzer_activo = true;         
			t_buzzer = ahora;             // guarda cuando hizo el pitido
			registrar_actividad();        // reinicia la cuenta para que el sistema no se duerma
			evt_boton = true;             
		}
	}

	if (!(btn_pinr & (1 << btn_bit))) {
		if (!presionado) { 
			presionado = true; 
			largo_hecho = false; // reinicia el detector de pulsación larga
			t_press = ahora; // guarda el tiempo inicial en que se apretó
			} else if (!largo_hecho && (ahora - t_press >= pulsacion_larga_ms)) { 
			largo_hecho = true;     
			registrar_actividad();  
			evt_largo = true;       
		}
		} else {
		presionado = false; 
	}
}


// acciones al entrar a cada estado (cada una cambia estado_actual)

// configura el sistema para ingresar al ESTADO ACTIVO
static void ir_activo(void)
{
	estado_actual = estado_activo;  
	lcd_bl_mask = lcd_backlight;    // prende la luz de fondo de la lcd
	lcd_i2c_write(0x0c, lcd_command); // envia comando para encender la pantalla LCD por si venía de apagado
	ADCSRA |= (1 << ADEN);         
	registrar_actividad();          // reinicia la cuenta del temporizador de inactividad
	resincronizar_tareas();         // sincroniza los tiempos de las tareas periódicas al momento actual
	actualizar_pantalla_lcd();     
}


static void ir_standby(void)
{
	estado_actual = estado_standby; 
	led_port &= ~(1 << led_bit);    // Apaga el led  para ahorrar energía
	resincronizar_tareas();        
	actualizar_pantalla_lcd();     
}


static void ir_apagado(void)
{
	estado_actual = estado_apagado;   
	led_port &= ~(1 << led_bit); 
	lcd_bl_mask = 0x00;               // apaga la luz de fondo de la LCD
	lcd_i2c_write(0x08, lcd_command);
	ADCSRA &= ~(1 << ADEN);           // apaga el conversor ADC por completo para reducir el consumo
}


static void ir_seguro(void)
{
	estado_actual = estado_seguro;    
	buzz_port &= ~(1 << buzz_bit);    // apaga el buzzer
	buzzer_activo = false;           
	led_port |= (1 << led_bit);       // deja el led fijo 
	lcd_bl_mask = lcd_backlight;      // asegura que la luz de la pantalla esté prendida
	lcd_i2c_write(0x0c, lcd_command); 
	ADCSRA |= (1 << ADEN);            
	actualizar_pantalla_lcd();       
}


// maquina de estados con if / else if evalua las condiciones y decide los cambios de estado 

 

static void actualizar_fsm(void)
{

	
	uint32_t inactivo = millis() - t_actividad;   // tiempo desde la ultima actividad
	bool boton = evt_boton; 
	bool largo = evt_largo;
	evt_boton = false;      
	evt_largo = false;      

	// decide segun el estado actual
	if (estado_actual == estado_seguro) {
		if (largo) ir_activo();                   
		else       tarea_lcd();                   // si sigue en seguro sólo actualiza pantalla
	}
	else if (largo) {
		ir_seguro();                              // desde cualquier estado puede ir al seguro
	}
	else if (estado_actual == estado_activo) {
		if (inactivo >= tiempo_inact_standby_ms) {
			ir_standby();                         
			} else {
			tarea_led();                          // ejecuta la tarea del parpadeo del LED
			tarea_dht();                          
			tarea_lcd();                          
		}
	}
	else if (estado_actual == estado_standby) {
		if (boton)                                    ir_activo();   // standby a activo
		else if (inactivo >= tiempo_inact_apagado_ms) ir_apagado();  // standby a apagado 
		else                                          tarea_lcd();   // mantiene el refresco de pantalla
	}
	else { 
		if (boton) ir_activo();                  // apagado a activo
	}
}



// conf. la dirección de entrada o salida 
static void gpio_init(void)
{
	led_ddr |= (1 << led_bit);    
	buz_ddr |= (1 << buzz_bit);    
	btn_ddr &= ~(1 << btn_bit);   
	btn_port |= (1 << btn_bit);   // activa la resistencia pull-up para el boton
	dht_ddr &= ~(1 << dht_bit);   // Inicializa el pin del DHT11 como entrada
}

// configura el Timer 0 en modo CTC para generar la interrupción de la base de tiempo de 1 ms
static void timer0_init(void)
{
	TCCR0A = (1 << WGM01);              
	TCCR0B = (1 << CS01) | (1 << CS00); // Selecciona un Prescaler de 64
	OCR0A = 249;                        // Carga el tope de cuenta (16MHz / (64 * 1000Hz) - 1 = 249)
	TIMSK0 = (1 << OCIE0A);             // Habilita la interrupción por comparación con OCR0A
}

// Configura el Timer 1 en modo CTC para disparar la lectura del ADC cada 100 ms
static void timer1_init(void)
{
	TCCR1A = 0;                         
	TCCR1B = (1 << WGM12) | (1 << CS12); 
	OCR1A = 6249;                        // carga el tope para 100 ms (16MHz / (256 * 10Hz) - 1 = 6249)
	TIMSK1 = (1 << OCIE1A);              // habilita la interrupción por comparación con OCR1A
}

// configura el ADC sobre el canal ADC0
static void adc_init(void)
{
	ADMUX = (1 << REFS0) | (canal_adc & 0x0F); //  Vcc como referencia de voltaje y el canal ADC0
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enciende el ADC, habilita interrupción y prescaler de 128 (125 kHz)
}

// Configura INT0 para detectar el botón

static void int0_init(void)
{
	EICRA = (1 << ISC01);  
	EIFR = (1 << INTF0);   
	EIMSK = (1 << INT0);   
}


int main(void)
{
	gpio_init();    // inicia la dirección de los pines digital E/S
	i2c_init();     // conf. la velocidad del bus de comunicacion I2C
	lcd_i2c_init(); // envia la secuencia de comandos de arranque a la pantalla LCD
	adc_init();     // configura los registros del conversor ADC
	timer0_init();  // inicializa el Timer 0 para la base de tiempos de 1 ms
	timer1_init();  // inicializa el Timer 1 para disparar el ADC cada 100 ms
	int0_init();    // habilita la interrupción externa del botón
	sei();          // habilita las interrupciones globales del microcontrolador

	t_actividad = millis();    
	resincronizar_tareas();    // ajusta los tiempos iniciales de ejecución de las tareas
	actualizar_pantalla_lcd(); // escribe el estado inicial en la pantalla LCD

	while (1) {
		tarea_pulsador();   
		tarea_buzzer();     
		actualizar_fsm();   
	}
}

