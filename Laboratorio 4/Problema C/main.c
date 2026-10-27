/*
 LABORATORIO 4 - PROBLEMA C
 Sistema de alarma con dos LDR, mensajes por Bluetooth (HC-05) y cuatro pulsadores.

 ENTRADAS:
   A0 / PC0 : LDR1 (cocina)
   A1 / PC1 : LDR2 (ventana)
   D2 / PD2 : Boton M (bit 0: 0=AUTO, 1=MANUAL)
   D3 / PD3 : Boton S (bit 1: 0=SONIDO, 1=SILENCIO)
   D4 / PD4 : Boton E (bit 2: 0=OFF, 1=ARMADA)
   D5 / PD5 : Boton F (bit 3: 0=NORMAL, 1=FORZADA)
   
 SALIDAS:
    D8 / PB0 : LED verde
   D9 / PB1 : LED rojo
   D10 / PB2: buzzer
   
 LCD CON  I2C:
    SDA -> A4 / PC4
    SCL -> A5 / PC5
	
  BLUETOOTH HC-05:
    HC-05 TXD -> D0/PD0/RX
    HC-05 RXD <- D1/PD1/TX
 
Los cuatro botones usan pull-ups
  Al reiniciar: control[3:0] = 0000 (AUTO, SONIDO, OFF, NORMAL)
  Las pulsaciones se detectan con Timer0 cada 1 ms, incluso si se esta
  transmitiendo un estado por UART. Antirrebote: 25 ms.
  Cada boton tiene ademas un bloqueo individual de 1 s despues de cada pulsacion
  No se utilizan delays (bloqueantes)
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

#define BAUD               9600UL
#define UBRR_VAL           ((F_CPU / (16UL * BAUD)) - 1UL)

#define LIMITE_OSCURO      380U
#define LIMITE_CLARO       440U
#define CONFIRMACIONES     4U       // 4 muestras de 50 ms = unos 200 ms
#define ANTIRREBOTE_MS     25U      // 25 lecturas estables a intervalo de 1 ms
#define BLOQUEO_BOTON_MS   1000U    // 1 segundo entre pulsaciones aceptadas de UN MISMO boton
#define INTERVALO_LDR_MS   50UL
#define INTERVALO_INFO_MS  1000UL		// envia info al celular cada 1.5 s
#define INTERVALO_LCD_MS   200UL
#define PAGINA_LCD_MS     2500UL

#define LED_VERDE          PB0
#define LED_ROJO           PB1
#define BUZZER             PB2

#define BIT_M              0U
#define BIT_S              1U
#define BIT_E              2U
#define BIT_F              3U
#define MASCARA(b)         (1U << (b))
#define RX_BUFFER_TAM      128U

// Maquina de estados
typedef enum {
    EST_OFF,
    EST_MANUAL_ON,
    EST_MANUAL_SIL_ON,
    EST_AUTO_ON,
    EST_AUTO_SIL_ON,
    EST_ALARMA,
    EST_ALARMA_SIN_SONIDO
} estado_t;

typedef struct {
    uint16_t adc;
    uint8_t tapado;           // 1 = la luz esta bloqueada
    uint8_t confirmaciones;   // Para filtrar cambios breves
} ldr_t;

typedef struct {
    uint8_t pin;              // Pin fisico de PORTD (PD2 a PD5)
    uint8_t estable;          // 1=libre; 0=presionado, validado
    uint8_t candidato;        // Lectura candidata a estado nuevo
    uint8_t contador;         // Milisegundos consecutivos de esa lectura
    uint16_t bloqueo_ms;      // Bloqueo individual tras una pulsacion valida
} boton_t;

volatile uint8_t rx_buffer[RX_BUFFER_TAM];
volatile uint8_t rx_cabeza = 0;
volatile uint8_t rx_cola = 0;

static ldr_t ldr1 = {0, 0, 0};
static ldr_t ldr2 = {0, 0, 0};
static boton_t botones[4] = {
    {PD2, 1, 1, 0, 0}, // M -> bit 0
    {PD3, 1, 1, 0, 0}, // S -> bit 1
    {PD4, 1, 1, 0, 0}, // E -> bit 2
    {PD5, 1, 1, 0, 0}  // F -> bit 3
};

// La interrupcion guarda los cambios en este registro de eventos.
// XOR acumula la paridad de las pulsaciones para no perderlas si el
// programa principal esta ocupado enviando mensajes por Bluetooth.

static volatile uint8_t eventos_pulsadores = 0U;
static volatile uint32_t milisegundos = 0UL;

static uint8_t control = 0U;   // Inicialmente 0000. NO se guarda en EEPROM.
static uint8_t zona = 0U;      // bits 4 y 5: 00 libre, 01 LDR1, 10 LDR2, 11 ambas
static uint8_t ultima_zona = 0U;
static uint8_t alarma_memoria = 0U; // Alarma enclavada hasta desarmar
static estado_t estado = EST_OFF;

// ----------------------- COMUNICACION UART -----------------------
ISR(USART_RX_vect) {
    uint8_t dato = UDR0;
    uint8_t siguiente = (uint8_t)((rx_cabeza + 1U) & (RX_BUFFER_TAM - 1U));
    if (siguiente != rx_cola) {
        rx_buffer[rx_cabeza] = dato;
        rx_cabeza = siguiente;
    }
}

static void uart_iniciar(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL & 0xFFU);
    UCSR0A = 0;
    UCSR0B = MASCARA(RXEN0) | MASCARA(TXEN0) | MASCARA(RXCIE0);
    UCSR0C = MASCARA(UCSZ01) | MASCARA(UCSZ00);			 // 8N1
}

static void uart_caracter(char c) {
    while (!(UCSR0A & MASCARA(UDRE0))) { }
    UDR0 = (uint8_t)c;
}

static void uart_texto(const char *s) {
    while (*s != '\0') uart_caracter(*s++);
}

static void uart_texto_flash(PGM_P s) {
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') uart_caracter(c);
}

static void uart_numero(uint16_t n) {
    char digitos[6];
    uint8_t i = 0;
    do {
        digitos[i++] = (char)('0' + n % 10U);
        n /= 10U;
    } while (n != 0U);
    while (i > 0U) uart_caracter(digitos[--i]);
}

static void uart_binario(uint8_t valor, uint8_t cantidad) {
    while (cantidad > 0U) {
        --cantidad;
        uart_caracter((valor & MASCARA(cantidad)) ? '1' : '0');
    }
}

static uint8_t uart_leer(char *c) {
    uint8_t disponible = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (rx_cabeza != rx_cola) {
            *c = (char)rx_buffer[rx_cola];
            rx_cola = (uint8_t)((rx_cola + 1U) & (RX_BUFFER_TAM - 1U));
            disponible = 1U;
        }
    }
    return disponible;
}

// -------------------- CUATRO PULSADORES --------------------------
static void botones_iniciar(void) {
    uint8_t i;
    const uint8_t mascara = MASCARA(PD2) | MASCARA(PD3) |
                             MASCARA(PD4) | MASCARA(PD5);
    DDRD &= (uint8_t)~mascara;			// D2-D5 como entradas
    PORTD |= mascara;					// Resistencias pull-up
    _delay_ms(5);					  // Tiempo de arranqe
		
    // Si se deja un boton apretado durante RESET, los bits siguen en 0000
    //  no se toma como pulsacion hasta que se suelte y aprete de nuevo
	
    for (i = 0U; i < 4U; ++i) {
        uint8_t actual = (PIND & MASCARA(botones[i].pin)) ? 1U : 0U;
        botones[i].estable = actual;
        botones[i].candidato = actual;
        botones[i].contador = 0U;
        botones[i].bloqueo_ms = 0U;		 // Al reiniciar se puede pulsar sin espera previa
    }
}

static void temporizador_iniciar(void) {
    // Timer0 en CTC, interrupcion cada 1 ms
    // 16 MHz / 64 / (249 + 1) = 1000 interrupciones por segundo
    TCCR0A = MASCARA(WGM01);
    TCCR0B = 0U;
    OCR0A = 249U;
    TCNT0 = 0U;
    TIMSK0 |= MASCARA(OCIE0A);
    TCCR0B = MASCARA(CS01) | MASCARA(CS00);			// Prescaler 64
}

ISR(TIMER0_COMPA_vect) {
    uint8_t i;
    ++milisegundos;
    for (i = 0U; i < 4U; ++i) {
        uint8_t actual = (PIND & MASCARA(botones[i].pin)) ? 1U : 0U;
        boton_t *b = &botones[i];

        // El tiempo de bloqueo se descuenta POR BOTON, cada 1 ms
        // Esto permite pulsar otros botones sin esperar ni detener el programa
        if (b->bloqueo_ms > 0U) {
            --b->bloqueo_ms;
        }

        // Debounce: la lectura debe permanecer estable 25 ms
        if (actual != b->candidato) {
            b->candidato = actual;
            b->contador = 0U;
        } else if (actual != b->estable) {
            if (++b->contador >= ANTIRREBOTE_MS) {
                b->estable = actual;
                b->contador = 0U;

               // Solo al presionar
                if (actual == 0U && b->bloqueo_ms == 0U) {
                    eventos_pulsadores ^= (uint8_t)MASCARA(i);
                    b->bloqueo_ms = BLOQUEO_BOTON_MS;
                }
                // Si lo pulsan durante el bloqueo se ignora
                // Si lo mantienen apretado, tampoco se ejecuta al pasar
                // el segundo, se debe soltar y volver a presionar
            }
        } else {
            b->contador = 0U;
        }
    }
}

static uint32_t tiempo_ms(void) {
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = milisegundos; }
    return t;
}

static void aplicar_pulsaciones(void) {
    uint8_t eventos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        eventos = eventos_pulsadores;
        eventos_pulsadores = 0U;
    }
    // i=0 M, i=1 S, i=2 E, i=3 F.
    // Invertimos los bits qe fueron pulsados
	
    control ^= (uint8_t)(eventos & 0x0FU);
}

// ----------------------- SENSORES LDR ----------------------------
static void adc_iniciar(void) {
    ADMUX = MASCARA(REFS0);					// Referencia ADC = AVcc
    ADCSRA = MASCARA(ADEN) | MASCARA(ADPS2) |
             MASCARA(ADPS1) | MASCARA(ADPS0);
    DIDR0 = MASCARA(ADC0D) | MASCARA(ADC1D);
}

static uint16_t adc_leer(uint8_t canal) {
    ADMUX = (uint8_t)((ADMUX & 0xF0U) | (canal & 0x0FU));
    _delay_us(20);
    ADCSRA |= MASCARA(ADSC);
    while (ADCSRA & MASCARA(ADSC)) { }
    return ADC;
}

static void ldr_actualizar(ldr_t *sensor, uint16_t valor) {
    uint8_t lectura = sensor->tapado;
    sensor->adc = valor;

    if (!sensor->tapado && valor < LIMITE_OSCURO) lectura = 1U;
    if ( sensor->tapado && valor > LIMITE_CLARO)  lectura = 0U;

    if (lectura != sensor->tapado) {
        if (++sensor->confirmaciones >= CONFIRMACIONES) {
            sensor->tapado = lectura;
            sensor->confirmaciones = 0U;
        }
    } else {
        sensor->confirmaciones = 0U;
    }
}

static void leer_sensores(void) {
    ldr_actualizar(&ldr1, adc_leer(0));
    ldr_actualizar(&ldr2, adc_leer(1));
    zona = (uint8_t)(ldr1.tapado | (ldr2.tapado << 1));
}

// ------------------- REGISTROS Y ESTADOS -------------------------
static uint8_t bit_control(uint8_t bit) {
    return (control & MASCARA(bit)) ? 1U : 0U;
}

static uint8_t registro_control(void) {
    return (uint8_t)((control & 0x0FU) | (zona << 4));
}

static uint8_t registro_flags(void) {
    uint8_t flags = 0U;
    if (PORTB & MASCARA(LED_VERDE)) flags |= MASCARA(0);
    if (PORTB & MASCARA(LED_ROJO))  flags |= MASCARA(1);
    if (PORTB & MASCARA(BUZZER))   flags |= MASCARA(2);
    return flags;
}

static const char *estado_nombre(estado_t e) {
    switch (e) {
        case EST_OFF:               return "OFF";
        case EST_MANUAL_ON:         return "MANUAL/ON";
        case EST_MANUAL_SIL_ON:     return "MANUAL/SIL/ON";
        case EST_AUTO_ON:           return "AUTO/ON";
        case EST_AUTO_SIL_ON:       return "AUTO/SIL/ON";
        case EST_ALARMA:            return "ALARMA";
        default:                    return "ALARMA SIN SONIDO";
    }
}

static const char *zona_nombre(uint8_t z) {
    switch (z) {
        case 1:  return "COCINA (LDR1)";
        case 2:  return "VENTANA (LDR2)";
        case 3:  return "LIVING";
        default: return "DESPEJADO";
    }
}


// -------------------- LCD 16x2 POR I2C ---------------------------
// Manejo directo de I2C
// P4-P7=D4-D7, el bus funciona a 100 kHz

#define LCD_RS    0x01U
#define LCD_E     0x04U
#define LCD_LUZ   0x08U

static uint8_t lcd_direccion = 0U;
static uint8_t lcd_lista = 0U;
static char lcd_cache[2][17] = {{0}, {0}};

static uint8_t twi_esperar(void) {
    uint16_t limite = 60000U;
    while (!(TWCR & MASCARA(TWINT))) {
        if (--limite == 0U) return 0U;
    }
    return 1U;
}

static void twi_detener(void) {
    uint16_t limite = 60000U;
    TWCR = MASCARA(TWINT) | MASCARA(TWEN) | MASCARA(TWSTO);
    while ((TWCR & MASCARA(TWSTO)) && --limite) { }
}

static uint8_t twi_empezar(void) {
    TWCR = MASCARA(TWINT) | MASCARA(TWEN) | MASCARA(TWSTA);
    if (!twi_esperar()) return 0U;
    return ((TWSR & 0xF8U) == 0x08U || (TWSR & 0xF8U) == 0x10U);
}

static uint8_t twi_escribir(uint8_t dato) {
    TWDR = dato;
    TWCR = MASCARA(TWINT) | MASCARA(TWEN);
    if (!twi_esperar()) return 0U;
    return ((TWSR & 0xF8U) == 0x18U || (TWSR & 0xF8U) == 0x28U);
}

static void twi_iniciar(void) {
    TWSR = 0U;                    // Prescaler 1
    TWBR = 72U;                   // SCL = 100 kHz con 16 MHz
    TWCR = MASCARA(TWEN);
}

static uint8_t twi_detectar(uint8_t direccion) {
    uint8_t ok;
    if (!twi_empezar()) { twi_detener(); return 0U; }
    ok = twi_escribir((uint8_t)(direccion << 1U));
    twi_detener();
    return ok;
}

// Escribe una secuenciav en una sola transaccion I2C
static uint8_t lcd_transmitir(uint8_t alto, uint8_t bajo,
                              uint8_t incluir_bajo) {
    uint8_t ok = 1U;
    if (!twi_empezar()) { twi_detener(); return 0U; }
    if (!twi_escribir((uint8_t)(lcd_direccion << 1U))) ok = 0U;
    if (ok && !twi_escribir((uint8_t)(alto | LCD_LUZ | LCD_E))) ok = 0U;
    if (ok && !twi_escribir((uint8_t)(alto | LCD_LUZ))) ok = 0U;
    if (ok && incluir_bajo) {
        if (!twi_escribir((uint8_t)(bajo | LCD_LUZ | LCD_E))) ok = 0U;
        if (ok && !twi_escribir((uint8_t)(bajo | LCD_LUZ))) ok = 0U;
    }
    twi_detener();
    return ok;
}

static uint8_t lcd_comando(uint8_t dato) {
    if (!lcd_transmitir((uint8_t)(dato & 0xF0U),
                       (uint8_t)((dato << 4U) & 0xF0U), 1U)) return 0U;
    if (dato == 0x01U || dato == 0x02U) _delay_ms(2);
    else _delay_us(50);
    return 1U;
}

static uint8_t lcd_caracter(char c) {
    uint8_t v = (uint8_t)c;
    if (!lcd_transmitir((uint8_t)((v & 0xF0U) | LCD_RS),
                       (uint8_t)(((v << 4U) & 0xF0U) | LCD_RS), 1U)) return 0U;
    _delay_us(50);
    return 1U;
}

static void lcd_iniciar(void) {
    twi_iniciar();
    _delay_ms(50);               // Encendido LCD HD44780

    if (twi_detectar(0x27U)) lcd_direccion = 0x27U;
    else if (twi_detectar(0x3FU)) lcd_direccion = 0x3FU;
    else { lcd_lista = 0U; return; }

    // Secuencia de inicio obligatoria del LCD
	
    if (!lcd_transmitir(0x30U, 0U, 0U)) return;
    _delay_ms(5);
    if (!lcd_transmitir(0x30U, 0U, 0U)) return;
    _delay_us(200);
    if (!lcd_transmitir(0x30U, 0U, 0U)) return;
    _delay_us(200);
    if (!lcd_transmitir(0x20U, 0U, 0U)) return;
    _delay_us(200);

    if (!lcd_comando(0x28U)) return;	// 4 bits, 2 lineas, 5x8
    if (!lcd_comando(0x08U)) return;	// Apagar pantalla
    if (!lcd_comando(0x01U)) return;	// Limpiar (solo al iniciar)
    if (!lcd_comando(0x06U)) return;	// Cursor avanza
    if (!lcd_comando(0x0CU)) return;	// Display encendido, cursor oculto
    lcd_lista = 1U;
}

// Completar las 16 columnas con espacios, sin usar sprintf
static void lcd_linea_vacia(char linea[17]) {
    uint8_t i;
    for (i = 0U; i < 16U; ++i) linea[i] = ' ';
    linea[16] = '\0';
}

static void lcd_poner(char linea[17], uint8_t posicion, const char *s) {
    while (*s != '\0' && posicion < 16U) linea[posicion++] = *s++;
}

static void lcd_poner_bits(char linea[17], uint8_t posicion,
                           uint8_t n, uint8_t cantidad) {
    uint8_t i;
    for (i = 0U; i < cantidad && posicion + i < 16U; ++i)
        linea[posicion + i] = (n & MASCARA(cantidad - 1U - i)) ? '1' : '0';
}

/*
static void lcd_poner_adc(char linea[17], uint8_t posicion, uint16_t valor) {
    uint8_t i;
    for (i = 0U; i < 4U && posicion + i < 16U; ++i) {
        if (posicion + 3U - i < 16U)
            linea[posicion + 3U - i] = (char)('0' + (valor % 10U));
        valor /= 10U;
    }
}
																				Coment-out desp de pruebas*/
static const char *lcd_zona_nombre(uint8_t zona_actual) {
    switch (zona_actual) {
        case 1U: return "COCINA";
        case 2U: return "VENTANA";
        case 3U: return "LIVING";
        default: return "DESPEJADO";
    }
}

static void lcd_escribir_linea(uint8_t fila, const char texto[17]) {
    uint8_t i;
    if (!lcd_lista || memcmp(lcd_cache[fila], texto, 16U) == 0) return;
    if (!lcd_comando(fila == 0U ? 0x80U : 0xC0U)) {
        lcd_lista = 0U;
        return;
    }
    for (i = 0U; i < 16U; ++i) {
        if (!lcd_caracter(texto[i])) {
            lcd_lista = 0U;
            return;
        }
    }
    memcpy(lcd_cache[fila], texto, 16U);
}

// Pagina 0: lectura rapida. Pagina 1: zona y registros.
// Pagina 2: ADC de cada LDR								(sacar despues de calibracion.)
//			 En OFF: La config elegida

static void lcd_actualizar(uint32_t ahora) {
    char arriba[17], abajo[17];
    uint8_t pagina = (uint8_t)((ahora / PAGINA_LCD_MS) % 3UL);
    uint8_t flags = registro_flags();
    uint8_t activo = (estado == EST_ALARMA || estado == EST_ALARMA_SIN_SONIDO);

    if (!lcd_lista) return;
    lcd_linea_vacia(arriba);
    lcd_linea_vacia(abajo);

    if (estado == EST_OFF) {
        if (pagina == 0U) {
            lcd_poner(arriba, 0U, "OFF / M:");
            lcd_poner(arriba, 9U, bit_control(BIT_M) ? "MANUAL" : "AUTO");
            lcd_poner(abajo, 0U, "S:");
            lcd_poner(abajo, 2U, bit_control(BIT_S) ? "OFF" : "ON");
            lcd_poner(abajo, 7U, "F:");
            lcd_poner(abajo, 9U, bit_control(BIT_F) ? "FORZADA" : "NORMAL");
        } else /*if (pagina == 1U)*/ {
            lcd_poner(arriba, 0U, "OFF / SELECCION");
            lcd_poner(abajo, 0U, "F E S M :");
            lcd_poner_bits(abajo, 10U, control, 4U);
			 }  
/*			else {
            lcd_poner(arriba, 0U, "LDR1 ADC:");
            lcd_poner_adc(arriba, 10U, ldr1.adc);
            lcd_poner(abajo, 0U, "LDR2 ADC:");
            lcd_poner_adc(abajo, 10U, ldr2.adc);
			 }												 Sacar desp de Calibrcion	*/	
    } else if (pagina == 0U) {
        if (activo) {
            lcd_poner(arriba, 0U, estado == EST_ALARMA ?
                      "ALARMA SONANDO" : "ALARMA EN SILENCIO");
            lcd_poner(abajo, 0U, "INTR:");
            lcd_poner(abajo, 5U, ultima_zona ? lcd_zona_nombre(ultima_zona) : "FORZADA");
        } else {
            lcd_poner(arriba, 0U, estado_nombre(estado));
            lcd_poner(abajo, 0U, "L1:");
            lcd_poner(abajo, 3U, ldr1.tapado ? "!!" : "Ok");
            lcd_poner(abajo, 7U, "L2:");
            lcd_poner(abajo, 10U, ldr2.tapado ? "!!" : "Ok");
        }
    } else/* if (pagina == 1U)*/ {
        lcd_poner(arriba, 0U, "ZONA:");
        lcd_poner(arriba, 6U, (activo && ultima_zona == 0U) ?
                  "FORZADA" : lcd_zona_nombre(activo ? ultima_zona : zona));
        lcd_poner(abajo, 0U, "C:");
        lcd_poner_bits(abajo, 2U, registro_control(), 4U);
        lcd_poner(abajo, 7U, "FLAGS:");
        lcd_poner_bits(abajo, 13U, flags, 3U);
    }
	/* else {
        lcd_poner(arriba, 0U, "LDR1 ADC:");
        lcd_poner_adc(arriba, 10U, ldr1.adc);
        lcd_poner(abajo, 0U, "LDR2 ADC:");
        lcd_poner_adc(abajo, 10U, ldr2.adc);
        }																	Sacar post pruebas */

    lcd_escribir_linea(0U, arriba);
    lcd_escribir_linea(1U, abajo);
}


static void actualizar_maquina(void) {
    if (!bit_control(BIT_E)) {
        // E=0 desarma y borra la alarma
        alarma_memoria = 0U;
        ultima_zona = 0U;
        estado = EST_OFF;
    } else {
        // E=1 habilita la deteccion de intrusiones en ambos modos
        // F fuerza la alarma solo en modo manual
        uint8_t forzado_manual = bit_control(BIT_M) && bit_control(BIT_F);
        if (zona != 0U || forzado_manual) {
            if (!alarma_memoria || (ultima_zona == 0U && zona != 0U)) {
                ultima_zona = zona;
            }
            alarma_memoria = 1U;
        }

        if (alarma_memoria) {
            estado = bit_control(BIT_S) ? EST_ALARMA_SIN_SONIDO : EST_ALARMA;
        } else if (bit_control(BIT_M)) {
            estado = bit_control(BIT_S) ? EST_MANUAL_SIL_ON : EST_MANUAL_ON;
        } else {
            estado = bit_control(BIT_S) ? EST_AUTO_SIL_ON : EST_AUTO_ON;
        }
    }

    // LEDs y buzzer muestran el estado actual
    PORTB &= (uint8_t)~(MASCARA(LED_VERDE) | MASCARA(LED_ROJO) |
                        MASCARA(BUZZER));
    if (estado == EST_ALARMA || estado == EST_ALARMA_SIN_SONIDO) {
        PORTB |= MASCARA(LED_ROJO);
        if (estado == EST_ALARMA) PORTB |= MASCARA(BUZZER);
    } else if (estado != EST_OFF) {
        PORTB |= MASCARA(LED_VERDE);
    }
}

// ------------------------- BLUETOOTH -----------------------------
static void enviar_estado(void) {
    uart_texto_flash(PSTR("\r\nESTADO: ")); uart_texto(estado_nombre(estado));
    uart_texto_flash(PSTR("\r\nCONTROL F E S M: ")); uart_binario(control & 0x0FU, 4);
    uart_texto_flash(PSTR(" | M=")); uart_numero(bit_control(BIT_M));
    uart_texto_flash(PSTR(" S=")); uart_numero(bit_control(BIT_S));
    uart_texto_flash(PSTR(" E=")); uart_numero(bit_control(BIT_E));
    uart_texto_flash(PSTR(" F=")); uart_numero(bit_control(BIT_F));

 //   uart_texto_flash(PSTR("\r\nLDR1: ")); uart_numero(ldr1.adc);
  //  uart_texto_flash(PSTR(" | LDR2: ")); uart_numero(ldr2.adc); 
    uart_texto(ldr1.tapado ? " [MOVIMIENTO DETECTADO!]" : " [OK]");

    uart_texto(ldr2.tapado ? " [MOVIMIENTO DETECTADO!]" : " [OK]");

    uart_texto_flash(PSTR("\r\nZONA: ")); uart_numero(zona);
    uart_texto_flash(PSTR(" - ")); uart_texto(zona_nombre(zona));
    uart_texto_flash(PSTR(" | ULTIMA: ")); uart_texto(zona_nombre(ultima_zona));
    uart_texto_flash(PSTR("\r\nREG CONTROL [5:0]: "));
    uart_binario(registro_control(), 6);
    uart_texto_flash(PSTR(" | FLAGS [2:0]: "));
    uart_binario(registro_flags(), 3);
    uart_texto_flash(PSTR("\r\n"));
}

static void ayuda(void) {
    uart_texto_flash(PSTR("\r\nCOMANDOS (terminar con ENTER):\r\n"));
    uart_texto_flash(PSTR("M / S / E / F = alterna el bit indicado\r\n"));
    uart_texto_flash(PSTR("AUTO / MANUAL = CAMBIA M\r\n"));
    uart_texto_flash(PSTR("ARMAR / DESARMAR = CAMBIA E\r\n"));
    uart_texto_flash(PSTR("SILENCIAR / SONIDO = CAMBIA S en 1 / 0\r\n"));
    uart_texto_flash(PSTR("FORZAR / NORMAL = CAMBIA F (solo en MODO MANUAL)\r\n"));
    uart_texto_flash(PSTR("ESTADO = muestra entradas, control y flags\r\n"));
    uart_texto_flash(PSTR("AYUDA = lista los comandos\r\n"));
}

static void procesar_comando(const char *cmd) {
    if (strcmp(cmd, "M") == 0) {
        control ^= MASCARA(BIT_M);
    } else if (strcmp(cmd, "S") == 0) {
        control ^= MASCARA(BIT_S);
    } else if (strcmp(cmd, "E") == 0) {
        control ^= MASCARA(BIT_E);
    } else if (strcmp(cmd, "F") == 0) {
        control ^= MASCARA(BIT_F);
    } else if (strcmp(cmd, "AUTO") == 0) {
        control &= (uint8_t)~MASCARA(BIT_M);
    } else if (strcmp(cmd, "MANUAL") == 0) {
        control |= MASCARA(BIT_M);
    } else if (strcmp(cmd, "ARMAR") == 0) {
        control |= MASCARA(BIT_E);
    } else if (strcmp(cmd, "DESARMAR") == 0) {
        control &= (uint8_t)~MASCARA(BIT_E);
    } else if (strcmp(cmd, "SILENCIAR") == 0) {
        control |= MASCARA(BIT_S);
    } else if (strcmp(cmd, "SONIDO") == 0) {
        control &= (uint8_t)~MASCARA(BIT_S);
    } else if (strcmp(cmd, "FORZAR") == 0) {
        control |= MASCARA(BIT_F);
    } else if (strcmp(cmd, "NORMAL") == 0) {
        control &= (uint8_t)~MASCARA(BIT_F);
    } else if (strcmp(cmd, "ESTADO") == 0) {
        actualizar_maquina();
        enviar_estado();
        return;
    } else if (strcmp(cmd, "AYUDA") == 0) {
        ayuda();
        return;
    } else {
        uart_texto_flash(PSTR("Comando desconocido. Escribi AYUDA.\r\n"));
        return;
    }
    uart_texto_flash(PSTR("OK. Nuevo control F E S M: "));
    uart_binario(control & 0x0FU, 4);
    uart_texto_flash(PSTR("\r\n"));
}

static void revisar_bluetooth(void) {
	static char comando[32];
	static uint8_t longitud = 0U;
	static uint32_t ultimo_caracter_ms = 0UL;
	char c;

	while (uart_leer(&c)) {
		if (c == '\r' || c == '\n') {
			if (longitud > 0U) {
				comando[longitud] = '\0';

				uart_texto_flash(PSTR("\r\nBT recibido: "));
				uart_texto(comando);
				uart_texto_flash(PSTR("\r\n"));

				procesar_comando(comando);
				longitud = 0U;
			}
			} else if ((uint8_t)c >= 32U && (uint8_t)c <= 126U) {
			if (c >= 'a' && c <= 'z')
			c = (char)(c - 'a' + 'A');

			if (longitud < sizeof(comando) - 1U) {
				comando[longitud++] = c;
				ultimo_caracter_ms = tiempo_ms();
				} else {
				longitud = 0U;
				uart_texto_flash(PSTR("ERROR: comando demasiado largo\r\n"));
			}
		}
	}

	// (Procesa el comando aunque el celular no envie ENTER, la APP de ABtC daba problemas)
	if (longitud > 0U &&
	(uint32_t)(tiempo_ms() - ultimo_caracter_ms) >= 250UL) {

		comando[longitud] = '\0';

		uart_texto_flash(PSTR("\r\nBT recibido: "));
		uart_texto(comando);
		uart_texto_flash(PSTR("\r\n"));

		procesar_comando(comando);
		longitud = 0U;
	}
}

// --------------------------- MAIN --------------------------------
int main(void) {
    estado_t estado_previo;
    uint8_t zona_previa;
    uint8_t control_previo;
 uint32_t ultimo_ldr;
 // uint32_t ultima_info;
    uint32_t ultimo_lcd;

    // Reinicio: los 4 bits SIEMPRE empiezan en 0000.
    control = 0U;
    alarma_memoria = 0U;
    estado = EST_OFF;
    DDRB |= MASCARA(LED_VERDE) | MASCARA(LED_ROJO) | MASCARA(BUZZER);
    PORTB &= (uint8_t)~(MASCARA(LED_VERDE) | MASCARA(LED_ROJO) |
                         MASCARA(BUZZER));
    DDRC &= (uint8_t)~(MASCARA(PC0) | MASCARA(PC1));
    PORTC &= (uint8_t)~(MASCARA(PC0) | MASCARA(PC1));

    botones_iniciar();
    adc_iniciar();
    lcd_iniciar();
    uart_iniciar();
    temporizador_iniciar();
    sei();

    leer_sensores();
    actualizar_maquina();
    estado_previo = estado;
    zona_previa = zona;
    control_previo = registro_control();
   ultimo_ldr = tiempo_ms();
  // ultima_info = ultimo_ldr;
    ultimo_lcd = ultimo_ldr;
    lcd_actualizar(ultimo_ldr);

    uart_texto_flash(PSTR("\r\nALARMA 2 LDR + 4 BOTONES - ATmega328P\r\n"));
    uart_texto_flash(PSTR("Control inicial F E S M: 0000\r\n"));
    uart_texto_flash(PSTR("Pulsadores: antirrebote 25 ms + bloqueo individual  1s .\r\n"));
    uart_texto_flash(PSTR("Enviar AYUDA para comandos.\r\n"));
    if (lcd_lista) uart_texto_flash(PSTR("LCD I2C detectada\r\n"));
    else uart_texto_flash(PSTR("AVISO: LCD I2C no detectada \r\n"));

    while (1) {
        uint32_t ahora;
        // El ISR de Timer0 revisa los pulsadores aun durante los mensajes UART
        aplicar_pulsaciones();
        revisar_bluetooth();

        ahora = tiempo_ms();
        if ((uint32_t)(ahora - ultimo_ldr) >= INTERVALO_LDR_MS) {
            ultimo_ldr = ahora;
            leer_sensores();
        }
        actualizar_maquina();

        if ((uint32_t)(ahora - ultimo_lcd) >= INTERVALO_LCD_MS) {
            ultimo_lcd = ahora;
            lcd_actualizar(ahora);
        }

        if (estado != estado_previo || zona != zona_previa ||
            registro_control() != control_previo) {
            if ((estado_previo != EST_ALARMA &&
                 estado_previo != EST_ALARMA_SIN_SONIDO) &&
                (estado == EST_ALARMA || estado == EST_ALARMA_SIN_SONIDO)) {
                uart_texto_flash(PSTR("\r\n*** ALERTA INTRUSION / ALARMA ***\r\n"));
                uart_texto_flash(PSTR("ZONA: "));
                uart_texto(zona_nombre(ultima_zona));
                uart_texto_flash(PSTR("\r\n"));
            }
     // Mensaje breve para no bloquear las lecturas ADC demasiado tiempi
            uart_texto_flash(PSTR("CTRL FESM=")); uart_binario(control & 0x0FU, 4U);
            uart_texto_flash(PSTR("; ESTADO=")); uart_texto(estado_nombre(estado));
            uart_texto_flash(PSTR("; ZONA=")); uart_numero(zona);
            uart_texto_flash(PSTR("\r\n"));
            estado_previo = estado;
            zona_previa = zona;
            control_previo = registro_control();
        }

  /*      // Telemetria/Notificaciones resumen cada 1 segundo (segun el valor de INTERVALO_INFO_MS)
		ESTADO entrega todos los detalles.
        if ((uint32_t)(ahora - ultima_info) >= INTERVALO_INFO_MS) {
            ultima_info = ahora;
        uart_texto_flash(PSTR("LDR1=")); uart_numero(ldr1.adc);
        uart_texto_flash(PSTR(" LDR2=")); uart_numero(ldr2.adc);
            uart_texto_flash(PSTR(" ZONA=")); uart_numero(zona);
            uart_texto_flash(PSTR("\r\n"));
        }
		*/
  // Se comentaron fuera algunas lineas de codigo, ya que algunas se usaron durante el desarrollo o calibracion
  // del sistema (como es el caso de las LDR/sensores), y podrian resutar molestas (como el spam de estado y
  // sensores cada 1 segundo, o una 3era pantalla con esta misma informacion.
    }
}
