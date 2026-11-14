/*		Laboratorio 5 - Parte D

 
  LCD: SDA = A4 , SCL = A5
  D2 (PD2/INT0): SIGUIENTE y DESPERTAR / SALIR
  D3 (PD3):      OK / EJECUTAR
  D9 (PB1/OC1A): salida para CH2 de osciloscopio (tiempo de despertar)
  D13 (PB5):     LED del Arduino, WDT reset
  A0 (ADC0):
 
  PCF8574 backpack comun: P0=RS, P1=RW, P2=E, P3=BL,
                        P4..P7=DB4..DB7. La variante puede diferir.
  La direccion de 7 bits se detecta en 0x20-0x27 y 0x38-0x3F.
  No se mide la corriente del MCU por separado si se alimenta la placa
  con LCD y modulo I2C conectados: tener en cuenta corrientes externas.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>

#define B_SIGUIENTE PD2
#define B_OK        PD3
#define PIN_PRUEBA  PB1
#define LED_RESET   PB5

#define LCD_ADDR_FIJA 0x00U

#define PCF_RS (1U<<0)
#define PCF_RW (1U<<1)
#define PCF_EN (1U<<2)
#define PCF_BL (1U<<3)

#define PRR_VALIDOS ((1U<<PRADC)|(1U<<PRUSART0)|(1U<<PRSPI)| \
                     (1U<<PRTIM1)|(1U<<PRTIM0)|(1U<<PRTIM2)|(1U<<PRTWI))

#define CATEGORIAS 5U
#define N_SLEEP 4U
#define N_PRR 10U
#define N_WDT 5U
#define N_CLK 6U
#define EXP_NINGUNO 0U
#define EXP_SLEEP   1U
#define EXP_WDT     2U

static volatile uint8_t prueba_actual = EXP_NINGUNO;
static volatile uint8_t despertar_o_salir = 0U;
static volatile uint8_t evento_wdt = 0U;
static uint8_t lcd_direccion = 0U;
static uint8_t lcd_presente = 0U;
static uint8_t luz_lcd = 1U;

volatile uint8_t origen_reset __attribute__((section(".noinit")));
void capturar_reset(void) __attribute__((naked, used, section(".init3")));

void capturar_reset(void)
{
    __asm__ __volatile__(
        "in r24, %[mcusr]\n\t"
        "sts origen_reset, r24\n\t"
        "clr r25\n\t"
        "out %[mcusr], r25\n\t"
        "wdr\n\t"
        "ldi r24, %[unlock]\n\t"
        "sts %[wdt], r24\n\t"
        "sts %[wdt], r25\n\t"
        :
        : [mcusr] "I" (_SFR_IO_ADDR(MCUSR)),
          [wdt]   "n" (_SFR_MEM_ADDR(WDTCSR)),
          [unlock] "M" ((1U<<WDCE)|(1U<<WDE))
        : "r24", "r25", "memory"
    );
}

ISR(INT0_vect)				  //	Primer evento de despertar: flanco en D9

{
    if (prueba_actual == EXP_SLEEP) PORTB |= (1U << PIN_PRUEBA);
    despertar_o_salir = 1U;
    EIMSK &= (uint8_t)~(1U << INT0);
}

ISR(WDT_vect)				// Cada flanco de D9 corresponde a un periodo del WDT
{
    PINB = (1U << PIN_PRUEBA);
    evento_wdt = 1U;
}

// -------------------------- TWI  --------------------------

static void twi_init(void)
{
    PRR &= (uint8_t)~(1U<<PRTWI);				 // Clock del TWI habilitado
    DDRC &= (uint8_t)~((1U<<PC4)|(1U<<PC5));
    PORTC |= (1U<<PC4)|(1U<<PC5);
    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);		// 100 kHz
    TWCR = (1U<<TWEN);
}

static uint8_t twi_esperar(void)
{
    uint16_t espera = 35000U;				// Un Timeout para no quedar bloqueado si los SDA/SCL fallan
    while (!(TWCR & (1U<<TWINT)) && espera) espera--;
    return (TWCR & (1U<<TWINT)) != 0U;
}

static void twi_stop(void)
{
    uint16_t espera = 10000U;
    TWCR = (1U<<TWINT)|(1U<<TWSTO)|(1U<<TWEN);		// Evita un START nuevo si el STOP sigue pendiente
    while ((TWCR & (1U<<TWSTO)) && espera) espera--;
}

static uint8_t twi_iniciar_escritura(uint8_t direccion)
{
    uint8_t estado;
    TWCR = (1U<<TWINT)|(1U<<TWSTA)|(1U<<TWEN);
    if (!twi_esperar()) { twi_stop(); return 0U; }
    estado = (uint8_t)(TWSR & 0xF8U);
    if (estado != 0x08U && estado != 0x10U) {
        twi_stop(); return 0U;
    }
    TWDR = (uint8_t)(direccion << 1);			 // 7 bits + escritura 
    TWCR = (1U<<TWINT)|(1U<<TWEN);
    if (!twi_esperar() || (TWSR & 0xF8U) != 0x18U) {
        twi_stop(); return 0U;
    }
    return 1U;
}

static uint8_t twi_escribir(uint8_t dato)
{
    TWDR = dato;
    TWCR = (1U<<TWINT)|(1U<<TWEN);
    return twi_esperar() && ((TWSR & 0xF8U) == 0x28U);
}

static uint8_t twi_enviar_pcf(uint8_t dato)
{
    uint8_t bien;
    if (!lcd_presente) return 0U;
    if (!twi_iniciar_escritura(lcd_direccion)) {
        lcd_presente = 0U;
        return 0U;
    }
    bien = twi_escribir(dato);
    twi_stop();
    if (!bien) lcd_presente = 0U;
    return bien;
}

static uint8_t detectar_lcd(void)		// Escanea las direcciones de la lcd
{
    uint8_t a;
    if (LCD_ADDR_FIJA != 0U) {
        if (twi_iniciar_escritura(LCD_ADDR_FIJA)) {
            twi_stop();
            lcd_direccion = LCD_ADDR_FIJA;
            return 1U;
        }
        return 0U;
    }
    if (twi_iniciar_escritura(0x27U)) {    //	Probar direcciones comunes primero y despues las restantes
        twi_stop(); lcd_direccion = 0x27U; return 1U;
    }
    if (twi_iniciar_escritura(0x3FU)) {
        twi_stop(); lcd_direccion = 0x3FU; return 1U;
    }
    for (a = 0x20U; a <= 0x27U; a++) {
        if (twi_iniciar_escritura(a)) {
            twi_stop(); lcd_direccion = a; return 1U;
        }
    }
    for (a = 0x38U; a <= 0x3FU; a++) {
        if (twi_iniciar_escritura(a)) {
            twi_stop(); lcd_direccion = a; return 1U;
        }
    }
    return 0U;
}

//------------------------------------

static uint8_t pcf_luz(void) { return luz_lcd ? PCF_BL : 0U; }

static void lcd_nibble(uint8_t nibble_alto, uint8_t es_dato)
{
    uint8_t salida = (uint8_t)((nibble_alto & 0xF0U) |
                        (es_dato ? PCF_RS : 0U) | pcf_luz());
    (void)twi_enviar_pcf(salida);
    (void)twi_enviar_pcf((uint8_t)(salida | PCF_EN));
    _delay_us(2);
    (void)twi_enviar_pcf(salida);					// flanco de bajada
    _delay_us(50);
}

static void lcd_write(uint8_t valor, uint8_t es_dato)
{
    if (!lcd_presente) return;
    lcd_nibble(valor, es_dato);
    lcd_nibble((uint8_t)(valor << 4), es_dato);
    if (!es_dato && (valor == 0x01U || valor == 0x02U)) _delay_ms(2);
}
static void lcd_cmd(uint8_t cmd) { lcd_write(cmd, 0U); }

static void lcd_init(void)
{
    twi_init();
    lcd_presente = detectar_lcd();
    if (!lcd_presente) return;
    luz_lcd = 1U;
    _delay_ms(50);
    lcd_nibble(0x30U, 0U); _delay_ms(5);
    lcd_nibble(0x30U, 0U); _delay_us(160);
    lcd_nibble(0x30U, 0U);
    lcd_nibble(0x20U, 0U);
    lcd_cmd(0x28U);
    lcd_cmd(0x0CU);						//	encendida, sin el  cursor
    lcd_cmd(0x06U);
    lcd_cmd(0x01U);
}

static void lcd_line(uint8_t linea, const char *s)
{
    uint8_t n = 0U;
    if (!lcd_presente) return;
    lcd_cmd((uint8_t)(0x80U | (linea ? 0x40U : 0U)));
    while (*s && n < 16U) { lcd_write((uint8_t)*s++, 1U); n++; }
    while (n < 16U) { lcd_write(' ', 1U); n++; }
}

static void pantalla(const char *arriba, const char *abajo)
{
    lcd_line(0U, arriba);
    lcd_line(1U, abajo);
}

static void backlight(uint8_t encendida)
{
    luz_lcd = (encendida != 0U);
    (void)twi_enviar_pcf(pcf_luz());				// EN=0, RW=0, RS=0
}

// ---------------- Botones y recursos ----------------

static uint8_t pulsado(uint8_t p) { return !(PIND & (1U<<p)); }
static void soltar(uint8_t p)
{
    while (pulsado(p)) { __asm__ __volatile__("nop"); }
    _delay_ms(30);
}

//	 0 = ninguno, 1 = SIGUIENTE, 2 = OK
static uint8_t boton_menu(void)
{
    if (pulsado(B_SIGUIENTE)) {
        _delay_ms(25);
        if (pulsado(B_SIGUIENTE)) { soltar(B_SIGUIENTE); return 1U; }
    }
    if (pulsado(B_OK)) {
        _delay_ms(25);
        if (pulsado(B_OK)) { soltar(B_OK); return 2U; }
    }
    return 0U;
}

static void preparar_int0(void)
{
    DDRD &= (uint8_t)~((1U<<B_SIGUIENTE)|(1U<<B_OK));
    PORTD |= (1U<<B_SIGUIENTE)|(1U<<B_OK);
    EICRA &= (uint8_t)~((1U<<ISC01)|(1U<<ISC00));
    EIFR = (1U<<INTF0);
    EIMSK &= (uint8_t)~(1U<<INT0);
}
static void armar_int0(void)
{
    EIFR = (1U<<INTF0);
    EIMSK |= (1U<<INT0);
}
static void desarmar_int0(void) { EIMSK &= (uint8_t)~(1U<<INT0); }

static void limpiar_perifericos(void)
{
    cli();
    desarmar_int0();
    PRR = 0U;							// reactiva antes de escribir en los registros de los perifericos
    TIMSK0=0U; TIMSK1=0U; TIMSK2=0U;
    TCCR0A=0U; TCCR0B=0U;
    TCCR1A=0U; TCCR1B=0U;
    TCCR2A=0U; TCCR2B=0U;
    UCSR0B=0U; SPCR=0U; TWCR=0U; ADCSRA=0U;
    ACSR = (1U<<ACD);
    prueba_actual = EXP_NINGUNO;
    despertar_o_salir = 0U;
    evento_wdt = 0U;
    DDRB |= (1U<<PIN_PRUEBA)|(1U<<LED_RESET);
    PORTB &= (uint8_t)~((1U<<PIN_PRUEBA)|(1U<<LED_RESET));
    PRR = (uint8_t)(PRR_VALIDOS & ~(1U<<PRTWI));
    sei();
}

static void restaurar_menu(void)
{
    // En las pruebas de CLKPR se restablecen los 16 MHz antes de esta funcion
    wdt_disable();
    limpiar_perifericos();
    twi_init();				// Obligatoria despues de haber deshabilitado PRTWI
    backlight(1U);
    lcd_cmd(0x0CU);
}

// ---------------------- WDT y el CLK --------------------------- 
static void configurar_wdt_irq(uint8_t bits)
{
    uint8_t sreg = SREG;
    const uint8_t unlock = (1U<<WDCE)|(1U<<WDE);
    const uint8_t final = (uint8_t)((1U<<WDIE)|bits);
    cli();
    wdt_reset();
    MCUSR &= (uint8_t)~(1U<<WDRF);
    __asm__ __volatile__(
        "sts %[reg], %[a]\n\t"
        "sts %[reg], %[b]\n\t"
        : : [reg] "n" (_SFR_MEM_ADDR(WDTCSR)),
            [a] "r" (unlock), [b] "r" (final) : "memory");
    SREG = sreg;
}

static void ajustar_clkpr(uint8_t codigo)
{
    uint8_t sreg = SREG;
    const uint8_t unlock = (1U<<CLKPCE);
    cli();
    __asm__ __volatile__(
        "sts %[reg], %[a]\n\t"
        "sts %[reg], %[b]\n\t"
        : : [reg] "n" (_SFR_MEM_ADDR(CLKPR)),
            [a] "r" (unlock), [b] "r" (codigo) : "memory");
    SREG = sreg;
}

//	----------- Modos SLEEP y el DESPERTAR -----------------

static void ensayo_sleep(uint8_t modo)
{
    if (modo == 1U) {				   //	ADC esta activo solo en el modo ADC Noise Reduction
        PRR = (uint8_t)(PRR_VALIDOS & ~(1U<<PRADC) & ~(1U<<PRTWI));
        ADMUX = (1U<<REFS0);
        ADCSRA = (1U<<ADEN)|(1U<<ADPS2)|(1U<<ADPS1)|(1U<<ADPS0);
        DIDR0 |= (1U<<ADC0D);
        set_sleep_mode(SLEEP_MODE_ADC);
    } else {
        ADCSRA = 0U;
        PRR = (uint8_t)(PRR_VALIDOS & ~(1U<<PRTWI));
        if (modo == 0U) set_sleep_mode(SLEEP_MODE_IDLE);
        else set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    }
    pantalla("SLEEP: MIDIENDO", "D2 para salir");
    _delay_ms(450);
    backlight(0U);    // Al ir a Dormiir se apaga TWI, pero la LCD mantiene el ultimo estado
    PRR |= (1U<<PRTWI);
    PORTB &= (uint8_t)~(1U<<PIN_PRUEBA);
    prueba_actual = EXP_SLEEP;
    despertar_o_salir = 0U;
    cli();
    if (!pulsado(B_SIGUIENTE)) {
        armar_int0();
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
        desarmar_int0();
    } else {
        sei();
    }
    if (pulsado(B_SIGUIENTE)) soltar(B_SIGUIENTE);
    restaurar_menu();
    pantalla("MCU DESPIERTO", "Volviendo menu");
    _delay_ms(550);
}

//	-------------------- PRR ----(Desactivar algunos Perifericos) -----------

static const uint8_t mascaras_prr[9] = {
    0U, (1U<<PRADC), (1U<<PRUSART0), (1U<<PRSPI),
    (1U<<PRTIM0), (1U<<PRTIM1), (1U<<PRTIM2), (1U<<PRTWI),
    PRR_VALIDOS
};

static void iniciar_perifericos_prr(void)
{
    PRR = 0U;
    TIMSK0=0U; TIMSK1=0U; TIMSK2=0U;
    TCCR0B=(1U<<CS01)|(1U<<CS00); 
    TCCR1B=(1U<<CS11)|(1U<<CS10);
    TCCR2B=(1U<<CS22); 
    UBRR0H=0U; UBRR0L=103U;
    UCSR0A=0U; UCSR0C=(1U<<UCSZ01)|(1U<<UCSZ00);
    UCSR0B=(1U<<RXEN0)|(1U<<TXEN0);
    DDRB |= (1U<<DDB2)|(1U<<DDB3)|(1U<<DDB5);
    PORTB |= (1U<<PB2);
    SPCR=(1U<<SPE)|(1U<<MSTR)|(1U<<SPR0);
    TWSR=0U; TWBR=72U; TWCR=(1U<<TWEN);
    ADMUX=(1U<<REFS0);
    ADCSRB=0U;
    ADCSRA=(1U<<ADEN)|(1U<<ADATE)|(1U<<ADSC)|
           (1U<<ADPS2)|(1U<<ADPS1)|(1U<<ADPS0);
    DIDR0 |= (1U<<ADC0D);
}

static void ensayo_prr(uint8_t indice)
{
    uint8_t mascara = mascaras_prr[indice];
    pantalla("PRR EN MEDICION", "D2 para salir");
    _delay_ms(450);
    backlight(0U);						 // NO enviar I2C durante la medicion
    iniciar_perifericos_prr();
    if (mascara & (1U<<PRADC)) ADCSRA &= (uint8_t)~(1U<<ADEN);
    PRR = mascara;
    while (!pulsado(B_SIGUIENTE)) { __asm__ __volatile__("nop"); }
    PRR = 0U;							//habilitar TWI antes de intentar mostrar mensajes
    soltar(B_SIGUIENTE);
    restaurar_menu();
    pantalla("PRR COMPLETADO", "Anotar consumo");
    _delay_ms(600);
}

//	----------- WDT como interrupcion ----------------------

static const uint8_t bits_wdt[4] = {
    0U, (1U<<WDP2), (1U<<WDP2)|(1U<<WDP1), (1U<<WDP3)|(1U<<WDP0)
};

static void ensayo_wdt_irq(uint8_t indice)
{
    pantalla("WDT INTERRUPCION", "D2 para salir");
    _delay_ms(450);
    backlight(0U);
    PRR = PRR_VALIDOS; 
    PORTB &= (uint8_t)~(1U<<PIN_PRUEBA);
    prueba_actual=EXP_WDT;
    despertar_o_salir=0U;
    evento_wdt=0U;
    configurar_wdt_irq(bits_wdt[indice]);
    for (;;) {
        cli();
        if (despertar_o_salir || pulsado(B_SIGUIENTE)) { sei(); break; }
        if (evento_wdt) {
            evento_wdt=0U;            // Reconfigura para nuevo periodo
            configurar_wdt_irq(bits_wdt[indice]);
        }
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        armar_int0();
        sei();
        sleep_cpu();
        sleep_disable();
    }
    cli();
    desarmar_int0();
    wdt_disable();
    sei();
    if (pulsado(B_SIGUIENTE)) soltar(B_SIGUIENTE);
    restaurar_menu();
    pantalla("WDT INT OK", "NO produce reset");
    _delay_ms(650);
}

//------------ WDT como reset --------------
static void ensayo_wdt_reset(void)
{
    pantalla("WDT RESET ~1s", "D2: simular fallo");
    _delay_ms(650);
    backlight(0U);
    PRR = PRR_VALIDOS;
    wdt_enable(WDTO_1S);
    for (;;) {
        if (pulsado(B_SIGUIENTE)) {            // Bloqueo deliberado, sin wdr, el WDT activa un reset
            for (;;) { __asm__ __volatile__("nop"); }
        }
        PORTB ^= (1U<<LED_RESET);
        _delay_ms(100);
        wdt_reset();
    }
}

//	------------ Prescaler del CLK ------- CLKPR ---------------
static const uint8_t codigo_clk[5] = {0U,1U,3U,6U,7U};
static const uint16_t divisores[5] = {1U,2U,8U,64U,128U};

static void ensayo_clkpr(uint8_t indice)
{
    uint16_t divisor = divisores[indice];		// Solo escribe ne la lcd a 16 MHz
    pantalla("CLKPR: MIDIENDO", "D2 para salir");
    _delay_ms(450);
    backlight(0U);
    ADCSRA = 0U;
    PRR = (uint8_t)(PRR_VALIDOS & ~(1U<<PRTIM1));
    DDRB |= (1U<<PIN_PRUEBA);
    TCCR1A=(1U<<COM1A0);			//  Le hace un Toggle a D9, para que la señal sea cuadrada
    TCCR1B=(1U<<WGM12);				// CTC
    OCR1A=(uint16_t)((F_CPU/(uint32_t)divisor/8UL/100UL)-1UL);
    TCNT1=0U;
    TCCR1B |= (1U<<CS11);
    ajustar_clkpr(codigo_clk[indice]);
    while (!pulsado(B_SIGUIENTE)) { __asm__ __volatile__("nop"); }
    ajustar_clkpr(0U);					// restaura la frecuancia antes de escribir en la pantalla (16 MHz)
    TCCR1A=0U; TCCR1B=0U;
    PRR=0U;
    soltar(B_SIGUIENTE);
    restaurar_menu();
    pantalla("CLKPR COMPLETO", "Anotar corriente");
    _delay_ms(600);
}

//--------------------- Menu------------------------
static const char *const categorias[CATEGORIAS] = {
    "A1/A3 - SLEEP", "A2 - PRR", "A4 - WDT INT",
    "A4 - WDT RESET", "A5 - CLKPR"
};
static const char *const n_sleep[N_SLEEP] = {
    "IDLE", "ADC NOISE RED.", "POWER-DOWN", "< VOLVER"
};
static const char *const n_prr[N_PRR] = {
    "REF: TODO ON", "APAGAR ADC", "APAGAR USART0", "APAGAR SPI",
    "APAGAR TIMER0", "APAGAR TIMER1", "APAGAR TIMER2", "APAGAR TWI",
    "APAGAR TODOS", "< VOLVER"
};
static const char *const n_wdt[N_WDT] = {
    "16 ms", "250 ms", "1 segundo", "8 segundos", "< VOLVER"
};
static const char *const n_clk[N_CLK] = {
    "16 MHz /1", "8 MHz /2", "2 MHz /8", "250 kHz /64",
    "125 kHz /128", "< VOLVER"
};

static uint8_t elegir_opcion(const char *titulo,
                             const char *const *nombres, uint8_t cantidad)
{
    uint8_t s=0U;
    pantalla(titulo, nombres[s]);
    for (;;) {
        uint8_t evento = boton_menu();
        if (evento == 1U) {
            s=(uint8_t)((s+1U)%cantidad);
            pantalla(titulo, nombres[s]);			// actualizar solo si hay un cambio
        } else if (evento == 2U) {
            return s;
        }
    }
}

static void aviso_reset(void)
{
    uint8_t i;
    uint8_t hubo_wdt = ((origen_reset & (1U<<WDRF)) != 0U);
    uint8_t veces = hubo_wdt ? 3U : 1U;
    if (hubo_wdt) pantalla("REINICIO POR WDT", "Recuperacion OK");
    else pantalla("LAB 5 - PARTE D", "Menu de ensayos");
    for (i=0U; i<veces; i++) {
        PORTB |= (1U<<LED_RESET); _delay_ms(120);
        PORTB &= (uint8_t)~(1U<<LED_RESET); _delay_ms(120);
    }
    _delay_ms(450);
}

int main(void)
{
    uint8_t categoria=0U;
    ajustar_clkpr(0U);
    preparar_int0();
    limpiar_perifericos();
    lcd_init();
    while (!lcd_presente) {    //	Si la LCD no responde, se avisa con el LED de D13 e intenta redetectar
        PORTB ^= (1U<<LED_RESET);
        _delay_ms(350);
        lcd_init();
    }
    aviso_reset();
    pantalla("LAB5 D - SIG/OK", categorias[categoria]);
    for (;;) {
        uint8_t ev, sel;
        ev = boton_menu();
        if (ev == 1U) {
            categoria=(uint8_t)((categoria+1U)%CATEGORIAS);
            pantalla("LAB5 D - SIG/OK", categorias[categoria]);
        } else if (ev == 2U) {
            if (categoria == 0U) {
                sel=elegir_opcion("MODO SLEEP", n_sleep, N_SLEEP);
                if (sel < (N_SLEEP-1U)) ensayo_sleep(sel);
            } else if (categoria == 1U) {
                sel=elegir_opcion("PERIF. PRR", n_prr, N_PRR);
                if (sel < (N_PRR-1U)) ensayo_prr(sel);
            } else if (categoria == 2U) {
                sel=elegir_opcion("WDT INTERRUPCION", n_wdt, N_WDT);
                if (sel < (N_WDT-1U)) ensayo_wdt_irq(sel);
            } else if (categoria == 3U) {
                ensayo_wdt_reset();
            } else if (categoria == 4U) {
                sel=elegir_opcion("DIVISOR CLKPR", n_clk, N_CLK);
                if (sel < (N_CLK-1U)) ensayo_clkpr(sel);
            }
            pantalla("LAB5 D - SIG/OK", categorias[categoria]);
        }
    }
}
