/*
 * encoder.c
 * encoders as5600 (o mt6701) en I2C1, I2C2 e I2C3
 *
 * como cada encoder esta en su propio bus se pueden leer los 3 al mismo tiempo.
 * cada 1 ms el lazo de control arranca una lectura por interrupcion (primero escribe
 * q registro quiere leer y despues lee con repeated start) y en el ms siguiente
 * agarra el resultado. todo con registros, nada de hal
 */
#include "encoder.h"
#include "app_pins.h"
#include "hw.h"

#if ENC_MODEL == ENC_AS5600
#define ENC_ADDR   0x36u      // direccion fija del as5600, por eso va uno por bus
#define ENC_REG    0x0Bu      // arranco en STATUS y el chip sigue solo con RAW ANGLE alto y bajo
#define ENC_NB     3u         // leo 3 bytes
#else
#define ENC_ADDR   0x06u
#define ENC_REG    0x03u      // angulo [13:6] y despues angulo [5:0] corrido 2 lugares
#define ENC_NB     2u
#endif

// todas las banderas de "borrar" del i2c juntas
#define I2C_ICR_ALL  (I2C_ICR_ADDRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | \
                      I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_PECCF | I2C_ICR_TIMOUTCF | I2C_ICR_ALERTCF)
// todas las interrupciones q uso
#define I2C_IE_ALL   (I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | \
                      I2C_CR1_TCIE | I2C_CR1_ERRIE)

// estados de cada bus: libre, escribiendo el registro, leyendo, termino bien, termino mal
enum { B_IDLE = 0, B_WRITE, B_READ, B_DONE, B_ERR };

typedef struct {
    I2C_TypeDef  *i2c;
    GPIO_TypeDef *scl_port;
    uint32_t      scl_pin;
    GPIO_TypeDef *sda_port;
    uint32_t      sda_pin;
    volatile uint8_t state;
    volatile uint8_t idx;       // cuantos bytes llevo leidos
    volatile uint8_t nack;      // el sensor no contesto
    volatile uint8_t buf[4];    // los bytes q llegan
    uint8_t  stuck;             // cuantas veces seguidas la lectura no termino a tiempo
    uint8_t  fails;             // errores seguidos (para saber cuando destrabar el bus)
    uint32_t errors;            // errores totales, se ven con el comando W
} bus_t;

static const uint8_t enc_present[N_AXES] = ENC_PRESENT_INIT;   // q ejes tienen encoder

static bus_t bus[N_AXES] = {
    { ENC1_I2C, ENC1_SCL_PORT, ENC1_SCL_PIN, ENC1_SDA_PORT, ENC1_SDA_PIN, 0, 0, 0, {0}, 0, 0, 0 },
    { ENC2_I2C, ENC2_SCL_PORT, ENC2_SCL_PIN, ENC2_SDA_PORT, ENC2_SDA_PIN, 0, 0, 0, {0}, 0, 0, 0 },
    { ENC3_I2C, ENC3_SCL_PORT, ENC3_SCL_PIN, ENC3_SDA_PORT, ENC3_SDA_PIN, 0, 0, 0, {0}, 0, 0, 0 },
};

// apagar y prender el periferico lo resetea entero (el manual pide unos ciclos con PE en 0)
static void periph_reset(bus_t *b)
{
    I2C_TypeDef *I = b->i2c;
    I->CR1 &= ~I2C_CR1_PE;
    (void)I->CR1; (void)I->CR1; (void)I->CR1;
    I->CR1 |= I2C_CR1_PE;
    b->state = B_IDLE;
}

static void pins_af(bus_t *b)
{
    hw_gpio_af(b->scl_port, b->scl_pin, ENC_I2C_AF, 1, 1);   // open drain con pull up interno
    hw_gpio_af(b->sda_port, b->sda_pin, ENC_I2C_AF, 1, 1);
}

/* destrabar el bus: aveces un sensor se queda colgado con SDA en 0 (por ejemplo si
 * hubo ruido en medio de una lectura). la solucion clasica es mandar 9 pulsos de
 * clock a mano y despues un STOP, asi el sensor termina lo q estaba haciendo y suelta */
static void bus_recover(bus_t *b)
{
    b->i2c->CR1 &= ~I2C_CR1_PE;
    hw_gpio_out(b->scl_port, b->scl_pin, 1, 1);
    hw_gpio_out(b->sda_port, b->sda_pin, 1, 1);
    hw_delay_us(5);
    for (int k = 0; k < 9; k++) {
        hw_pin_write(b->scl_port, b->scl_pin, 0); hw_delay_us(5);
        hw_pin_write(b->scl_port, b->scl_pin, 1); hw_delay_us(5);
    }
    // condicion de STOP: SDA sube mientras SCL esta en 1
    hw_pin_write(b->sda_port, b->sda_pin, 0); hw_delay_us(5);
    hw_pin_write(b->scl_port, b->scl_pin, 1); hw_delay_us(5);
    hw_pin_write(b->sda_port, b->sda_pin, 1); hw_delay_us(5);
    pins_af(b);
    b->i2c->ICR = I2C_ICR_ALL;
    b->i2c->CR1 |= I2C_CR1_PE;
    b->state = B_IDLE;
}

/* ---------- lecturas y escrituras esperando (bloqueantes), solo se usan al arrancar ---------- */

// espera a q aparezca una bandera, o error, o se canse de esperar
static int wait_flag(I2C_TypeDef *I, uint32_t flag)
{
    for (uint32_t t = 0; t < 200000u; t++) {
        uint32_t isr = I->ISR;
        if (isr & flag) return 0;
        if (isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) return -1;
    }
    return -2;
}

// escribe n bytes a partir del registro reg
static int poll_write(bus_t *b, uint8_t reg, const uint8_t *d, uint32_t n)
{
    I2C_TypeDef *I = b->i2c;
    I->ICR = I2C_ICR_ALL;
    // n+1 bytes porq primero va el numero de registro, AUTOEND manda el STOP solo al final
    I->CR2 = (ENC_ADDR << 1) | ((n + 1u) << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
    int r = wait_flag(I, I2C_ISR_TXIS);
    if (!r) I->TXDR = reg;
    for (uint32_t k = 0; !r && k < n; k++) {
        r = wait_flag(I, I2C_ISR_TXIS);
        if (!r) I->TXDR = d[k];
    }
    if (!r) r = wait_flag(I, I2C_ISR_STOPF);
    if (r) { wait_flag(I, I2C_ISR_STOPF); }
    I->ICR = I2C_ICR_ALL;
    return r;
}

// lee n bytes a partir del registro reg
static int poll_read(bus_t *b, uint8_t reg, uint8_t *d, uint32_t n)
{
    I2C_TypeDef *I = b->i2c;
    I->ICR = I2C_ICR_ALL;
    I->CR2 = (ENC_ADDR << 1) | (1u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;   // escribo 1 byte (el registro), sin STOP
    int r = wait_flag(I, I2C_ISR_TXIS);
    if (!r) { I->TXDR = reg; r = wait_flag(I, I2C_ISR_TC); }
    if (!r) {
        // repeated start pero ahora en lectura
        I->CR2 = (ENC_ADDR << 1) | (n << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN |
                 I2C_CR2_AUTOEND | I2C_CR2_START;
        for (uint32_t k = 0; !r && k < n; k++) {
            r = wait_flag(I, I2C_ISR_RXNE);
            if (!r) d[k] = (uint8_t)I->RXDR;
        }
    }
    wait_flag(I, I2C_ISR_STOPF);
    I->ICR = I2C_ICR_ALL;
    return r;
}

/* ---------------- interrupciones ---------------- */

/* la misma rutina sirve para los 3 buses. va siguiendo la lectura paso a paso:
 * TXIS -> mando el numero de registro
 * TC   -> ya se mando, arranco la lectura con repeated start
 * RXNE -> llego un byte, lo guardo
 * STOPF-> termino, me fijo si salio todo bien */
static void ev_handler(bus_t *b)
{
    I2C_TypeDef *I = b->i2c;
    uint32_t isr = I->ISR;

    if (isr & I2C_ISR_NACKF) { I->ICR = I2C_ICR_NACKCF; b->nack = 1; }   // el sensor no contesto (el stop sale solo)
    if (isr & I2C_ISR_RXNE) {
        uint8_t v = (uint8_t)I->RXDR;
        if (b->idx < ENC_NB) b->buf[b->idx++] = v;
    }
    if (isr & I2C_ISR_TXIS) I->TXDR = ENC_REG;
    if (isr & I2C_ISR_TC) {
        I->CR2 = (ENC_ADDR << 1) | (ENC_NB << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN |
                 I2C_CR2_AUTOEND | I2C_CR2_START;
        b->state = B_READ;
    }
    if (isr & I2C_ISR_STOPF) {
        I->ICR = I2C_ICR_STOPCF;
        b->state = (b->state == B_READ && b->idx == ENC_NB && !b->nack) ? B_DONE : B_ERR;
    }
}

// errores de bus (ruido, arbitraje perdido, etc): borro todo y reseteo el periferico
static void er_handler(bus_t *b)
{
    b->i2c->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_TIMOUTCF | I2C_ICR_PECCF;
    periph_reset(b);
    b->state = B_ERR;
}

void I2C1_EV_IRQHandler(void) { ev_handler(&bus[0]); }
void I2C1_ER_IRQHandler(void) { er_handler(&bus[0]); }
void I2C2_EV_IRQHandler(void) { ev_handler(&bus[1]); }
void I2C2_ER_IRQHandler(void) { er_handler(&bus[1]); }
void I2C3_EV_IRQHandler(void) { ev_handler(&bus[2]); }
void I2C3_ER_IRQHandler(void) { er_handler(&bus[2]); }

/* ---------------- lo q usa el resto del programa ---------------- */

int enc_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN | RCC_APB1ENR1_I2C2EN | RCC_APB1ENR1_I2C3EN;
    (void)RCC->APB1ENR1;
    // clock de los i2c = PCLK1 (80 MHz), el TIMINGR esta calculado para eso
    RCC->CCIPR &= ~(RCC_CCIPR_I2C1SEL | RCC_CCIPR_I2C2SEL | RCC_CCIPR_I2C3SEL);

    static const IRQn_Type ev_irq[N_AXES] = { I2C1_EV_IRQn, I2C2_EV_IRQn, I2C3_EV_IRQn };
    static const IRQn_Type er_irq[N_AXES] = { I2C1_ER_IRQn, I2C2_ER_IRQn, I2C3_ER_IRQn };

    hw_delay_us(20000);   // le doy tiempo al sensor para q arranque

    int mask = 0;
    for (int i = 0; i < N_AXES; i++) {
        if (!enc_present[i]) continue;   // eje sin encoder, ni toco ese bus
        bus_t *b = &bus[i];
        I2C_TypeDef *I = b->i2c;
        I->CR1 = 0;
        bus_recover(b);              // por las dudas destrabo el bus, y de paso deja los pines listos
        I->CR1 = 0;
        I->TIMINGR = I2C_TIMINGR_VAL;
        I->CR1 = I2C_CR1_PE;

#if ENC_MODEL == ENC_AS5600
        /* registro CONF: filtro lento 8x (atrasa mas o menos 1,1 ms) y filtro rapido
         * con umbral de 7 cuentas, sin histeresis. haci responde rapido pero sin tanto ruido */
        uint8_t conf[2] = { (uint8_t)((0x2u << 2) | 0x1u), 0x00 };
        poll_write(b, 0x07, conf, 2);
#endif
        uint8_t tmp[4];
        if (poll_read(b, ENC_REG, tmp, ENC_NB) == 0) mask |= (1 << i);   // contesto, anda

        // de aca en adelante todo por interrupcion
        I->CR1 = I2C_CR1_PE | I2C_IE_ALL;
        b->state = B_IDLE;
        NVIC_SetPriority(ev_irq[i], 1);
        NVIC_SetPriority(er_irq[i], 1);
        NVIC_ClearPendingIRQ(ev_irq[i]);
        NVIC_ClearPendingIRQ(er_irq[i]);
        NVIC_EnableIRQ(ev_irq[i]);
        NVIC_EnableIRQ(er_irq[i]);
    }
    return mask;
}

static void start_one(bus_t *b)
{
    I2C_TypeDef *I = b->i2c;
    if (b->state == B_WRITE || b->state == B_READ) {   // la lectura anterior todavia no termino
        if (++b->stuck >= 3u) { b->stuck = 0; b->errors++; bus_recover(b); }   // 3 veces seguidas = colgado
        return;
    }
    if (b->state == B_DONE || b->state == B_ERR) return; // hay un resultado q nadie leyo, no lo piso
    b->stuck = 0;
    if (I->ISR & I2C_ISR_BUSY) {                         // el bus esta ocupado y no deveria
        if (++b->fails >= 10u) { b->fails = 0; bus_recover(b); }
        return;
    }
    b->idx = 0;
    b->nack = 0;
    b->state = B_WRITE;
    I->ICR = I2C_ICR_ALL;
    I->CR2 = (ENC_ADDR << 1) | (1u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;   // arranca, lo demas lo hace la interrupcion
}

void enc_start_all(void)
{
    for (int i = 0; i < N_AXES; i++)
        if (enc_present[i]) start_one(&bus[i]);
}

int enc_collect(int i, uint16_t *raw, uint8_t *status)
{
    if (!enc_present[i]) return 0;       // sin encoder nunca hay lectura
    bus_t *b = &bus[i];
    if (b->state == B_DONE) {
#if ENC_MODEL == ENC_AS5600
        *status = b->buf[0];
        *raw = (uint16_t)(((b->buf[1] & 0x0Fu) << 8) | b->buf[2]);   // 12 bits: 4 del primero y 8 del segundo
#else
        *status = ENC_ST_MD;   // el mt6701 no tiene status, asumo q el iman esta
        *raw = (uint16_t)(((uint16_t)b->buf[0] << 6) | (b->buf[1] >> 2));
#endif
        b->state = B_IDLE;
        b->fails = 0;
        return 1;
    }
    if (b->state == B_ERR) {
        b->state = B_IDLE;
        b->errors++;
        if (++b->fails >= 10u) { b->fails = 0; bus_recover(b); }   // muchos errores seguidos, destrabo
        return -1;
    }
    return 0;
}

uint32_t enc_errors(int i)
{
    return bus[i].errors;
}
