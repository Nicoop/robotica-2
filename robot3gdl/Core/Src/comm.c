/*
 * comm.c
 * USART2 por registros, recibe y manda por interrupcion con buffers circulares
 * PA2 = TX y PA3 = RX, estan conectados al st-link asi q no hay q cablear nada
 */
#include "comm.h"
#include "hw.h"

#define RXSZ 1024u   // buffer de recepcion, con el control de flujo nunca se llena
#define TXSZ 2048u   // buffer de transmision

static volatile uint8_t  rxb[RXSZ];
static volatile uint32_t rxh, rxt;   // h = donde escribe la interrupcion, t = donde lee el programa
static volatile uint32_t lines_in;   // lineas q llegaron (lo sube la interrupcion)
static uint32_t          lines_out;  // lineas q ya lei (lo sube el bucle principal)
static volatile uint8_t  rx_ovf;     // se lleno el buffer y se perdio algo
static volatile uint32_t rx_err;     // caracteres perdidos o rotos en el hardware (overrun / ruido)

static volatile uint8_t  txb[TXSZ];
static volatile uint32_t txh, txt;

void comm_init(uint32_t baud)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    (void)RCC->APB1ENR1;
    hw_gpio_af(GPIOA, 2, 7, 0, 0);   // AF7 = usart2
    hw_gpio_af(GPIOA, 3, 7, 0, 1);   // al RX le pongo pull up para q no lea basura si queda al aire

    USART2->CR1 = 0;            // lo apago primero porq el cubemx ya lo deja configurado con la hal
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->BRR = (hw_pclk1_hz() + baud / 2u) / baud;   // divisor redondeado: 80M/115200 = 694
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;   // tx, rx, interrupcion de rx

    rxh = rxt = 0;
    txh = txt = 0;
    lines_in = lines_out = 0;
    /* prioridad 1: por arriba del lazo de control (TIM7). el uart tiene lugar para UN solo caracter,
     * si la interrupcion tarda mas de 87 us en atenderlo se pisa con el siguiente (overrun) y la
     * linea llega rota -> err,cs. la interrupcion es cortita asi q no le saca tiempo a nadie */
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;

    // si hubo error (overrun, frame, paridad) lo borro, si no la interrupcion se queda trabada
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_PE)) {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_PECF;
        rx_err++;                       // lo cuento para avisar (sirve para saber de donde vienen los err,cs)
    }

    // llego un caracter, lo meto en el buffer
    if (isr & USART_ISR_RXNE) {
        uint8_t c = (uint8_t)USART2->RDR;
        uint32_t n = (rxh + 1u) % RXSZ;
        if (n != rxt) {
            rxb[rxh] = c;
            rxh = n;
            if (c == '\n') lines_in++;   // cuento las lineas completas
        } else {
            rx_ovf = 1;
        }
    }

    // el transmisor esta libre: mando el siguiente caracter o apago la interrupcion si no hay mas
    if ((USART2->CR1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE)) {
        if (txt != txh) {
            USART2->TDR = txb[txt];
            txt = (txt + 1u) % TXSZ;
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

uint32_t comm_lines(void)
{
    return lines_in - lines_out;
}

int comm_peek(void)
{
    if (rxt == rxh) return -1;
    return rxb[rxt];
}

int comm_read_line(char *buf, int max)
{
    int n = 0, too_long = 0;
    while (rxt != rxh) {
        char c = (char)rxb[rxt];
        rxt = (rxt + 1u) % RXSZ;
        if (c == '\n') break;
        if (c == '\r') continue;     // por si viene de windows con \r\n
        if (n < max - 1) buf[n++] = c;
        else too_long = 1;           // no entra, sigo sacando igual hasta el fin de linea
    }
    buf[n] = 0;
    lines_out++;
    return too_long ? -1 : n;
}

void comm_puts(const char *s)
{
    while (*s) {
        uint32_t n = (txh + 1u) % TXSZ;
        while (n == txt) { /* buffer lleno, espero a q la interrupcion vaya mandando */ }
        txb[txh] = (uint8_t)*s++;
        txh = n;
        USART2->CR1 |= USART_CR1_TXEIE;   // prendo la interrupcion para q empieze a mandar
    }
}

uint32_t comm_rx_errors(void)
{
    return rx_err;
}

int comm_overflow(void)
{
    if (!rx_ovf) return 0;
    uint32_t pm = hw_irq_save();
    rx_ovf = 0;
    rxt = rxh;                  // tiro todo lo q habia, ya esta corrupto igual
    lines_out = lines_in;
    hw_irq_restore(pm);
    return 1;
}
