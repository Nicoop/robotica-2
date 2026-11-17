/*
 * hw.h — Utilidades de bajo nivel (registros directos, sin HAL)
 */
#ifndef HW_H
#define HW_H

#include "stm32l4xx.h"
#include "app_pins.h"

#define SIG_LEVEL(active)  (SIG_ACTIVE_LOW ? !(active) : !!(active))

static inline void hw_gpio_clk(GPIO_TypeDef *p)
{
    if (p == GPIOA)      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    else if (p == GPIOB) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    else if (p == GPIOC) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    else if (p == GPIOD) RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    else if (p == GPIOH) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
    (void)RCC->AHB2ENR;
}

static inline void hw_pin_write(GPIO_TypeDef *p, uint32_t pin, uint32_t level)
{
    if (level) p->BSRR = (1u << pin);
    else       p->BRR  = (1u << pin);
}

static inline void hw_gpio_out(GPIO_TypeDef *p, uint32_t pin, uint32_t open_drain, uint32_t level)
{
    hw_gpio_clk(p);
    hw_pin_write(p, pin, level);
    p->OTYPER  = (p->OTYPER & ~(1u << pin)) | ((open_drain ? 1u : 0u) << pin);
    p->OSPEEDR = (p->OSPEEDR & ~(3u << (2u * pin))) | (2u << (2u * pin));
    p->PUPDR  &= ~(3u << (2u * pin));
    p->MODER   = (p->MODER & ~(3u << (2u * pin))) | (1u << (2u * pin));
}

static inline void hw_gpio_af(GPIO_TypeDef *p, uint32_t pin, uint32_t af, uint32_t open_drain, uint32_t pullup)
{
    hw_gpio_clk(p);
    uint32_t r = pin >> 3, s = 4u * (pin & 7u);
    p->AFR[r]  = (p->AFR[r] & ~(0xFu << s)) | (af << s);
    p->OTYPER  = (p->OTYPER & ~(1u << pin)) | ((open_drain ? 1u : 0u) << pin);
    p->OSPEEDR = (p->OSPEEDR & ~(3u << (2u * pin))) | (2u << (2u * pin));
    p->PUPDR   = (p->PUPDR & ~(3u << (2u * pin))) | ((pullup ? 1u : 0u) << (2u * pin));
    p->MODER   = (p->MODER & ~(3u << (2u * pin))) | (2u << (2u * pin));
}

static inline uint32_t hw_irq_save(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}

static inline void hw_irq_restore(uint32_t pm)
{
    __set_PRIMASK(pm);
}

/* Frecuencia de PCLK1 a partir de SystemCoreClock (HCLK) */
static inline uint32_t hw_pclk1_hz(void)
{
    static const uint8_t sh[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    return SystemCoreClock >> sh[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
}

/* Reloj de los timers del APB1 (x2 si el prescaler del APB1 es distinto de 1) */
static inline uint32_t hw_tim_apb1_hz(void)
{
    uint32_t pp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    return (pp < 4u) ? hw_pclk1_hz() : 2u * hw_pclk1_hz();
}

/* Espera activa aproximada (no usar en el lazo de control salvo recuperación de bus) */
static inline void hw_delay_us(uint32_t us)
{
    volatile uint32_t n = (SystemCoreClock / 4000000u) * us;
    while (n--) { __NOP(); }
}

#endif
