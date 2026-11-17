/*
 * stepgen.c — Generador de pulsos STEP/DIR por interrupción de TIM6
 *
 * TIM6 interrumpe a STEP_TICK_HZ (50 kHz). En cada tick se apagan los pulsos
 * del tick anterior y cada eje decide si emite un pulso nuevo (stepgen_core.h).
 * El lazo de control (1 kHz) fija con sg_set_target() a qué posición tiene que
 * llegar cada eje al final del próximo milisegundo; acá se reparte ese avance
 * uniformemente en los 50 ticks.
 */
#include "stepgen.h"
#include "app_config.h"
#include "app_pins.h"
#include "hw.h"

static const uint8_t dir_inv[N_AXES] = DIR_INVERT_INIT;
static GPIO_TypeDef * const step_port[N_AXES] = { STEP1_PORT, STEP2_PORT, STEP3_PORT };
static const uint32_t step_pin[N_AXES]        = { STEP1_PIN,  STEP2_PIN,  STEP3_PIN  };
static GPIO_TypeDef * const dir_port[N_AXES]  = { DIR1_PORT,  DIR2_PORT,  DIR3_PORT  };
static const uint32_t dir_pin[N_AXES]         = { DIR1_PIN,   DIR2_PIN,   DIR3_PIN   };

#define SG_TICKS_PER_CTRL  (STEP_TICK_HZ / CTRL_HZ)

#if SIG_ACTIVE_LOW
#define SG_ASSERT(i)   (step_port[(i)]->BRR  = (1u << step_pin[(i)]))
#define SG_RELEASE(i)  (step_port[(i)]->BSRR = (1u << step_pin[(i)]))
#else
#define SG_ASSERT(i)   (step_port[(i)]->BSRR = (1u << step_pin[(i)]))
#define SG_RELEASE(i)  (step_port[(i)]->BRR  = (1u << step_pin[(i)]))
#endif

static inline void sg_write_dir(int i, int d)
{
    uint32_t active = ((d > 0) ? 1u : 0u) ^ dir_inv[i];
    hw_pin_write(dir_port[i], dir_pin[i], SIG_LEVEL(active));
}
#define SG_SET_DIR(i, d) sg_write_dir((i), (d))

#include "stepgen_core.h"

static sg_axis_t ax[N_AXES];
static volatile uint8_t sg_run;

void sg_init(void)
{
    for (int i = 0; i < N_AXES; i++) {
        hw_gpio_out(step_port[i], step_pin[i], SIG_OPEN_DRAIN, SIG_LEVEL(0));
        hw_gpio_out(dir_port[i], dir_pin[i], SIG_OPEN_DRAIN, SIG_LEVEL(0));
        ax[i].steps = 0;
        ax[i].frac = 0;
        ax[i].rate = 0;
        ax[i].pend = 0;
        ax[i].busy = 0;
        sg_write_dir(i, 1);
        ax[i].dir = 1;
    }
    sg_run = 0;

    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    (void)RCC->APB1ENR1;
    TIM6->CR1 = 0;
    TIM6->PSC = 0;
    TIM6->ARR = hw_tim_apb1_hz() / STEP_TICK_HZ - 1u;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 0);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR = 0;
    SG_RELEASE(0);
    SG_RELEASE(1);
    SG_RELEASE(2);
    if (!sg_run) return;
    sg_axis_tick(&ax[0], 0);
    sg_axis_tick(&ax[1], 1);
    sg_axis_tick(&ax[2], 2);
}

void sg_enable(uint8_t on)
{
    if (!on) {
        sg_run = 0;
        for (int i = 0; i < N_AXES; i++) {
            ax[i].rate = 0;
            ax[i].pend = 0;
            ax[i].busy = 0;
        }
    } else {
        sg_run = 1;
    }
}

float sg_get_pos(int i)
{
    uint32_t pm = hw_irq_save();
    int32_t s = ax[i].steps + ax[i].pend;
    int32_t f = ax[i].frac;
    hw_irq_restore(pm);
    return (float)s + (float)f * (1.0f / (float)SG_ONE);
}

void sg_set_target(int i, float p)
{
    float r = (p - sg_get_pos(i)) * ((float)SG_ONE / (float)SG_TICKS_PER_CTRL);
    if (r > (float)SG_RATE_MAX) r = (float)SG_RATE_MAX;
    else if (r < -(float)SG_RATE_MAX) r = -(float)SG_RATE_MAX;
    ax[i].rate = (int32_t)r;
}

void sg_stop_axis(int i)
{
    ax[i].rate = 0;
}

void sg_set_pos(int i, int32_t steps)
{
    uint32_t pm = hw_irq_save();
    ax[i].steps = steps;
    ax[i].frac = 0;
    ax[i].rate = 0;
    ax[i].pend = 0;
    hw_irq_restore(pm);
}
