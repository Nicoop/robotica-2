/*
 * stepgen_core.h — Lógica del generador de pasos por eje (DDA)
 *
 * Se incluye desde stepgen.c y desde la simulación de PC. Antes de incluir
 * hay que definir SG_ASSERT(i) (activar pulso STEP del eje i) y
 * SG_SET_DIR(i, d) (poner DIR del eje i en d = +1 / -1).
 *
 * Cada tick (20 µs a 50 kHz) se suma 'rate' a un acumulador de fracción de
 * µpaso. Cuando la posición comandada supera medio µpaso se emite un pulso.
 * Reglas de temporización para el driver:
 *  - el pulso dura un tick (20 µs) y después hay al menos un tick sin pulso,
 *  - DIR cambia sólo cuando no hay pulso activo y el pulso siguiente sale un
 *    tick después (≥ 20 µs de setup de DIR).
 */
#ifndef STEPGEN_CORE_H
#define STEPGEN_CORE_H

#include <stdint.h>

typedef struct {
    volatile int32_t steps;   /* µpasos emitidos */
    volatile int32_t frac;    /* fracción de µpaso en Q16 (aprox. [-0.5, 0.5)) */
    volatile int32_t rate;    /* µpasos por tick en Q16, |rate| <= SG_RATE_MAX */
    volatile int8_t  dir;     /* estado actual del pin DIR: +1 / -1 */
    volatile int8_t  pend;    /* pulso pendiente tras cambio de DIR */
    volatile int8_t  busy;    /* tick de reposo tras un pulso */
} sg_axis_t;

#define SG_ONE       65536
#define SG_HALF      32768
#define SG_RATE_MAX  29491     /* 0,45 µpaso/tick → 22,5 kHz a 50 kHz */

static inline void sg_axis_tick(sg_axis_t *a, int i)
{
    int32_t f = a->frac + a->rate;

    if (a->pend) {                       /* DIR ya cambió en el tick anterior */
        SG_ASSERT(i);
        a->steps += a->pend;
        a->pend = 0;
        a->busy = 1;
        a->frac = f;
        return;
    }
    if (a->busy) {                       /* tick de reposo después del pulso */
        a->busy = 0;
        a->frac = f;
        return;
    }
    if (f >= SG_HALF) {
        f -= SG_ONE;
        if (a->dir != 1) { SG_SET_DIR(i, 1); a->dir = 1; a->pend = 1; }
        else             { SG_ASSERT(i); a->steps += 1; a->busy = 1; }
    } else if (f < -SG_HALF) {
        f += SG_ONE;
        if (a->dir != -1) { SG_SET_DIR(i, -1); a->dir = -1; a->pend = -1; }
        else              { SG_ASSERT(i); a->steps -= 1; a->busy = 1; }
    }
    a->frac = f;
}

#endif
