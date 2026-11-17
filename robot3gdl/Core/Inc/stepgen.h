/*
 * stepgen.h — Generador de pulsos STEP/DIR para los tres ejes (TIM6, 50 kHz)
 */
#ifndef STEPGEN_H
#define STEPGEN_H

#include <stdint.h>

void  sg_init(void);
void  sg_enable(uint8_t on);              /* 0: no emite pulsos */
void  sg_set_target(int i, float p);      /* posición (µpasos) a alcanzar en el próximo ms */
void  sg_stop_axis(int i);                /* velocidad 0 */
void  sg_set_pos(int i, int32_t steps);   /* fija el contador de posición */
float sg_get_pos(int i);                  /* posición comandada actual (µpasos) */

#endif
