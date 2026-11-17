/*
 * control.h — Corrección de posición a partir del encoder
 *
 * El perfil trapezoidal manda el movimiento (prealimentación). Esta parte mira
 * la diferencia entre la referencia y lo que mide el encoder, y si supera
 * CORR_DB_ON acumula una corrección (acción integral) que se suma a la
 * posición comandada, hasta que el error baja de CORR_DB_OFF. Así compensa
 * pasos perdidos y la deformación por carga sin oscilar alrededor del cero.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

typedef struct {
    float   c;        /* corrección acumulada (µpasos) */
    uint8_t active;
} corr_t;

void  corr_reset(corr_t *k);
float corr_update(corr_t *k, float e, float dt);   /* e = referencia - encoder (µpasos) */

#endif
