/*
 * planner.h — Planificador con look-ahead y perfiles trapezoidales
 *
 * Los puntos se encolan como segmentos rectos en espacio articular. Las tres
 * articulaciones recorren cada segmento sincronizadas (empiezan y terminan
 * juntas). La velocidad sobre la trayectoria sigue un perfil trapezoidal y los
 * segmentos se empalman sin frenar en cada punto.
 *
 * Este módulo no toca hardware: se puede compilar y probar en la PC.
 */
#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>
#include "app_config.h"

/* ----- Lado productor (bucle principal) ----- */
void     planner_init(void);
uint32_t planner_free(void);                 /* lugares libres en la cola */
int      planner_empty(void);                /* 1 si no hay movimiento pendiente */
/* Encola un punto destino (grados). feed = velocidad sobre la trayectoria (°/s),
 * accel = aceleración sobre la trayectoria (°/s²).
 * Devuelve 1 encolado, 0 ignorado (demasiado cerca), -1 cola llena, -2 frenando. */
int      planner_add(const float q[N_AXES], float feed, float accel);
void     planner_get_end(float q[N_AXES]);   /* posición final de lo encolado */
/* Estas dos deben llamarse con interrupciones bloqueadas */
void     planner_reset(const float q[N_AXES]); /* vacía la cola y fija la posición */
void     planner_flush(void);                  /* vacía la cola en la posición actual */

/* ----- Lado ejecutor (tick de control) ----- */
void  exec_tick(float dt, float q_out[N_AXES]);
void  exec_get(float q_out[N_AXES]);
float exec_speed(void);
void  exec_hold(void);        /* frena hasta detenerse siguiendo la trayectoria */
int   exec_hold_done(void);

#endif
