/*
 * storage.h — Calibración y límites guardados en la última página de flash
 */
#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>

#define CAL_HAS_REF  0x1u
#define CAL_HAS_LIM  0x2u

typedef struct {
    uint32_t magic;
    uint32_t flags;
    float    q_cal[3];     /* ángulos articulares de la pose de referencia (grados) */
    uint32_t raw_cal[3];   /* lectura de cada encoder en esa pose */
    float    jmin[3];
    float    jmax[3];
    uint32_t crc;
    uint32_t pad;          /* tamaño múltiplo de 8 bytes */
} cal_t;

int storage_load(cal_t *c);   /* 1 si había datos válidos */
int storage_save(cal_t *c);   /* 0 si se grabó bien */

#endif
