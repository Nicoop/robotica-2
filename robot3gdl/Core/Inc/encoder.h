/*
 * encoder.h — Lectura de los encoders magnéticos (AS5600 / MT6701) por I2C
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "app_config.h"

#if ENC_MODEL == ENC_AS5600
#define ENC_COUNTS   4096
#else
#define ENC_COUNTS   16384
#endif

/* bits del registro STATUS del AS5600 */
#define ENC_ST_MD    0x20   /* imán detectado */
#define ENC_ST_ML    0x10   /* imán demasiado débil / lejos */
#define ENC_ST_MH    0x08   /* imán demasiado fuerte / cerca */

int      enc_init(void);              /* devuelve máscara de encoders que respondieron */
void     enc_start_all(void);         /* lanza una lectura en los tres buses */
/* 1: lectura nueva, 0: todavía no terminó, -1: error */
int      enc_collect(int i, uint16_t *raw, uint8_t *status);
uint32_t enc_errors(int i);

#endif
