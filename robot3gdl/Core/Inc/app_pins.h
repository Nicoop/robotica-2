/*
 * app_pins.h — Asignación de pines (Nucleo-L476RG)
 *
 * Drivers TB6600 / DM542 en ÁNODO COMÚN:
 *   PUL+, DIR+, ENA+  →  5V de la Nucleo
 *   PUL-, DIR-, ENA-  →  pines de abajo (configurados open-drain, tolerantes a 5V)
 */
#ifndef APP_PINS_H
#define APP_PINS_H

#include "stm32l4xx.h"

/* ----- Eje 1 ----- */
#define STEP1_PORT  GPIOA
#define STEP1_PIN   10      /* D2 */
#define DIR1_PORT   GPIOB
#define DIR1_PIN    5       /* D4 */
/* ----- Eje 2 ----- */
#define STEP2_PORT  GPIOB
#define STEP2_PIN   4       /* D5 */
#define DIR2_PORT   GPIOB
#define DIR2_PIN    10      /* D6 */
/* ----- Eje 3 ----- */
#define STEP3_PORT  GPIOA
#define STEP3_PIN   8       /* D7 */
#define DIR3_PORT   GPIOA
#define DIR3_PIN    9       /* D8 */
/* ----- Enable común a los tres drivers ----- */
#define ENA_PORT    GPIOC
#define ENA_PIN     7       /* D9 */

/* LED verde de la placa (LD2) */
#define LED_PORT    GPIOA
#define LED_PIN     5

/* ----- Encoders (un bus I2C por encoder, AF4) ----- */
#define ENC1_I2C       I2C1
#define ENC1_SCL_PORT  GPIOB
#define ENC1_SCL_PIN   8     /* D15 */
#define ENC1_SDA_PORT  GPIOB
#define ENC1_SDA_PIN   9     /* D14 */

#define ENC2_I2C       I2C2
#define ENC2_SCL_PORT  GPIOB
#define ENC2_SCL_PIN   13    /* morpho CN10-30 */
#define ENC2_SDA_PORT  GPIOB
#define ENC2_SDA_PIN   14    /* morpho CN10-28 */

#define ENC3_I2C       I2C3
#define ENC3_SCL_PORT  GPIOC
#define ENC3_SCL_PIN   0     /* A5 */
#define ENC3_SDA_PORT  GPIOC
#define ENC3_SDA_PIN   1     /* A4 */

#define ENC_I2C_AF     4

/* ----- Polaridad de las señales hacia los drivers ----- */
#define SIG_OPEN_DRAIN       1  /* 1: open-drain (ánodo común a 5V). 0: push-pull (cátodo común) */
#define SIG_ACTIVE_LOW       1  /* con ánodo común la señal "activa" es nivel bajo */
#define ENA_ASSERT_RELEASES  1  /* TB6600/DM542: ENA activo = motor libre (sin torque) */

#endif
