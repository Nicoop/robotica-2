/*
 * storage.c — Escritura de la calibración en flash (STM32L476, banco 2, página 255)
 * La L476RG tiene 1 MB en dos bancos de 512 KB; el programa corre desde el
 * banco 1, así que borrar y grabar el banco 2 no detiene la ejecución.
 */
#include "storage.h"
#include "stm32l4xx.h"
#include <string.h>
#include <stddef.h>

#define CAL_ADDR   0x080FF800u
#define CAL_PAGE   255u
#define CAL_MAGIC  0x52334731u   /* "R3G1" */
#define FLASH_ERRS 0xC3FAu       /* OPERR..OPTVERR */

static uint32_t cal_crc(const cal_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    uint32_t h = 2166136261u;
    for (size_t k = 0; k < offsetof(cal_t, crc); k++) {
        h ^= p[k];
        h *= 16777619u;
    }
    return h;
}

int storage_load(cal_t *c)
{
    memcpy(c, (const void *)CAL_ADDR, sizeof *c);
    if (c->magic != CAL_MAGIC || c->crc != cal_crc(c)) {
        memset(c, 0, sizeof *c);
        return 0;
    }
    return 1;
}

static int wait_bsy(void)
{
    for (uint32_t t = 0; FLASH->SR & FLASH_SR_BSY; t++)
        if (t > 8000000u) return -1;
    return 0;
}

int storage_save(cal_t *c)
{
    c->magic = CAL_MAGIC;
    c->pad = 0;
    c->crc = cal_crc(c);

    if (wait_bsy()) return -1;
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    FLASH->SR = FLASH_ERRS | FLASH_SR_EOP;

    /* borrar la página */
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_PNB | FLASH_CR_PG | FLASH_CR_MER1 | FLASH_CR_MER2)) |
                FLASH_CR_PER | FLASH_CR_BKER | (CAL_PAGE << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    int r = wait_bsy();
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_BKER | FLASH_CR_PNB);
    if (!r && (FLASH->SR & FLASH_ERRS)) r = -2;

    /* programar de a doble palabra (64 bits) */
    if (!r) {
        const uint32_t *src = (const uint32_t *)c;
        volatile uint32_t *dst = (volatile uint32_t *)CAL_ADDR;
        FLASH->CR |= FLASH_CR_PG;
        for (uint32_t k = 0; k < sizeof(cal_t) / 4u; k += 2u) {
            dst[k] = src[k];
            dst[k + 1u] = src[k + 1u];
            if (wait_bsy()) { r = -3; break; }
            if (FLASH->SR & FLASH_ERRS) { r = -4; break; }
        }
        FLASH->CR &= ~FLASH_CR_PG;
    }
    FLASH->CR |= FLASH_CR_LOCK;

    /* invalidar la caché de datos para leer lo recién grabado */
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;

    if (!r) {
        cal_t chk;
        if (!storage_load(&chk) || memcmp(&chk, c, sizeof chk) != 0) r = -5;
    }
    return r;
}
