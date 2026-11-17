/*
 * control.c — Corrección integral con histéresis (ver control.h)
 */
#include "control.h"
#include "app_config.h"
#include <math.h>

void corr_reset(corr_t *k)
{
    k->c = 0.0f;
    k->active = 0;
}

float corr_update(corr_t *k, float e, float dt)
{
    float ae = fabsf(e);
    if (!k->active && ae > CORR_DB_ON) k->active = 1;
    else if (k->active && ae < CORR_DB_OFF) k->active = 0;

    if (k->active) {
        float dc = CORR_KI * e * dt;
        float lim = CORR_RATE_MAX * dt;
        if (dc > lim) dc = lim;
        else if (dc < -lim) dc = -lim;
        k->c += dc;
        if (k->c > CORR_MAX) k->c = CORR_MAX;
        else if (k->c < -CORR_MAX) k->c = -CORR_MAX;
    }
    return k->c;
}
