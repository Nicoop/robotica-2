/*
 * planner.c — Planificador con look-ahead y ejecutor de perfiles trapezoidales
 *
 * Cola circular de bloques (segmentos). El bucle principal agrega bloques en
 * 'head'; el tick de control (ISR) ejecuta el bloque 'tail'.
 *
 * Planificación (al agregar cada punto):
 *  - velocidad nominal y aceleración del bloque limitadas por los máximos de
 *    cada articulación (proyectados sobre la dirección del segmento),
 *  - velocidad máxima en el vértice entre bloques (salto de velocidad y
 *    curvatura),
 *  - pasada hacia atrás: que siempre se pueda frenar a 0 al final de la cola,
 *  - pasada hacia adelante: que las velocidades de entrada sean alcanzables.
 *
 * Ejecución (cada 1 ms): perfil trapezoidal "en línea". En cada tick se
 * acelera hacia la velocidad nominal pero nunca por encima de la velocidad que
 * todavía permite frenar hasta la velocidad de salida del bloque. Si llegan
 * puntos nuevos, la velocidad de salida sube y el movimiento sigue sin cortes.
 */
#include "planner.h"
#include <math.h>

#if defined(__ARM_ARCH)
#define PL_BARRIER() __asm volatile ("dmb" ::: "memory")
#else
#define PL_BARRIER() __sync_synchronize()
#endif

#if (PLANNER_SIZE & (PLANNER_SIZE - 1u)) != 0
#error "PLANNER_SIZE debe ser potencia de 2"
#endif
#define PMASK (PLANNER_SIZE - 1u)

typedef struct {
    float q0[N_AXES];       /* punto inicial (grados) */
    float u[N_AXES];        /* dirección unitaria en espacio articular */
    float L;                /* longitud (grados, norma euclídea) */
    float v_nom;            /* velocidad nominal sobre el segmento */
    float a;                /* aceleración sobre el segmento */
    float v_max_entry;      /* límite de velocidad en el vértice de entrada */
    float v_rev;            /* resultado de la pasada hacia atrás */
    volatile float v_entry; /* velocidad de entrada planificada */
} block_t;

static block_t B[PLANNER_SIZE];
static volatile uint32_t head;   /* próximo lugar libre (escribe el bucle principal) */
static volatile uint32_t tail;   /* bloque en ejecución (escribe el ejecutor) */
static float end_pos[N_AXES];

static const float vmax[N_AXES] = JOINT_VMAX_INIT;
static const float amax[N_AXES] = JOINT_AMAX_INIT;
static const float jerk[N_AXES] = JOINT_JERK_INIT;

/* estado del ejecutor */
static float ex_s;                /* avance dentro del bloque (grados) */
static float ex_v;                /* velocidad actual sobre la trayectoria */
static float ex_q[N_AXES];        /* referencia actual */
static volatile uint8_t ex_hold;
static volatile uint8_t ex_hold_done;

/* ------------------------------------------------------------------ */

void planner_init(void)
{
    float z[N_AXES] = {0};
    planner_reset(z);
}

uint32_t planner_free(void)
{
    uint32_t used = (head - tail) & PMASK;
    return PLANNER_SIZE - 1u - used;
}

int planner_empty(void)
{
    return head == tail;
}

void planner_get_end(float q[N_AXES])
{
    for (int i = 0; i < N_AXES; i++) q[i] = end_pos[i];
}

void planner_reset(const float q[N_AXES])
{
    head = 0;
    tail = 0;
    for (int i = 0; i < N_AXES; i++) {
        end_pos[i] = q[i];
        ex_q[i] = q[i];
    }
    ex_s = 0.0f;
    ex_v = 0.0f;
    ex_hold = 0;
    ex_hold_done = 0;
}

void planner_flush(void)
{
    float q[N_AXES];
    for (int i = 0; i < N_AXES; i++) q[i] = ex_q[i];
    planner_reset(q);
}

/* Recalcula las velocidades de entrada de los bloques en cola */
static void recalc(void)
{
    uint32_t t = tail;
    uint32_t h = head;
    if (h == t) return;
    uint32_t first = (t + 1u) & PMASK;     /* primer bloque modificable */
    if (first == h) return;                /* sólo está el que se ejecuta */
    uint32_t last = (h - 1u) & PMASK;

    /* Pasada hacia atrás: el último bloque termina en 0 */
    uint32_t k = last, start = first;
    float next = 0.0f;
    for (;;) {
        block_t *b = &B[k];
        float ve = sqrtf(next * next + 2.0f * b->a * b->L);
        if (ve > b->v_max_entry) ve = b->v_max_entry;
        if (k != last && ve == b->v_rev) { start = k; break; }  /* nada cambia más atrás */
        b->v_rev = ve;
        next = ve;
        if (k == first) { start = first; break; }
        k = (k - 1u) & PMASK;
    }

    /* Pasada hacia adelante */
    k = start;
    if (k == first) {
        B[k].v_entry = B[k].v_rev;
    } else {
        block_t *p = &B[(k - 1u) & PMASK];
        float vf = sqrtf(p->v_entry * p->v_entry + 2.0f * p->a * p->L);
        B[k].v_entry = (vf < B[k].v_rev) ? vf : B[k].v_rev;
    }
    while (k != last) {
        uint32_t n = (k + 1u) & PMASK;
        block_t *p = &B[k];
        float vf = sqrtf(p->v_entry * p->v_entry + 2.0f * p->a * p->L);
        B[n].v_entry = (vf < B[n].v_rev) ? vf : B[n].v_rev;
        k = n;
    }
}

int planner_add(const float q[N_AXES], float feed, float accel)
{
    if (ex_hold) return -2;
    if (planner_free() == 0u) return -1;

    float d[N_AXES], L2 = 0.0f;
    for (int i = 0; i < N_AXES; i++) {
        d[i] = q[i] - end_pos[i];
        L2 += d[i] * d[i];
    }
    float L = sqrtf(L2);
    if (L < MIN_SEG_LEN) return 0;

    uint32_t h = head;
    block_t *nb = &B[h];
    float v = feed, a = accel;
    for (int i = 0; i < N_AXES; i++) {
        nb->q0[i] = end_pos[i];
        nb->u[i] = d[i] / L;
        float ad = fabsf(d[i]);
        if (ad > 1e-9f) {
            float r = L / ad;
            if (vmax[i] * r < v) v = vmax[i] * r;
            if (amax[i] * r < a) a = amax[i] * r;
        }
    }
    nb->L = L;
    nb->v_nom = v;
    nb->a = a;
    nb->v_entry = 0.0f;
    nb->v_rev = -1.0f;

    /* Velocidad máxima en el vértice con el bloque anterior */
    float vj = 0.0f;
    if (h != tail) {
        block_t *pb = &B[(h - 1u) & PMASK];
        vj = (pb->v_nom < v) ? pb->v_nom : v;
        float Lm = 0.5f * (pb->L + L);
        for (int i = 0; i < N_AXES; i++) {
            float du = fabsf(nb->u[i] - pb->u[i]);
            if (du > 1e-6f) {
                float l1 = jerk[i] / du;                 /* salto instantáneo de velocidad */
                if (l1 < vj) vj = l1;
                float l2 = sqrtf(amax[i] * Lm / du);     /* curvatura: v²·Δu/L ≤ amax */
                if (l2 < vj) vj = l2;
            }
        }
    }
    nb->v_max_entry = vj;

    PL_BARRIER();
    head = (h + 1u) & PMASK;
    for (int i = 0; i < N_AXES; i++) end_pos[i] = q[i];

    recalc();
    return 1;
}

/* ------------------------------------------------------------------ */
/*                       Ejecutor (tick de control)                    */
/* ------------------------------------------------------------------ */

static void finish_last(block_t *b, uint32_t next_index)
{
    for (int i = 0; i < N_AXES; i++) ex_q[i] = b->q0[i] + b->u[i] * b->L;
    ex_s = 0.0f;                    /* el próximo bloque arranca desde su inicio */
    ex_v = 0.0f;
    tail = next_index;              /* cola vacía */
    if (ex_hold) ex_hold_done = 1;
}

void exec_tick(float dt, float q_out[N_AXES])
{
    uint32_t t = tail;
    if (t == head) {
        ex_v = 0.0f;
        if (ex_hold) ex_hold_done = 1;
        goto out;
    }

    block_t *b = &B[t];
    uint32_t n = (t + 1u) & PMASK;
    int has_next = (n != head);
    float v_exit = has_next ? B[n].v_entry : 0.0f;

    float rem = b->L - ex_s;
    if (rem < 0.0f) rem = 0.0f;
    /* velocidad máxima que todavía permite frenar hasta v_exit al final del bloque */
    float v_allow = sqrtf(v_exit * v_exit + 2.0f * b->a * rem);
    float vt = ex_hold ? 0.0f : b->v_nom;
    if (v_allow < vt) vt = v_allow;

    float v0 = ex_v, v1;
    if (v0 < vt) {
        v1 = v0 + b->a * dt;
        if (v1 > vt) v1 = vt;
    } else {
        float dec = (ex_hold && v0 <= v_allow) ? b->a : 1.25f * b->a;
        v1 = v0 - dec * dt;
        if (v1 < vt) v1 = vt;
    }
    ex_s += 0.5f * (v0 + v1) * dt;
    ex_v = v1;

    /* avanzar de bloque (puede cruzar varios si son muy cortos) */
    while (ex_s >= b->L) {
        if (has_next) {
            ex_s -= b->L;
            t = n;
            tail = t;
            b = &B[t];
            n = (t + 1u) & PMASK;
            has_next = (n != head);
        } else {
            finish_last(b, n);
            goto out;
        }
    }
    /* final de la cola: casi en el punto y casi quieto */
    if (!has_next && (b->L - ex_s) < 1e-5f && ex_v < 1e-3f) {
        finish_last(b, n);
        goto out;
    }
    if (ex_hold && ex_v <= 0.0f) ex_hold_done = 1;

    for (int i = 0; i < N_AXES; i++) ex_q[i] = b->q0[i] + b->u[i] * ex_s;

out:
    for (int i = 0; i < N_AXES; i++) q_out[i] = ex_q[i];
}

void exec_get(float q_out[N_AXES])
{
    for (int i = 0; i < N_AXES; i++) q_out[i] = ex_q[i];
}

float exec_speed(void)
{
    return ex_v;
}

void exec_hold(void)
{
    ex_hold = 1;
}

int exec_hold_done(void)
{
    return ex_hold_done;
}
