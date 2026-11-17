/*
 * app.c
 * el programa principal del robot: lazo de control, comandos por serie y estados
 *
 * interrupciones q hay (entre parentesis la prioridad, 0 es la mas alta):
 *   TIM6  50 kHz  (0)  saca los pulsos de los motores   -> stepgen.c
 *   I2Cx          (1)  lee los encoders                  -> encoder.c
 *   TIM7   1 kHz  (2)  planificador y lazo de control    -> aca
 *   USART2        (3)  comunicacion con matlab           -> comm.c
 * el bucle principal se encarga de los comandos, el control de flujo, el timeout y las fallas
 *
 * unidades: para afuera (matlab) todo en grados de articulacion, adentro en micropasos del motor
 */
#include "app.h"
#include "app_config.h"
#include "app_pins.h"
#include "hw.h"
#include "planner.h"
#include "stepgen.h"
#include "encoder.h"
#include "comm.h"
#include "storage.h"
#include "control.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define DT  (1.0f / (float)CTRL_HZ)   // periodo del lazo de control en segundos (0,001)

enum { FAULT_NONE = 0, FAULT_STALL, FAULT_ENC };   // tipos de falla: ninguna, motor trabado, encoder

static const float   gear[N_AXES]     = GEAR_RATIO_INIT;
static const uint8_t enc_inv[N_AXES]  = ENC_INVERT_INIT;
static const uint8_t enc_present[N_AXES] = ENC_PRESENT_INIT;   // 0 = ese eje no tiene encoder (lazo abierto)
static const float   jmin_def[N_AXES] = JOINT_MIN_INIT;
static const float   jmax_def[N_AXES] = JOINT_MAX_INIT;

static float spd[N_AXES];      // micropasos por grado de articulacion (con la reduccion incluida)
static float k_enc;            // micropasos de motor por cada cuenta del encoder
static float jmin[N_AXES], jmax[N_AXES];

/* ---- variables q comparten el bucle principal y la interrupcion de control (por eso volatile) ---- */
static volatile uint32_t ms_now;                       // milisegundos desde q arranco
static volatile uint8_t  motors_on, referenced, freeze;
static volatile uint8_t  fault_code, fault_axis, fault_new;

static volatile uint16_t enc_raw[N_AXES];   // ultima lectura de cada encoder (0 a 4095)
static volatile uint8_t  enc_st[N_AXES];    // status del encoder (si ve el iman y eso)
static volatile uint8_t  enc_have[N_AXES];  // ya llego por lo menos una lectura
static volatile int32_t  enc_acc[N_AXES];   // cuentas acumuladas contando las vueltas completas
static volatile float    enc_off[N_AXES];   // a cuantos upasos equivale enc_acc = 0 (sale de la referencia)
static volatile uint16_t enc_age[N_AXES];   // hace cuantos ms q no llega una lectura buena

/* filtro por software de los encoders (ver ENC_MEDIAN y ENC_LPF_HZ en app_config.h).
 * se filtra enc_acc (las cuentas con las vueltas incluidas) y de ahi en adelante se usa enc_f */
static volatile float    enc_f[N_AXES];     // enc_acc filtrado (en cuentas, con coma)
static int32_t  enc_m1[N_AXES], enc_m2[N_AXES];   // las 2 lecturas anteriores para la mediana
static uint8_t  enc_fok[N_AXES];            // el filtro ya arranco
static float    enc_alpha = 1.0f;           // cuanto pesa cada lectura nueva en el pasa bajos
static uint32_t enc_lag = 2u + (uint32_t)ENC_DELAY_TICKS;   // con cuantos ms de atraso se compara

#define HIST 16u   // tiene q ser potencia de 2 y mayor q el atraso total del encoder
static float    pref_hist[HIST][N_AXES];    // referencia de los ultimos ms, para comparar con el encoder q llega atrasado
static uint32_t hist_i;
static corr_t   corr[N_AXES];
static volatile float q_now[N_AXES];        // donde deberia estar el robot ahora (grados)

/* perfil en S (ver SCURVE_MS en app_config.h): promedio movil de la referencia del planificador.
 * guarda los ultimos SCURVE_MS valores de q y la referencia q se usa es el promedio */
#if SCURVE_MS > 0
#if SCURVE_MS > 200
#error "SCURVE_MS muy grande (maximo 200)"
#endif
static float    sc_buf[SCURVE_MS][N_AXES];  // ultimos valores q salieron del planificador
static uint32_t sc_i;                       // donde va el proximo
#endif
static volatile uint16_t sc_settle;         // ms q faltan para q el promedio termine de llegar (0 = quieto)

/* ---- estas solo las usa el bucle principal ---- */
static uint8_t  streaming, holding, comm_lost, lim_dirty;
static float    stream_v = PATH_V_DEFAULT, stream_a = PATH_A_DEFAULT;   // velocidad y aceleracion del ultimo S
static uint32_t last_rx_ms;                 // cuando llego algo por ultima vez (para el timeout)
static cal_t    cal;                        // calibracion leida de la flash

/* ======================================================================== */
/*                            funciones de ayuda                             */
/* ======================================================================== */

// posicion del motor segun el encoder, en micropasos
static inline float enc_usteps(int i)
{
    if (!enc_present[i]) return sg_get_pos(i);   // sin encoder: lo unico q se sabe son los pasos q se mandaron
    float c = enc_f[i] * k_enc;   // la posicion filtrada, no la cruda
    return enc_off[i] + (enc_inv[i] ? -c : c);
}

/* acoplamiento mecanico entre el eje 2 y el 3: la transmision del eje 3 pasa por el eje de
 * rotacion del 2, entonces si se mueve el 2 con el motor 3 quieto, el eslabon 3 igual se mueve
 * COUPLING_32 grados por cada grado del eje 2. aca se compensa:
 *   lo q tiene q girar el motor 3 = q3 - COUPLING_32 * q2   (en grados de articulacion)
 *   el angulo real del eslabon 3   = motor3 + COUPLING_32 * q2 */
static inline float joint_to_motor(int i, const float q[N_AXES])
{
    if (i == 2) return q[2] - COUPLING_32 * q[1];
    return q[i];
}

static void motors_to_joints(const float m[N_AXES], float q[N_AXES])
{
    for (int i = 0; i < N_AXES; i++) q[i] = m[i];
    q[2] = m[2] + COUPLING_32 * m[1];
}

// prende o suelta los motores con el pin ENA, uno solo para los 3 drivers (q en el tb6600
// funciona al reves: activar ENA = driver apagado = motor sin torque)
static void ena_set(uint8_t motors_enabled)
{
    uint32_t assert_ena = ENA_ASSERT_RELEASES ? !motors_enabled : motors_enabled;
    hw_pin_write(ENA_PORT, ENA_PIN, SIG_LEVEL(assert_ena));
}

// se llama desde la interrupcion cuando algo anda mal. congela el movimiento y
// deja anotado q paso para q el bucle principal avise por serie
static void raise_fault(uint8_t code, uint8_t axis)
{
    if (!fault_code) {          // me quedo con la primer falla, las q vienen despues suelen ser concecuencia
        fault_code = code;
        fault_axis = axis;
        fault_new = 1;
    }
    freeze = 1;
}

/* ---- mandar numeros por serie sin printf (el printf con float ocupa un monton y aveces ni anda) ---- */
static void out_u(uint32_t v)
{
    char b[12];
    int n = 0;
    do { b[n++] = (char)('0' + v % 10u); v /= 10u; } while (v && n < 11);   // saca los digitos al reves
    char r[12];
    for (int k = 0; k < n; k++) r[k] = b[n - 1 - k];                        // y los doy vuelta
    r[n] = 0;
    comm_puts(r);
}

// numero con decimales: lo multiplico por 10^dec, lo redondeo y lo imprimo en dos partes
static void out_f(float v, int dec)
{
    if (v != v) { comm_puts("nan"); return; }    // un nan es distinto de si mismo, truco viejo
    if (v < 0.0f) { comm_puts("-"); v = -v; }
    if (v > 4.0e6f) v = 4.0e6f;                  // para q no se desborde el entero
    uint32_t m = 1;
    for (int k = 0; k < dec; k++) m *= 10u;
    uint32_t x = (uint32_t)(v * (float)m + 0.5f);
    out_u(x / m);
    if (dec > 0) {
        char b[8];
        uint32_t f = x % m;
        for (int k = dec - 1; k >= 0; k--) { b[k] = (char)('0' + f % 10u); f /= 10u; }   // con ceros adelante
        b[dec] = 0;
        comm_puts(".");
        comm_puts(b);
    }
}

// respuestas tipicas a los comandos
static void reply_ok(void)      { comm_puts("ok\n"); }
static void reply_ok_free(void) { comm_puts("ok,"); out_u(planner_free()); comm_puts("\n"); }   // ok + lugares libres
static void reply_err(const char *e) { comm_puts("err,"); comm_puts(e); comm_puts("\n"); }
static void reply_err_ax(const char *e, int ax)
{
    comm_puts("err,"); comm_puts(e); comm_puts(","); out_u((uint32_t)ax + 1u); comm_puts("\n");   // ejes numerados desde 1
}

/* ======================================================================== */
/*               lazo de control, interrupcion del TIM7 cada 1 ms            */
/* ======================================================================== */

// mediana de 3 numeros (el del medio)
static inline int32_t med3(int32_t a, int32_t b, int32_t c)
{
    if (a > b) { int32_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return (a > b) ? a : b;
}

// pone el filtro de un encoder justo en el valor actual de enc_acc (sin historia vieja)
static void enc_filter_reset(int i)
{
    enc_m1[i] = enc_m2[i] = enc_acc[i];
    enc_f[i] = (float)enc_acc[i];
    enc_fok[i] = 1;
}

// le pasa una lectura nueva al filtro (mediana de 3 y despues pasa bajos)
static inline void enc_filter_step(int i)
{
    int32_t x = enc_acc[i];
    if (!enc_fok[i]) enc_filter_reset(i);
#if ENC_MEDIAN
    int32_t m = med3(x, enc_m1[i], enc_m2[i]);
    enc_m2[i] = enc_m1[i];
    enc_m1[i] = x;
#else
    int32_t m = x;
#endif
    enc_f[i] += enc_alpha * ((float)m - enc_f[i]);
}

// llena el promedio del perfil en S con una posicion fija (sin historia, no se mueve)
static void sc_reset(const float q[N_AXES])
{
#if SCURVE_MS > 0
    for (uint32_t k = 0; k < SCURVE_MS; k++)
        for (int i = 0; i < N_AXES; i++) sc_buf[k][i] = q[i];
    sc_i = 0;
#else
    (void)q;
#endif
    sc_settle = 0;
}

// perfil en S: mete la q nueva del planificador y la reemplaza por el promedio de los ultimos SCURVE_MS ms
static inline void sc_filter(float q[N_AXES])
{
#if SCURVE_MS > 0
    uint32_t prev = (sc_i + SCURVE_MS - 1u) % SCURVE_MS;
    uint8_t cambio = 0;
    for (int i = 0; i < N_AXES; i++) {
        if (fabsf(q[i] - sc_buf[prev][i]) > 1e-6f) cambio = 1;
        sc_buf[sc_i][i] = q[i];
    }
    sc_i = (sc_i + 1u) % SCURVE_MS;
    // la suma se hace entera cada vez (son 150 sumas, nada para el micro) asi no se acumula error
    for (int i = 0; i < N_AXES; i++) {
        float acc = 0.0f;
        for (uint32_t k = 0; k < SCURVE_MS; k++) acc += sc_buf[k][i];
        q[i] = acc / (float)SCURVE_MS;
    }
    if (cambio) sc_settle = (uint16_t)SCURVE_MS;
    else if (sc_settle > 0u) sc_settle--;
#else
    (void)q;
#endif
}

void TIM7_IRQHandler(void)
{
    TIM7->SR = 0;
    ms_now++;

    /* 1) agarro las lecturas de los encoders q se pidieron el ms anterior y cuento vueltas.
     *    el encoder solo sabe el angulo dentro de una vuelta, entonces si pasa de 4095 a 0
     *    es q dio una vuelta mas. como en 1 ms el motor gira muy poquito, si el salto es
     *    mayor a media vuelta seguro q fue eso */
    for (int i = 0; i < N_AXES; i++) {
        uint16_t raw;
        uint8_t st;
        if (enc_collect(i, &raw, &st) > 0) {
            if (enc_have[i]) {
                int32_t d = (int32_t)raw - (int32_t)enc_raw[i];
                if (d > ENC_COUNTS / 2) d -= ENC_COUNTS;
                else if (d < -(ENC_COUNTS / 2)) d += ENC_COUNTS;
                // filtro de lecturas imposibles: el motor no puede girar tanto en 1 ms, asi q si el salto
                // es enorme es una lectura mala (ruido en el i2c, iman debil) y la descarto
                int32_t lim = ENC_MAX_JUMP * ((int32_t)enc_age[i] + 1);
                if (d > lim || d < -lim) {
                    if (enc_age[i] < 0xFFFFu) enc_age[i]++;
                    continue;
                }
                enc_acc[i] += d;
            }
            enc_raw[i] = raw;
            enc_st[i] = st;
            enc_have[i] = 1;
            enc_age[i] = 0;
            enc_filter_step(i);           // filtro por software (mediana + pasa bajos)
        } else if (enc_age[i] < 0xFFFFu) {
            enc_age[i]++;                 // no llego nada, la ultima lectura va quedando vieja
        }
    }
    enc_start_all();                      // pido las lecturas para el ms q viene

    /* 2) si un encoder no responde por mucho tiempo ya no se puede confiar en el conteo de
     *    vueltas, asi q se pierde la referencia y hay q volver a referenciar */
    if (referenced) {
        for (int i = 0; i < N_AXES; i++) {
            if (enc_present[i] && enc_age[i] > ENC_STALE_LOST_MS) {
                referenced = 0;
                raise_fault(FAULT_ENC, (uint8_t)i);
                break;
            }
        }
    }

    float q[N_AXES];
    if (motors_on && referenced) {
        /* 3) le pido al planificador donde tiene q estar el robot en este ms */
        if (freeze) exec_get(q);
        else exec_tick(DT, q);
        sc_filter(q);                     // perfil en S (promedio movil), de aca en adelante todo usa la q suavizada

        hist_i = (hist_i + 1u) & (HIST - 1u);
        for (int i = 0; i < N_AXES; i++) pref_hist[hist_i][i] = joint_to_motor(i, q) * spd[i];
        // la lectura q agarre recien se pidio hace 1 ms y el filtro del encoder atrasa otro ms mas o menos,
        // entonces la comparo con la referencia de hace unos ms y no con la de ahora
        uint32_t lag = (hist_i - enc_lag) & (HIST - 1u);   // enc_lag incluye el atraso de los filtros
        (void)lag;

        for (int i = 0; i < N_AXES; i++) {
            float c = corr[i].c;
#if CORR_ENABLE
            if (enc_present[i] && enc_age[i] == 0u && !freeze) {   // solo si tiene encoder y la lectura es nueva
                float e = pref_hist[lag][i] - enc_usteps(i);   // error = donde deberia estar - donde esta
                c = corr_update(&corr[i], e, DT);
                // motor trabado: el error es muy grande o la correccion ya puso todo lo q podia y no sirvio
                if (fabsf(e) > STALL_ERR || fabsf(c) >= CORR_MAX) raise_fault(FAULT_STALL, (uint8_t)i);
            }
#endif
            /* 4) le digo al generador de pasos a donde tiene q llegar al final del proximo ms
             *    (referencia + correccion) */
            sg_set_target(i, joint_to_motor(i, q) * spd[i] + c);
        }
    } else {
        // motores sueltos o sin referencia: no se mueve nada y la posicion la saco del encoder
        sc_settle = 0;                    // el perfil en S no corre, no hay nada terminando de llegar
        float m[N_AXES];
        for (int i = 0; i < N_AXES; i++) {
            sg_stop_axis(i);
            m[i] = referenced ? enc_usteps(i) / spd[i] : 0.0f;
        }
        motors_to_joints(m, q);
    }
    for (int i = 0; i < N_AXES; i++) q_now[i] = q[i];
}

// TIM7 a 1 kHz: prescaler para q cuente a 1 MHz y q desborde cada 1000 cuentas
static void ctrl_timer_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
    (void)RCC->APB1ENR1;
    TIM7->CR1 = 0;
    TIM7->PSC = hw_tim_apb1_hz() / 1000000u - 1u;   // 80 MHz / 80 = 1 MHz
    TIM7->ARR = 1000000u / CTRL_HZ - 1u;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0;
    TIM7->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM7_IRQn, 2);   // mas baja q los pulsos y el i2c, asi esos nunca esperan
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* ======================================================================== */
/*                     estados, motores y referenciado                       */
/* ======================================================================== */

/* pone el contador de pasos, la referencia y el planificador en la posicion q mide el encoder.
 * se usa al habilitar los motores, haci si alguien movio el brazo a mano no pega un salto */
static void sync_to_encoder(void)
{
    float q[N_AXES], m[N_AXES];
    uint32_t pm = hw_irq_save();
    for (int i = 0; i < N_AXES; i++) {
        float p = enc_usteps(i);
        sg_set_pos(i, (int32_t)floorf(p + 0.5f));
        corr_reset(&corr[i]);
        for (uint32_t h = 0; h < HIST; h++) pref_hist[h][i] = p;   // relleno la historia para q no de error de una
        m[i] = p / spd[i];
    }
    motors_to_joints(m, q);   // de posicion de motores a angulos de articulacion
    for (int i = 0; i < N_AXES; i++) q_now[i] = q[i];
    planner_reset(q);
    sc_reset(q);                // el perfil en S arranca quieto en la posicion del encoder
    freeze = 0;
    hw_irq_restore(pm);
}

static void motors_set(uint8_t on)
{
    if (on) {
        if (referenced) sync_to_encoder();   // primero sincronizo y despues habilito, el orden importa
        sg_enable(1);
        motors_on = 1;
        ena_set(1);
    } else {
        motors_on = 0;
        sg_enable(0);
        ena_set(0);
    }
}

// lleva una diferencia de cuentas al rango -media vuelta a +media vuelta
static int wrap_counts(int32_t d)
{
    while (d >= ENC_COUNTS / 2) d -= ENC_COUNTS;
    while (d < -(ENC_COUNTS / 2)) d += ENC_COUNTS;
    return d;
}

/* referenciado (la opcion 1 q elegimos): el robot esta mas o menos en la pose de calibracion,
 * a menos de media vuelta de motor en cada articulacion. como el encoder sabe el angulo
 * exacto dentro de la vuelta, comparando con lo q leia en la calibracion se saca la posicion
 * exacta. con la reduccion media vuelta de motor son 45 grados en las 2 primeras y 30 en la tercera */
static int do_reference(void)
{
    for (int i = 0; i < N_AXES; i++)
        if (enc_present[i] && (!enc_have[i] || enc_age[i] > 20u)) { reply_err_ax("enc", i); return -1; }
#if ENC_MODEL == ENC_AS5600
    for (int i = 0; i < N_AXES; i++)
        if (enc_present[i] && !(enc_st[i] & ENC_ST_MD)) { reply_err_ax("magnet", i); return -1; }   // sin iman no hay nada q hacer
#endif
    float qc[N_AXES];
    for (int i = 0; i < N_AXES; i++) qc[i] = cal.q_cal[i];
    uint32_t pm = hw_irq_save();
    for (int i = 0; i < N_AXES; i++) {
        float mc = joint_to_motor(i, qc);   // donde estaba el motor (en grados) en la pose de calibracion
        if (!enc_present[i]) {
            // sin encoder no hay forma de medir: se asume q esta justo en la pose de calibracion
            sg_set_pos(i, (int32_t)floorf(mc * spd[i] + 0.5f));
            continue;
        }
        int32_t d = wrap_counts((int32_t)enc_raw[i] - (int32_t)cal.raw_cal[i]);   // cuanto giro el motor desde la calibracion
        float du = (float)d * k_enc;
        if (enc_inv[i]) du = -du;
        float mi = mc + du / spd[i];   // posicion del motor = la de calibracion + lo q se movio
        enc_acc[i] = 0;
        enc_filter_reset(i);           // el filtro arranca de cero junto con el conteo, si no pegaria un salto
        enc_off[i] = mi * spd[i];
    }
    referenced = 1;
    if (fault_code == FAULT_ENC) { fault_code = FAULT_NONE; fault_new = 0; }
    // si los motores ya estaban prendidos sincronizo aca mismo, con las interrupciones apagadas,
    // asi el lazo de control no alcanza a ver una mezcla de la posicion vieja y la nueva
    if (motors_on) sync_to_encoder();
    hw_irq_restore(pm);

#if ENC_MODEL == ENC_AS5600
    // avisos si el iman esta muy lejos o muy cerca (anda igual pero puede tener mas ruido)
    for (int i = 0; i < N_AXES; i++) {
        if (!enc_present[i]) continue;
        if (enc_st[i] & ENC_ST_ML) { comm_puts("!warn,magnet_weak,"); out_u((uint32_t)i + 1u); comm_puts("\n"); }
        if (enc_st[i] & ENC_ST_MH) { comm_puts("!warn,magnet_strong,"); out_u((uint32_t)i + 1u); comm_puts("\n"); }
    }
#endif
    motors_set(1);
    return 0;
}

static void clear_fault(void)
{
    fault_code = FAULT_NONE;
    fault_new = 0;
    if (referenced && motors_on) sync_to_encoder();
    freeze = 0;
}

// frenar suave siguiendo la trayectoria (si es q se esta moviendo)
static void start_hold(void)
{
    if (!planner_empty()) {
        exec_hold();
        holding = 1;
    }
}

// devuelve q eje se pasa del limite, o -1 si esta todo bien
static int limit_violation(const float q[N_AXES])
{
    for (int i = 0; i < N_AXES; i++)
        if (q[i] < jmin[i] || q[i] > jmax[i]) return i;
    return -1;
}

static const char *state_name(void)
{
    if (fault_code)          return "FAULT";
    if (!referenced)         return "NOREF";
    if (holding)             return "HOLD";
    if (!planner_empty() || sc_settle) return "RUN";   // con el perfil en S termina de llegar SCURVE_MS ms despues
    return "IDLE";
}

static int is_busy(void)
{
    return streaming || holding || !planner_empty() || sc_settle;
}

/* ======================================================================== */
/*                       protocolo (los comandos de matlab)                  */
/* ======================================================================== */

// un caracter hexa a numero, -1 si no es hexa
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// texto a float, pero mas estricto q strtof solo: si sobra basura o es un disparate da error
static int pf(const char *s, float *out)
{
    char *e;
    float v = strtof(s, &e);
    if (e == s) return 0;
    while (*e == ' ') e++;
    if (*e) return 0;
    if (!(v == v) || v > 1e6f || v < -1e6f) return 0;
    *out = v;
    return 1;
}

// separa la linea por las comas (modifica la linea, cambia las comas por 0)
static int split(char *s, char *tok[], int max)
{
    int n = 0;
    tok[n++] = s;
    for (char *p = s; *p && n < max; p++) {
        if (*p == ',') { *p = 0; tok[n++] = p + 1; }
    }
    return n;
}

// respuesta del Q: st,estado,motores,qref1,qref2,qref3,qenc1,qenc2,qenc3,libres,falla
static void print_status(void)
{
    comm_puts("st,");
    comm_puts(state_name());
    comm_puts(",");
    out_u(motors_on);
    for (int i = 0; i < N_AXES; i++) { comm_puts(","); out_f(q_now[i], 3); }
    float m[N_AXES], qe[N_AXES];
    for (int i = 0; i < N_AXES; i++) m[i] = referenced ? enc_usteps(i) / spd[i] : 0.0f;
    motors_to_joints(m, qe);   // lo q miden los encoders, pasado a angulos de articulacion
    for (int i = 0; i < N_AXES; i++) {
        comm_puts(",");
        out_f(qe[i], 3);
    }
    comm_puts(",");
    out_u(planner_free());
    comm_puts(",");
    out_u(fault_code);
    comm_puts("\n");
}

// respuesta del W: lectura cruda, status y errores de cada encoder. sirve para ver si los encoders andan
static void print_raw(void)
{
    comm_puts("raw");
    for (int i = 0; i < N_AXES; i++) {
        comm_puts(",");
        out_u(enc_raw[i]);
        comm_puts(",");
        out_u(enc_st[i]);
        comm_puts(",");
        out_u(enc_errors(i));
    }
    comm_puts("\n");
}

static void print_help(void)
{
    comm_puts("!S,v[,a]  inicia stream (v en grados/s, a en grados/s2)\n"
              "!P,q1,q2,q3  punto (grados)\n"
              "!E  fin de stream   X  parar / borrar falla\n"
              "!J,eje,delta[,v[,a]]  mover relativo   G,q1,q2,q3[,v[,a]]  ir a pose\n"
              "!M,0|1  liberar / habilitar motores\n"
              "!C,q1,q2,q3  calibrar: el robot esta en esa pose\n"
              "!R  referenciar con la calibracion guardada\n"
              "!L,eje,min,max  limites articulares\n"
              "!Q  estado   W  lecturas crudas de encoders\n");
}

// procesa una linea q se resivio por serie
static void handle_line(char *line)
{
    /* saco los espacios de adelante y de atras */
    while (*line == ' ' || *line == '\t') line++;
    size_t len = strlen(line);
    while (len && (line[len - 1] == ' ' || line[len - 1] == '\t')) line[--len] = 0;
    if (!len) return;

    /* checksum opcional "*HH": XOR de todos los caracteres antes del asterisco.
     * matlab siempre lo manda, desde una terminal a mano se puede no poner */
    char *star = strchr(line, '*');
    if (star) {
        uint8_t cs = 0;
        for (char *p = line; p < star; p++) cs ^= (uint8_t)*p;
        int h1 = hexval(star[1]), h2 = (h1 >= 0) ? hexval(star[2]) : -1;
        if (h1 < 0 || h2 < 0 || star[3] != 0 || cs != (uint8_t)((h1 << 4) | h2)) {
            reply_err("cs");
            if (streaming) { streaming = 0; start_hold(); }   // si llego algo roto en medio de una trayectoria mejor frenar
            return;
        }
        *star = 0;   // corto la linea antes del asterisco y sigo
    }

    char *tok[8];
    int n = split(line, tok, 8);
    char c = tok[0][0];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');   // acepta minusculas tambien
    if (tok[0][1] != 0) { reply_err("cmd"); return; }       // los comandos son de una sola letra

    switch (c) {

    case 'P': {                                     // un punto de la trayectoria
        float q[N_AXES];
        if (!streaming) { reply_err("state"); return; }
        if (n < 4 || !pf(tok[1], &q[0]) || !pf(tok[2], &q[1]) || !pf(tok[3], &q[2])) {
            reply_err("syntax"); streaming = 0; start_hold(); return;
        }
        int ax = limit_violation(q);
        if (ax >= 0) { reply_err_ax("limit", ax); streaming = 0; start_hold(); return; }   // fuera de limites: frena
        int r = planner_add(q, stream_v, stream_a);
        if (r < 0) { reply_err("busy"); return; }
        reply_ok_free();
        break;
    }

    case 'S': {                                     // arranca una trayectoria, trae la velocidad (y opcional la aceleracion)
        float v = PATH_V_DEFAULT, a = PATH_A_DEFAULT;
        if (fault_code)  { reply_err("fault"); return; }
        if (!referenced) { reply_err("noref"); return; }
        if (holding)     { reply_err("busy"); return; }
        if (n >= 2 && !pf(tok[1], &v)) { reply_err("syntax"); return; }
        if (n >= 3 && !pf(tok[2], &a)) { reply_err("syntax"); return; }
        if (v <= 0.0f || a <= 0.0f)    { reply_err("syntax"); return; }
        stream_v = v;
        stream_a = a;
        if (!motors_on) motors_set(1);
        streaming = 1;
        comm_lost = 0;
        last_rx_ms = ms_now;
        reply_ok_free();
        break;
    }

    case 'E':                                       // fin normal: termina lo q tiene y se queda quieto con torque
        streaming = 0;
        reply_ok();
        break;

    case 'X':                                       // parar, y si habia una falla la borra
        streaming = 0;
        if (fault_code) clear_fault();
        else start_hold();
        reply_ok();
        break;

    case 'J':                                       // mover un eje relativo (jog)
    case 'G': {                                     // ir a una pose
        float q[N_AXES], v = JOG_V_DEFAULT, acc = PATH_A_DEFAULT;   // la aceleracion es opcional, va al final
        if (fault_code)  { reply_err("fault"); return; }
        if (!referenced) { reply_err("noref"); return; }
        if (streaming || holding) { reply_err("busy"); return; }
        planner_get_end(q);                         // arranco desde donde termina lo q ya esta cargado
        if (c == 'J') {
            float ax, d;
            if (n < 3 || !pf(tok[1], &ax) || !pf(tok[2], &d)) { reply_err("syntax"); return; }
            int a = (int)ax - 1;
            if (a < 0 || a >= N_AXES) { reply_err("syntax"); return; }
            if (n >= 4 && !pf(tok[3], &v)) { reply_err("syntax"); return; }
            if (n >= 5 && !pf(tok[4], &acc)) { reply_err("syntax"); return; }   // J,eje,d,v,a
            q[a] += d;
        } else {
            if (n < 4 || !pf(tok[1], &q[0]) || !pf(tok[2], &q[1]) || !pf(tok[3], &q[2])) {
                reply_err("syntax"); return;
            }
            if (n >= 5 && !pf(tok[4], &v)) { reply_err("syntax"); return; }
            if (n >= 6 && !pf(tok[5], &acc)) { reply_err("syntax"); return; }  // G,q1,q2,q3,v,a
        }
        if (v <= 0.0f || acc <= 0.0f) { reply_err("syntax"); return; }
        int ax = limit_violation(q);
        if (ax >= 0) { reply_err_ax("limit", ax); return; }
        if (!motors_on) motors_set(1);
        if (planner_add(q, v, acc) < 0) { reply_err("busy"); return; }   // el planificador igual la recorta a JOINT_AMAX de cada eje
        reply_ok();
        break;
    }

    case 'M': {                                     // motores: 0 sueltos, 1 con torque
        float v;
        if (n < 2 || !pf(tok[1], &v)) { reply_err("syntax"); return; }
        if (is_busy()) { reply_err("busy"); return; }   // no dejo soltar en movimiento
        motors_set(v != 0.0f);
        reply_ok();
        break;
    }

    case 'C': {                                     // calibrar: "el robot esta ahora en esta pose"
        float q[N_AXES];
        if (is_busy()) { reply_err("busy"); return; }
        if (n < 4 || !pf(tok[1], &q[0]) || !pf(tok[2], &q[1]) || !pf(tok[3], &q[2])) {
            reply_err("syntax"); return;
        }
        for (int i = 0; i < N_AXES; i++)
            if (enc_present[i] && (!enc_have[i] || enc_age[i] > 20u)) { reply_err_ax("enc", i); return; }
        for (int i = 0; i < N_AXES; i++) {
            cal.q_cal[i] = q[i];
            cal.raw_cal[i] = enc_raw[i];            // me guardo lo q lee cada encoder en esta pose
        }
        cal.flags |= CAL_HAS_REF;
        if (storage_save(&cal) != 0) { reply_err("flash"); return; }
        if (do_reference() == 0) reply_ok();
        break;
    }

    case 'R':                                       // referenciar con la calibracion guardada
        if (is_busy()) { reply_err("busy"); return; }
        if (!(cal.flags & CAL_HAS_REF)) { reply_err("nocal"); return; }
        if (fault_code == FAULT_STALL) clear_fault();
        if (do_reference() == 0) reply_ok();
        break;

    case 'L': {                                     // limites de una articulacion
        float ax, mn, mx;
        if (n < 4 || !pf(tok[1], &ax) || !pf(tok[2], &mn) || !pf(tok[3], &mx)) {
            reply_err("syntax"); return;
        }
        int a = (int)ax - 1;
        if (a < 0 || a >= N_AXES || mn >= mx) { reply_err("syntax"); return; }
        jmin[a] = mn;
        jmax[a] = mx;
        for (int i = 0; i < N_AXES; i++) { cal.jmin[i] = jmin[i]; cal.jmax[i] = jmax[i]; }
        cal.flags |= CAL_HAS_LIM;
        if (is_busy()) { lim_dirty = 1; reply_ok(); return; }   // si se esta moviendo lo grabo despues, cuando pare
        if (storage_save(&cal) != 0) { reply_err("flash"); return; }
        reply_ok();
        break;
    }

    case 'Q': print_status(); break;
    case 'W': print_raw();    break;
    case '?': print_help(); reply_ok(); break;
    default:  reply_err("cmd"); break;
    }
}

/* ======================================================================== */
/*                       arranque y bucle principal                          */
/* ======================================================================== */

void app_init(void)
{
    // conversiones: 200 pasos * 8 micropasos * reduccion / 360 grados
    for (int i = 0; i < N_AXES; i++) {
        spd[i] = MICROSTEPS * MOTOR_FULL_STEPS * gear[i] / 360.0f;
        jmin[i] = jmin_def[i];
        jmax[i] = jmax_def[i];
    }
    k_enc = MICROSTEPS * MOTOR_FULL_STEPS / (float)ENC_COUNTS;   // 1600 / 4096 upasos por cuenta

    /* filtro de los encoders: alpha del pasa bajos y cuanto atrasa todo junto.
     * el pasa bajos de primer orden atrasa (1-alpha)/alpha muestras, la mediana 1 */
    {
        float extra = 0.0f;
        if (ENC_LPF_HZ > 0.0f) {
            enc_alpha = 1.0f - expf(-2.0f * 3.14159265f * ENC_LPF_HZ * DT);
            extra += (1.0f - enc_alpha) / enc_alpha;
        }
        if (ENC_MEDIAN) extra += 1.0f;
        uint32_t lag = 2u + (uint32_t)ENC_DELAY_TICKS + (uint32_t)(extra + 0.5f);
        if (lag > HIST - 1u) lag = HIST - 1u;
        enc_lag = lag;
    }

    hw_gpio_out(LED_PORT, LED_PIN, 0, 0);
    // el ENA arranca como diga MOTORS_HOLD_AT_BOOT (asi no hay un instante con los motores sueltos)
    hw_gpio_out(ENA_PORT, ENA_PIN, SIG_OPEN_DRAIN,
                SIG_LEVEL(ENA_ASSERT_RELEASES ? !MOTORS_HOLD_AT_BOOT : MOTORS_HOLD_AT_BOOT));
    motors_on = 0;                      // el firmware no mueve nada hasta q le pidan (o se referencie)
    ena_set(MOTORS_HOLD_AT_BOOT);       // 1 = drivers con torque, el brazo no se cae; 0 = sueltos para acomodarlo a mano

    planner_init();
    sg_init();
    comm_init(UART_BAUD);
    int enc_mask = enc_init();

    // si hay calibracion y limites guardados los cargo
    if (storage_load(&cal)) {
        if (cal.flags & CAL_HAS_LIM) {
            for (int i = 0; i < N_AXES; i++) { jmin[i] = cal.jmin[i]; jmax[i] = cal.jmax[i]; }
        }
    } else {
        memset(&cal, 0, sizeof cal);
    }

    ctrl_timer_init();   // a partir de aca corre el lazo de control

    // mensaje de arranque, para saber desde matlab q la placa se reseteo
    comm_puts("!ready,robot3gdl," FW_VERSION ",cal=");
    out_u((cal.flags & CAL_HAS_REF) ? 1u : 0u);
    comm_puts("\n");
    if (SystemCoreClock != 80000000u) {   // todo esta calculado para 80 MHz, si no es asi aviso
        comm_puts("!warn,clock,"); out_u(SystemCoreClock); comm_puts("\n");
    }
    for (int i = 0; i < N_AXES; i++) {
        if (!enc_present[i]) { comm_puts("!warn,no_encoder,"); out_u((uint32_t)i + 1u); comm_puts("\n"); }
        else if (!(enc_mask & (1 << i))) { comm_puts("!warn,enc_init,"); out_u((uint32_t)i + 1u); comm_puts("\n"); }
    }

#if AUTO_REF_AT_BOOT
    // si hay una calibracion guardada, se referencia solo al arrancar (como mandar R)
    if (cal.flags & CAL_HAS_REF) {
        uint32_t t0 = ms_now;
        while ((uint32_t)(ms_now - t0) < 50u) { }   // espero 50 ms a q lleguen las primeras lecturas de los encoders
        if (do_reference() == 0) {
            comm_puts("!auto_ref\n");
#if AUTO_HOME_AT_BOOT
            float home[N_AXES] = HOME_POSE;
            planner_add(home, JOG_V_DEFAULT, PATH_A_DEFAULT);   // y despues va solo a la posicion de inicio
            comm_puts("!going_home\n");
#endif
        }
    }
#endif
}

void app_loop(void)
{
    static char line[96];

    /* 1) leo los comandos q llegaron. aca esta el control de flujo: si llega un P y la cola
     *    esta llena lo dejo esperando en el buffer y no contesto. porqe matlab no manda mas
     *    hasta q le contesten, nunca se desborda nada */
    while (comm_lines() > 0u) {
        int c = comm_peek();
        if ((c == 'P' || c == 'p') && streaming && planner_free() == 0u) break;
        int r = comm_read_line(line, sizeof line);
        last_rx_ms = ms_now;
        if (r < 0) { reply_err("long"); continue; }
        handle_line(line);
    }
    // si hay lineas esperando es q la pc esta viva, aunque no este mandando nada nuevo
    if (comm_lines() > 0u) last_rx_ms = ms_now;
    if (comm_overflow()) {
        comm_puts("!rx_overflow\n");
        if (streaming) { streaming = 0; start_hold(); }
    }
    {
        // si el uart perdio caracteres aviso (una vez por tanda), la linea q venia llega como err,cs
        static uint32_t rx_err_ant;
        uint32_t re = comm_rx_errors();
        if (re != rx_err_ant) {
            rx_err_ant = re;
            comm_puts("!warn,uart_rx,"); out_u(re); comm_puts("\n");
        }
    }

    /* 2) se corto la comunicacion en medio de una trayectoria (se desenchufo el usb, se colgo matlab, etc).
     *    frena siguiendo la trayectoria y despues suelta los motores */
    if (streaming && (uint32_t)(ms_now - last_rx_ms) > COMM_TIMEOUT_MS) {
        streaming = 0;
        comm_lost = 1;
        comm_puts("!comm_lost\n");
        if (!planner_empty()) start_hold();
        else if (RELEASE_ON_COMM_LOSS) { motors_set(0); comm_puts("!released\n"); }   // ya estaba quieto, suelto directo
    }

    /* 3) termino de frenar: vacio la cola y si fue por corte de comunicacion suelto los motores */
    if (holding && exec_hold_done() && !sc_settle) {   // espero q el perfil en S tambien termine de frenar
        uint32_t pm = hw_irq_save();
        planner_flush();
        hw_irq_restore(pm);
        holding = 0;
        if (comm_lost && RELEASE_ON_COMM_LOSS) { motors_set(0); comm_puts("!released\n"); }
        else comm_puts("!stopped\n");
    }

    /* 4) la interrupcion detecto una falla: freno todo y aviso */
    if (fault_new) {
        fault_new = 0;
        streaming = 0;
        holding = 0;
        uint32_t pm = hw_irq_save();
        planner_flush();
        hw_irq_restore(pm);
        if (fault_code == FAULT_STALL) {
            sync_to_encoder();          // se queda donde esta realmente el motor, no donde deberia estar
            comm_puts("!fault,stall,");
        } else {
            comm_puts("!fault,encoder,");
        }
        out_u((uint32_t)fault_axis + 1u);
        comm_puts("\n");
    }

    /* 5) limites q se cambiaron mientras se movia, los grabo ahora q esta quieto */
    if (lim_dirty && !is_busy()) {
        lim_dirty = 0;
        if (storage_save(&cal) != 0) comm_puts("!warn,flash\n");
    }

    /* 6) el led: lento = sin referencia, rapido = moviendose, muy rapido = falla, fijo = listo */
    uint32_t t = ms_now;
    uint32_t on;
    if (fault_code)               on = (t / 50u) & 1u;
    else if (!referenced)         on = (t / 500u) & 1u;
    else if (!planner_empty())    on = (t / 100u) & 1u;
    else                          on = motors_on;
    hw_pin_write(LED_PORT, LED_PIN, on);
}
