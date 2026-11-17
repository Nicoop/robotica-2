/*
 * app_config.h
 * aca estan todos los numeros del robot, si queres cambiar algo casi seguro es aca
 * y no en el resto del codigo
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define FW_VERSION "1.0"
#define N_AXES 3   // son 3 articulaciones, no lo toques salvo q le agregues un eje

/* ======================= mecanica ======================= */
#define MOTOR_FULL_STEPS   200.0f                 // paps de 1,8 grados => 200 pasos por vuelta
#define MICROSTEPS         32.0f                  // ojo tiene q ser lo mismo q pusiste en los switches del driver
#define GEAR_RATIO_INIT    { 4.0f, 4.0f, 4.5f }   // cuantas vueltas da el motor para q la articulacion de una
/* acoplamiento mecanico: cuantos grados se mueve el eslabon 3 (respecto del 2) por cada grado
 * q se mueve el eje 2 con el motor 3 quieto. 0 = sin acoplamiento. se mide con la prueba del readme */
#define COUPLING_32        -0.333333f  // medido: eje 2 se mueve 90 grados -> eslabon 3 se mueve 30 (signo verificado en el robot)

/* si alguna articulacion se mueve para el otro lado poner 1 en DIR_INVERT
 * si el encoder cuenta al reves que el motor poner 1 en ENC_INVERT
 * (esto se prueba con el comando J, esta explicado en el readme) */
#define DIR_INVERT_INIT    { 0, 1, 0 }
#define ENC_INVERT_INIT    { 1, 1, 0 }

/* ============ limites de cada articulacion en grados ============
 * por ahora estan re abiertos porq todavia no sabemos los reales.
 * despues se cambian con el comando L y quedan guardados en la flash */
#define JOINT_MIN_INIT     { -180.0f, -180.0f, -180.0f }
#define JOINT_MAX_INIT     {  180.0f,  180.0f,  180.0f }

/* ============ velocidades y aceleraciones maximas ============ */
#define JOINT_VMAX_INIT    { 200.0f, 200.0f, 200.0f } // grados/s de la articulacion (no del motor). 200 = 133 rpm en el motor
#define JOINT_AMAX_INIT    { 400.0f, 400.0f, 400.0f } // grados/s2. con 400 llega a 200 grados/s en medio segundo
/* cuanto puede saltar la velocidad de golpe en una esquina de la trayectoria (grados/s)
 * si lo subis dobla mas rapido en las esquinas pero mas brusco */
#define JOINT_JERK_INIT    { 3.0f, 3.0f, 4.0f }

/* ======================= trayectoria ======================= */
#define PATH_V_DEFAULT     20.0f   // velocidad q usa si el comando S no trae ninguna
#define PATH_A_DEFAULT     60.0f   // lo mismo pero aceleracion
#define JOG_V_DEFAULT      10.0f   // velocidad para mover a mano con J y G
/* perfil en S: la referencia q sale del planificador (trapezoidal) se pasa por un promedio
 * movil de SCURVE_MS milisegundos. un trapecio promediado queda en S: la aceleracion ya no
 * aparece de golpe sino q sube y baja en SCURVE_MS ms (el jerk queda en aceleracion/SCURVE_MS).
 * tambien suaviza los saltos de velocidad en las esquinas. a cambio todo llega SCURVE_MS ms mas
 * tarde y en las curvas muy cerradas redondea un poquito. 0 = sin perfil en S (trapecio puro) */
#define SCURVE_MS          50u     // entre 20 y 150. mas = mas suave pero mas lento en arrancar y frenar
#define PLANNER_SIZE       512u    // cuantos puntos entran en la cola, tiene q ser potencia de 2 si o si
#define MIN_SEG_LEN        1e-4f   // si dos puntos estan mas cerca q esto el segundo se ignora

/* ======================= tiempos ======================= */
#define STEP_TICK_HZ       50000u  // cada cuanto salta la interrupcion q saca los pulsos
#define CTRL_HZ            1000u   // cada cuanto corre el lazo de control (1 ms)

/* ======================= comunicacion ======================= */
#define UART_BAUD             115200u
#define COMM_TIMEOUT_MS       500u   // si en medio de una trayectoria no llega nada en este tiempo se asume q se corto
#define RELEASE_ON_COMM_LOSS  1      // 1 = frena y suelta los motores, 0 = frena y se queda quieto con torque
#define MOTORS_HOLD_AT_BOOT   1      // 1 = al prender los drivers quedan con torque (el brazo no se cae), 0 = arrancan sueltos
#define AUTO_REF_AT_BOOT      1      // 1 = al prender se referencia solo con la calibracion guardada (como mandar R)
#define AUTO_HOME_AT_BOOT     0      // 1 = despues de referenciarse va solo a HOME_POSE
#define HOME_POSE             { 0.0f, 0.0f, 0.0f }   // posicion de inicio en grados

/* ======================= encoders ======================= */
#define ENC_AS5600   1
#define ENC_MT6701   2
#define ENC_MODEL    ENC_AS5600     // si resulta q tus placas son MT6701 cambia esto y listo
/* valor del registro TIMINGR del i2c con el clock a 80 MHz
 * 0x00702991 = 400 kHz, 0x10909CEC = 100 kHz
 * si los encoders tiran errores por ruido o cables largos proba con 100 kHz */
#define I2C_TIMINGR_VAL      0x10909CECu   // 100 kHz (mas robusto al ruido de los motores)
#define ENC_STALE_LOST_MS    100u    // si un encoder no responde por este tiempo se pierde la referencia
#define ENC_MAX_JUMP         100     // cuentas por ms: un salto mayor es una lectura mala y se descarta (el motor real gira < 5 cuentas/ms)
/* q ejes tienen encoder conectado (1 = si, 0 = no). un eje sin encoder anda en lazo abierto:
 * se mueve igual pero no corrige pasos perdidos ni detecta si se traba, y al referenciar
 * (comando R) tiene q estar EXACTO en la pose de calibracion porq no hay nada q lo mida.
 * cuando tengas el encoder q falta pone 1 de nuevo y listo */
#define ENC_PRESENT_INIT     { 1, 1, 1 }

/* ============ correccion con los encoders ============ */
#define CORR_ENABLE      1        // 0 la apaga (para la prueba de sentido de giro conviene apagarla)
#define CORR_DB_ON       24.0f    // upasos de error a partir de los cuales empieza a corregir
#define CORR_DB_OFF      6.0f     // upasos de error donde deja de corregir (asi no queda temblando)
#define CORR_KI          20.0f    // q tan rapido corrige, 1/KI es mas o menos el tiempo q tarda
#define CORR_RATE_MAX    8000.0f  // velocidad maxima a la q corrige en upasos/s
#define CORR_MAX         3200.0f  // lo maximo q puede corregir (media vuelta de motor), si llega aca es q algo anda mal
#define STALL_ERR        1600.0f   // si el error pasa esto (un cuarto de vuelta) se asume q el motor esta trabado
#define ENC_DELAY_TICKS  1        // el filtro del encoder atrasa mas o menos 1 ms, esto lo compensa
/* ============ filtro de los encoders por software ============
 * 1) mediana de 3 lecturas: mata los picos sueltos. atrasa 1 ms
 * 2) pasa bajos de primer orden con corte en ENC_LPF_HZ: saca el temblequeo.
 *    mas bajo = mas liso pero mas atrasado (60 Hz unos 2 ms, 30 Hz unos 5 ms)
 * el atraso se compensa solo. si el lazo oscila o tironea subi ENC_LPF_HZ, no lo bajes de 20 */
#define ENC_MEDIAN       1        // 1 = usa la mediana de 3, 0 = no
#define ENC_LPF_HZ       60.0f    // corte del pasa bajos en Hz, 0 = sin pasa bajos
#endif
