/*
 * comm.h
 * comunicacion por el USART2, q es el q sale por el usb del st-link como puerto COM
 */
#ifndef COMM_H
#define COMM_H

#include <stdint.h>

void     comm_init(uint32_t baud);
uint32_t comm_lines(void);                    // cuantas lineas completas hay esperando
int      comm_peek(void);                     // primer caracter de la proxima linea sin sacarla
int      comm_read_line(char *buf, int max);  // saca una linea, devuelve -1 si era muy larga
void     comm_puts(const char *s);            // manda texto. ojo, no usar desde una interrupcion
int      comm_overflow(void);                 // 1 si se perdieron datos (y lo limpia)
uint32_t comm_rx_errors(void);              // caracteres perdidos/rotos en el uart desde q arranco

#endif
