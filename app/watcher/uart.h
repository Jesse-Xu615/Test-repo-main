#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>

int uart_init(uint8_t num, int baud);
char uart_rx(void);
int wc_printf(const char *format, ...);
int wc_sprintf(char *out, const char *format, ...);

#endif //_UART_H_
