#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>

#ifdef CONFIG_IRQD_ENABLE_DEBUG
int uart_init(uint8_t num, int baud);
char uart_rx(void);
int irqd_printf(const char *format, ...);
#else
#define uart_init(...)
#define irqd_printf(...)
#endif

#endif //_UART_H_
