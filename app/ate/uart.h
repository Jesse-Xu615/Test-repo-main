#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>

#ifdef CONFIG_ATE_DEBUG_UART
int uart_init(void);
int uart_printf(const char *format, ...);
#else
#define uart_init()
#define uart_printf(...)
#endif

#endif //_UART_H_
