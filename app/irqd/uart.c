#include <stdint.h>
#include <stdlib.h>

#include "soc.h"
#include "hal/io.h"

#define UART_PORT0  0
#define UART_PORT1  1
#define UART_PORT   UART_PORT0

#if (UART_PORT == UART_PORT0)
#define UART_BASE                   UART0_BASE_ADDR
#define UART_IRQ_NUM                IRQn_UART1
#elif (UART_PORT == UART_PORT1)
#define UART_BASE                   UART1_BASE_ADDR
#define UART_IRQ_NUM                IRQn_UART2
#endif

#define OFT_ATCUART_HWC             0x10
#define OFT_ATCUART_OSC             0x14
#define OFT_ATCUART_RBR             0x20
#define OFT_ATCUART_THR             0x20
#define OFT_ATCUART_DLL             0x20
#define OFT_ATCUART_IER             0x24
#define OFT_ATCUART_DLM             0x24
#define OFT_ATCUART_IIR             0x28
#define OFT_ATCUART_FCR             0x28
#define OFT_ATCUART_LCR             0x2C
#define OFT_ATCUART_MCR             0x30
#define OFT_ATCUART_LSR             0x34
#define OFT_ATCUART_MSR             0x38
#define OFT_ATCUART_SCR             0x3C

#ifndef abs
#define abs(x) ((x) < 0 ? -(x) : (x))
#endif

int uart_init(uint8_t num, int baud)
{
	int os = readl(UART_BASE + OFT_ATCUART_OSC) & 0x1F;
	int lc = readl(UART_BASE + OFT_ATCUART_LCR);
#ifdef CONFIG_SYS_FPGA
	int freq = 20000000;
#else
	int freq = 40000000;
#endif
	int div;

#if (UART_PORT == UART_PORT0)
	/* set gpio 21 = mode 0, gpio 22 = mode 4 */
	uint32_t pmx = readl(GPIO_BASE_ADDR + 0x28);
	pmx &= ~0x0FF00000;
	writel(pmx, GPIO_BASE_ADDR + 0x28);
#elif (UART_PORT == UART_PORT1)
	/* set gpio 0 = mode 0, gpio1 = mode 0 */
	uint32_t pmx = readl(GPIO_BASE_ADDR + 0x20);
	pmx &= ~(0x000000FF);
	pmx |= 0x00000000;
	writel(pmx, GPIO_BASE_ADDR + 0x20);
#endif

	(void)num; /* TODO */

	div = freq / (baud * os);
	if (abs(div * baud * os - freq) > abs((div +1) * baud * os - freq)) {
		div++;
	}

	lc |= 0x80;
	writel(lc, UART_BASE + OFT_ATCUART_LCR);

	writel((uint8_t)(div >> 8), UART_BASE + OFT_ATCUART_DLM);
	writel((uint8_t)div, UART_BASE + OFT_ATCUART_DLL);

	lc &= ~(0x80);
	writel(lc, UART_BASE + OFT_ATCUART_LCR);

	writel(3, UART_BASE + OFT_ATCUART_LCR);
	writel(5, UART_BASE + OFT_ATCUART_FCR);

	/* UART RX Interrupt disable */
	writel(0, UART_BASE + OFT_ATCUART_IER);
	__nds__plic_disable_interrupt(UART_IRQ_NUM);

	return 0;
}

void uart_tx(char c)
{
	writel(c, UART_BASE + OFT_ATCUART_THR);
	while (1) {
		if ((readl(UART_BASE + OFT_ATCUART_LSR) & 0x60) == 0x60) {
			break;
		}
	}
}

//#define uart_tx(c) putchar(c)
#define putchar	uart_tx


#include <stdarg.h>

static void printchar(char **str, int c)
{

	if (str) {
		**str = c;
		++(*str);
	}
	else (void)putchar(c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}

	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}

static int print( char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char *)((long)va_arg( args, int ));
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

int irqd_printf(const char *format, ...)
{
	va_list args;

	va_start( args, format );
	return print( 0, format, args );
}
