#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>

#define MTIME_TO_USEC(m)   ((m) / (CONFIG_XTAL_CLOCK_HZ / CONFIG_MTIME_CLK_DIV / 1000000))
#define USEC_TO_MTIME(u)   ((u) * (CONFIG_XTAL_CLOCK_HZ / CONFIG_MTIME_CLK_DIV / 1000000))

extern uint64_t cur_mtime(void);
extern void udelay(uint32_t usec);

#endif //_TIMER_H_
