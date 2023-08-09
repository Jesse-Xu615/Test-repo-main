#include <stdint.h>
#include <stdlib.h>

#include <sys/queue.h>
#include "soc.h"
#include "hal/io.h"
#include "hal/systimer.h"
#include "hal/rtc.h"
#include "uart.h"

#include <pm_rram.h>

#define PIT1_BASE_ADDR              0xF1200000
#define TIMER_INTEN                 0x14
#define TIMER_INTST                 0x18
#define TIMER_CHEN                  0x1C
#define TIMER_CHCTL(ch)             (0x20 + ch * 0x10)
#define TIMER_CHRLD(ch)             (0x24 + ch * 0x10)
#define TIMER_CHCNT(ch)             (0x28 + ch * 0x10)

#define SYS_TIMER_CFG0              (*(volatile uint32_t *)(0xF01002A8)) /* control */
#define SYS_TIMER_CFG1              (*(volatile uint32_t *)(0xF01002AC)) /* timer snap */
#define SYS_TIMER_CFG2              (*(volatile uint32_t *)(0xF01002B0)) /* rtc snap */
#define SYS_TIMER_CFG3              (*(volatile uint32_t *)(0xF01002B4)) /* clkratio */

#define SYS_TIMER_CTRL_SNAP         (1 << 0)
#define SYS_TIMER_CTRL_LOAD         (1 << 1)
#define SYS_TIMER_CTRL_SNAP_INTR    (1 << 2) /* int status: snap not set at all */
#define SYS_TIMER_CTRL_LOAD_INTR    (1 << 3) /* int status: load */
#define SYS_TIMER_CTRL_CMP_INTR     (1 << 4) /* int status: cmp */
#define SYS_TIMER_CTRL_CLR_INTR     (1 << 5)
#define SYS_TIMER_CTRL_EN           (1 << 8)
#define SYS_TIMER_CTRL_SNAP_INTR_EN (1 << 9)
#define SYS_TIMER_CTRL_LOAD_INTR_EN (1 << 10)
#define SYS_TIMER_CTRL_CMP_INTR_EN  (1 << 11)

volatile uint8_t g_snap_complete;
uint32_t g_prev_pit;
uint32_t g_prev_rtc;

extern void set_irq(uint8_t irq, uint8_t enable);

static void systimer_snap(void)
{
	g_snap_complete = 0;

	SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_SNAP_INTR_EN;
	SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_SNAP;

	while (1) {
		if (g_snap_complete) {
			break;
		}
	}
}

void systimer_int_handler(void)
{
	g_snap_complete = 1;
	SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_CLR_INTR;
}

void systimer_init(void)
{
	uint32_t status;

	writel(0x01, PIT1_BASE_ADDR + TIMER_CHCTL(0)); /* External & 32BIT */
	writel(0xFFFFFFFF, PIT1_BASE_ADDR + TIMER_CHRLD(0)); /* Counter init */
	writel(0x01, PIT1_BASE_ADDR + TIMER_CHEN); /* Enable */

	SYS_TIMER_CFG0 = SYS_TIMER_CTRL_EN;

	SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_LOAD_INTR_EN;
	SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_LOAD;

	do {
		status = SYS_TIMER_CFG0;
	} while (!(status & SYS_TIMER_CTRL_LOAD_INTR));

	SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_CLR_INTR;
	SYS_TIMER_CFG0 &= ~SYS_TIMER_CTRL_LOAD_INTR_EN;
}

void systimer_snap_first(void)
{
	set_irq(IRQn_BLE_TIMER, 1);

	systimer_snap();

	g_prev_pit = SYS_TIMER_CFG1;
	g_prev_rtc = SYS_TIMER_CFG2;
}

void systimer_snap_second(void)
{
	uint32_t now_pit;
	uint32_t now_rtc;
	uint32_t pit_diff;
	uint32_t rtc_diff;
	uint32_t ratio;
	struct pm_rram_info *rram = (struct pm_rram_info *)SCM2010_PM_RRAM_INFO_ADDR;

	/* retry until snap diff is over 500usec */
	do {
		systimer_snap();

		now_pit = SYS_TIMER_CFG1;
		now_rtc = SYS_TIMER_CFG2;

		if (now_pit >= g_prev_pit) {
			pit_diff = now_pit - g_prev_pit;
		} else {
			pit_diff = 0xFFFFFFFF - g_prev_pit + now_pit;
		}
	} while (pit_diff < (SNAP_SHORT_INTERVAL_USEC * 20));

	if (now_rtc >= g_prev_rtc) {
		rtc_diff = now_rtc - g_prev_rtc;
	} else {
		rtc_diff = RTC_COUNT_MAX - g_prev_rtc + now_rtc;
	}

	ratio = (uint64_t)pit_diff * 100 / rtc_diff;

	rram->ratio = SNAP_MOVING_AVERAGE_WEIGHT(rram->ratio, ratio);

	SYS_TIMER_CFG3 = SNAP_RATIO_TO_REG(rram->ratio);

	set_irq(IRQn_BLE_TIMER, 0);
	SYS_TIMER_CFG0 = SYS_TIMER_CTRL_EN;
}

