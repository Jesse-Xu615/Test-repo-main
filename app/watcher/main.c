/*
 * Copyright 2021-2023 Senscomm Semiconductor Co., Ltd.	All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdint.h>
#include <stdio.h>

#include "soc.h"
#include "hal/io.h"
#include "uart.h"
#include "hal/rtc.h"

#include "pm_rram.h"

uint8_t uart_received;

void uart_int_handler(void)
{
	uart_rx();
	uart_received = 1;
}


#define GPIO_DBG

#ifdef GPIO_DBG
#define pm_gpio_out(x)  (*(uint32_t *)(GPIO_BASE_ADDR + 0x10) |= (1 << x))
#define pm_gpio_set(x)  (*(uint32_t *)(GPIO_BASE_ADDR + 0x14) |= (1 << x))
#define pm_gpio_clr(x)  (*(uint32_t *)(GPIO_BASE_ADDR + 0x14) &= ~(1 << x))

void gpio_dbg_start(void)
{
	writel(readl(GPIO_BASE_ADDR + 0x28) | 0x80000000, GPIO_BASE_ADDR + 0x28);
	writel(readl(GPIO_BASE_ADDR + 0x2C) | 0x00000008, GPIO_BASE_ADDR + 0x2C);

	pm_gpio_out(23);
	pm_gpio_set(23);
}
#else
#define pm_gpio_out(x)
#define pm_gpio_set(x)
#define pm_gpio_clr(x)
#define gpio_dbg_start(...)
#endif

extern uint32_t scm2020_wc_dtim_parser_run(uint64_t *duration, uint8_t watcher_first, uint8_t mode_flag);

#define D25_HART_ID                      0
#define N22_HART_ID                      1
#define PLIC_INT_ENABLE_REG(reg)         ( \
                                          NDS_PLIC_BASE + \
                                          PLIC_ENABLE_OFFSET + \
                                          (D25_HART_ID << PLIC_ENABLE_SHIFT_PER_TARGET) + \
                                          (reg << 2))

#define PLIC_N22_INT_ENABLE_REG(reg)    ( \
                                         NDS_PLIC_BASE + \
                                         PLIC_ENABLE_OFFSET + \
                                         (N22_HART_ID << PLIC_ENABLE_SHIFT_PER_TARGET) + \
                                         (reg << 2))

#define NEXT_WAKEUP_MARGIN               23000             // 23000us
#define FULL_BOOT_CHK_MARGIN             3                 // 3tick (90us)
#define NEXT_SCHEULE_MARGIN              USEC_TO_MTIME(90) // 90us

#define MTIME_TO_USEC(m)                 ((m) / (CONFIG_XTAL_CLOCK_HZ / CONFIG_MTIME_CLK_DIV / 1000000))
#define USEC_TO_MTIME(u)                 ((u) * (CONFIG_XTAL_CLOCK_HZ / CONFIG_MTIME_CLK_DIV / 1000000))


#define MTIME_BASE_ADDRESS               0xE6000000

#define rd_mtime()                       (*(volatile uint64_t *)(MTIME_BASE_ADDRESS))
#define wr_mtime(v)                      (*(volatile uint64_t *)(MTIME_BASE_ADDRESS) = (v))
#define wr_mtimecmp(v)                   (*(volatile uint64_t *)(MTIME_BASE_ADDRESS + 8) = (v))

#define RTC_FIXED_CONVERSTION            0

/* extern function */
extern void rtc_alarm_set(uint32_t count);
extern uint32_t rtc_time_get(void);
extern void systimer_init(void);
extern void systimer_snap_first(void);
extern void systimer_snap_second(void);

uint32_t watcher_start_rtc = 0;
static void watcher_dbg_start(void)
{
	watcher_start_rtc = rtc_time_get();
	wc_printf("\n>>> Watcher <<<\n");
}
static void watcher_dbg_time_start(void)
{
	pm_gpio_out(24);
	pm_gpio_out(6);
	if (!watcher_start_rtc)
		watcher_start_rtc = rtc_time_get();
}
static void watcher_dbg_time_end(void)
{
	watcher_start_rtc = 0;
}

uint32_t watcher_usec_to_tick(uint32_t usec)
{
#if (RTC_FIXED_CONVERSTION)
	uint64_t tick;

	/* Faster calculation but could be off 1 full tick since we do not
	 * add residual back. Adding back the residual is commented out below, but
	 * shown.
	 */

	tick = (1ULL << 32) * 32768 / 1000000 * usec;

	/* Residual for 32768 Hz. */

	tick += ((uint64_t)usec * (1526122139 + 1)) >> 32;

	return tick >> 32;
#else
	struct pm_rram_info *rram = (struct pm_rram_info *)SCM2010_PM_RRAM_INFO_ADDR;
	uint32_t tick = (uint64_t)usec * (100 * 20) / rram->ratio;
	return tick;
#endif
}

uint32_t watcher_tick_to_usec(uint32_t tick)
{
#if (RTC_FIXED_CONVERSTION)
	uint32_t usec;
	uint32_t shift;

	shift = __builtin_popcount(32768 - 1) - 6;

	usec = ((tick >> shift) * 15625) +
		(((tick & ~(~0U << shift)) * 15625) >> shift);
	return usec;
#else
	struct pm_rram_info *rram = (struct pm_rram_info *)SCM2010_PM_RRAM_INFO_ADDR;
	uint64_t usec = ((uint64_t)tick * rram->ratio / (100 * 20));
	return (uint32_t)usec;
#endif
}


static void restore_mtime(void)
{
	struct pm_rram_info *rram = (struct pm_rram_info *)SCM2010_PM_RRAM_INFO_ADDR;
	uint32_t rtc_cur;
	uint32_t rtc_elapsed;
	uint64_t mtime_elapsed;

	rtc_cur = rtc_time_get();
	if (rtc_cur >= rram->rtc_value) {
		rtc_elapsed = rtc_cur - rram->rtc_value;
	} else {
		rtc_elapsed = rtc_cur + (RTC_COUNT_MAX - rram->rtc_value);
	}

	mtime_elapsed = USEC_TO_MTIME(watcher_tick_to_usec(rtc_elapsed));

	wr_mtimecmp(UINT64_MAX);
	wr_mtime(mtime_elapsed + rram->mtime_value);
}

void set_irq(uint8_t irq, uint8_t enable)
{
	uint32_t mask;
	uint32_t offset;

	offset = (irq >> 5);

	mask = readl(PLIC_INT_ENABLE_REG(offset));
	if (enable) {
		mask |= (1 << (irq & 0x1F));
	} else {
		mask &= ~(1 << (irq & 0x1F));
	}
	writel(mask, PLIC_INT_ENABLE_REG(offset));
}

void watcher_dbg_dtim_int_ctrl(uint8_t flags)
{
#ifdef GPIO_DBG
	if (flags)
		pm_gpio_set(24);
	else
		pm_gpio_clr(24);
#endif
}

void watcher_dbg_dtim_ctrl(uint8_t flags)
{
#ifdef GPIO_DBG
	if (flags)
		pm_gpio_set(6);
	else
		pm_gpio_clr(6);
#endif
}

void run_watcher(struct pm_rram_info *rram)
{
	uint8_t watcher_first_flag = 0;
	uint8_t hiber_mode_flag = 0;
	uint32_t fullflag;
	uint64_t wakeup_duration_us;

	systimer_snap_first();

	if (!(rram->exec_watcher)) {
		watcher_first_flag = 1;
	}

	rram->exec_watcher = 1;

#ifdef CONFIG_PM_TEST
	(void)watcher_first_flag;
	(void)hiber_mode_flag;
	fullflag = 0;

	if (rram->reserved1 == 2 || rram->reserved1 == 3) {
		wakeup_duration_us = 1000 * 100; //100ms
	} else if (rram->reserved1 == 4) {
		wakeup_duration_us = 1000 * 100; //100ms

		wc_printf("full\n");
		fullflag = 1;
	}
#else
	/* in Hibernation & over flash writing, let's apply Deep sleep wakeup time */
	if (rram->pm_mode == SCM2010_PM_RRAM_MODE_HIBERNATION && (rram->sync_flag & SCM2010_PM_RRAM_SAVE_FLASH)) {
		hiber_mode_flag = 1;
	}

    /* DTIM Parser Configuration and run */
    fullflag = scm2020_wc_dtim_parser_run(&wakeup_duration_us, watcher_first_flag, hiber_mode_flag);
#endif


	systimer_snap_second();

	pm_gpio_clr(23);

	if (fullflag) {
		rram->wakeup_flag &= ~SCM2010_PM_RRAM_WAKEUP_WATCHER;
		rram->wakeup_flag |= SCM2010_PM_RRAM_WAKEUP_RESET;
		return;
	} else {
		uint64_t wakeup_time;
		uint64_t duration;
		uint32_t rtc_idle;
		uint32_t rtc_cur;
		uint32_t rtc_next;

		wakeup_time = rd_mtime() + USEC_TO_MTIME(wakeup_duration_us);

		if (wakeup_time > rram->next_mtime_value + USEC_TO_MTIME(NEXT_WAKEUP_MARGIN)) {
			rram->wakeup_flag &= ~SCM2010_PM_RRAM_WAKEUP_WATCHER;

			if (rram->next_mtime_value < rd_mtime()) {
				rram->wakeup_flag |= SCM2010_PM_RRAM_WAKEUP_RESET;
				return;
			}

			duration = MTIME_TO_USEC(rram->next_mtime_value - rd_mtime());
			rtc_idle = watcher_usec_to_tick(duration);
		} else {
			rtc_idle = watcher_usec_to_tick(wakeup_duration_us);
		}

		if (rtc_idle < FULL_BOOT_CHK_MARGIN) {
			rram->wakeup_flag &= ~SCM2010_PM_RRAM_WAKEUP_WATCHER;
			rram->wakeup_flag |= SCM2010_PM_RRAM_WAKEUP_RESET;
			return;
		}

		rtc_cur = rtc_time_get();

		if (rtc_idle > (RTC_COUNT_MAX - rtc_cur)) {
			rtc_next = rtc_idle - (RTC_COUNT_MAX - rtc_cur);
		} else {
			rtc_next = rtc_idle + rtc_cur;
		}

		rtc_alarm_set(rtc_next);
	}
}

void change_clock(void)
{
	int32_t v;

	/* Chnage core clock to PLL 160Mhz */

	v = readl(SYS(SYS_CLK_CTRL));

	/* APB DIV 2 of AHB */
	v &= ~(0x07 << 9);
	v |= 1 << 9;

	/* AHB DIV 2 of N22 */
	v &= ~(0x07 << 6);
	v |= 1 << 6;

	/* N22 DIV 1 of D25 */
	v &= ~(0x07 << 3);
	v |= 0 << 3;

	/* D25 DIV 3 (160Mhz) of PLL */
	v &= ~(0x07 << 0);
	v |= 2 << 0;

	writel(v, SYS(SYS_CLK_CTRL));

	/* ROOT CLK change to PLL(480Mhz) */
	v |= 1 << 12;
	writel(v, SYS(SYS_CLK_CTRL));
}

void wakeup_wfi(void)
{
	while (1) {
		__asm volatile ("wfi");
	}
}

int main(void)
{
	struct pm_rram_info *rram = (struct pm_rram_info *)SCM2010_PM_RRAM_INFO_ADDR;

	change_clock();

	/*
	 * if the watcher has started (reached here), it means reset vector was set to the watcher
	 * set it back to the WFI to prepare for the next wake up
	 */
	writel((uint32_t)wakeup_wfi, SMU(RESET_VECTOR(0)));

	gpio_dbg_start();
	uart_init(1, 115200);

	/* systimer initialize */
	systimer_init();

	/* mtimer retore */
	restore_mtime();

	/* Enable interrupts (MEIE) */
	set_csr(NDS_MIE, MIP_MEIP);

	/* Set Machine Status */
	set_csr(NDS_MSTATUS, MSTATUS_MIE);

	/* Watcher dbg start , Case HI, need start time, so move here */
	watcher_dbg_start();

	/*
	 * for the initial watcher execution from active
	 * if PM state is hibernation preparation
	 * then all that's required is to wait for the next RTC interrupt
	 * for the other watcher case, wakeup and run the watcher right away
	 */
	if (rram->sync_flag & SCM2010_PM_RRAM_SAVE_FLASH) {
		/* enable RTC interrupt */
		set_irq(IRQn_RTC_ALARM, 1);
		__asm volatile ("wfi");

		/* During Hibernation preparing, N22 interrupt Disable */
		/* Without it, UART Wakeup is executed twice */
		writel(readl(PLIC_N22_INT_ENABLE_REG(0)) & ~IRQn_RTC_ALARM, PLIC_N22_INT_ENABLE_REG(0));
	}

	while (1) {
		/* case of hibernation preparing,  next watcher need new dbg start time */
		watcher_dbg_time_start();

		/* run dtim parser & next wakeup */
		run_watcher(rram);

		if (uart_received) {
			/*
			 * if uart is recevied, proceed with full wakeup
			 * hoping that wise will catch the next uart interrupt
			 */
			wc_printf("UART full\n");
			rram->wakeup_flag &= ~SCM2010_PM_RRAM_WAKEUP_WATCHER;
			rram->wakeup_flag |= SCM2010_PM_RRAM_WAKEUP_RESET;
		}

		if (rram->wakeup_flag == SCM2010_PM_RRAM_WAKEUP_RESET) {

			/* the watcher decided to go to full wakeup (reset) */

			if (rram->sync_flag & SCM2010_PM_RRAM_SAVE_FLASH) {
				/* cancel hibernation, N22 will run full wakeup routine */
				rram->sync_flag |= SCM2010_PM_RRAM_SAVE_CANCEL;
				while (1) {
					__asm volatile ("wfi");
				}
			} else {
				/* reset peer, peer core will run full wakeup routine */

				if (__nds__mfsr(NDS_MHARTID) == 0) {
					writel(0x10, SYS(CORE_RESET_CTRL));
				} else {
					writel(0x01, SYS(CORE_RESET_CTRL));
				}

				while (1) {
					__asm volatile ("wfi");
				}
			}

		} else {

			/* the next wakeup type is watcher */

			if (rram->sync_flag & SCM2010_PM_RRAM_SAVE_FLASH) {
				/*
				 * when hibernation is done, it will enter low power mode
				 * when rtc interrupt occurs, it will run the watcher again
				 */
				__asm volatile ("wfi");
#ifdef CONFIG_PM_TEST
				wc_printf("Preparing HIB\n");
#endif
			} else {
				/* wfi to enter the low power mode */
#ifdef GPIO_DBG
				uint32_t val;
				/* last point to check N22 status is WFI*/
				val = readl(SMU(LOWPOWER_CTRL));
				while (!(val & 1 << (12))) {
					val = readl(SMU(LOWPOWER_CTRL));
				}
#endif
				__asm volatile ("wfi");
			}
		}

		/* clear the watcher start time for Hibernation watcher bcn rx */
		watcher_dbg_time_end();
	}

	return 0;
}
