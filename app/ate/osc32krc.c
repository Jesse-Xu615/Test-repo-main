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
#include <soc.h>

#include "hal/io.h"
#include "mmap.h"
#include "timer.h"

#include "uart.h"


/* atcrtc rtc registers */
#define OFT_RTC_32K     0x04
#define OFT_RTC_CNT     0x10
#define OFT_RTC_ALM     0x14
#define OFT_RTC_CTR     0x18
#define OFT_RTC_STS     0x1c

#define RTC_CFG         0x218 /* RTC_CFG */

#define PIT1_BASE_ADDR      0xF1200000
#define TIMER_INTEN         0x14
#define TIMER_INTST         0x18
#define TIMER_CHEN          0x1C
#define TIMER_CHCTL(ch)     (0x20 + ch * 0x10)
#define TIMER_CHRLD(ch)     (0x24 + ch * 0x10)
#define TIMER_CHCNT(ch)     (0x28 + ch * 0x10)

#define SYS_TIMER_CFG0              (*(volatile uint32_t *)(0xF01002A8)) /* control */
#define SYS_TIMER_CFG1              (*(volatile uint32_t *)(0xF01002AC)) /* timer snap */
#define SYS_TIMER_CFG2              (*(volatile uint32_t *)(0xF01002B0)) /* rtc snap */
#define SYS_TIMER_CFG3              (*(volatile uint32_t *)(0xF01002B4)) /* clkratio */
#define SYS_TIMER_COMPARE           (*(volatile uint32_t *)(0xF01002DC))
#define SYS_TIMER_COUNTER_VAL       (*(volatile uint32_t *)(0xF01002E0))

#define SYS_TIMER_CTRL_SNAP         (1 << 0)
#define SYS_TIMER_CTRL_LOAD         (1 << 1)
#define SYS_TIMER_CTRL_SNAP_INTR    (1 << 2) /* int status: snap not set at all */
#define SYS_TIMER_CTRL_LOAD_INTR    (1 << 3) /* int status: load */
#define SYS_TIMER_CTRL_CMP_INTR     (1 << 4) /* int status: cmp */
#define SYS_TIMER_CTRL_INTR_MASK    (3 << 3)
#define SYS_TIMER_CTRL_CLR_INTR     (1 << 5)
#define SYS_TIMER_CTRL_EN           (1 << 8)
#define SYS_TIMER_CTRL_SNAP_INTR_EN (1 << 9)
#define SYS_TIMER_CTRL_LOAD_INTR_EN (1 << 10)
#define SYS_TIMER_CTRL_INTR_EN      (1 << 11)

volatile int snap_try;
volatile int snap_done;
uint32_t rtc0;
uint32_t rtc1;
uint32_t pit0;
uint32_t pit1;

int osc_timer_interrupt(void)
{
    uart_printf("osc32krc interrupt %d\n", snap_try);

    /* clear the interrupt */
    SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_CLR_INTR;

    if (snap_try == 0) {
        pit0 = SYS_TIMER_CFG1;
        rtc0 = SYS_TIMER_CFG2;
    } else {
        pit1 = SYS_TIMER_CFG1;
        rtc1 = SYS_TIMER_CFG2;
    }

    snap_done = 1;

    return 0;
}

int osc_verify(void)
{
    /* calulate the ratio and verify the value against the expected range */
    #define RATIO_MIN   (int)(610.0 * 0.8) /* -20% */
    #define RATIO_MAX   (int)(610.0 * 1.4) /* +40% */

    uint64_t ratio;
    uint32_t ratio_int;
    uint32_t ratio_dec;
    uint32_t pit_diff;
    uint32_t rtc_diff;

    pit_diff = pit1 - pit0;
    rtc_diff = rtc1 - rtc0;

    ratio = (uint64_t)pit_diff * 100 / rtc_diff;
    ratio_int = (uint32_t)(ratio / 100);
    ratio_dec = (uint32_t)(ratio % 100);

    uart_printf("pit  : %d-%d=%d\n", pit1, pit0, pit_diff);
    uart_printf("rtc  : %d-%d=%d\n", rtc1, rtc0, rtc_diff);
    uart_printf("ratio: %d.%d\n", ratio_int, ratio_dec);
    uart_printf("exp  : %d~%d\n", RATIO_MIN, RATIO_MAX);

    (void)ratio_dec;

    if (ratio_int < RATIO_MAX && ratio_int > RATIO_MIN) {
        return 0;
    }

    return -1;
}

int osc32krc(void)
{
#ifdef CONFIG_ATE_32KRC
    uart_printf("Test osc32krc\n");

    /* enable global interrupt */
    set_csr(NDS_MIE, MIP_MEIP);
    set_csr(NDS_MSTATUS, MSTATUS_MIE);

	/* enable RTC APB clock */
    writel((1 << 4), SMU(RTC_CFG));

    /* BT PIT1 (20Mhz Timer) */
    writel(0x01, PIT1_BASE_ADDR + TIMER_CHCTL(0)); /* External & 32BIT */
    writel(0xFFFFFFFF, PIT1_BASE_ADDR + TIMER_CHRLD(0)); /* Counter init */
    writel(0x01, PIT1_BASE_ADDR + TIMER_CHEN); /* Enable */

    /* BT timer control */
    SYS_TIMER_COMPARE = 0;
    SYS_TIMER_CFG0 = 0;
    SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_EN;

    /* enable snap, load interrupt */
    SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_SNAP_INTR_EN | SYS_TIMER_CTRL_LOAD_INTR_EN;
    /* set default ratio */
    SYS_TIMER_CFG3 = (610 << 8) | (35 * 255 / 100);

    /* enable RTC to make SNAP/LOAD to work */
    if ((readl(RTC_BASE_ADDR + 0x18) & 0x01) == 0x00) {
        writel(0x01, RTC_BASE_ADDR + 0x18);
    }

    /* enable PLIC interrupt */
    __nds__plic_enable_interrupt(IRQn_BLE_TIMER);

    udelay(500);

    /* try snap and wait completion */
    snap_try = 0;
    snap_done = 0;
    SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_SNAP;
    while (snap_done == 0);

    udelay(500);

    /* try snap and wait completion */
    snap_try = 1;
    snap_done = 0;
    SYS_TIMER_CFG0 |= SYS_TIMER_CTRL_SNAP;
    while (snap_done == 0);

    if (osc_verify() != 0) {
        return -1;
    }

    uart_printf("Test osc32krc ok\n");

    clear_csr(NDS_MSTATUS, MSTATUS_MIE);
    clear_csr(NDS_MIE, MIP_MEIP);

#endif
    return 0;
}

