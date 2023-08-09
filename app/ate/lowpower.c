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

#include "uart.h"

#ifdef CONFIG_ATE_LOWPOWER

#define PMU_DLCPD_MSR_OFFSET        0x00
#define PMU_DLCPD_MPACR_OFFSET      0x04
#define PMU_DLCPD_MPADR_OFFSET      0x08
#define PMU_DLCPD_IMR_OFFSET        0x0C
#define PMU_DLCPD_IFR_OFFSET        0x10
#define PMU_DLCPD_IOK_IFR_OFFSET    0x14
#define PMU_DLCPD_IDL_IFR_OFFSET    0x18
#define PMU_DLCPD_IDN_IFR_OFFSET    0x1C
#define PMU_DLCPD_IUO_IFR_OFFSET    0x20

#define PMU_DLCPD_IMR_PICL_OK_M     (1 << 6)
#define PMU_DLCPD_IFR_PICL_OK_F     (1 << 6)

/* WIU_ICR_1 */
#define PMU_SLEEP_OFFSET            0x06
/* WIU_ICR_2 */
#define PMU_WAKEUP_OFFSET           0x08

enum pmu_sequence_mode {
    ACTIVE_TO_DEEP_SLEEP_0              = 0,
    ACTIVE_TO_DEEP_SLEEP_0_IO_OFF       = 1,
    ACTIVE_TO_DEEP_SLEEP_1              = 2,
    ACTIVE_TO_DEEP_SLEEP_1_IO_OFF       = 3,
    ACTIVE_TO_HIBERNATION               = 4,
    ACTIVE_TO_HIBERNATION_IO_OFF        = 5,
    ACTIVE_IO_OFF_TO_LIGHT_SLEEP_IO_OFF = 6,
    ACTIVE_TO_SLEEP                     = 7,
    ACTIVE_TO_SLEEP_IO_OFF              = 8,
    ACTIVE_TO_IDLE                      = 9,
    ACTIVE_TO_IDLE_IO_OFF               = 10,
    ACTIVE_TO_LIGHT_SLEEP               = 11,
    ACTIVE_TO_ACTIVE_IO_OFF             = 12,
    ACTIVE_IO_OFF_TO_ACTIVE             = 13,
    WAKEUP_TO_ACTIVE                    = 14,
    WAKEUP_TO_ACTIVE_IO_OFF             = 15,
};

static void lowpower_pmu_write_data(u8 offset, u16 data)
{
    u32 v;

    /* mask of PICL_OK interrupt */
    v = readl(PMU_BASE_ADDR + PMU_DLCPD_IMR_OFFSET);
    v |= PMU_DLCPD_IMR_PICL_OK_M;
    writel(v, PMU_BASE_ADDR + PMU_DLCPD_IMR_OFFSET);

    v = readl(PMU_BASE_ADDR + PMU_DLCPD_IFR_OFFSET);
    if (v & PMU_DLCPD_IFR_PICL_OK_F) {
        /* if remain, clear PICL_OK_F */
        v |= PMU_DLCPD_IFR_PICL_OK_F;
        writel(v, PMU_BASE_ADDR + PMU_DLCPD_IFR_OFFSET);
    }

    /* set PICL access data */
    writel(data, PMU_BASE_ADDR + PMU_DLCPD_MPADR_OFFSET);

    /* set PICL access control */
    v = offset   | /* WIU register offset */
        (1 << 8) | /* WIU ICU index */
        (1 << 28); /* PICL access start */
    writel(v, PMU_BASE_ADDR + PMU_DLCPD_MPACR_OFFSET);

    /* wait PICL_OK */
    do {
        v = readl(PMU_BASE_ADDR + PMU_DLCPD_IFR_OFFSET);
    } while(!(v & PMU_DLCPD_IFR_PICL_OK_F));

    /* clear PICL_OK_F */
    writel(v, PMU_BASE_ADDR + PMU_DLCPD_IFR_OFFSET);

    /* unmask of PICL_OK interrupt */
    v = readl(PMU_BASE_ADDR + PMU_DLCPD_IMR_OFFSET);
    v &= ~PMU_DLCPD_IMR_PICL_OK_M;
    writel(v, PMU_BASE_ADDR + PMU_DLCPD_IMR_OFFSET);
}
#endif

int lowpower(void)
{
#ifdef CONFIG_ATE_LOWPOWER
    u32 v;

    /*
     * If bootmode is UART download mode, N22 is stayed in WFI.
     * So device can be entered sleep mode.
     */
    uart_printf("Test lowpower\n");

    /* set GPIO0 wakeup */
    /* set mode GPIO 0 */
    v = readl(GPIO_BASE_ADDR + 0x20); /* GPIO_MODE */
    v &= ~(0x0000000F);
    v |= (0x00000008);
    writel(v, GPIO_BASE_ADDR + 0x20);

    /* output disable */
    v = readl(GPIO_BASE_ADDR + 0x10); /* GPIO_OEN */
    v &= ~(1 << 0);
    writel(v, GPIO_BASE_ADDR + 0x10);

    /* input enable */
    v = readl(GPIO_BASE_ADDR + 0x0C); /* GPIO_IE */
    v |= (1 << 0);
    writel(v, GPIO_BASE_ADDR + 0x0C);

    /* edge clear */
    v = readl(GPIO_BASE_ADDR + 0x50); /* GPIO_EDGE_CLR */
    v |= (1 << 0);
    writel(v, GPIO_BASE_ADDR + 0x50); /* GPIO_EDGE_CLR */

    v = readl(GPIO_BASE_ADDR + 0x50); /* GPIO_EDGE_CLR */
    v &= ~(1 << 0);
    writel(v, GPIO_BASE_ADDR + 0x50); /* GPIO_EDGE_CLR */

    /* interrupt enable */
    v = readl(GPIO_BASE_ADDR + 0x30); /* GPIO_RISE */
    v |= (1 << 0);
    writel(v, GPIO_BASE_ADDR + 0x30); /* GPIO_RISE */

    /* active to IO off hibernation */
    lowpower_pmu_write_data(PMU_SLEEP_OFFSET, ACTIVE_TO_HIBERNATION_IO_OFF);

    /* wakeup to active  */
    lowpower_pmu_write_data(PMU_WAKEUP_OFFSET, WAKEUP_TO_ACTIVE);

    /* lowpower control */
    v = readl(SMU(LOWPOWER_CTRL));
    v |= (1 << 0);  /* set GPIO wakeup */
    v |= (1 << 14); /* enable sleep */
    writel(v, SMU(LOWPOWER_CTRL));

    /* enter WFI */
    __asm volatile( "wfi" );
#endif
    return 0;
}
