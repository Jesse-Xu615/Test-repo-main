/*
 * Copyright 2023-2024 Senscomm Semiconductor Co., Ltd.	All rights reserved.
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

#define ADC_CENTER_VALUE        0x800

volatile int adc_done;

void auxadc_interrupt(void)
{
    u32 v;

    /* clear adc interrupt */
    v = readl(SYS(AUXADC_CTRL));
    v |= (1 << 16);
    writel(v, SYS(AUXADC_CTRL));

    adc_done = 1;
}

int auxadc_calibration(void)
{
    int offset_reg_val;
    int offset;
    int data;
    u32 v;

    v = readl(SYS(AUXADC_CFG));
    v &= ~(0x07);        /* VIN0 ~ VIN15 could be floating */
    v &= ~(0x3f << 8);   /* OFFSET[5:0] must be set all 0 */
    v |= 1 << 6;         /* DISH = 1: shoting VIP and VIN for offset calribration */
    v |= 1 << 7;         /* SDIF = 1: offset calibration must be done at diffrentail mode */
    writel(v, SYS(AUXADC_CFG));

    adc_done = 0;

    /* trigger adc */
    v = readl(SYS(AUXADC_CTRL));
    v |= (1 << 0);
    writel(v, SYS(AUXADC_CTRL));

    /* wait complete */
    while (adc_done == 0);

    data = readl(SYS(AUXADC_DATA(0))) & 0xFFF;

    offset = data - ADC_CENTER_VALUE;

    if (offset > 0) {
        /* 2's component minus */
        offset = offset ^ 0x1F;
        offset_reg_val = (offset + 1) & 0x1F;
        offset_reg_val |= 1 << 5;
    } else {
        /* plus */
        offset *= -1;
        offset_reg_val = offset & 0x1F;
    }

    v = readl(SYS(AUXADC_CFG));
    v &= ~(1 << 6 | 1 << 7);
    writel(v, SYS(AUXADC_CFG));

    return offset_reg_val;
}

int auxadc(void)
{
#ifdef CONFIG_ATE_AUXADC
    u16 adc_data[4];
    u32 v;
    int offset;
    int i;

    uart_printf("Test auxadc\n");

    /* enable global interrupt */
    set_csr(NDS_MIE, MIP_MEIP);
    set_csr(NDS_MSTATUS, MSTATUS_MIE);

    /* enable PLIC interrupt */
    __nds__plic_enable_interrupt(IRQn_AUXADC);

    /* set GPIO mode (1)*/
    /* set mode GPIO 0/1/2/3 */
    v = readl(GPIO_BASE_ADDR + 0x20); /* GPIO_MODE */
    v &= ~(0x0000FFFF);
    v |= (0x8888);
    writel(v, GPIO_BASE_ADDR + 0x20);

    /* output disable */
    v = readl(GPIO_BASE_ADDR + 0x10); /* GPIO_OEN */
    v &= ~(1 << 0 | 1 << 1 | 1 << 2 | 1 << 3);
    writel(v, GPIO_BASE_ADDR + 0x10);

    /* input enable */
    v = readl(GPIO_BASE_ADDR + 0x0C); /* GPIO_IE */
    v |= (1 << 0 | 1 << 1 | 1 << 2 | 1 << 3);
    writel(v, GPIO_BASE_ADDR + 0x0C);

    v = readl(SYS(AUXADC_CFG));

    /* enable ADC clock (2)*/
    v |= (1 << 3);   /* AUXADC_CLK_EN*/

    /* enable ADC clock (3)*/
    v |= (1 << 5);  /* AUXADC_ADCEN */

    /* enable offset measurement control (4)*/
    v |= (1 << 15); /* AUXADC_EN_VOBUF */

    /* set AUXADC_CFG->SDIF = 0(signled mode) (5)*/
    v &= ~(1 << 7); /* AUXADC_SDIF */

    writel(v, SYS(AUXADC_CFG));

    /* ADC single mode conversion AUXADC_MODE = 0 */
    v = readl(SYS(AUXADC_CTRL));
    v &= ~(1 << 1);
    writel(v, SYS(AUXADC_CTRL));

    /* offset calibration and set offset value (6)*/
    offset = auxadc_calibration();

    /* set ADC offset */
    v = readl(SYS(AUXADC_CFG));
    v &= ~(0x3F << 8);
    v |= (offset << 8);
    writel(v, SYS(AUXADC_CFG));

    for (i = 0; i < 4; i++) {
        adc_done = 0;

        /* set single channel AUXADC_CFG->SAIN slelected (7) */
        v = readl(SYS(AUXADC_CFG));
        v &= ~(0x07);
        v |= (4 + i);
        writel(v, SYS(AUXADC_CFG));

        /* trigger ADC sample (8) */
        v = readl(SYS(AUXADC_CTRL));
        v |= (1 << 0);
        writel(v, SYS(AUXADC_CTRL));

        /* wait complete */
        while (adc_done == 0);

        if (i == 0) {
            /* ch 4 */
            adc_data[i] = (u16)(readl(SYS(AUXADC_DATA(2))) & 0xFFF);
        } else if (i == 1) {
            /* ch 5 */
            adc_data[i] = (u16)((readl(SYS(AUXADC_DATA(2))) >> 16) & 0xFFF);
        } else if (i == 2) {
            /* ch 6 */
            adc_data[i] = (u16)(readl(SYS(AUXADC_DATA(3))) & 0xFFF);
        } else {
            /* ch 7 */
            adc_data[i] = (u16)((readl(SYS(AUXADC_DATA(3))) >> 16) & 0xFFF);
        }

        /* change single channel and repeat the above step (9) */
    }

    /* check single channel data (10) */
    for (i = 0; i < 4; i++) {
        /* 1.65V on VIN, the output range can be set to 0x7FF ~ 0x801 */
        if (adc_data[i] < 0x7ff || adc_data[i] > 0x801) {
            return -1;
        }
    }
#endif

	return 0;
}

