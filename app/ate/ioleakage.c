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

#include "hal/io.h"
#include "mmap.h"
#include "uart.h"

#define NUM_GPIO	24

int ioleakage(void)
{
#ifdef CONFIG_ATE_IOLEAKAGE
    uint8_t gpio;
	uint8_t bank, shift;
	uint32_t addr, reg;

    uart_printf("Test ioleakage\n");

	for (gpio = 0; gpio <= NUM_GPIO; gpio++) {
		/* do not change IO for signalling GPIO */
		if (gpio == CONFIG_ATE_GPIO_DONE || gpio == CONFIG_ATE_GPIO_VERDICT) {
			continue;
		}

		/* Set pinmux to GPIO. */
		bank = gpio / 8;
		addr = IOMUX_BASE_ADDR + bank * 4;
		shift = (gpio - bank * 8) * 4;
		reg = readl(addr);
		reg &= ~(0xf << shift);
		reg |= (0x8 << shift);
		writel(reg, addr);

		/* Enable GPIO input. */
		addr = GPIO_BASE_ADDR + 0x0c; /* GPIO_IE */
		reg = readl(addr);
		reg |= (1 << gpio);
		writel(reg, addr);

		/* Disable GPIO output. */
		addr = GPIO_BASE_ADDR + 0x10; /* GPIO_OEN */
		reg = readl(addr);
		reg &= ~(1 << gpio);
		writel(reg, addr);
	}

	/* configure the IO to be input without pull-up/down */
	addr = GPIO_BASE_ADDR + 0x04; /* GPIO_PU */
	writel(0, addr);
	addr = GPIO_BASE_ADDR + 0x08; /* GPIO_PD */
	writel(0, addr);

    uart_printf("Test ioleakage ok\n");
#endif
	return 0;
}
