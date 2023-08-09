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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "hal/io.h"
#include "mmap.h"

#include "uart.h"
#include "timer.h"

#define __weak__ __attribute__((__weak__))

__weak__ int porbor(void) 	 { return 0; };
__weak__ int efuse(void) 	 { return 0; };
__weak__ int auxadc(void)	 { return 0; };
__weak__ int sysmem(void)	 { return 0; };
__weak__ int osc32krc(void)	 { return 0; };
__weak__ int usbphy(void)	 { return 0; };
__weak__ int lowpower(void)	 { return 0; };
__weak__ int spiflash(void)	 { return 0; };
__weak__ int ioleakage(void) 	 { return 0; };

#ifndef CONFIG_ATE_DEBUG_UART

static uint8_t gpio_done = CONFIG_ATE_GPIO_DONE;
static uint8_t gpio_pass = CONFIG_ATE_GPIO_VERDICT;

static void enable_gpio_out(uint8_t gpio)
{
	uint8_t bank, shift;
	uint32_t addr, reg;

	/* Set pinmux to GPIO. */
	bank = gpio / 8;
	addr = IOMUX_BASE_ADDR + bank * 4;
	shift = (gpio - bank * 8) * 4;
	reg = readl(addr);
	reg &= ~(0xf << shift);
	reg |= (0x8 << shift);
	writel(reg, addr);

	/* Disable GPIO input. */
	addr = GPIO_BASE_ADDR + 0x0c; /* GPIO_IE */
	reg = readl(addr);
	reg &= ~(1 << gpio);
	writel(reg, addr);

	/* Enable GPIO output. */
	addr = GPIO_BASE_ADDR + 0x10; /* GPIO_OEN */
	reg = readl(addr);
	reg |= (1 << gpio);
	writel(reg, addr);
}

static void control_gpio_out(uint8_t gpio, bool high)
{
	uint32_t addr, reg;

	addr = GPIO_BASE_ADDR + 0x14; /* GPIO_OUT */
	reg = readl(addr);
	reg &= ~(1 << gpio);
	if (high)
		reg |= (1 << gpio);
	writel(reg, addr);
}

#endif

static void setup(void)
{
#ifdef CONFIG_ATE_MAX_BUS_CLK
	uint32_t a, v;

	/*
	 * 1. Set APB divider to 1/2.
	 * 2. Set AHB divider to 1/2.
	 * 3. Set N22 divider to 1.
	 * 4. Set D25 divider to 3 or 2.
	 * 5. Switch to PLL.
	 */
	a = (uint32_t)SYS(SYS_CLK_CTRL);
	v = readl(a);

	v = (v & ~(7 << 9)) | (1 << 9);
	writel(v, SYS(SYS_CLK_CTRL));

	v = (v & ~(7 << 6)) | (1 << 6);
	writel(v, SYS(SYS_CLK_CTRL));

	v = (v & ~(7 << 3)) | (0 << 3);
	writel(v, SYS(SYS_CLK_CTRL));

	v = (v & ~(7 << 0)) | (2 << 0);
	writel(v, SYS(SYS_CLK_CTRL));

	v = (v & ~(1 << 12)) | (1 << 12);
	writel(v, SYS(SYS_CLK_CTRL));

#endif
#ifndef CONFIG_ATE_DEBUG_UART
	enable_gpio_out(gpio_done);
	enable_gpio_out(gpio_pass);

	control_gpio_out(gpio_done, false);
	control_gpio_out(gpio_pass, false);
#else
    uart_init();
    uart_printf("\x1b[2J\x1b[1;1HATE Test\n");
#endif
}

static void done(void)
{
#ifndef CONFIG_ATE_DEBUG_UART
	control_gpio_out(gpio_done, true);
#else
    uart_printf("\n\nDone\n");
#endif
}

static void verdict(bool success)
{
#ifndef CONFIG_ATE_DEBUG_UART
	control_gpio_out(CONFIG_ATE_GPIO_VERDICT, success);
#else
    uart_printf("\n\n%s\n", success ? "Good" : "Bad" );
#endif
}

int main(void)
{
	uint64_t st __maybe_unused = cur_mtime();

	setup();

	if (porbor() < 0
			|| efuse() < 0
			|| auxadc() < 0
			|| sysmem() < 0
			|| osc32krc() < 0
			|| usbphy() < 0
			|| lowpower() < 0
			|| spiflash() < 0
			|| ioleakage() < 0) {
		verdict(false);
	} else
		verdict(true);

	done();

	uart_printf("\n\n%d msec elapsed.\n", MTIME_TO_USEC((uint32_t)cur_mtime() - (uint32_t)st) / 1000);

    /* do not return */
    while (1) {
        ;
    }

	return 0;
}
