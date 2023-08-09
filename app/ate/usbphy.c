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

#include "mmap.h"
#include "timer.h"
#include "uart.h"

//#define TEST_HS

/* POWER */
#define SOFT_CON	(1 << 6)

/* TESTMODE */
#define FORCE_FS 	(1 << 5)
#define FORCE_HS 	(1 << 4)
#define TEST_K 		(1 << 2)
#define TEST_J 		(1 << 1)

volatile uint8_t *power = (volatile uint8_t *)(USB_BASE_ADDR + 0x01);
volatile uint8_t *testm = (volatile uint8_t *)(USB_BASE_ADDR + 0x0F);

int usbphy(void)
{
#ifdef CONFIG_ATE_USBPHY
    uart_printf("Test USB PHY\n");

	/* Connect to the bus. */
#ifdef TEST_HS
	*testm = FORCE_HS;
#else
	*testm = FORCE_FS;
#endif
	*power = *power | SOFT_CON;

	udelay(5000);

    uart_printf("Force K\n");

	*testm = TEST_K;

	udelay(5000);

	uart_printf("Force J\n");

	*testm = TEST_J;

	udelay(5000);

	/* Disconnect from the bus. */
	*power = *power & ~SOFT_CON;

#endif
	return 0;
}

