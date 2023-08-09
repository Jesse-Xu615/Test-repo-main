/*
* Copyright 2023-2025 Senscomm Semiconductor Co., Ltd.	All rights reserved.
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

#include "hal/compiler.h"

#include "soc.h"
#include "uart.h"
#include "sdio.h"

int main(void)
{
    uart_init(0, 115200);

    irqd_printf("irq_dispatcher started.\n");

    /* Enable interrupts (MEIE) */
    set_csr(NDS_MIE, MIP_MEIP);

    /* Set Machine Status */
    set_csr(NDS_MSTATUS, MSTATUS_MIE);

	sdio_init();

    while (1) {
		__asm volatile ("wfi");

    }

    return 0;
}
