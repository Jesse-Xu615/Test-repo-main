/*
* Copyright 2021-2023 Senscomm Semiconductor Co., Ltd.    All rights reserved.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <stdint.h>

#include "soc.h"

#define __weak__ __attribute__((__weak__))

__weak__ void osc_timer_interrupt(void) {};
__weak__ void auxadc_interrupt(void) 	{};

void trap_entry(void) __attribute__ ((interrupt ("machine") , aligned(4)));
void trap_entry(void)
{
#ifdef CONFIG_ATE_SUPPORT_INT
    volatile long mcause = read_csr(NDS_MCAUSE);

    if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_EXT)) {

        unsigned int irq_source;
        irq_source = __nds__plic_claim_interrupt();

        if (irq_source == IRQn_BLE_TIMER) {
            osc_timer_interrupt();
        }

        if (irq_source == IRQn_AUXADC) {
            auxadc_interrupt();
        }

        __nds__plic_complete_interrupt(irq_source);
    }
#endif
}

