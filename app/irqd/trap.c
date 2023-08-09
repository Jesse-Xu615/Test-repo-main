/*
* Copyright 2023-2025 Senscomm Semiconductor Co., Ltd.    All rights reserved.
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
#include "sdio.h"

__attribute__((weak)) void mtime_handler(void)
{
    clear_csr(NDS_MIE, MIP_MTIP);
}

__attribute__((weak)) void mswi_handler(void)
{
    clear_csr(NDS_MIE, MIP_MSIP);
}

__attribute__((weak)) void syscall_handler(long n, long a0, long a1, long a2, long a3)
{
}

__attribute__((weak)) long except_handler(long cause, long epc)
{
    return epc;
}

void irqd_trap_entry(void) __attribute__ ((interrupt ("machine") , aligned(4)));
void irqd_trap_entry(void)
{
    long mcause = read_csr(NDS_MCAUSE);
    long mepc = read_csr(NDS_MEPC);
    long mstatus = read_csr(NDS_MSTATUS);
    long mxstatus = read_csr(NDS_MXSTATUS);
#ifdef __riscv_dsp
    int ucode = read_csr(NDS_UCODE);
#endif
#ifdef __riscv_flen
    int fcsr = read_fcsr();
#endif

    /* clobbers list for ecall */
#ifdef __riscv_32e
    __asm volatile("" : : :"t0", "a0", "a1", "a2", "a3");
#else
    __asm volatile("" : : :"a7", "a0", "a1", "a2", "a3");
#endif

    /* Do your trap handling */
    if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_EXT)) {
        unsigned int irq_source;
        /* Machine-level interrupt from PLIC */
        irq_source = __nds__plic_claim_interrupt();

        if (irq_source == IRQn_SDIO) {
            sdio_int_handler();
        }

        __nds__plic_complete_interrupt(irq_source);
    } else if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_TIMER)) {
        /* Machine timer interrupt */
        mtime_handler();
    } else if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_SOFT)) {
        /* Machine SWI interrupt */
        mswi_handler();
        /* Machine SWI is connected to PLIC_SW source 1 */
        __nds__plic_sw_complete_interrupt((__nds__mfsr(NDS_MHARTID) + 1));
    } else if (!(mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == TRAP_M_ECALL)) {
        /* Machine Syscal call */
        __asm volatile(
                "mv a4, a3\n"
                "mv a3, a2\n"
                "mv a2, a1\n"
                "mv a1, a0\n"
#ifdef __riscv_32e
                "mv a0, t0\n"
#else
                "mv a0, a7\n"
#endif
                "call syscall_handler\n"
                : : : "a4"
        );
        mepc += 4;
    } else {
        /* Unhandled Trap */
        mepc = except_handler(mcause, mepc);
        while (1) {
            __asm volatile( "ebreak" );
        }
    }

    /* Restore CSR */
    write_csr(NDS_MSTATUS, mstatus);
    write_csr(NDS_MEPC, mepc);
    write_csr(NDS_MXSTATUS, mxstatus);
#ifdef __riscv_dsp
    write_csr(NDS_UCODE, ucode);
#endif
#ifdef __riscv_flen
    write_fcsr(fcsr);
#endif
}

