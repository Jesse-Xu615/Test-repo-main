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

/*
 * Copyright (c) 2012-2017 Andes Technology Corporation
 * All rights reserved.
 *
 */

	.section .init, "ax"

	.global reset_vecor

reset_vector:

	.global _start
	.type _start,@function

_start:
	/* Initialize global pointer */
	.option push
	.option norelax
	la gp, __global_pointer$
	.option pop

	/* Initialize stack pointer */
	la t0, _stack
	mv sp, t0

	/* Initial machine trap-vector Base */
	la t0, trap_entry
	csrw mtvec, t0

#ifdef CONFIG_ICACHE_ENABLE
    /* Enable I-Cache */
	li t0, 0x1
	csrs mcache_ctl, t0
#endif

#ifdef CONFIG_DCACHE_ENABLE
    /* Enable D-Cache */
	li t0, 0x2
	csrs mcache_ctl, t0
#else
    /* Enable D-Cache */
	li t0, 0x2
	csrc mcache_ctl, t0
#endif

    /* Disable all interrupts (i.e. timer, external) in mie */
    csrw mie, zero

	/* System reset handler */
	call main
