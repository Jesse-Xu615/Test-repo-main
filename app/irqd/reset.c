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

#include <stdint.h>
#include <string.h>

#include "soc.h"

extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _ldata;

static void __memcpy(void *ptr2, void *ptr1, unsigned len)
{
	uint8_t *dst = ptr2, *src = ptr1;

	if (src == dst)
		return;

	while (src < ((uint8_t *)ptr1 + len))
		*dst++ = *src++;
}

static void __memset(void *ptr, int value, size_t size)
{
    uint8_t *dest = ptr;

    while (size--)
        *dest++ = (uint8_t)value;
}

extern int main(void);

void reset_handler(void)
{
    /* Enable Misaligned access */
    set_csr(NDS_MMISC_CTL, (1 << 6));

    /* Clear Machine Status */
    clear_csr(NDS_MSTATUS, MSTATUS_MIE);

    /* Clear Machine Exception PC */
    write_csr(NDS_MEPC, 0);

    /* Clear Machine Extended Status */
    clear_csr(NDS_MXSTATUS, (0x3 << 6));

    /* clear bss */
    __memset(&_sbss, 0, (int32_t)(&_ebss) - (int32_t)(&_sbss));

    /* initialize RW variables */
    __memcpy(&_sdata, &_ldata, (int32_t)(&_edata) - (int32_t)(&_sdata));

    /* Entry function */
    main();
}

