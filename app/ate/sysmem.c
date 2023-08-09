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

#include "uart.h"

#define MEMVAL(addr)  (*((uint32_t *)(addr)))

int sysmem(void)
{
#ifdef CONFIG_ATE_SYSMEM
    int i;

    uart_printf("Test sysmem\n");

    for (i = 0; i < 16384; i += 4) {
        MEMVAL(0x40007FFF - i) = (0x55550000 + i);
        MEMVAL(0x40000000 + i) = (0x55557FFF - i);
        if (MEMVAL(0x40007FFF - i) != (0x55550000 + i)) {
            uart_printf("fail high at i=%d\n", i);
            return -1;
        }
        if (MEMVAL(0x40000000 + i) != (0x55557FFF - i)) {
            uart_printf("fail low at i=%d\n", i);
            return -1;
        }
    }

    uart_printf("Test sysmem ok\n");
#endif
	return 0;
}
