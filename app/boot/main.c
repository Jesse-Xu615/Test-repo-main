/*
 * Copyright 2018-2019 Senscomm Semiconductor Co., Ltd.	All rights reserved.
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
#include <stdlib.h>

#include "hal/types.h"
#include "hal/spi-flash.h"

#define TIMEOUT_IN_SEC	(CONFIG_BOOT_TIMEOUT)

__weak void boot(flash_part_t *p)
{
	/* need to be done per SoC */
}

int main(void)
{
	int timeout = TIMEOUT_IN_SEC;
	flash_part_t *p;

	 while (timeout){
		printf("\n\x1b[2J\x1b[1;1HPress any key...%d\x1b[0m\n", timeout);
		if(getchar_timeout(1000) > 0)
			return 0;
		timeout--;
	}

	p = flash_lookup_partition("nuttx", NULL);

	boot(p);
	
	/* will never be reached */
	return 0;
}
