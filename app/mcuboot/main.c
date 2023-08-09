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

#include "hal/types.h"

#define TIMEOUT_IN_SEC	(CONFIG_BOOT_TIMEOUT)

__weak void mcuboot_main(void)
{
    while(1);
}

int main(void)
{
	int timeout = TIMEOUT_IN_SEC;

	 while (timeout){
		printf("\n\x1b[2J\x1b[1;1HPress any key...%d\x1b[0m\n", timeout);
		if(getchar_timeout(1000) > 0)
			return 0;
		timeout--;
	}

     mcuboot_main();

     return 0;
}
