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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "hal/kernel.h"
#include "hal/device.h"
#include "hal/timer.h"
#include "hal/console.h"
#include "hal/kmem.h"

#include "u-boot/xyzModem.h"
#include "head.h"

/* #define DEBUG */

#ifdef DEBUG
#define debug(arg ...) printf(arg)
#define error(arg ...) printf(arg)
#else
#define debug(arg ...)
#define error(arg ...)
#endif

static int getcxmodem(void) {
	int ret = getchar_timeout(0);
	if (ret >= 0)
		return (char) ret;

	return -1;
}

int download_by_ymodem(boot_hdr_t *bhdr)
{
	connection_info_t info = {
		.mode = xyzModem_ymodem,
	};
	int size = 0, err, res;
	char *buf = NULL, *dest;
	boot_hdr_t *hdr;

	buf = malloc(1024);
	if (buf == NULL) {
		error("No memory available for receiving\n");
		return -1;
	}

	dest = buf;

	if ((res = xyzModem_stream_open(&info, &err)) != 0) {
		error("%s\n", xyzModem_error(err));
		goto out;
	}

	while ((res = xyzModem_stream_read(dest, 1024, &err)) > 0) {
		if (size == 0) {

			/* Parse a boot header to find the proper destination
			 * and copy the rest to it.
			 */

			int copy = res - sizeof(*hdr);
			hdr = (boot_hdr_t *)dest;
			dest = (char *)uswap_32(hdr->vma);
			memcpy(dest, hdr + 1, copy);
			memcpy(bhdr, hdr, sizeof(*hdr));
			res = copy;
		}

		size += res;
		dest += res;
        /* Reading less than 1024 bytes is not necessarily an error. */
        err = 0;
	}

	debug("## Total Size = 0x%08x = %d Bytes\n", size, size);
 out:
	xyzModem_stream_close(&err);
	xyzModem_stream_terminate(false, getcxmodem);
	free(buf);

	return (err == 0) ? 0 : -1;
}
