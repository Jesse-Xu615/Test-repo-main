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

#include "head.h"
#include "tusb.h"
#include "tusb_main.h"

/* #define DEBUG */

#ifdef DEBUG
#define debug(arg ...) printf(arg)
#define error(arg ...) printf(arg)
#else
#define debug(arg ...)
#define error(arg ...)
#endif


static struct _dfu_ctx {
	boot_hdr_t hdr;
	u8 *write_ptr;
	osSemaphoreId_t dfu_sem_id;
} dfu_ctx;

static int dfu_prog(void *ctx, void const *buf, size_t size)
{
	struct _dfu_ctx *dctx = ctx;
	boot_hdr_t *h;
	u8 *src = (u8 *)buf;
	size_t dsz = size;

	if (dctx->hdr.magic == 0) { /* new beginning */
		h = (boot_hdr_t *)src;
		if (h->magic != uswap_32(0x48787031)) {
			/* invalid */
			error("Invalid magic:0x%x\n", h->magic);
			return 0;
		}
		memcpy(&dctx->hdr, h, sizeof(*h));
		dctx->write_ptr = (u8 *)uswap_32(dctx->hdr.vma);
		src += sizeof(*h);
		dsz -= sizeof(*h);
	}

	memcpy(dctx->write_ptr, src, dsz);
	dctx->write_ptr += dsz;

	/* return original size */
    return size;
}

static int dfu_done(void *ctx)
{
	struct _dfu_ctx *dctx = ctx;
    osStatus_t res;

    res = osSemaphoreRelease(dctx->dfu_sem_id);

    return (res == osOK ? 0 : -1);
}

extern void start_tusb(void);

int download_by_dfu(boot_hdr_t *bhdr)
{
	struct _dfu_ctx *dctx = &dfu_ctx;
	int size __maybe_unused = 0, nblk = 0;
    osStatus_t res;

    dctx->dfu_sem_id = osSemaphoreNew(1, 0, NULL);
	dctx->hdr.magic = 0;

	tud_disconnect();

	tusb_dfu_start(dctx, dfu_prog, dfu_done);

	start_tusb();

	while ((res = osSemaphoreAcquire(dctx->dfu_sem_id, osWaitForever)) == osOK) {
		nblk++;
		size = (u32)dctx->write_ptr - uswap_32(dctx->hdr.vma);
		debug("## Total Size = 0x%08x = %d Bytes in block-%d.\n", size, size, nblk);

		if (dctx->hdr.type == 0) {
			/* It's a boot block. Let's return it. */
			break;
		}
		/* Invaliddate the boot header. */
		dctx->hdr.magic = 0;
	}

	memcpy(bhdr, &dctx->hdr, sizeof(*bhdr));

	tud_disconnect();

    osSemaphoreDelete(dctx->dfu_sem_id);

	return (res == osOK) ? 0 : -1;
}
