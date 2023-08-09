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
#include "hal/efuse.h"

#include "hal/cmsis/cmsis_os2.h"

#include "sdio-br.h"

#include "head.h"

/* #define DEBUG */
#ifdef DEBUG
#define debug(arg ...) printf(arg)
#define error(arg ...) printf(arg)
#else
#define debug(arg ...)
#define error(arg ...)
#endif

#define SDIO_MAX_DATA_SIZE  2048
#define SDIO_REQUEST_SIZE   sizeof(struct sdio_req)

#define SDIO_WAIT_TIME      500 //ms

#define SDIO_BOOT_CMPLETE   1
#define SDIO_DATA_COMPLETE  2
#define SDIO_BOOT_FAIL      3

#define SDIO_OCR_FLAG       (1 << 4)
#define SDIO_OCR_MASK       (0xFFFFF)

#define _attribute_ram_sec_     __attribute__((section(".sdio_ram")))

_attribute_ram_sec_ static u8 req_buf[SDIO_MAX_DATA_SIZE + SDIO_REQUEST_SIZE];

static struct _dfu_ctx {
    boot_hdr_t hdr;
    u8 *write_ptr;
    osSemaphoreId_t dfu_sem_id;
} dfu_ctx;

static int config_ocr(u32 *ocr)
{
    struct device *dev = device_get_by_name("efuse-scm2010");
    u32 flags;

    if (!dev) {
        return -1;
    }

    flags = efuse_read(dev, 4);
    if (flags & SDIO_OCR_FLAG) {
        *ocr = efuse_read(dev, 17) & SDIO_OCR_MASK;
    } else {
        return -1;
    }

    return 0;
}

static void sdio_req_cmpl(struct sdio_req *req)
{
    struct device *dev = device_get_by_name("sdio");
    struct _dfu_ctx *dctx = &dfu_ctx;
    boot_hdr_t *h;
    u8 *src = (u8 *)req->buf;
    size_t dsz = req->len;
    u32 cur_size;

    if (dctx->hdr.magic == 0) { /* new beginning */
        h = (boot_hdr_t *)src;
        if (h->magic != uswap_32(0x48787031)) {
            /* invalid */
            error("Invalid magic:0x%x\n", h->magic);
            osSemaphoreRelease(dctx->dfu_sem_id);
            return;
        }

        memcpy(&dctx->hdr, h, sizeof(*h));
        dctx->write_ptr = (u8 *)uswap_32(dctx->hdr.vma);
        src += sizeof(*h);
        dsz -= sizeof(*h);
    }

    cur_size = (u32)dctx->write_ptr - uswap_32(dctx->hdr.vma);
    if (cur_size + dsz > uswap_32(dctx->hdr.size)) {
        dsz = uswap_32(dctx->hdr.size) - cur_size;
    }

    memcpy(dctx->write_ptr, src, dsz);
    dctx->write_ptr += dsz;

    req->len = 0;
    req->req_len = 0;
    sdio_rx(dev, req);
    sdio_rx_ready(dev, SDIO_FN_DFU);
}

int download_by_sdio(boot_hdr_t *bhdr)
{
    struct device *dev = device_get_by_name("sdio");
    struct sdio_req *req = (struct sdio_req *)req_buf;
    struct _dfu_ctx *dctx = &dfu_ctx;
    int download_complete = -1;
    u32 ocr = 0;
    osStatus_t res;

    if (!config_ocr(&ocr) && ocr) {
        sdio_set_ocr(dev, ocr);
    }

    dctx->dfu_sem_id = osSemaphoreNew(1, 0, NULL);

    req->cmpl = sdio_req_cmpl;
    req->fn_num = SDIO_FN_DFU;
    req->len = 0;
    req->req_len = 0;

    sdio_rx(dev, req);
    sdio_rx_ready(dev, SDIO_FN_DFU);

    while (1) {
        u8 cmpl = 0;

        sdio_download_complete(dev, &cmpl);

        if (cmpl == SDIO_BOOT_CMPLETE) {
            download_complete = 0;
            break;
        } else if (cmpl == SDIO_DATA_COMPLETE) {
            dctx->hdr.magic = 0;
        } else if (cmpl == SDIO_BOOT_FAIL) {
            break;
        }

        res = osSemaphoreAcquire(dctx->dfu_sem_id, SDIO_WAIT_TIME);
        if (res != osErrorTimeout) {
            break;
        }
    }

    memcpy(bhdr, &dctx->hdr, sizeof(*bhdr));

    return download_complete;
}
