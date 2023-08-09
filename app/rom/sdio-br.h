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

#ifndef _SDIO_H_
#define _SDIO_H_

#include <hal/kernel.h>
#include <hal/device.h>
#include <hal/device.h>
#include <hal/types.h>

#define SDIO_BLOCK_SIZE 512

enum {
    SDIO_FN_WIFI    = 1,
    SDIO_FN_BLE     = 2,
    SDIO_FN_REV3    = 3,
    SDIO_FN_REV4    = 4,
    SDIO_FN_REV5    = 5,
    SDIO_FN_REV6    = 6,
    SDIO_FN_DFU     = 7,
    SDIO_FN_MAX,
};

struct sdio_req {
    struct list_head entry;
    void (*cmpl)(struct sdio_req *req);
    u8    fn_num; /* sdio function number */
    u8    rsvd;
    u32   len;
    u32   req_len;
    void *priv;
#ifdef CONFIG_SDIO_BOOT
    u8 buf[0];
#else
    /*
     * actually we will link the mbuf->data with this buf,
     * rather than a continuous fixed memory like SDIO BOOT does
     */
    u8 *buf;
#endif
};

struct sdio_ops {
    int (*tx_request)(struct device *dev, struct sdio_req *req);
    int (*rx_request)(struct device *dev, struct sdio_req *req);
#ifdef CONFIG_SDIO_BOOT
    int (*set_ocr)(struct device *dev, u32 ocr);
    int (*rx_ready)(struct device *dev, u8 fn_num);
    int (*dn_cmpl)(struct device *dev, u8 *cmpl);
#endif
};

#define sdio_get_ops(dev) ((struct sdio_ops *)(dev)->driver->ops)

static __inline__ int sdio_tx(struct device *dev, struct sdio_req *req)
{
    if (!dev) {
        return -ENODEV;
    }

    if (!sdio_get_ops(dev)->tx_request) {
        return -ENOSYS;
    }

    return sdio_get_ops(dev)->tx_request(dev, req);
}

static __inline__ int sdio_rx(struct device *dev, struct sdio_req *req)
{
    if (!dev) {
        return -ENODEV;
    }

    if (!sdio_get_ops(dev)->rx_request) {
        return -ENOSYS;
    }

    return sdio_get_ops(dev)->rx_request(dev, req);
}

#ifdef CONFIG_SDIO_BOOT
static __inline__ int sdio_set_ocr(struct device *dev, u32 ocr)
{
    if (!dev) {
        return -ENODEV;
    }

    if (!sdio_get_ops(dev)->set_ocr) {
        return -ENOSYS;
    }

    return sdio_get_ops(dev)->set_ocr(dev, ocr);
}

static __inline__ int sdio_rx_ready(struct device *dev, u8 fn_num)
{
    if (!dev) {
        return -ENODEV;
    }

    if (!sdio_get_ops(dev)->rx_ready) {
        return -ENOSYS;
    }

    return sdio_get_ops(dev)->rx_ready(dev, fn_num);
}

static __inline__ int sdio_download_complete(struct device *dev, u8 *cmpl)
{
    if (!dev) {
        return -ENODEV;
    }

    if (!sdio_get_ops(dev)->dn_cmpl) {
        return -ENOSYS;
    }

    return sdio_get_ops(dev)->dn_cmpl(dev, cmpl);
}
#endif

#ifdef CONFIG_SDIO_NETIF
typedef int (*trx_data_hdl_cb)(struct sdio_req *req);
#endif

/* SDIO Debugging log */
#define CONFIG_SDIO_DBG_LOG

#ifdef CONFIG_SDIO_DBG_LOG
#define sdio_dbg_log printk
#else
#define sdio_dbg_log(args...)
#endif

#endif //_SDIO_H_
