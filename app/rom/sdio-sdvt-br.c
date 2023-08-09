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

#include <string.h>

#include <soc.h>
#include <hal/kernel.h>
#include <hal/console.h>
#include <hal/device.h>
#include <hal/clk.h>
#include <hal/pinctrl.h>
#include <hal/timer.h>
#include "sdio-br.h"


#ifdef CONFIG_SDIO_BOOT
#define GPIO_MODE1 *((volatile unsigned int *)0xf0700024)
#define GPIO_MODE2 *((volatile unsigned int *)0xf0700028)
#else
static const char *sdio_pins[] = {"data2", "data3", "clk", "cmd", "data0", "data1"};
#endif

#define sdio_addr(device, oft) (u32 *)((device->base[0] + oft))

#define SDVT_SDIO_CFG(n) (0x2dc + (n * 4))
#define SDVT_SDIO_BASE_CFG(n) (0x304 + (n * 4))
#define SDVT_SDIO_CTRL (0x34c)

#define SDVT_SDIO_DEV_CONTROL 0x00
#define SDVT_SDIO_DEV_CCCR0 0x04
#define SDVT_SDIO_DEV_CCCR1_BLK_SIZE 0x08
#define SDVT_SDIO_DEV_CCCR2 0x0C
#define SDVT_SDIO_DEV_CCCR_BUS_SUSPEND 0x10
#define SDVT_SDIO_DEV_CCCR_STATUS 0x14
#define SDVT_SDIO_DEV_VENDOR_UNUQUE0 0x18
#define SDVT_SDIO_DEV_VENDOR_UNUQUE1 0x1C
#define SDVT_SDIO_DEV_VENDOR_UNUQUE2 0x20
#define SDVT_SDIO_DEV_VENDOR_UNUQUE3 0x24
#define SDVT_SDIO_DEV_FN1_FBR0 0x28
#define SDVT_SDIO_DEV_FN1_FBR1 0x2C
#define SDVT_SDIO_DEV_FN2_FBR0 0x30
#define SDVT_SDIO_DEV_FN2_FBR1 0x34
#define SDVT_SDIO_DEV_FN3_FBR0 0x38
#define SDVT_SDIO_DEV_FN3_FBR1 0x3C
#define SDVT_SDIO_DEV_FN4_FBR0 0x40
#define SDVT_SDIO_DEV_FN4_FBR1 0x44
#define SDVT_SDIO_DEV_FN5_FBR0 0x48
#define SDVT_SDIO_DEV_FN5_FBR1 0x4C
#define SDVT_SDIO_DEV_FN6_FBR0 0x50
#define SDVT_SDIO_DEV_FN6_FBR1 0x54
#define SDVT_SDIO_DEV_FN7_FBR0 0x58
#define SDVT_SDIO_DEV_FN7_FBR1 0x5C
#define SDVT_SDIO_DEV_FUNC_READY_DRUATION 0x60
#define SDVT_SDIO_DEV_SD_MODE_TIMING0 0x64
#define SDVT_SDIO_DEV_SD_MODE_TIMING1 0x68
#define SDVT_SDIO_DEV_T_INIT_PERIOD 0x6C
#define SDVT_SDIO_DEV_SPI_TIMING 0x70
#define SDVT_SDIO_DEV_NAC_TIMING 0x74
#define SDVT_SDIO_DEV_PULLUP_EN 0x78
#define SDVT_SDIO_DEV_IRQ_ENABLE 0x7C
#define SDVT_SDIO_DEV_IRQ_STATUS 0x80
#define SDVT_SDIO_DEV_SOC_TIMEOUT 0x84
#define SDVT_SDIO_DEV_CMD_INFO 0x88
#define SDVT_SDIO_DEV_DAT_ADDR 0x8C
#define SDVT_SDIO_DEV_DAT_BLOCK_SIZE 0x90
#define SDVT_SDIO_DEV_DAT_BLOCK_CNT 0x94
#define SDVT_SDIO_DEV_INT_INIT_PROCESS 0x98
#define SDVT_SDIO_DEV_ONE_SEC_TIMNEOUT 0x9C
#define SDVT_SDIO_DEV_RCA 0xA0
#define SDVT_SDIO_DEV_OCR 0xA4
#define SDVT_SDIO_DEV_CARD_STATUS 0xA8

#define SDVT_SDIO_IRQ_STATUS_WR_CMD_RECV (1 << 0)
#define SDVT_SDIO_IRQ_STATUS_RD_CMD_RECV (1 << 1)
#define SDVT_SDIO_IRQ_STATUS_RD_BLCOK_DONE (1 << 2)
#define SDVT_SDIO_IRQ_STATUS_RD_NEXT_BLOCK (1 << 3)
#define SDVT_SDIO_IRQ_STATUS_WR_BLOCK_READY (1 << 4)
#define SDVT_SDIO_IRQ_STATUS_XFER_FULL_DONE (1 << 5)
#define SDVT_SDIO_IRQ_STATUS_CMD_UPDATE (1 << 6)
#define SDVT_SDIO_IRQ_STATUS_CMD_ABORT (1 << 7)
#define SDVT_SDIO_IRQ_STATUS_CMD_CRC_ERROR (1 << 8)
#define SDVT_SDIO_IRQ_STATUS_DATA_CRC_ERROR (1 << 9)
#define SDVT_SDIO_IRQ_STATUS_FUNC0_CIS (1 << 10)
#define SDVT_SDIO_IRQ_STATUS_SUSPEND_ACCEPT (1 << 11)
#define SDVT_SDIO_IRQ_STATUS_RESUME_ACCEPT (1 << 12)
#define SDVT_SDIO_IRQ_STATUS_OUT_OF_RANGE (1 << 13)
#define SDVT_SDIO_IRQ_STATUS_ERASE_RESET (1 << 14)

#define SCM_SDIO_VID 0x3364
#define SCM_SDIO_PID 0x4010

#define CONFIG_SDIO_MAX_TRAN_SPEED (15000000)
/*
#define CONFIG_SDIO_RES_WORKAROUND
*/

#ifdef CONFIG_SDIO_BOOT
#define _attribute_dma_sec_ __attribute__((section(".sdio_ram")))
#else
#define _attribute_dma_sec_ __ram_dma_desc__
#endif

_attribute_dma_sec_ static u8 cis_ram[512];

struct sdvt_sdio_ctx
{
    struct device   *dev;
    struct list_head tx_desc_queue;		/* TX queue for WiFi tx processing */
    struct list_head tx_process_queue;	/* TX process queue in ISR */

	/* Multiple functions(wifi, dfu) can be used in RX */
    struct list_head rx_queue[SDIO_FN_MAX];

#ifdef CONFIG_SDIO_NETIF
    trx_data_hdl_cb  rx_handler;		/* rx pkt handler callback */
    trx_data_hdl_cb  tx_resume;			/* after tx, mbuf release callback */
#endif
#ifdef CONFIG_SDIO_BOOT
#ifdef CONFIG_SDIO_RES_WORKAROUND
    u8 connected;
#endif
#endif
};

struct sdvt_sdio_ctx sdio_ctx;

#ifdef CONFIG_SDIO_NETIF
/* temporary fixed size solution*/
/* for small size packet like Control or event packet, we will change and optimize */
#define TRX_DESC_BUF_SIZE (3 * SDIO_BLOCK_SIZE)
#define RX_DESC_BUF_NUM 15	/* number of RX Desc in Func WiFi */
#define TX_DESC_BUF_NUM 15	/* number of TX Desc in Func WiFi */

static struct sdio_req rx_desc_buffer[RX_DESC_BUF_NUM];
static struct sdio_req tx_desc_buffer[TX_DESC_BUF_NUM];

/* Reserved RX Buffer in Func WiFi */
_attribute_dma_sec_ static u8 rcv_buffer[TRX_DESC_BUF_SIZE * RX_DESC_BUF_NUM];
#endif /* CONFIG_SDIO_NETIF */

/* if Read len is under min_size(16), data can be not correct, so add padding */
#define CONFIG_SDIO_RX_MIN_ADD_PADDING
#ifdef CONFIG_SDIO_RX_MIN_ADD_PADDING
#define SDIO_RX_MIN_SIZE	16
#endif

/* These variables are required for using command */
#ifdef CONFIG_CMD_SDIO
static u32 sdio_irq_status[100];
static u32 irq_idx = 0;
#endif

static inline u32 sdvt_sdio_readl(struct device *dev, u32 offset)
{
    u32 *addr, v;

    addr = sdio_addr(dev, offset);
    v    = readl(addr);

    return v;
}

static inline void sdvt_sdio_writel(u32 val, struct device *dev, u32 offset)
{
    writel(val, sdio_addr(dev, offset));
}

static inline void sdvt_sdio_writew(u16 val, struct device *dev, u32 offset)
{
    writew(val, sdio_addr(dev, offset));
}

static inline void sdvt_sdio_writeb(u8 val, struct device *dev, u32 offset)
{
    writeb(val, sdio_addr(dev, offset));
}

static u8 sdvt_sdio_clk_cfg(void)
{
    u8 v;

    switch (CONFIG_SDIO_MAX_TRAN_SPEED) {
    case 400000:
        v = 0x48;
        break;
    case 15000000:
        v = 0x22;
        break;
    case 25000000:
        v = 0x32;
        break;
    case 50000000:
        v = 0x5B;
        break;
    default:
        v = 0x22;
        break;
    }

    return v;
}

static void sdvt_sdio_init(struct device *dev)
{
    u8 cis0[17] = {
        /* CIS 0 */
        0x21,                       // TPL_CODE_CISTPL_FUNCID
        0x02,                       // Link to next tuple
        0x0C,                       // Card function code
        0x00,                       // Not used
        0x22,                       // TPL_CODE_CISTPL_FUNCE
        0x04,                       // Link to next tuple
        0x00,                       // Extended data
        0x00,                       // Only block size function 0 can support (512)
        0x02,                       // Together with previous byte
        0x22,                       // Transfer rate (15 Mbit/sec)
        0x20,                       // TPL_CODE_CISTPL_MANFID
        0x04,                       // Link to next tuple
        SCM_SDIO_VID & 0xff,        // SDIO manufacturer code 0x0296
        (SCM_SDIO_VID >> 8) & 0xff, // Used with previous byte
        SCM_SDIO_PID & 0xff,        // Part number/revision number OEM ID = 0x5347
        (SCM_SDIO_PID >> 8) & 0xff, // Used with previous byte
        0xFF,                       // End of Tuple Chain
    };

    u8 cis[49] = {
        /* fnunction CIS */
        0x21, // TPL_CODE_CISTPL_FUNCID
        0x02, // Link to next tuple
        0x0C, // Card function type
        0x00, // Not used
        0x22, // TPL_CODE_CISTPL_FUNCE
        0x2A, // Link to next tuple
        0x01, // Type of extended data
        0x01, // Wakeup support
        0x20, // X.Y revision
        0x00, // No serial number
        0x00, // No serial number
        0x00, // No serial number
        0x00, // No serial number
        0x00, // Size of the CSA space available for this function in bytes (0)
        0x00, // Used with previous
        0x02, // Used with previous
        0x00, // Used with previous
        0x03, // CSA property: Bit 0 - 0 implies R/W capability in CSA
        // Bit 1 - 0 implies the Host may reformat the file system
        0x00, // Maximum block size (512 bytes)
        0x02, // Used with previous
        0x00, // OCR value of the function
        0x80, // Used with previous
        0xFF, // Used with previous
        0x00, // Used with previous
        0x08, // Minimum power required by this function (8 mA)
        0x0A, // ADDED => Average power required by this function when operating (10 mA)
        0x0F, // ADDED => Maximum power required by this function when operating (15 mA)
        0x01, // Stand by is not supported
        0x01, // Used with previous
        0x01, // Used with previous
        0x00, // Minimum BW
        0x00, // Used with previous
        0x00, // Optional BW
        0x00, // Used with previous
        0xE8, // Card required timeout: 1000ms
        0x03,
        0x00, // Average Power required by this function when operating. (10 mA)
        0x00,
        0x00, // Maximum Power required by this function when operating. (15 mA)
        0x00,
        0x00, // High Powermode average(High power function is not supported)
        0x00,
        0x00, // High power mode - peak power(High power function is not supported)
        0x00,
        0x00, // Low Power Mode average(Low power function is not supported)
        0x00,
        0x00, // Low Power Peak(Low power function is not supported)
        0x00,
        0xFF // End of Tuple Chain
    };

    u32 v;
    u32 cis_offset;

    /* SDIO reset and clock and AHB enable */
    *SYS(SDVT_SDIO_CTRL) = 0x013;

    /*
     * ocr
     * If set to 0xff8000, there is no response to CMD5 with
     * OCR selected.
     */

    v = readl(SDVT_SDIO_CFG(0) + SYS_BASE);
    v = (v & 0xFF000000) | 0x200000;
    writel(v, SDVT_SDIO_CFG(0) + SYS_BASE);

    /* func0 cis ptr */
    v = readl(SDVT_SDIO_CFG(1) + SYS_BASE);
    v = (v & ~0x1ffff00) | (0x1000 << 8);
    writel(v, SDVT_SDIO_CFG(1) + SYS_BASE);

    /* configure max speed */
    cis0[9] = sdvt_sdio_clk_cfg();

    /* cis0 data copy to ram */
    memcpy(cis_ram, cis0, sizeof(cis0));
    cis_offset = sizeof(cis0);

    /* func1 cis ptr */
    v = readl(SDVT_SDIO_CFG(2) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(2) + SYS_BASE);

    /* cis1 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /* func2 cis ptr */
    v = readl(SDVT_SDIO_CFG(3) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(3) + SYS_BASE);

    /* cis2 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /* func3 cis ptr */
    v = readl(SDVT_SDIO_CFG(4) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(4) + SYS_BASE);

    /* cis3 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /* func4 cis ptr */
    v = readl(SDVT_SDIO_CFG(5) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(5) + SYS_BASE);

    /* cis4 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /* func5 cis ptr */
    v = readl(SDVT_SDIO_CFG(6) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(6) + SYS_BASE);

    /* cis5 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /* func6 cis ptr */
    v = readl(SDVT_SDIO_CFG(7) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(7) + SYS_BASE);

    /* cis6 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /* func7 cis ptr */
    v = readl(SDVT_SDIO_CFG(8) + SYS_BASE);
    v = (v & ~0x1FFFF) | (0x1000 + cis_offset);
    writel(v, SDVT_SDIO_CFG(8) + SYS_BASE);

    /* cis7 data copy to ram */
    memcpy(cis_ram + cis_offset, cis, sizeof(cis));
    cis_offset += sizeof(cis);

    /*
     * func0 io base address
     * Not includes CCCR and FBR locations.
     */
    v = (u32)cis_ram - 0x1000;
    writel(v, SDVT_SDIO_BASE_CFG(0) + SYS_BASE);

    /* write RCA value 1 */
    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_RCA);
    v = v | 1;
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_RCA);

    /*
     * set bus width 4bit mode
     */
    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR0);
    v &= ~(3 << 24);
    v |= (2 << 24);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_CCCR0);

    /*
     * set card capability (support multi block transfer)
     * set high speed
     */
    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR2);
    v &= 0xFFFF0000;
    v |= 1 << 0; // card support direct command durting data (SDC)
    v |= 1 << 1; // card support multi block (SMB)
    v |= 1 << 2; // card support read wait (SRW)
    v |= 1 << 3; // card support suspend/resume (SBS)
    v |= 1 << 4; // support interrupt between block of data in 4-bit sd mode (S4MI)
    v |= 0 << 6; // low speed card (LCS)
    v |= 0 << 7; // low speed card support 4bit (4BLS)
    if (cis0[9] == 0x5B) {
        v |= 1 << 8; // high speed (SHS)
    } else {
        v |= 0 << 8; // high speed (SHS)
    }
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_CCCR2);

    /* set ncr timing */
    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_SD_MODE_TIMING0);
    v &= 0xFFFFFF00;
    v |= 0x08;
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_SD_MODE_TIMING0);

    /* ext standards SDIO & block size 512*/
    v = 0x0F | SDIO_BLOCK_SIZE << 16;
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN1_FBR0);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN2_FBR0);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN3_FBR0);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN4_FBR0);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN5_FBR0);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN6_FBR0);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_FN7_FBR0);

    sdvt_sdio_writel(0, dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);
}

static int sdvt_sdio_setup_dma(struct sdio_req *req)
{
    u32 v;

    v = (u32)req->buf;
    writel(v, SDVT_SDIO_BASE_CFG(req->fn_num * 2) + SYS_BASE);

    return 0;
}

static int sdvt_sdio_tx_request(struct device *dev, struct sdio_req *req)
{
    struct sdvt_sdio_ctx *priv         = dev->priv;
    u8                    need_trigger = 0;
    u32                   flags;

    if (!req) {
        return -1;
    }

    if (req->fn_num >= SDIO_FN_MAX) {
        return -1;
    }

    if (list_empty(&priv->tx_process_queue)) {
        need_trigger = 1;
    }

    local_irq_save(flags);
    list_add_tail(&req->entry, &priv->tx_process_queue);
    local_irq_restore(flags);

    if (need_trigger) {
        u32 v;
        v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR0);
        /* we need to make sure we are in 1-bit mode before trigger interrupt*/
        while ((v & (2 << 24)) != 0) {
            udelay(100);
            v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR0);
        }

        sdvt_sdio_setup_dma(req);

        /* write tx len */
        sdvt_sdio_writel(req->req_len, dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);

        v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR_STATUS);

        if (!(((v >> 8) & 0x0F) & (1 << req->fn_num))) {
            /* INT pending */
            v |= (1 << req->fn_num) << 8;

            sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_CCCR_STATUS);
        }
    }
    return 0;
}

static int sdvt_sdio_rx_request(struct device *dev, struct sdio_req *req)
{
    struct sdvt_sdio_ctx *priv = dev->priv;
    u32 flags;

    if (!req) {
        return -1;
    }

    if (req->fn_num >= SDIO_FN_MAX) {
        return -1;
    }

    local_irq_save(flags);
    list_add_tail(&req->entry, &priv->rx_queue[req->fn_num]);
    local_irq_restore(flags);

    return 0;
}

#ifdef CONFIG_SDIO_BOOT
static int sdvt_sdio_rx_ready(struct device *dev, u8 fn_num)
{
    u32 v;

    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);
    v |= 1 << fn_num;
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);

    return 0;
}

static int sdvt_sdio_set_ocr(struct device *dev, u32 ocr)
{
    u32 v;

    v = readl(SDVT_SDIO_CFG(0) + SYS_BASE);
    v = (v & 0xFF000000) | (ocr & 0xFFFFF) << 4;
    writel(v, SDVT_SDIO_CFG(0) + SYS_BASE);

    return 0;
}

static int sdvt_sdio_rx_ready_clear(struct device *dev, u8 fn_num)
{
    u32 v;

    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);
    v &= ~(1 << fn_num);
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);

    return 0;
}

static int sdvt_sdio_fw_dn_cmpl(struct device *dev, u8 *cmpl)
{
    u32 v;

    v     = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);
    *cmpl = (v >> 8) & 0xFF;
    v &= 0xFFFF00FF;
    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);

    return 0;
}
#endif

struct sdio_ops sdvt_sdio_ops = {
    .tx_request = sdvt_sdio_tx_request,
    .rx_request = sdvt_sdio_rx_request,
#ifdef CONFIG_SDIO_BOOT
    .rx_ready = sdvt_sdio_rx_ready,
    .set_ocr  = sdvt_sdio_set_ocr,
    .dn_cmpl  = sdvt_sdio_fw_dn_cmpl,
#endif
};

#ifdef CONFIG_SDIO_BOOT
#ifdef CONFIG_SDIO_RES_WORKAROUND
static void sdvt_sdio_res_errata(struct device *dev)
{
    u32 v;

    /*
     * Some hosts write card reset without distinction
     * between power on and re-intialize
     * SDVT IP cannot clear RES for card reset at power on in HW
     * SW clear workaround
     */

    while (1) {
        v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR0);
        if (v & (1 << 19)) {
            break;
        }
    }

    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR0);
    while (1) {
        v &= ~(1 << 19);
        sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_CCCR0);
        v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR0);
        if (!(v & (1 << 19))) {
            break;
        }
    }
}
#endif
#endif

/*
(How to Invoke interrupt for sending data to Host)
Bit[15:8] of CCCR0 register is related to INT_ENABLE. It indicates interrupt for which function was enabled.
When interrupt and interrupt pending was enabled, then the DAT LINE(o_sdio_data[1]) of bit will be driven low.
It will take care interrupt timing for write and read

(How to send data to Host)
Once Read command is received, the device will send response for that command.
1. The CMD_INFO, DATA_ADDR, DAT_BLOCK_SIZE, DAT_BLOCK_CNT AND  IRQ_ENBLE registers  will be loaded.
2. IRQ_RD_CMD_RECEIVED IRQ will be HIGH.
3. IRQ_RD_NEXT_BLOCK will be high, when DMA start internal read for data and  IRQ_RD_BLOCK_DONE will be High when that particular read data was sampled in DMA.
4. For next Block of read data, the step 3 repeat.
5. IRQ_XFER_FULL_DONE will be high when Particular read transfer is completed.
*/

/* SDIO Interrupt Service Routine */
static int sdvt_sdio_irq(int irq, void *data)
{
    struct device        *dev  = data;
    struct sdvt_sdio_ctx *priv = dev->priv;
    volatile u32         irq_status;
    struct sdio_req     *req = NULL;
    u32                 cmd_info;
    u8                  fn_num;
    u8                  write;
    u32                 v;

    irq_status = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_IRQ_STATUS);
    cmd_info   = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CMD_INFO);
    fn_num     = (cmd_info >> 8) & 0x7;
    write      = (cmd_info >> 1) & 0x1;

#ifdef CONFIG_CMD_SDIO
    if (irq_idx < 100) {
        sdio_irq_status[irq_idx++] = irq_status;
    }
#endif
    if (irq_status & SDVT_SDIO_IRQ_STATUS_WR_CMD_RECV) {
        if (!list_empty(&priv->rx_queue[fn_num])) {
            req = list_first_entry(&priv->rx_queue[fn_num], struct sdio_req, entry);

            if (req->req_len == 0) {
                /* we use a vendor register to indicate data len */
                req->req_len = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_VENDOR_UNUQUE1);
#ifdef CONFIG_SDIO_RX_MIN_ADD_PADDING
                /* if sdio read length is under min_size, sometimes data is not correct */
                if (req->req_len < SDIO_RX_MIN_SIZE)
                    req->req_len = SDIO_RX_MIN_SIZE;
#endif
                sdvt_sdio_setup_dma(req);
            }
        }
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_RD_CMD_RECV) {
    }

    /* RD_BLOCK_DONE status should be processed before XFER_FULL_DONE */
    if (irq_status & SDVT_SDIO_IRQ_STATUS_RD_BLCOK_DONE) {
        if (!list_empty(&priv->tx_process_queue)) {
            req = list_first_entry(&priv->tx_process_queue, struct sdio_req, entry);
            req->len += sdvt_sdio_readl(dev, SDVT_SDIO_DEV_DAT_BLOCK_SIZE);
        }
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_WR_BLOCK_READY) {
        if (!list_empty(&priv->rx_queue[fn_num])) {
            req = list_first_entry(&priv->rx_queue[fn_num], struct sdio_req, entry);

#ifdef CONFIG_SDIO_RX_MIN_ADD_PADDING
		if (req->req_len <= SDIO_RX_MIN_SIZE && req->len == 0)
			req->len = req->req_len;
		else
#endif
	        req->len += sdvt_sdio_readl(dev, SDVT_SDIO_DEV_DAT_BLOCK_SIZE);
        }
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_XFER_FULL_DONE) {
        if (write) {
            if (!list_empty(&priv->rx_queue[fn_num])) {
                req = list_first_entry(&priv->rx_queue[fn_num], struct sdio_req, entry);
                if (req->req_len != req->len) {
                    /* not yet complete */
                    req = NULL;
                }
            } else {
                req = NULL;
            }

#ifdef CONFIG_SDIO_BOOT
            if (req) {
                sdvt_sdio_rx_ready_clear(dev, fn_num);
            }
#endif
        } else {
            if (!list_empty(&priv->tx_process_queue)) {
                req = list_first_entry(&priv->tx_process_queue, struct sdio_req, entry);
                if (req->req_len == req->len) {
                    v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR_STATUS);
                    v &= ~((1 << fn_num) << 8);
                    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_CCCR_STATUS);
                } else {
                    /* not yet complete */
                    req = NULL;
                }
            } else {
                req = NULL;
            }
        }

        if (req) {
            list_del(&req->entry);

            if (req->cmpl)
                req->cmpl(req);
        }
        /* data still remained in tx queue list */
        if (!write) {
            if (!list_empty(&priv->tx_process_queue)) {
                req = list_first_entry(&priv->tx_process_queue, struct sdio_req, entry);
                sdvt_sdio_setup_dma(req);

                /* write tx len */
                sdvt_sdio_writel(req->req_len, dev, SDVT_SDIO_DEV_VENDOR_UNUQUE0);

                v = sdvt_sdio_readl(dev, SDVT_SDIO_DEV_CCCR_STATUS);

                if (!(((v >> 8) & 0x0F) & (1 << req->fn_num))) {
                    /* INT pending */
                    v |= (1 << req->fn_num) << 8;

                    sdvt_sdio_writel(v, dev, SDVT_SDIO_DEV_CCCR_STATUS);
                }
            }
        }
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_RD_NEXT_BLOCK) {
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_CMD_UPDATE) {
#ifdef CONFIG_SDIO_BOOT
#ifdef CONFIG_SDIO_RES_WORKAROUND
        if (!priv->connected) {
            priv->connected = 1;
        }
#endif
#endif
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_CMD_ABORT) {
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_CMD_CRC_ERROR) {
#ifdef CONFIG_SDIO_BOOT
#ifdef CONFIG_SDIO_RES_WORKAROUND
        if (!priv->connected) {
            priv->connected = 1;
            sdvt_sdio_res_errata(dev);
        }
#endif
#endif
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_FUNC0_CIS) {
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_SUSPEND_ACCEPT) {
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_RESUME_ACCEPT) {
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_OUT_OF_RANGE) {
    }

    if (irq_status & SDVT_SDIO_IRQ_STATUS_ERASE_RESET) {
    }

    sdvt_sdio_writel(irq_status, dev, SDVT_SDIO_DEV_IRQ_STATUS);

    return 0;
}

#ifdef CONFIG_SDIO_BOOT
static void sdvt_sdio_set_pin(void)
{
    /* sdio data3[15] */
    GPIO_MODE1 &= 0x0FFFFFFF;
    GPIO_MODE1 |= 0x1 << 28;

    /* sdio data3[16], clk[17], cmd[18], data0[19], data1[20] */
    GPIO_MODE2 &= 0xFFF00000;
    GPIO_MODE2 |= 0x11111;
}
#endif

#ifdef CONFIG_SDIO_NETIF

/* TX callback function */
static void sdio_resume_tx_desc(struct sdio_req *req)
{
    struct device        *dev  = sdio_ctx.dev;
    struct sdvt_sdio_ctx *priv = dev->priv;

    req->len     = 0;
    req->req_len = 0;

    if (sdio_ctx.tx_resume)
        sdio_ctx.tx_resume(req);

    list_add_tail(&req->entry, &priv->tx_desc_queue);
}

/* RX callback function */
/* Transfering Data to RX handler */
static void sdio_rx_complete(struct sdio_req *req)
{
    if (sdio_ctx.rx_handler)
        sdio_ctx.rx_handler(req);

    return;
}

/* Release SDIO RX buffer desc, attach rx queue again */
int sdio_resume_rx_desc(struct sdio_req *req)
{
    struct device        *dev  = sdio_ctx.dev;
    struct sdvt_sdio_ctx *priv = dev->priv;
    u32                   flags;

    if (!req) {
        return -1;
    }

    if (req->fn_num >= SDIO_FN_MAX) {
        return -1;
    }

    memset(req->buf, 0, req->len);

    req->len     = 0;
    req->req_len = 0;
    req->fn_num  = SDIO_FN_WIFI;
    req->cmpl    = sdio_rx_complete; /* RX Callback functioin */

    local_irq_save(flags);
    list_add_tail(&req->entry, &priv->rx_queue[req->fn_num]);
    local_irq_restore(flags);

    return 0;
}

void sdio_tx_process(u8 *tx_buf, int tx_len, void *private, u8 rsvd)
{
    struct device        *dev = sdio_ctx.dev;
    struct sdio_req      *tx_desc_handle;
    struct sdvt_sdio_ctx *priv = dev->priv;

    /* to get the available tx descriptor */
    while (list_empty(&priv->tx_desc_queue)) {
        udelay(2);
    }

    tx_desc_handle = list_first_entry(&priv->tx_desc_queue, struct sdio_req, entry);
    list_del(&tx_desc_handle->entry);

    tx_desc_handle->len     = 0;
    tx_desc_handle->req_len = tx_len;
    tx_desc_handle->buf     = tx_buf;
    tx_desc_handle->priv    = private;
    tx_desc_handle->rsvd    = rsvd;

    /* attach SDIO TX Queue */
    sdio_tx(dev, tx_desc_handle);
}

/* RX desc initialize and attach rx queue list */
static void sdio_init_rx_desc(struct device *dev)
{
    struct sdio_req      *rx_desc_handle;
    struct sdvt_sdio_ctx *priv = dev->priv;

    for (int i = 0; i < RX_DESC_BUF_NUM; i++) {
        rx_desc_handle = &rx_desc_buffer[i];
        /* We use fixed size DMA buffer in RX buffer desc */
        rx_desc_handle->buf     = &rcv_buffer[i * TRX_DESC_BUF_SIZE];
        rx_desc_handle->fn_num  = SDIO_FN_WIFI;
        rx_desc_handle->len     = 0;
        rx_desc_handle->req_len = 0;
        rx_desc_handle->cmpl    = sdio_rx_complete; /* RX Callback functioin */
        list_add_tail(&rx_desc_handle->entry, &priv->rx_queue[SDIO_FN_WIFI]);
    }
    return;
}

/* TX desc initialize and info is cleared */
static void sdio_init_tx_desc(struct device *dev)
{
    struct sdio_req      *tx_desc_handle;
    struct sdvt_sdio_ctx *priv = dev->priv;

    for (int i = 0; i < TX_DESC_BUF_NUM; i++) {
        tx_desc_handle          = &tx_desc_buffer[i];
        tx_desc_handle->fn_num  = SDIO_FN_WIFI;
        tx_desc_handle->len     = 0;
        tx_desc_handle->req_len = 0;
        tx_desc_handle->cmpl    = sdio_resume_tx_desc; /* TX Callback functioin */

        list_add_tail(&tx_desc_handle->entry, &priv->tx_desc_queue);
    }

    return;
}

void sdio_rx_handler_register(trx_data_hdl_cb cb)
{
    sdio_ctx.rx_handler = cb;
}

void sdio_tx_resume_register(trx_data_hdl_cb cb)
{
    sdio_ctx.tx_resume = cb;
}

#endif /* CONFIG_SDIO_NETIF */

static int sdvt_sdio_probe(struct device *dev)
{
#ifndef CONFIG_SDIO_BOOT
    struct pinctrl_pin_map *pmap[6];
#endif
    int ret = 0;
    int i;

#ifdef CONFIG_SDIO_BOOT
	sdvt_sdio_set_pin();
#else
    for (i = 0; i < ARRAY_SIZE(sdio_pins); i++) {
        pmap[i] = pinctrl_lookup_platform_pinmap(dev, sdio_pins[i]);
        if (pmap[i] == NULL) {
            goto free_pin;
        }

        if (pinctrl_request_pin(dev, pmap[i]->id, pmap[i]->pin) < 0) {
            goto free_pin;
        }
    }
#endif
    memset(&sdio_ctx, 0, sizeof(struct sdvt_sdio_ctx));
    sdio_ctx.dev = dev;

    INIT_LIST_HEAD(&sdio_ctx.tx_desc_queue);
    INIT_LIST_HEAD(&sdio_ctx.tx_process_queue);
    for (i = 1; i < SDIO_FN_MAX; i++) {
        INIT_LIST_HEAD(&sdio_ctx.rx_queue[i]);
    }

    dev->priv = &sdio_ctx;

#ifdef CONFIG_SDIO_NETIF
    sdio_init_rx_desc(dev);
    sdio_init_tx_desc(dev);
#endif
    sdvt_sdio_init(dev);

    ret = request_irq(dev->irq[0], sdvt_sdio_irq, dev_name(dev), dev->pri[0], dev);
    if (ret) {
        sdio_dbg_log("%s irq req is failed(%d)\n", __func__, ret);
        return -1;
    }

    sdio_dbg_log("sdio driver initialized\n");

    return 0;

#ifndef CONFIG_SDIO_BOOT
free_pin:
    sdio_dbg_log("sdio pin err\n");
    for (; i >= 0; i--) {
        if (pmap[i] && pmap[i]->pin != -1)
            gpio_free(dev, pmap[i]->id, pmap[i]->pin);
    }
#endif

    return -EBUSY;
}

static declare_driver(sdio) = {
    .name  = "sdio",
    .probe = sdvt_sdio_probe,
    .ops   = &sdvt_sdio_ops,
};

#ifdef CONFIG_CMD_SDIO
#include <cli.h>
#include <stdio.h>

void sdio_show_status(void)
{
    struct sdio_req *tx_desc_handle;
    struct sdio_req *rx_desc_handle;
    int              i;

    printf("[SDIO] IRQ Status register\n");
    for (i = 0; i < irq_idx; i++) {
        printf("%08x\n", sdio_irq_status[i]);
    }

    irq_idx = 0;

    for (int i = 0; i < TX_DESC_BUF_NUM; i++) {
        tx_desc_handle = (struct sdio_req *)(&tx_desc_buffer[i]);
        printf("%d tx_desc(%p) len(%d) req_len(%d)\n", i, tx_desc_handle, tx_desc_handle->len, tx_desc_handle->req_len);
    }
    for (int i = 0; i < RX_DESC_BUF_NUM; i++) {
        rx_desc_handle = (struct sdio_req *)(&rx_desc_buffer[i]);
        printf("%d rx_desc(%p) len(%d) req_len(%d)\n", i, rx_desc_handle, rx_desc_handle->len, rx_desc_handle->req_len);
    }
}

int do_read_sdio_reg(int argc, char *argv[])
{
    u32 v;
    u32 i;

    printf("[SDIO] register\n");
    for (i = 0xe0a00000; i <= (0xe0a00000 + 0xa8); i += 4) {
        v = readl(i);

        printf("[0x%08x]0x%08x\n", i - 0xe0a00000, v);
    }
    return CMD_RET_SUCCESS;
}

#endif
