#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "soc.h"
#include "hal/io.h"
#include "uart.h"

#include "irqd.h"

#define SREG_CCCR_STATUS 		(0x14)
#define SREG_VENDOR_UNIQUE2 	(0x20)
#define SREG_IRQ_STATUS 		(0x80)
#define SREG_CMD_INFO 			(0x88)
#define SREG_DAT_ADDR 			(0x8c)
#define SREG_DAT_BLOCK_SIZE 	(0x90)
#define SREG_DAT_BLOCK_CNT 		(0x94)

#define IRQ_RD_BLOCK_DONE			(1 << 2)
#define IRQ_XFER_FULL_DONE			(1 << 5)
#define IRQ_CMD_UPDATE				(1 << 6)
#define IRQ_CMD_CRC_ERROR			(1 << 8)
#define IRQ_DAT_CRC_ERROR			(1 << 9)

#define INT_PENDING_SHFT			(8)
#define INT_PENDING_MASK			(0x0000ff00)

#define CMD_RW_SHFT					(1)
#define CMD_RW_MASK					(0x00000002)
#define CMD_IDX_SHFT				(2)
#define CMD_IDX_MASK				(0x000000fc)
#define CMD_FUNC_NUM_SHFT			(8)
#define CMD_FUNC_NUM_MASK			(0x00000700)

#define SDIO_CMD53_CMD_IDX			(0x35) /* IO_RW_EXTENDED Command(CMD53) 110101b */

#define rbfm(f) 					(f##_MASK) 	/* mask */
#define rbfs(f) 					(f##_SHFT) 	/* shift */
#define rbfset(v, f, x) 			((v) |= ((x) << rbfs(f)) & rbfm(f))
#define rbfclr(v, f) 				((v) &= ~rbfm(f))
#define rbfmod(v, f, x) 			{rbfclr(v, f); rbfset(v, f, x);}
#define rbfget(v, f) 				(((v) & rbfm(f)) >> rbfs(f))
#define rbfzero(v) 					((v) = 0)

#define DMB	asm volatile ("" : : : "memory")
#define dmb()	DMB

extern char  __fifo_start[];

volatile static sdvt_tx_chain_t *g_txchain;
volatile static sdvt_interrupt_fifo_t *global_int_fifo;
static sdvt_interrupt_fifo_t int_fifo; /* local copy */

static u32 read_sreg(u32 reg)
{
	u32 addr = (u32)SDIO_BASE_ADDR + reg;
	u32 v = readl(addr);

	return v;
}

static void write_sreg(u32 val, u32 reg)
{
	u32 addr = (u32)SDIO_BASE_ADDR + reg;
	writel(val, addr);
}

static int sdio_add_to_fifo(u16 status, u16 cmdinfo, u32 blksz, u32 blkcnt)
{
	sdvt_interrupt_fifo_t *fifo = &int_fifo;
	sdvt_interrupt_t *ielem;

	fifo->cons_idx = global_int_fifo->cons_idx;

	/* Indices must be converted to u8. */
    if ((u8)(fifo->prod_idx) == (u8)(fifo->cons_idx - 1))
      return -1;

	ielem = &fifo->ielems[(u8)fifo->prod_idx];
	ielem->irq_status = status;
	ielem->cmd_info = cmdinfo;
	ielem->data_blk_sz = blksz;
	ielem->data_blk_cnt = blkcnt;

	fifo->prod_idx++;

	global_int_fifo->prod_idx = fifo->prod_idx;

	__nds__plic_sw_set_pending(IRQD_SW_INT);

	return 0;
}

#if defined(IRQ_DBG) || defined(CONFIG_SDIO_OOB_GPIO_INT)
void control_gpio_out(uint8_t gpio, uint8_t high)
{
	uint32_t addr, reg;

	addr = GPIO_BASE_ADDR + 0x14; /* GPIO_OUT */
	reg  = readl(addr);
	reg &= ~(1 << gpio);
	if (high)
		reg |= (1 << gpio);
	writel(reg, addr);
}
#endif

#ifdef IRQ_DBG
static void enable_gpio_out(uint8_t gpio)
{
	uint8_t  bank, shift;
	uint32_t addr, reg;

	/* Set pinmux to GPIO. */
	bank  = gpio / 8;
	addr  = IOMUX_BASE_ADDR + bank * 4;
	shift = (gpio - bank * 8) * 4;
	reg   = readl(addr);
	reg &= ~(0xf << shift);
	reg |= (0x8 << shift);
	writel(reg, addr);

	/* Disable GPIO input. */
	addr = GPIO_BASE_ADDR + 0x0c; /* GPIO_IE */
	reg  = readl(addr);
	reg &= ~(1 << gpio);
	writel(reg, addr);

	/* Enable GPIO output. */
	addr = GPIO_BASE_ADDR + 0x10; /* GPIO_OEN */
	reg  = readl(addr);
	reg |= (1 << gpio);
	writel(reg, addr);
}

#ifdef MATCH_IRQ_DEBUG
uint8_t g_sdio_cmdupdate_init = 0;

#define SDIO_DEBUG_CMDUPDATE  \
		if (irq_status & IRQ_CMD_UPDATE) {  \
			control_gpio_out(23, 1);  \
			if (((cmd_info & CMD_IDX_MASK) >> CMD_IDX_SHFT) == SDIO_CMD53_CMD_IDX) {  \
				if (g_sdio_cmdupdate_init != 0) {  \
					control_gpio_out(24, 1);  \
					while (1);  \
				}  \
				g_sdio_cmdupdate_init = 1;  \
			}  \
		}

#define SDIO_DEBUG_FULLDONE  \
	if (irq_status & IRQ_XFER_FULL_DONE) {  \
		control_gpio_out(23, 0);  \
		if (((cmd_info & CMD_IDX_MASK) >> CMD_IDX_SHFT) == SDIO_CMD53_CMD_IDX) {  \
			if (g_sdio_cmdupdate_init != 1) {  \
				control_gpio_out(24, 1);  \
				while (1);  \
			}  \
			g_sdio_cmdupdate_init = 0;  \
		}  \
	}

#define SDIO_CRC_ERR_TRIG
#else
#define SDIO_DEBUG_CMDUPDATE  \
	if (irq_status & IRQ_CMD_UPDATE) {  \
		control_gpio_out(23, 1);  \
	}

#define SDIO_DEBUG_FULLDONE  \
	if (irq_status & IRQ_XFER_FULL_DONE) {  \
		control_gpio_out(23, 0);  \
	}

#define SDIO_CRC_ERR_TRIG  \
	if (irq_status & (IRQ_CMD_CRC_ERROR | IRQ_DAT_CRC_ERROR)) {  \
		control_gpio_out(24, 1);  \
	}
#endif
#else
#define SDIO_DEBUG_CMDUPDATE
#define SDIO_DEBUG_FULLDONE
#define SDIO_CRC_ERR_TRIG
#endif

void sdio_init(void)
{
	g_txchain = (sdvt_tx_chain_t *)__fifo_start;
	g_txchain->cons_idx = g_txchain->cnt = 0;

	global_int_fifo = (sdvt_interrupt_fifo_t *)(g_txchain + 1);
	global_int_fifo->prod_idx = global_int_fifo->cons_idx = 0;
	global_int_fifo->ielems = (sdvt_interrupt_t *)(global_int_fifo + 1);

	int_fifo.prod_idx = int_fifo.cons_idx = 0;
	int_fifo.ielems = global_int_fifo->ielems;

#ifdef IRQ_DBG
	enable_gpio_out(23);
	enable_gpio_out(24);
	
	control_gpio_out(23, 0);
	control_gpio_out(24, 0);
#endif

	/* Enable local SDIO interrupt. */
	__nds__plic_enable_interrupt(IRQn_SDIO);
}

static bool write = false;
static u16 txcons_len = 0;

void sdio_int_handler(void)
{
	u32 v __maybe_unused;
	u32 irq_status;
	u32 cmd_info;
	u32 data_blk_sz;
	u32 data_blk_cnt;
	bool cmd53;

	do {
		irq_status = read_sreg(SREG_IRQ_STATUS);
		if (!irq_status)
			break;

/*
		if (__builtin_popcount(irq_status) > 2)
			irqd_printf("?");
*/

		write_sreg(irq_status, SREG_IRQ_STATUS);

		cmd_info = read_sreg(SREG_CMD_INFO);
		data_blk_sz = read_sreg(SREG_DAT_BLOCK_SIZE);
		data_blk_cnt = read_sreg(SREG_DAT_BLOCK_CNT);

		if (irq_status & IRQ_CMD_UPDATE) {
			write = rbfget(cmd_info, CMD_RW) ? true : false;
			cmd53 = (rbfget(cmd_info, CMD_IDX) == SDIO_CMD53_CMD_IDX) ? true : false;
			if (!write && cmd53) {
#ifdef CONFIG_SDIO_OOB_GPIO_INT
				control_gpio_out(CONFIG_SDIO_OOB_GPIO_INT_PIN, 0);
#else
				u8 int_pending;
				u8 fn_num;

				fn_num = rbfget(cmd_info, CMD_FUNC_NUM);
				v = read_sreg(SREG_CCCR_STATUS);
				int_pending = rbfget(v, INT_PENDING);
				int_pending &= ~(1 << fn_num);
				rbfmod(v, INT_PENDING, int_pending);
				write_sreg(v, SREG_CCCR_STATUS);
#endif
			}
		}
		/* RD_BLOCK_DONE status should be processed before XFER_FULL_DONE */
		if (irq_status & IRQ_RD_BLOCK_DONE) {
			txcons_len += data_blk_sz;
			irq_status &= ~IRQ_RD_BLOCK_DONE;
		}
		if (!write && (irq_status & IRQ_XFER_FULL_DONE)) {
			if (txcons_len < g_txchain->txelems[g_txchain->cons_idx].len) {
				irq_status &= ~IRQ_XFER_FULL_DONE;
			} else {
				g_txchain->cons_idx++;
				txcons_len = 0;
				if (g_txchain->cons_idx < g_txchain->cnt) {
					writel(g_txchain->txelems[g_txchain->cons_idx].addr, g_txchain->dma_addr);
					dmb();
#ifdef CONFIG_SDIO_OOB_GPIO_INT
					control_gpio_out(CONFIG_SDIO_OOB_GPIO_INT_PIN, 1);
#else
					/* INT pending */
					v = read_sreg(SREG_CCCR_STATUS);
					rbfset(v, INT_PENDING, (1 << g_txchain->fn));
					write_sreg(v, SREG_CCCR_STATUS);
#endif
					irq_status &= ~IRQ_XFER_FULL_DONE;
				}
			}
		}

		SDIO_DEBUG_CMDUPDATE
		SDIO_DEBUG_FULLDONE
		SDIO_CRC_ERR_TRIG

		if (irq_status) {
			sdio_add_to_fifo(irq_status, cmd_info, data_blk_sz, data_blk_cnt);
		}
	} while (irq_status);
}
