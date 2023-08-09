/*
* Copyright 2021-2023 Senscomm Semiconductor Co., Ltd.    All rights reserved.
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
#include <stdint.h>
#include <stdlib.h>

#include "soc.h"
#include "hal/io.h"
#include "uart.h"
#include "dtimparse.h"

#include "pm_watcher.h"

extern uint32_t rtc_time_get(void);

#define CONFIG_FREEBSD_MBUF_CACHE_SIZE 1024
#define CONFIG_FREEBSD_MBUF_CHUNK_SIZE 1096

#include "scm2020_shmem.h"
#include "ieee80211.h"
#include "mbuf.h"

/* Average log delay */
#define WATCHER_LOG_DELAY_TIME (10 * 1000)

/* Need additional seep time case of NO BCN */
#define WATCHER_NOBCN_ADDITIONAL_SLEEP_TIME (2500)

/* Need more waiting, Hibernation & First more waiting Beacon */
#define WATCHER_HIB_FIRST_WAKEUP_MORE_WAIT 10

/* Log enable and First , more waiting Beacon */
#define WATCHER_LOG_FIRST_WAKEUP_MORE_WAIT 5

static u8 watcher_macdtim_interrupt_cnt = 0;
static u8 watcher_txdone_interrupt_cnt = 0;

/* If packet has wrong and check it, enable */
/* #define RX_PACKET_DUMP */
static u8 watcher_rxbuf_interrupt_cnt = 0;

/* For checking RX interrupt, enable in advance */
static bool watcher_rxbuf_int_proc = 0;

static u8 watcher_tx_pkt_type = 0;
static void scm2020_ps_make_arp_resp(uint8_t *arp_header);
static void inline scm2020_wc_txdone(struct watcher_tx_pkt_ctx *tx_pkt, u8 pkt_type, u32 tx_start);

static void (*wc_crypto_setiv)(uint8_t *ivp) = NULL;

extern void watcher_dbg_dtim_int_ctrl(uint8_t flags);
extern void watcher_dbg_dtim_ctrl(uint8_t flags);
void set_wlan_irq(u8 enable)
{
	if (enable)
		__nds__plic_enable_interrupt(IRQn_WLAN0);
	else
		__nds__plic_disable_interrupt(IRQn_WLAN0);
}

u8 get_watcher_logflag(void)
{
	return WATCHER_LOG_DISFLAG;
}

static void __inline wc_udelay(u32 udelay)
{
	u32 now = wc_ktime();

	while (wc_ktime() < (now + udelay))
		asm("nop");
}

static inline u32 wc_mac_readl(u32 offset)
{
	return readl(MAC_BASE_START + MAC_REG_OFFSET + offset);
}

static inline void wc_mac_writel(u32 val, u32 offset)
{
	writel(val, MAC_BASE_START + MAC_REG_OFFSET + offset);
}

static inline u32 wc_phy_readl(u8 BLK_OFFSET, u16 offset)
{
	return readl(PHY_BASE_START + PHY_BLK_OFFSET(BLK_OFFSET) + offset);
}

static inline void wc_phy_writel(u32 val, u8 BLK_OFFSET, u16 offset)
{
	writel(val, PHY_BASE_START + PHY_BLK_OFFSET(BLK_OFFSET) + offset);
}

#ifdef PM_SOC_QFN40_FIB
static inline void wc_phy_init(void)
{
/* RFIF control signal assignment */
#define RFI_CTRLSIG_SHDN	RFI_CTRLSIG_CFG(0)
#define RFI_CTRLSIG_2G_PA_EN	RFI_CTRLSIG_CFG(1)
#define RFI_CTRLSIG_5G_PA_EN	RFI_CTRLSIG_CFG(2)
#define RFI_CTRLSIG_EXT_SW_RX	RFI_CTRLSIG_CFG(3)
#define RFI_CTRLSIG_EXT_SW_TX	RFI_CTRLSIG_CFG(4)
#define RFI_CTRLSIG_RF_RX_EN	RFI_CTRLSIG_CFG(5)
#define RFI_CTRLSIG_RF_TX_EN	RFI_CTRLSIG_CFG(6)
#define RFI_CTRLSIG_ADC		RFI_CTRLSIG_CFG(7)
#define RFI_CTRLSIG_DAC		RFI_CTRLSIG_CFG(8)
#define RFI_CTRLSIG_RX_EN	RFI_CTRLSIG_CFG(12)
#define RFI_CTRLSIG_TX_EN	RFI_CTRLSIG_CFG(13)
#define RFI_CTRLSIG_RX_CLK	RFI_CTRLSIG_CFG(14)
#define RFI_CTRLSIG_TX_CLK	RFI_CTRLSIG_CFG(15)

	/* PHY Control mode */

	wc_phy_writel(0x00000000, PHY_RFITF, FW_RFI_EN);

	/* SHDN: - ctrl[0], always on */
	/* SHDN: - ctrl[0], always on */
	wc_phy_writel(RFCTRL(1, HIGH, ALWAYS, TXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_SHDN);
	/* 2G PA En:1 - ctrl[1], during TXVALID */
	wc_phy_writel(RFCTRL(1, HIGH, TXVALID, TXVALID, 0, 0), PHY_RFITF, RFI_CTRLSIG_2G_PA_EN);
	/* 5G PA En - ctrl[2], always off */
	wc_phy_writel(RFCTRL(1, HIGH, TXVALID, ALWAYS, 0, 0), PHY_RFITF, RFI_CTRLSIG_5G_PA_EN);
	/* ExtSwitch Rx - ctrl[3], during RXEN */
	wc_phy_writel(RFCTRL(1, HIGH, RXEN, RXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_EXT_SW_RX);
	/* ExtSwitch Tx - ctrl[4], during TXEN */
	wc_phy_writel(RFCTRL(1, HIGH, TXEN, TXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_EXT_SW_TX);
	/* RF Rx Enable - ctrl[5], during RXEN */
	wc_phy_writel(RFCTRL(1, HIGH, RXEN, RXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_RF_RX_EN);
	/* RF Tx Enable - ctrl[6], during TXEN */
	wc_phy_writel(RFCTRL(1, HIGH, TXEN, TXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_RF_TX_EN);
	/* ADC control - ctrl[7], always on */
	wc_phy_writel(RFCTRL(1, HIGH, ALWAYS, RXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_ADC);
	/* DAC control - ctrl[8], during TXEN */
	wc_phy_writel(RFCTRL(1, HIGH, TXEN, TXEN, 1, 5), PHY_RFITF, RFI_CTRLSIG_DAC);
	/* RX_EN (digital I/Q) - ctrl[12], during RXEN */
	wc_phy_writel(RFCTRL(1, HIGH, RXEN, RXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_RX_EN);
	/* TX_EN (digital I/Q) - ctrl[13], during TXEN */
	wc_phy_writel(RFCTRL(1, HIGH, TXEN, TXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_TX_EN);
	/* RX_CLK  - ctrl[14], always (in FPGA) */
	wc_phy_writel(RFCTRL(1, HIGH, ALWAYS, TXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_RX_CLK);
	/* TX_CLK  - ctrl[15], always on (in FGPA) */
	wc_phy_writel(RFCTRL(1, HIGH, ALWAYS, TXEN, 0, 0), PHY_RFITF, RFI_CTRLSIG_TX_CLK);

	/* Tx DAC interface ON */
	wc_phy_writel(0x00000001, PHY_RFITF, FW_RFI_TXDAC_INTF);

	/* Rx ADC interface ON */
	wc_phy_writel(0x00000001, PHY_RFITF, FW_RFI_RXADC_INTF);

	/* TX digital manual gain */
	wc_phy_writel(0x000000A8, PHY_RFITF, PHY_RFI_TX_DIG_GAIN_MAN);
	/* RX digital manual gain disable */
	wc_phy_writel(0x00000000, PHY_RFITF, PHY_RFI_RX_DIG_GAIN_MAN);
	/* Rx HPF duration */
	wc_phy_writel(0x00000808, PHY_RFITF, FW_RXHPF_CFG1);
	/* Rx HPF BW selection */
	wc_phy_writel(0x00000000, PHY_RFITF, FW_RXHPF_CFG2);

	/* PHY power en */
	wc_phy_writel(0x00000001, PHY_MODEM, FW_PWR_EN);
	/* AGC LUT index range */
	wc_phy_writel(0x00005E5A, PHY_MODEM, PHY_AGC_INDEX_CFG);

	/* Power low TH change */
	wc_phy_writel(0x001100FE, PHY_MODEM, PHY_AGC_PWRMSR_CFG);
	/* Init power range */
	wc_phy_writel(0x01A2012C, PHY_MODEM, PHY_AGC_INITPWR_RANGE);
	/* Target power */
	wc_phy_writel(0x015E01A2, PHY_MODEM, PHY_AGC_TARGET_PWR);

	/* Gain adj wait time */
	wc_phy_writel(0x00280020, PHY_MODEM, PHY_AGC_WAIT_CNT2);
	/* Gain step control */
	wc_phy_writel(0x18181818, PHY_MODEM, PHY_AGC_GDOWN_CFG);

	/* AGC debug config default setting */
	wc_phy_writel(0x00007F11, PHY_MODEM, PHY_AGC_TEST_OPTION);

	/* Default value update for All version*/
	/* Initial fine/coarse gain adj wait counter */
	wc_phy_writel(0x00280020, PHY_MODEM, PHY_AGC_WAIT_CNT1);
	/* SW reset */
	wc_phy_writel(0x00000001, PHY_MODEM, FW_PHY_SW_RESET);

	/* Temp Config */
	/* Don't Stop reg */
	wc_phy_writel(0xFFFFFFFF, PHY_MODEM, PHY_DONT_BE_STUCK);
	/* Turn off (GID, PAID) filtering */
	wc_phy_writel(0x00000000, PHY_MODEM, FW_VHT_CONFIG(0, 0));
	/* Turn off (GID, PAID) filtering */
	wc_phy_writel(0x00000000, PHY_MODEM, FW_VHT_CONFIG(0, 1));

	/* RSSI offset */
	wc_phy_writel(0x000000F4, PHY_MODEM, PHY_RSSI_OFFSET);
	/* SAT det config */
	wc_phy_writel(0x000C0708, PHY_MODEM, PHY_RFI_SATDET_CFG);
	/* HPF config when Init gain change */
	wc_phy_writel(0x000C0000, PHY_MODEM, FW_RXHPF_CFG2);
	/* HPF config when Init gain change */
	wc_phy_writel(0x00280808, PHY_MODEM, FW_RXHPF_CFG1);
	/* 11B min sen config */
	wc_phy_writel(0x0003E802, PHY_MODEM, PHY_11B_DET);
}

#if 1 /* PM_SOC_QFN40_FIB */
static u32 phy_rx_lut[95] = {
0x0028, 0x002A, 0x0128, 0x012A, 0x0228, 0x022A, 0x0328, 0x032A, 0x0428, 0x042A,
0x0528, 0x052A, 0x0628, 0x062A, 0x0728, 0x072A, 0x0828, 0x082A, 0x0928, 0x092A,
0x0A28, 0x0A2A, 0x1528, 0x152A, 0x1628, 0x162A, 0x1728, 0x172A, 0x1828, 0x182A,
0x1928, 0x192A, 0x1A28, 0x1A2A, 0x2328, 0x232A, 0x2428, 0x242A, 0x2528, 0x252A,
0x2628, 0x262A, 0x2728, 0x272A, 0x2828, 0x282A, 0x2928, 0x292A, 0x2A28, 0x2A2A,
//0x3528, 0x352A, 0x3628, 0x362A, 0x3728, 0x372A, 0x3828, 0x382A, 0x3928, 0x392A,
0x352A, 0x352C, 0x362A, 0x362C, 0x372A, 0x372C, 0x382A, 0x382C, 0x392A, 0x392C,
0x3A2A, 0x3A2C, 0x7529, 0x752B, 0x7629, 0x762B, 0x7729, 0x772B, 0x7829, 0x782B,
0x7929, 0x792B, 0x7A29, 0x7A2B, 0x7A2D, 0x7A2F, 0x7A31, 0x7A33, 0x7A35, 0x7A37,
0x7A39, 0x7A3B, 0x7A3D, 0x7A3F, 0x7A41, 0x7A43, 0x7A45, 0x7A47, 0x7A49, 0x7A4B,
0x7A4D, 0x7A4F, 0x7A51, 0x7A53, 0x7A55,
};
#else
static u32 phy_rx_lut[95] = {
	/* 2.4 GHz band */
	0x00002122, 0x00002220, 0x00002222, 0x00002320,
	0x00002322, 0x00002420, 0x00002422, 0x00002520,
	0x00002522, 0x00002620, 0x00002622, 0x00002720,
	0x00002722, 0x00002820, 0x00002822, 0x00002920,
	0x00002922, 0x00002A20, 0x00002A22, 0x00002B20,
	0x00002B22, 0x00002C20, 0x00002C22, 0x00002D20,
	0x00002D22, 0x00002E20, 0x00002E22, 0x00002F20,
	0x00002F22, 0x00004822, 0x00004920, 0x00004922,
	0x00004A20, 0x00004A22, 0x00004B20, 0x00004B22,
	0x00004C20, 0x00004C22, 0x00004D20, 0x00004D22,
	0x00004E20, 0x00004E22, 0x00004F20, 0x00004F22,
	0x00005020, 0x00005022, 0x00005120, 0x00006923,
	0x00006A21, 0x00006A23, 0x00006B21, 0x00006B23,
	0x00006C21, 0x00006C23, 0x00006D21, 0x00006D23,
	0x00006E21, 0x00006E23, 0x00006F21, 0x00006F23,
	0x00007021, 0x00007023, 0x00007121, 0x00007123,
	0x00007221, 0x00007223, 0x00007321, 0x00007323,
	0x00007421, 0x00007423, 0x00007521, 0x00007523,
	0x00007621, 0x00007623, 0x00007721, 0x00007723,
	0x00007821, 0x00007823, 0x00007921, 0x00007923,
	0x00007A21, 0x00007A23, 0x00007B21, 0x00007B23,
	0x00007C21, 0x00007C23, 0x00007D21, 0x00007D23,
	0x00007E21, 0x00007E23, 0x00007F21, 0x00007F23,
	0x00007F25, 0x00007F27, 0x00007F29
};
#endif

static inline void wc_phy_rx_lut_restore(void)
{
	u8 i = 0;

	wc_phy_writel(0x01010000, PHY_RFITF, FW_RXLUT_CFG0); /* Rx LUT APB access en */

	for (i = 0; i < 95; i+=2) {
		// Write Gain LUT
		wc_phy_writel((2<<8) | i, PHY_RFITF, FW_RFI_RXLUTV2_ADDR);
		wc_phy_writel(phy_rx_lut[i], PHY_RFITF, FW_RFI_RXLUTV2_WRDATA);
		wc_phy_writel(0x1, PHY_RFITF, FW_RFI_RXLUTV2_WRSTART);
	}

	wc_phy_writel(0x01000000, PHY_RFITF, FW_RXLUT_CFG0); /* Rx LUT APB access dis */
}

#endif

static void wc_hmac_enable(bool on)
{
	u32 v;
	if (on) {
		/* Enable MAC */
		v = wc_mac_readl(REG_MAC_CFG);
		if (bfget_m(v, MAC_CFG, EN))
			return;
		bfmod_m(v, MAC_CFG, EN, 1);
		wc_mac_writel(v, REG_MAC_CFG);
	} else {
		/* Disable MAC */
		v = wc_mac_readl(REG_MAC_CFG);
		if (!bfget_m(v, MAC_CFG, EN))
			return;
		bfclr_m(v, MAC_CFG, EN);
		wc_mac_writel(v, REG_MAC_CFG);
		/* Wait until RX DMA goes to IDLE */
		/* XXX: also need to check MAC_STATUS_RXE_BUSY ? */
		do {
			v = wc_mac_readl(REG_MAC_STATUS);
		} while (bfget_m(v, MAC_STATUS, DMA_BUSY));
	}
}

/**
 * It's completely different between Chip(SOC) and FPGA
 * In Xiaohu FPGA, Even if Power is off, RF reg setting is still remained.
 * Only PHY RF enable is enough.
 * In SOC, Setting the channel, Fix me lator
 */
#define	IEEE80211_CHAN_HT40U	0x00020000 /* HT 40 channel w/ ext above */
#define	IEEE80211_CHAN_HT40D	0x00040000 /* HT 40 channel w/ ext below */
#define	IEEE80211_CHAN_VHT40U	0x02000000 /* VHT40 channel, ext above */
#define	IEEE80211_CHAN_VHT40D	0x04000000 /* VHT40 channel, ext below */

#define	IEEE80211_CHAN_HT40	(IEEE80211_CHAN_HT40U | IEEE80211_CHAN_HT40D)
#define	IEEE80211_CHAN_VHT40	(IEEE80211_CHAN_VHT40U | IEEE80211_CHAN_VHT40D)
#define	IEEE80211_CHAN_40	(IEEE80211_CHAN_HT40 | IEEE80211_CHAN_VHT40)
static void scm2020_wc_set_channel_freq(u32 cfreq)
{
	bool mac_en;
	u32 val;
	u8 pri_loc, bw;

	if ((WATCHER_MAC_REG_PTR->reg_icflag & IEEE80211_CHAN_40) != 0) {
		bw = 1;
		pri_loc = WATCHER_MAC_REG_PTR->reg_icfreq > cfreq ? 1 : 0;
	} else {
		bw = 0;
		pri_loc = 0;
	}

	mac_en = !!(wc_mac_readl(REG_MAC_CFG) & MAC_CFG_EN_MASK);

	if (mac_en) {
		/*Disble MAC RX*/
		wc_hmac_enable(false);
	}

	/** do phy_set_channel with optimization
	 * setting FW_OPBW_PRICH, PHY_SYNC_CCFO_CSFO, PHY_PHAMP_CONV reg
	 */
	{
		u32 conv_ratio1, conv_ratio2, conv_11b_ratio;
		u32 opbw;

#ifdef PM_SOC_QFN40_FIB
		{
			u8 chan = (cfreq == 2484 ? 13 : (cfreq - 2412) / 5);
			if(chan > 14)
				chan = 14;

			wc_phy_writel(0x05000100 | (bw << 12) | chan, PHY_RFITF, FW_RFI_ACTT_CFG);
			if(bw == 1)
				wc_phy_writel(((wc_phy_readl(PHY_RFITF, FW_RFI_EN) >> 15) && 0x10) | 0x1, PHY_RFITF, FW_RFI_TX_INTP_FLT);
			else
				wc_phy_writel(0x1, PHY_RFITF, FW_RFI_TX_INTP_FLT);


			wc_phy_writel(0x000000B0, PHY_RFITF, PHY_RFI_TX_DIG_GAIN_MAN);
			wc_phy_writel(0x000128EC, PHY_RFITF, PHY_RFI_TXPWR_MINMAX);
		}
#endif

		/* Set RF Channel */
		val = readl(0xf0e00000 + 0x001 * 4);	// RF manual mode = standby
		val &= ~GENMASK(15, 0);
		writel(val | (0x26a0 << 0), 0xf0e00000 + 0x001 * 4);

		wc_udelay(10);

		val = readl(0xf0e00000 + 0x001 * 4);	// RF manual mode = Modem control
		val &= ~GENMASK(15, 0);
		writel(val | (0x22a0 << 0), 0xf0e00000 + 0x001 * 4);

		/* work normal , so can be removed */
		/* wc_udelay(50); */

		conv_11b_ratio = ((5 * 1048576) + cfreq / 2) / cfreq;
		wc_phy_writel(conv_11b_ratio, PHY_MODEM, FW_11B_CSFO_CONV_VINTF0);
		conv_ratio1 = (20 * 262144 + cfreq / 2) / cfreq;
		conv_ratio2 = (20 * 4194304 + cfreq / 2) / cfreq;
		wc_phy_writel(conv_ratio1, PHY_MODEM, PHY_SYNC_CCFO_CSFO);
		wc_phy_writel(conv_ratio2, PHY_MODEM, PHY_PHAMP_CONV);

		/* set the opbw = 0 */
		opbw = ((bw << FW_OPBW_VINTF0_SHIFT) & FW_OPBW_VINTF0_MASK) | ((pri_loc << FW_PRICH_VINTF0_SHIFT) & FW_PRICH_VINTF0_MASK);
		wc_phy_writel(opbw, PHY_MODEM, FW_OPBW_PRICH(0));

#ifdef PM_SOC_QFN40_FIB
		wc_phy_writel(1 - opbw, PHY_RFITF, FW_RFI_RXDECI_FLT);	// OP40 work-around
#endif
		/** PLL Lock time delay
		 * In Soc, RF Driver do delay or lock detector
		 * if RF Driver have 70us delay for PLL lock, remove it.
		 */
		wc_udelay(10);	//70
		wc_phy_writel(0x1, PHY_MODEM, FW_PHY_SW_RESET);
	}

	if (mac_en) {
		/*Enable MAC RX*/
		wc_hmac_enable(true);
	}
	return;
}

#define RX_PKT_WAIT_TIMEOUT	15	//First 20ms waiting, need optimization
#define RX_PTR(p)		((p) & ((WATCHER_PS_POLL_TX_PTR->n_rx_desc << 1) - 1))
#define RX_PTR2IDX(p)	((p) & (WATCHER_PS_POLL_TX_PTR->n_rx_desc - 1))

#define rx_ptr_READY()	({u32 __v; u16 __ptr;				\
				__v = wc_mac_readl(REG_RX_BUF_PTR0);		\
				__ptr = RX_PTR(bfget_m(__v, RX_BUF_PTR0, READY_PTR)); __ptr;})

#define rx_ptr_READ()	({u32 __v; u16 __ptr;				\
				__v = wc_mac_readl(REG_RX_BUF_PTR0);		\
				__ptr = RX_PTR(bfget_m(__v, RX_BUF_PTR0, READ_PTR)); __ptr;})
#define rx_ptr_WRITE()	({u32 __v; u16 __ptr;				\
				__v = wc_mac_readl( REG_RX_BUF_PTR1);		\
				__ptr = RX_PTR(bfget_m(__v, RX_BUF_PTR1, WRITE_PTR)); __ptr;})

#define advance_rx_ptr_READY()	do {					\
			u32 v; u16 ptr;							\
			v = wc_mac_readl(REG_RX_BUF_PTR0);		\
			ptr = rx_ptr_READY();					\
			ptr = RX_PTR(ptr + 1);					\
			bfmod_m(v, RX_BUF_PTR0, READY_PTR, ptr);	\
			wc_mac_writel(v, REG_RX_BUF_PTR0);		\
} while (0);

#define advance_rx_ptr_READ()	do {				\
			u32 v; u16 ptr;							\
			v = wc_mac_readl(REG_RX_BUF_PTR0);		\
			ptr = rx_ptr_READ();					\
			ptr = RX_PTR(ptr + 1);					\
			bfmod_m(v, RX_BUF_PTR0, READ_PTR, ptr);	\
			wc_mac_writel(v, REG_RX_BUF_PTR0);		\
} while (0);

#define is_room_READ()	({bool __room; u16 __wp, __rp;			\
			__wp = rx_ptr_WRITE();				\
			__rp = rx_ptr_READ();					\
			__room = __wp != __rp; __room;})

#define init_ptr0(which, val)		do {				\
			u32 v;							\
			v = wc_mac_readl(REG_RX_BUF_PTR0);		\
			bfclr_m(v, RX_BUF_PTR0, which##_PTR);	\
			wc_mac_writel(v, REG_RX_BUF_PTR0);		\
} while(0);

#define init_ptr1(which, val)		do {				\
			u32 v;							\
			v = wc_mac_readl(REG_RX_BUF_PTR1);		\
			bfclr_m(v, RX_BUF_PTR1, which##_PTR);	\
			wc_mac_writel(v, REG_RX_BUF_PTR1);		\
} while(0);

#define LOG2(x)	({ int __n, __k = 0; for (__n = 1; __n < x; __n = __n * 2) __k++; __k;})

#define read_ptr(which)	rx_ptr_##which()
#define advance_ptr(which)	advance_rx_ptr_##which()
#define is_room(which)	is_room_##which()

#ifndef BIT
#define BIT(x)  (0x1UL << (x))
#endif

#define RX_DESC_HW_READY	BIT(2)  /* already emptied by SW */
#define RX_DESC_SW_READY 	BIT(1)  /* ready to be emptied by SW */
#define RX_DESC_HW_OWNED 	BIT(0) 	/* ready to be written by HW */

struct rx_vector {
	/* word 1 (  0 -  31) */
	bf( 0,  6, mcs);
	bf( 7,  7, preamble_type);
	bf( 8, 19, l_length);
	bf(20, 22, format);
	bf(23, 23, fec_coding);
	bf(24, 31, rssi_legacy);

	/* word 2 ( 32 -  63) */
	bf( 0, 15, length);
	bf(16, 19, spatial_reuse);
	bf(20, 22, mcs_sig_b);
	bf(23, 23, dcm_sig_b);
	bf(24, 31, rssi);

	/* word 3 ( 64 -  95) */
	bf( 0, 19, psdu_length);
	bf(20, 22, ch_bandwidth);
	bf(23, 23, stbc);
	bf(24, 30, txop_duration);
	bf(31, 31, dcm);

	/* word 4 ( 96 - 127) */
	bf( 0,  7, ru_allocation);
	bf( 8, 15, rsvd1);
	bf(16, 23, td_best_sym_epwr_db);
	bf(24, 31, td_best_sym_dpwr_db);

	/* word 5 (128 - 159) */
	bf( 0,  7, td_worst_sym_epwr_db);
	bf( 8, 15, td_worst_sym_dpwr_db);
	bf(16, 23, avg_epwr_db);
	bf(24, 31, avg_dpwr_db);

	/* word 6 (160 - 191) */
	bf( 0, 10, sta_id_list);
	bf(11, 13, pe_duration);
	bf(14, 15, gi_type);
	bf(16, 24, partial_aid);
	bf(25, 27, num_sts);
	bf(28, 28, dyn_bandwidth_in_non_ht);
	bf(29, 30, ch_bandwidth_in_non_ht);
	bf(31, 31, doppler);

	/* word 7 (192 - 223) */
	bf( 0,  6, scrambler_initial_value);
	bf( 7,  7, uplink_flag);
	bf( 8, 13, bss_color);
	bf(14, 15, he_ltf_type);
	bf(16, 21, group_id);
	bf(22, 23, user_position);
	bf(24, 24, beam_change);
	bf(25, 25, beamformed);
	bf(26, 26, txop_ps_not_allowed);
	bf(27, 27, non_ht_modulation);
	bf(28, 28, aggregation);
	bf(29, 29, sounding);
	bf(30, 30, smoothing);
	bf(31, 31, lsigvalid);

	/* word 8 (224 - 255) */
	bf( 0,  3, spatial_reuse2);
	bf( 4,  7, spatial_reuse3);
	bf( 8, 11, spatial_reuse4);
	bf(12, 23, cfo_est);
	bf(24, 31, rcpi);
};

struct rx_info {
	/* word 1 */
	bf(0,  13, len);
	bf(14, 31, rsvd1);

	/* word 2 */
	bf( 0,  0, phy);
	bf( 1,  1, ok);
	bf( 2,  2, err_crc);
	bf( 3,  3, err_len);
	bf( 4,  4, err_key);
	bf( 5,  5, err_mic);
	bf( 6,  6, err_icv);
	bf( 7,  7, clear);
	bf( 8, 10, cipher);
	bf(11, 11, ampdu);
	bf(12, 12, eof);
	bf(13, 13, rsvd2);
	bf(14, 15, policy);
	bf(16, 19, tid);
	bf(20, 20, tail);
	bf(21, 31, rsvd3);

	/* word 3 */
	u32 tsf;
};

struct hw_rx_hdr {
	struct rx_vector rv;
	struct rx_info   ri;
	struct ieee80211_frame mh[0];
};

static inline void scm2020_wc_dtim_parser_stop(void)
{
	u32 val;

	/* all of interrupt is disabled */
	bfzero(val);
	wc_mac_writel(val, REG_INTR_ENABLE);

	/* wlan interrupt disable */
	set_wlan_irq(0);
}

#define KEY_SET_DONE_TIMEOUT	MSEC_TO_RTC(10)
static void scm2020_wc_key_load(struct watcher_ieee80211_key *wc_key, int action)
{
	const u8 *addr;
	union key_cmd kcmd = {{0}};
	u32 v, kaddr[2];
	int j;
	u8 remain;
	u32 start;

	addr = WATCHER_MAC_REG_PTR->reg_ni_bssid;
	if (wc_key->group) {
		kcmd.gtk = true;
	}

	kaddr[0] = get_unaligned_le32(addr);
	kaddr[1] = get_unaligned_le16(addr + 4);

	wc_mac_writel(kaddr[0], REG_SEC_MAC_ADDR_L32);
	wc_mac_writel(kaddr[1], REG_SEC_MAC_ADDR_H16);

	remain = wc_key->wk_keylen % sizeof(u32);

	for (j = 0; j < wc_key->wk_keylen/sizeof(u32); j++) {
		v = get_unaligned_le32(&wc_key->wk_key[j * 4]);
		wc_mac_writel(v, M_OFT_N(SEC_TEMP_KEY0, j));
	}

	if (remain) {
		v = 0;
		memcpy(&v, &wc_key->wk_key[j * 4], remain);
		wc_mac_writel(v, M_OFT_N(SEC_TEMP_KEY0, j));
	}

	if (wc_key->cipher == SC_CIPHER_TKIP) {
		const u8 *tx_mic, *rx_mic;

		tx_mic = wc_key->wk_txmic;
		rx_mic = wc_key->wk_rxmic;

		for (j = 0; j < 2; j++) {
			u32 tx, rx;
			tx = get_unaligned_le32(tx_mic + 4 * j);
			rx = get_unaligned_le32(rx_mic + 4 * j);
			wc_mac_writel(tx, M_OFT_N(SEC_TEMP_KEY4, j));
			wc_mac_writel(rx, M_OFT_N(SEC_TEMP_KEY6, j));
		}
	}

	kcmd.cipher = wc_key->cipher;
	kcmd.keyid = wc_key->wk_keyix;
	kcmd.spp = wc_key->wk_spp_amsdu;

	if(action == SC_KEY_CMD_DEL_BY_INDEX)
		kcmd.idx = wc_key->hw_key_idx;

	kcmd.cmd = action;
	wc_mac_writel(kcmd.v, REG_SEC_KEY_CMD);

	start = wc_ktime();
	do {
		v = wc_mac_readl(REG_SEC_KEY_CMD);
		if (bfget_m(v, SEC_KEY_CMD, KCMD) == 0) {

			if(action == SC_KEY_CMD_ADD)
				wc_key->hw_key_idx = bfget_p(wc_mac_readl(REG_SEC_KEY_STATUS), SEC_KEY_STATUS_ADD_IDX);

			return;
		}
	} while (wc_ktime() < (start + KEY_SET_DONE_TIMEOUT));

	wc_printf("[WATCHER] key command failed\n");
	return;
}

static void wc_ccmp_setiv(uint8_t *ivp)
{
	uint64_t wc_keytsc;

	WATCHER_SEC_KEY_PTR->txkeytsc++;
	wc_keytsc = WATCHER_SEC_KEY_PTR->txkeytsc;
	ivp[0] = wc_keytsc >> 0; /* PN0 */
	ivp[1] = wc_keytsc >> 8; /* PN1 */
	ivp[2] = 0;                 /* Reserved */
	ivp[3] = WATCHER_SEC_KEY_PTR->txkeyid | IEEE80211_WEP_EXTIV;       /* KeyID | ExtID */
	ivp[4] = wc_keytsc >> 16;        /* PN2 */
	ivp[5] = wc_keytsc >> 24;        /* PN3 */
	ivp[6] = wc_keytsc >> 32;        /* PN4 */
	ivp[7] = wc_keytsc >> 40;        /* PN5 */
}

static void wc_tkip_setiv(uint8_t *ivp)
{
	uint64_t wc_keytsc;

	WATCHER_SEC_KEY_PTR->txkeytsc++;
	wc_keytsc = WATCHER_SEC_KEY_PTR->txkeytsc;
	ivp[0] = wc_keytsc >> 8; /* TSC1 */
	ivp[1] = (ivp[0] | 0x20) & 0x7f;    /* WEP seed */
	ivp[2] = wc_keytsc >> 0; /* TSC0 */
	ivp[3] = WATCHER_SEC_KEY_PTR->txkeyid | IEEE80211_WEP_EXTIV;       /* KeyID | ExtID */
	ivp[4] = wc_keytsc >> 16;        /* TSC2 */
	ivp[5] = wc_keytsc >> 24;        /* TSC3 */
	ivp[6] = wc_keytsc >> 32;        /* TSC4 */
	ivp[7] = wc_keytsc >> 40;        /* TSC5 */
}

static void wc_wep_setiv(uint8_t *ivp)
{
	uint32_t iv;
	/*
	 * Skip 'bad' IVs from Fluhrer/Mantin/Shamir:
	 * (B, 255, N) with 3 <= B < 16 and 0 <= N <= 255
	 */
	iv = WATCHER_SEC_KEY_PTR->txkeytsc;
	if ((iv & 0xff00) == 0xff00) {
		int B = (iv & 0xff0000) >> 16;

		if (3 <= B && B < 16)
			iv += 0x0100;
	}
	WATCHER_SEC_KEY_PTR->txkeytsc = iv + 1;

	ivp[0] = iv >> 0;
	ivp[1] = iv >> 8;
	ivp[2] = iv >> 16;
	ivp[3] = WATCHER_SEC_KEY_PTR->txkeyid;
}

static inline void scm2020_wc_sec_key_restore(void)
{
	struct watcher_ieee80211_key *wc_key;
	int i;

	/* Maybe open mode */
	if (!WATCHER_SEC_KEY_PTR->sec_mode)
		return;

	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
		wc_key = &WATCHER_SEC_KEY_PTR->wc_key[i];

		if (wc_key->devkey) {
			if (wc_key->group && wc_key->hw_key_idx != -1) {
				scm2020_wc_key_load(wc_key, SC_KEY_CMD_DEL_BY_INDEX);
			}
			scm2020_wc_key_load(wc_key , SC_KEY_CMD_ADD);
		}
	}

	wc_key = &WATCHER_SEC_KEY_PTR->wc_ucastkey;
	if (wc_key->devkey)
		scm2020_wc_key_load(wc_key, SC_KEY_CMD_ADD);

	if (WATCHER_SEC_KEY_PTR->txcipher == SC_TXCIPHER_WEP) {
		wc_crypto_setiv = wc_wep_setiv;
	} else if (WATCHER_SEC_KEY_PTR->txcipher == SC_TXCIPHER_TKIP) {
		wc_crypto_setiv = wc_tkip_setiv;
	} else {
		wc_crypto_setiv = wc_ccmp_setiv;
	}

	return;
}

#ifdef RX_PACKET_DUMP
static void wc_rx_dump(uint8_t *buffer , unsigned short totlen)
{
	int x;

	wc_printf("===[PM WiFi] rx data[%u]===\n", totlen);
	for (x = 0; x < (totlen - 22); x++) {
		if (!(x % 16))
			wc_printf("\n");
		wc_printf("%x ", buffer[x]);
	}
	wc_printf("\n");
}
#endif

#ifdef CONFIG_WATCHER_FILTER_MULTICAST
static bool check_reg_mc_addr(const uint8_t dest[IEEE80211_ADDR_LEN])
{
	int index;
	unsigned long reg_mc_addr;

	/* If registered multicast addr is nothing */
	if (!WATCHER_MC_FILTER_PTR->tot_reg_mc)
		return 0;

	for (index = 0 ; index < CONFIG_WATCHER_ACCEPT_MAX_MC ; index++) {
		unsigned char b,c,d;

		if (!WATCHER_MC_FILTER_PTR->accept_mc[index].mc_addr)
			continue;

		/* get the multicast ip address */
		reg_mc_addr = WATCHER_MC_FILTER_PTR->accept_mc[index].mc_addr;
		b = (uint8_t)((reg_mc_addr & (uint32_t)0x00ff0000UL) >> 16);
		c = (uint8_t)((reg_mc_addr & (uint32_t)0x0000ff00UL) >> 8);
		d = (uint8_t)((reg_mc_addr & (uint32_t)0x000000ffUL));

		/* mac addresss is same with reg addr */
		if (ETHER_IS_FILTER_MC_OUI(dest) && dest[3] == b && dest[4] == c && dest[5] == d) {

			scm2020_ps_update_mc_time(index, rtc_time_get());
			/* Fix me lator , check the multicast ip address */
			wc_printf("multicast addr(0x%08x) match\n", reg_mc_addr);
			return 1;
		}
#ifdef WATCHER_FOR_DEBUG
		else {
			wc_printf("[MC]:%d addr(0x%08x) and (%x:%x:%x) \n", index, reg_mc_addr , b,c,d);
		}
#endif
	}
	return 0;
}
#endif

#if defined(CONFIG_WATCHER_FILTER_TCP) || defined(CONFIG_WATCHER_FILTER_UDP)
static bool check_reg_udp_tcp_port(char protocol , uint32_t dest_ip , uint16_t port_num)
{
	int index;

	/* checking for dest_ip is my ip or broadcast ip */
	if (dest_ip == IPADDR_BROADCAST)
		return 1;

	/* TCP Parsing */
	if (protocol == IP_TCP_PROTO) {
#ifdef CONFIG_WATCHER_FILTER_TCP
		/* If registered TCP Port is nothing */
		if (!WATCHER_TCP_FILTER_PTR->tot_reg_tcp)
			return 0;
		for (index = 0 ; index < CONFIG_WATCHER_ACCEPT_MAX_TCP ; index++) {

			if (!WATCHER_TCP_FILTER_PTR->accept_tcp[index].port_num)
				continue;

			/* tcp port is same with registered tcp port */
			if (WATCHER_TCP_FILTER_PTR->accept_tcp[index].port_num == port_num) {
				scm2020_ps_update_tcp_time(index, rtc_time_get());
				/* Bingo that's registered tcp port */
				wc_printf("[RX] TCP Reg port(0x%08x) match\n", port_num);
				return 1;
			}
		}
		return 0;
#else
		return 1;
#endif
	}
	else { /* UDP Parsing */
#ifdef CONFIG_WATCHER_FILTER_UDP
		/* If registered UDP Port is nothing */
		if (!WATCHER_UDP_FILTER_PTR->tot_reg_udp)
			return 0;
		for (index = 0 ; index < CONFIG_WATCHER_ACCEPT_MAX_UDP ; index++) {

			if (!WATCHER_UDP_FILTER_PTR->accept_udp[index].port_num)
				continue;

			/* udp port is same with registered udp port */
			if (WATCHER_UDP_FILTER_PTR->accept_udp[index].port_num == port_num) {
				scm2020_ps_update_udp_time(index, rtc_time_get());
				/* Bingo that's registered udp port */
				wc_printf("[RX] UDP Reg port(0x%08x) match\n", port_num);
				return 1;
			}
		}
		return 0;
#else
		return 1;
#endif
	}
}
#endif

/* For Wakeup full booting, stop rx interrupt */
static inline void wlan_wc_stop_rx_int(void)
{
	WATCHER_BCN_WINDOW_PTR->not_rx_desc_restore = 1;

	watcher_rxbuf_interrupt_cnt++;
	scm2020_wc_dtim_parser_stop();
}

/* ARP related Packet is buffered as arp info format */
static void scm2020_ps_put_arp_info(unsigned char *arp_header)
{
	int index = 0;

	/* Already ARP cache info is full, do not compare and save */
	if (WATCHER_ARP_CACHE_INFO_PTR->tot_arp_cache >= CONFIG_WATCHER_MAX_ARP_CACHE) {
		wc_printf("ARP INFO is full\n");
		return;
	}

	if (WATCHER_ARP_CACHE_INFO_PTR->tot_arp_cache > 0) {

		for (index = 0; index < WATCHER_ARP_CACHE_INFO_PTR->tot_arp_cache ; index++) {
			/* Check if same info */
			if (memcmp(WATCHER_ARP_CACHE_INFO_PTR->arp_info[index].sender_ip, &arp_header[14], 4) == 0) {
				wc_printf("have same ARP INFO\n");
				return;
			}
		}
	}

	/* put the ARP info(ip, mac) to retention mem */
	memcpy(WATCHER_ARP_CACHE_INFO_PTR->arp_info[index].sender_ip, &arp_header[14], 4);
	memcpy(WATCHER_ARP_CACHE_INFO_PTR->arp_info[index].sender_mac, &arp_header[8], 6);
	WATCHER_ARP_CACHE_INFO_PTR->tot_arp_cache++;
	wc_printf("ARP INFO is saved(total : %d)\n", WATCHER_ARP_CACHE_INFO_PTR->tot_arp_cache);
	return;
}

static void wlan_wc_chk_rx_int(void)
{
	int readidx;
	volatile struct scm_mac_shmem_map *shmem;

	if (read_ptr(READ) == read_ptr(WRITE)) {
		wc_printf("[%s, %d] Spurious interrupt (case 1) ignored\n", __func__, __LINE__);
		return; /* spurious */
	}
	shmem = (struct scm_mac_shmem_map *)(MAC_BASE_START + MAC_SHM_OFFSET);
	while (1) {
		uint8_t type;
		uint8_t subtype;
		uint16_t totlen;
		struct mbuf *m;
		struct hw_rx_hdr *hw;
		struct ieee80211_frame *mh;
		u8 frag;

		readidx = RX_PTR2IDX(read_ptr(READ));

		frag = shmem->rx[readidx].frag;
		if ((frag & 0x2) == 0) /* interested only in the first fragment */
			goto next;

		m = (struct mbuf *)(shmem->rx[readidx].pbuffer_l - 92);
		hw = mtod(m, struct hw_rx_hdr *);
		mh = hw->mh;

		totlen = hw->ri.len + sizeof(*hw);

		/* if receive encrypt frame, length will lack 4 byte*/
		if (hw->ri.err_key) {
			totlen += 4;
		}
		totlen = min(totlen, m->m_len);

		type = mh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
		subtype = mh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (type == IEEE80211_FC0_TYPE_DATA) {
			int header_size;
			uint8_t *llc_header;
			uint16_t ether_type;

			/* In case of UC, RX interrupt is not enabled yet, just return except BC*/
			if (!watcher_rxbuf_int_proc && !ETHER_IS_BROADCAST(mh->i_addr1))
				goto next;

			/* Only From DS */
			if ((mh->i_fc[1] & IEEE80211_FC1_DIR_MASK) != IEEE80211_FC1_DIR_FROMDS)
				goto next;

			/* don't count Null data frames */
			if (subtype == IEEE80211_FC0_SUBTYPE_NODATA ||
				subtype == IEEE80211_FC0_SUBTYPE_QOS_NULL) {
				/* drop the packet for L2 Keepalive */
				wc_printf("Null or QoS Null Drop the packet\n");
				/* If need active state in Full booting, this can be enabled */
#if 0
				WATCHER_BCN_WINDOW_PTR->pm_wifi_active_state = 1;
#endif
				goto next;
			}

			header_size = sizeof(struct ieee80211_frame);

			if (IEEE80211_QOS_HAS_SEQ(mh))
				header_size += 2;	/* qos field */

			if (WATCHER_SEC_KEY_PTR->sec_mode) {
				/* 8 bytes represent TKIP & CCMP header
				 * TKIP header consist of IV & Extended IV (8 byte in total)
				 * Out of 8 byte CCMP header, 6 used for PN,
				 * 1 reserved & remaining byte contain key ID values (2 bits for key id)
				 * WEP has 8 byte (4-IV, 4-ICV) encryption overhead, front 4-iv, tail 4-ICV
				 * let's neglect this security header processing
				 */
				if (WATCHER_SEC_KEY_PTR->txcipher == SC_TXCIPHER_WEP)	/* WEP or WEP104 */
					header_size += 4;
				else	/* CCMP or TKIP or other cipher */
					header_size += 8;
			}

			llc_header = (uint8_t *)(header_size + (uint8_t *)mh);

			/* If packet has normal llc/snap format, drop it in watcher */
			if (memcmp(llc_header, llcsnapheaderaddr, IEEE80211_ADDR_LEN) != 0) {
				wc_printf("[RX] is not LLC/SNAP --> drop\n");
				goto next;
			}
			/* DA BSSID SA */
			ether_type = (llc_header[IEEE80211_ADDR_LEN] << 8) | llc_header[IEEE80211_ADDR_LEN + 1];

			/* Check My UC Packet */
			/** If packet is valid(data packet), rx packet transfer to Wise
			 *  and trigger full booting
			 */
			if (memcmp(WATCHER_MAC_REG_PTR->reg_iv_myaddr, mh->i_addr1, IEEE80211_ADDR_LEN) == 0) {

#ifdef CONFIG_MORE_DATA_ACTIVE
				/* More bit checking */
				if (mh->i_fc[1] & IEEE80211_FC1_MORE_DATA) {
					wc_printf("More Data packet --> let's go Wise\n");
					WATCHER_BCN_WINDOW_PTR->pm_wifi_active_state = 1;
					wlan_wc_stop_rx_int();
					break;
				}
#endif
				if (ether_type == LLC_TYPE_ARP) {
					/* Destination is my mac address(UC) in ARP packet */
					/* ARP Reply or for updateing ARP Table or IP address conflict */
					/* So, Wise wakeup */
					wc_printf("[RX] UC-ARP\n");
				}
				else if (ether_type == LLC_TYPE_IPv4) {
					uint8_t *ip_header;
					uint32_t dest_ip;
					uint16_t port_num;

					/* Get the IP Header Start */
					ip_header = (uint8_t *)(IEEE80211_ADDR_LEN + 2 + (uint8_t *)llc_header);

#if defined(CONFIG_WATCHER_FILTER_TCP) || defined(CONFIG_WATCHER_FILTER_UDP)
					/* TCP/UDP Port filter */
					if (ip_header[9] == IP_UDP_PROTO || ip_header[9] == IP_TCP_PROTO) {
						uint8_t *udptcp_hdr;
						uint16_t iphdr_hlen = (ip_header[0] & IPv4_HLMASK) << 2;

						udptcp_hdr = (unsigned char *)(iphdr_hlen + ip_header);

						dest_ip = CONV_IP_ADDR(ip_header[12], ip_header[13], ip_header[14], ip_header[15]);
						port_num = udptcp_hdr[2] << 8 | udptcp_hdr[3];

						if (!check_reg_udp_tcp_port(ip_header[9], dest_ip, port_num)){
							wc_printf("[RX] UC-IP-TCP/UDP not reg port(%d) --> Drop\n" , port_num);
							goto next;
						}
					}
					else {
						wc_printf("[RX] UC-IP-NotTCP/UDP --> let's go Wise\n");
					}
#else
					wc_printf("[RX] UC-IP\n");
#endif
				}
				else if (ether_type == LLC_TYPE_PAE) {
					wc_printf("[RX] UC-PAE for EAPOL\n");
				}
				else {
					wc_printf("[RX] is not IP/ARP --> Drop\n");
					goto next;
				}
			}
			/* Check My BC/MC Packet */
			else {
				bool is_bc = false;
				bool is_mc = false;

				/* This is mine(src is my address) */
				if (memcmp(WATCHER_MAC_REG_PTR->reg_iv_myaddr, mh->i_addr3, IEEE80211_ADDR_LEN) == 0) {
					wc_printf("Src is mine --> Drop\n");
					goto next;
				}

				is_bc = ETHER_IS_BROADCAST(mh->i_addr1);
				is_mc = ETHER_IS_MULTICAST(mh->i_addr1);

				if (is_mc) {
					/* Only Multicast packet --> drop */
					if (!is_bc) {
#ifdef CONFIG_WATCHER_FILTER_MULTICAST
						if (!check_reg_mc_addr(mh->i_addr1)){
							wc_printf("MC --> Drop\n");
							goto next;
						}
						wc_printf("[RX] Match MC --> let's go Wise\n");
#else
					wc_printf("MC --> Drop\n");
					goto next;
#endif
					}
					else {
						/* In case of BC, ARP is processed */
						/* 1. ARP request & dest is my IP : answer ARP response in Watcher */
						if (ether_type == LLC_TYPE_ARP) {
							uint8_t *arp_header;
							uint16_t op_code;
							uint32_t dest_ip;

							/* IP address is not assigned yet, BC should be discarded  */
							if (!scm2020_ps_get_net_ipaddr()) {
								goto next;
							}

							/* Get the ARP Header Start */
							arp_header = (uint8_t *)(IEEE80211_ADDR_LEN + 2 + (uint8_t *)llc_header);

							op_code = (arp_header[6] << 8) | arp_header[7];
							dest_ip = CONV_IP_ADDR(arp_header[24], arp_header[25], arp_header[26], arp_header[27]);
							if (op_code == ARP_OP_REQUEST){ /* ARP request */

								/* 2. Request and My target IP, Compare target ip with my ip address */
								if (scm2020_ps_get_net_ipaddr() == dest_ip) {
#ifdef CONFIG_WATCHER_TX_ARP_RESP
									/* Make ARP Response */
									scm2020_ps_make_arp_resp(arp_header);
									wc_printf("[RX] ARP Request for Me --> make ARP reply\n");
									goto next;
#else
									wc_printf("[RX] ARP Request for Me --> let's go Wise\n");
#endif
								}
								/* 3. ARP request & dest is other IP, (Nuttx:Drop)/(Wise:check ARP announce & buffering) */
								else {
#ifdef CONFIG_WATCHER_IN_WISE_ONLY
									uint32_t src_ip;
									src_ip = CONV_IP_ADDR(arp_header[14], arp_header[15], arp_header[16], arp_header[17]);
									/* check if ARP announce packet */
									if (dest_ip == src_ip) {
										/* ARP RX Packet Buffering */
										/* At next wakeup, buffering info will be transfering to Net stack ARP Table */
										scm2020_ps_put_arp_info(arp_header);
									}
									else {
										wc_printf("[RX] ARP Request drop(target 0x%08x)\n", dest_ip);
									}
									goto next;
#else
									wc_printf("[RX] ARP Request (target 0x%08x)\n", dest_ip);
									goto next;
#endif
								}
							}
							/* BC & ARP reply : Drop */
#ifdef CONFIG_WATCHER_IN_WISE_ONLY
							else if (op_code == ARP_OP_REPLY){ /* ARP reply */
								/* Case of IP conflict or update ARP table */
								if (scm2020_ps_get_net_ipaddr() == dest_ip) {
									/* ARP RX Packet Buffering */
									/* At next wakeup, buffering info will be transfering to Net stack ARP Table */
									scm2020_ps_put_arp_info(arp_header);
								}
								goto next;
							}
#endif
							else
								goto next;
						}
						else {
							/* This is packet like Netbios (BC / IP) */
							wc_printf("[RX] Pkt is not ARP of BC --> Drop\n");
							goto next;
						}
					}
				}
				else {
					wc_printf("[RX] is not UC or BC/MC\n");
#ifdef RX_PACKET_DUMP
					wc_rx_dump((uint8_t *)mh, totlen);
#endif
					goto next;
				}
			}

			/* Here go Wise, otherwise goto next
			 * In full booting case, Stop the RX_BUF Interrupt
			 * If not stop interrupt, interrupt was kept, so need tunning
			 */
			wlan_wc_stop_rx_int();
			break;
		}
		else if (type == IEEE80211_FC0_TYPE_MGT) {

			/* Deauth or Diassoc of MGMT frame */
			if (subtype == IEEE80211_FC0_SUBTYPE_DEAUTH ||
				subtype == IEEE80211_FC0_SUBTYPE_DISASSOC) {

				/* Full booting for fast reconnecting */
				wc_printf("[RX] Deauth OR Diassoc --> goto Wise\n");

				wlan_wc_stop_rx_int();
				break;
			}
		}
next:
		advance_ptr(READ);

		shmem->rx[readidx].pbuffer_h = RX_DESC_HW_OWNED;
		advance_ptr(READY);
		if (!is_room(READ)) {
			break;
		}
	} /* End of While */
}

static inline void scm2020_wc_rx_desc_restore(void)
{
	uint8_t i;
	uint32_t v;

	/* disable RX operation immediately */
	v = wc_mac_readl(REG_RX_BUF_CFG);
	bfclr_m(v, RX_BUF_CFG, EN);
	wc_mac_writel(v, REG_RX_BUF_CFG);

	/* RX BUF_PTR1 write_ptr clearing & RX_BUF_PTR0 ready_ptr clearing */
	init_ptr1(WRITE, 0);
	init_ptr0(READY, 0);

	for (i = 0; i <WATCHER_PS_POLL_TX_PTR->n_rx_desc; i++) {
		advance_ptr(READY);
	}

	v = wc_mac_readl(REG_RX_BUF_CFG);
	bfmod_m(v, RX_BUF_CFG, BUF_NUM, LOG2(WATCHER_PS_POLL_TX_PTR->n_rx_desc));
	wc_mac_writel(v, REG_RX_BUF_CFG);

	/* RX_BUF_PTR0 read_ptr clearing */
	init_ptr0(READ, 0);

	/* Enable RX operation */
	v = wc_mac_readl(REG_RX_BUF_CFG);
	bfmod_m(v, RX_BUF_CFG, EN, 1);
	wc_mac_writel(v, REG_RX_BUF_CFG);

	return;
}

/* WiFi interrupt service routine in watcher */
void wlan_dtim_int_handler(void)
{
	unsigned int irqs;

	irqs = wc_mac_readl(REG_INTR_STATUS);
	wc_mac_writel(irqs, REG_INTR_CLEAR);

	/* In case of DTIM PARSER, notify to waiting routine */
	if (bfget_m(irqs, INTR_STATUS, RX_DTIM)) {
		watcher_macdtim_interrupt_cnt++;
		watcher_dbg_dtim_int_ctrl(1);
	}
	if (bfget_m(irqs, INTR_STATUS, TX_DONE)) {
		watcher_txdone_interrupt_cnt++;
	}
	if (bfget_m(irqs, INTR_STATUS, RX_BUF)) {
		wlan_wc_chk_rx_int();
	}
}

#ifdef WATCHER_TX_DUMP
static int _isprint(unsigned char c)
{
	if (c < 32)
		return 0;

	if (c >= 127)
		return 0;

	return 1;
}

static void wc_hexdump(void *buffer, size_t sz)
{
	unsigned char *start, *end, *buf = (unsigned char *)buffer;
	char *ptr, line[128];
	int cursor;

	start = (unsigned char *)(((uint32_t) buf) & ~7);
	end = (unsigned char *)((((uint32_t) buf) + sz + 7) & ~7);

	while (start < end) {

		ptr = line;
		ptr += wc_sprintf(ptr, "0x%08lx: ", (unsigned long) start);

		for (cursor = 0; cursor < 16; cursor++) {
			if ((start + cursor < buf) || (start + cursor >= buf + sz))
				ptr += wc_sprintf(ptr, "..");
			else
				ptr += wc_sprintf(ptr, "%02x", *(start + cursor));

			if ((cursor & 1))
				*ptr++ = ' ';
			if ((cursor & 3) == 3)
				*ptr++ = ' ';
		}
		ptr += wc_sprintf(ptr, "  ");

		/* ascii */
		for (cursor = 0; cursor < 16; cursor++) {
			if ((start + cursor < buf) || (start + cursor >= buf + sz))
				ptr += wc_sprintf(ptr, ".");
			else
				ptr += wc_sprintf(ptr, "%c", _isprint(start[cursor]) ? start[cursor] : ' ');
		}
		ptr += wc_sprintf(ptr, "\n");
		wc_printf("%s", line);
		start += 16;
	}
}

static void scm2020_wc_dump_tx()
{
	volatile struct scm_mac_shmem_map *shmem;
	volatile struct txdesc *txdesc;

	struct mentry *mentry;
	struct bdesc *bdesc;
	int i = 0;

	shmem = (struct scm_mac_shmem_map *)(MAC_BASE_START + MAC_SHM_OFFSET);
	txdesc = &shmem->tx[0];

	wc_printf("\n");
	wc_printf("shmem.tx[0] === \n");

	wc_printf("ptable_l:0x%08x, ptable_h:0x%08x, num:%d\n",
			txdesc->ptable_l,
			txdesc->ptable_h,
			txdesc->num);

	mentry = (struct mentry *)txdesc->ptable_l;
	for (i = 0; i < txdesc->num; i++, mentry++) {
		wc_printf("mentry[%d] === \n", i);
		wc_printf("pbd_l:0x%08x, pbd_h:0x%08x, aid:%d, tid:%d, sn:%d, noack:%d, len:%d\n",
				mentry->pbd_l,
				mentry->pbd_h,
				mentry->aid,
				mentry->tid,
				mentry->sn,
				mentry->noack,
				mentry->len);

		bdesc = (struct bdesc *)mentry->pbd_l;
		wc_printf("bdesc=== \n");
		wc_printf("pdata_l:0x%08x, pdata_h:0x%08x, len:%d\n",
				bdesc->pdata_l,
				bdesc->pdata_h,
				bdesc->len);
	}
}
#endif

static inline void scm2020_wc_set_bssid_mask(u8 bssid_indicator)
{
	u8 mask[IEEE80211_ADDR_LEN];
	u64 mask_u64;
	u32 vif = 0;

	mask_u64 = ether_addr_to_u64(etherbroadcastaddr);
	if (bssid_indicator > 0)
		mask_u64 &= ~GENMASK64(bssid_indicator - 1, 0);

	u64_to_ether_addr(mask_u64, mask);

	wc_mac_writel(get_unaligned_le32(mask), REG_VIF_MBSSID_MASK_L32(vif));
	wc_mac_writel(get_unaligned_le16(mask + 4), REG_VIF_MBSSID_MASK_H16(vif));
}

static inline void scm2020_wc_set_bssid(u8 non_trans_node, const u8 *assoc_bssid, const u8 *trans_bssid)
{
	u32 bssid_l, bssid_h;
	u32 vif = 0;

	/* BSSID0: associated BSSID */
	bssid_l = get_unaligned_le32(assoc_bssid);
	bssid_h = get_unaligned_le16(assoc_bssid + 4);
	/* BSS0_L#vif, BSS0_H#vif */
	wc_mac_writel(bssid_l, REG_VIF_BSS0_L(vif));
	wc_mac_writel(bssid_h, REG_VIF_BSS0_H(vif));

	if (non_trans_node) {
		/* BSSID1: transmitted BSSID */
		bssid_l = get_unaligned_le32(trans_bssid);
		bssid_h = get_unaligned_le16(trans_bssid + 4);
		wc_mac_writel(bssid_l, REG_VIF_BSS1_L(vif));
		wc_mac_writel(bssid_h, REG_VIF_BSS1_H(vif));
	} else {
		/* Set BSSID1 as the same value as BSSID0 firstly */
		wc_mac_writel(bssid_l, REG_VIF_BSS1_L(vif));
		wc_mac_writel(bssid_h, REG_VIF_BSS1_H(vif));
	}
}


static inline void scm2020_wc_config_rx_filter(u8 non_trans_node) __attribute__((unused));
static inline void scm2020_wc_config_rx_filter(u8 non_trans_node)
{
	u32 v, vif = 0;

	v = wc_mac_readl(REG_VIF_RX_FILTER0(vif));

	/* Disable AP_Mode filter */
	bfclr_m(v, VIF0_RX_FILTER0, AP_MODE);

	/*
	 * configures the interface's BSSID filter. If disabled, BSSID
	 * filter passes all the group addressed frames: otherwise, the filter
	 * passes only those frames whose BSSID matches at least one of BSSIDs of the vif.
	 */
	/* RX_FILTER0#vif - BSS_EN */
	if (non_trans_node) {
		bfmod_m(v, VIF0_RX_FILTER0, BSS_EN, 3);   /* BSS_0_1_EN */
	} else {
		bfmod_m(v, VIF0_RX_FILTER0, BSS_EN, 1);  /* BSS_0_EN */
	}

	/* RX_FILTER0#vif - BFE_EN */
	bfmod_m(v, VIF0_RX_FILTER0, BFE, 1);

	wc_mac_writel(v, REG_VIF_RX_FILTER0(vif));
}

static inline void scm2020_wc_restore_init()
{
	u32 v;

	/* Restore normal initialized Registers */

	/* 1. scm2020_hmac_init() */

	/* RX_CFG_PRO: promisc mode off */
	wc_mac_writel(0, REG_RX_CFG);

	/* BFM0#vif - CBF format, MCS to HT/VHT NDPA/NDP */
	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_bfm0, REG_VIF_BFM0(0));

	/* BFM1#vif - CBF format, MCS to HE NDPA/NDP */
	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_bfm1, REG_VIF_BFM1(0));

	/* TX_TO_CFG: HW tx timeout mechanism */
	wc_mac_writel(0x1fff0001, REG_TX_TO_CFG);

	/* TX_TO_11B_CFG: tx timeout 16ms for CCK rate */
	wc_mac_writel(0x00003fff, REG_TX_TO_11B_CFG);

	/* RX_TO_CFG: HW rx timeout mechanism */
	wc_mac_writel(0x00012c00, REG_RX_TO_CFG);

	/* RX_TO_11B_CFG: rx timeout for CCK rate */
	wc_mac_writel(0x00008000, REG_RX_TO_11B_CFG);

	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_dev_opt2, REG_DEV_OPT2);

	/* Run firmware binary into IMEM */
	wc_mac_writel(1, REG_MCU_CFG);

	/* 2. Phy Reg Init scm2020_phy_init() */
#ifdef PM_SOC_QFN40_FIB	/* CONFIG_RF_ACTT */
	/* For PM testing in FIB_V1 SoC */
	wc_phy_init();

	wc_phy_writel(0x00200000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_00);
	wc_phy_writel(0x00300000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_01);
	wc_phy_writel(0x00200000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_06);
	wc_phy_writel(0x002e0000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_07);
	wc_phy_writel(0x00350000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_10);
	wc_phy_writel(0x00200000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_11);
	wc_phy_writel(0x002a0000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_12);
	wc_phy_writel(0x00200000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_13);
	wc_phy_writel(0x00350000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_14);
	wc_phy_writel(0x00200000, PHY_RFITF, FW_RFI_CTRLSIG_CFG_15);

	wc_phy_writel(0x0000504e, PHY_MODEM, PHY_AGC_INDEX_CFG);
	wc_phy_writel(0x0012161a, PHY_MODEM, PHY_CS_CONFIG_THRD);

#endif

	/* FW_PWR_EN restoring */
	wc_phy_writel(0x00001FF1, PHY_MODEM, FW_PWR_EN);

	/* Init just RX lut ASSP(2 step) */
	wc_phy_rx_lut_restore();

	/* Enable 11b RSSI check : default is disable(1)*/
#if 0
	v = wc_phy_readl(PHY_MODEM, PHY_11B_CS_PARAM);
	bfmod_p(v, PHY_11B_CS_RSSICHK_OFF, 0);
	wc_phy_writel(v, PHY_MODEM, PHY_11B_CS_PARAM);
#endif
	/* my mac addr assign */
	v = get_unaligned_le32(WATCHER_MAC_REG_PTR->reg_iv_myaddr);
	wc_mac_writel(v, REG_VIF_MAC0_L(0));
	v = get_unaligned_le16(WATCHER_MAC_REG_PTR->reg_iv_myaddr + 4);
	wc_mac_writel(v, REG_VIF_MAC0_H(0));

	return;
}

/**
 * In watcher,
 * MAC/PHY Register should be restored to the connection state before working.
 */
static inline void scm2020_wc_pm_restore_run()
{
	int vif = 0;

	/* Without setting channel, no wifi interrupt */
	scm2020_wc_set_channel_freq(WATCHER_MAC_REG_PTR->reg_chanfreq);

	/* move to here , for running earlier */
	/* BSSID0: associated BSSID/transmitter BSSID[MBSSID] & BSSID Mask */
	scm2020_wc_set_bssid(WATCHER_BCN_WINDOW_PTR->non_trans_node,
		WATCHER_MAC_REG_PTR->reg_ni_bssid, WATCHER_MAC_REG_PTR->reg_transmitter_bssid);
	scm2020_wc_set_bssid_mask(WATCHER_BCN_WINDOW_PTR->ni_bssid_indicator);

#if 0
	/* RX Filter assign */
	scm2020_wc_config_rx_filter(WATCHER_BCN_WINDOW_PTR->non_trans_node);
#endif

	/* Aid assign */
	wc_mac_writel(IEEE80211_AID(WATCHER_MAC_REG_PTR->reg_ni_associd), REG_VIF_AID(0));

	/* basic rate assign */
	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_cfg1, REG_VIF_CFG1(vif));

	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_cfg0, REG_VIF_CFG0(vif));

	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_dev_cfg, REG_DEV_CFG);

	/* HT related assign */
	/* scm2020_phy_he_sta_id_filter */
	if (WATCHER_MAC_REG_PTR->reg_niflag & IEEE80211_NODE_HE) {
		wc_phy_writel(WATCHER_MAC_REG_PTR->reg_fw_he_config1, PHY_MODEM, FW_HE_CONFIG(1, vif));
		wc_phy_writel(WATCHER_MAC_REG_PTR->reg_fw_he_config2, PHY_MODEM, FW_HE_CONFIG(2, vif));

		wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_cfg_info, REG_VIF_CFG_INFO(vif));
	}

	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_min_mpdu_spacing, REG_VIF_MIN_MPDU_SPACING(vif));
}

#define HW_TX_HDR_LEN 28

static inline void scm2020_wc_txop_kick(struct watcher_pkt_hdr *m, int hwqueue, u8 pkt_type)
{
	volatile struct scm_mac_shmem_map *shmem;
	volatile struct mentry *me;
	volatile struct txdesc *txdesc;
	struct bdesc *bd0;
	int idx = 0;
	u16 seqno;
	uint16_t *seq;
	u32 cmd = 0, slot;
	struct ieee80211_frame *hdr = NULL;

	/* Get the MAC SHARED MEM */
	shmem = (struct scm_mac_shmem_map *)(MAC_BASE_START + MAC_SHM_OFFSET);

	/* Get the TX desc from shared mem */
	txdesc = &shmem->tx[hwqueue];

	switch (pkt_type) {
		case WATCHER_PS_POLL_PKT:
			me = (struct mentry *)(&WATCHER_PS_POLL_TX_PTR->me);
			bd0 = (struct bdesc *)(&WATCHER_PS_POLL_TX_PTR->bd);
			seqno = 0;
			break;
		case WATCHER_NULL_PKT:
			me = (struct mentry *)(&WATCHER_NULL_TX_PTR->me);
			bd0 = (struct bdesc *)(&WATCHER_NULL_TX_PTR->bd);
			break;
		case WATCHER_GARP_PKT:
			me = (struct mentry *)(&WATCHER_GARP_TX_PTR->me);
			bd0 = (struct bdesc *)(&WATCHER_GARP_TX_PTR->bd);
			break;
		case WATCHER_UDP_HOLE_PKT:
			me = (struct mentry *)(&WATCHER_UDP_HOLE_TX_PTR->me);
			bd0 = (struct bdesc *)(&WATCHER_UDP_HOLE_TX_PTR->bd);
			break;
		case WATCHER_TCP_HOLE_PKT:
			me = (struct mentry *)(&WATCHER_TCP_HOLE_TX_PTR->me);
			bd0 = (struct bdesc *)(&WATCHER_TCP_HOLE_TX_PTR->bd);
			break;
		case WATCHER_ARP_RESP_PKT:
			me = (struct mentry *)(&WATCHER_ARP_RESP_TX_PTR->me);
			bd0 = (struct bdesc *)(&WATCHER_ARP_RESP_TX_PTR->bd);
			break;
		default:
			return;

	}

	/* Get the seq number of data packet & increment */
	if (pkt_type != WATCHER_PS_POLL_PKT) {
		hdr = (struct ieee80211_frame *)(m->data + HW_TX_HDR_LEN);
		seqno = (WATCHER_BCN_WINDOW_PTR->nonqos_seqno++) & (IEEE80211_SEQ_RANGE - 1);
		seq = (uint16_t *)&hdr->i_seq[0];
		*seq = htole16(seqno << IEEE80211_SEQ_SEQ_SHIFT);

		if (pkt_type == WATCHER_GARP_PKT || pkt_type == WATCHER_ARP_RESP_PKT ||
			pkt_type == WATCHER_UDP_HOLE_PKT || pkt_type == WATCHER_TCP_HOLE_PKT) {
			if (wc_crypto_setiv)
				wc_crypto_setiv((uint8_t *)&hdr[1]);
		}
	}
	txdesc->ptable_l = (u32)me;
	txdesc->ptable_h = 0;

	{
		bd0->pdata_l = (u32)m->data;
		bd0->pdata_h = 0;
		bd0->len = m->len;

		me->pbd_l = (u32)bd0;
		me->pbd_h = 0;
		/* MPDU Length */
		me->len = m->len - HW_TX_HDR_LEN + 4 + WATCHER_SEC_KEY_PTR->ciphertrailer; // IEEE80211_CRC_LEN;
		me->aid = IEEE80211_AID(WATCHER_MAC_REG_PTR->reg_ni_associd);
		me->tid = 0;	//NON_TID(16)
		me->sn = seqno;

		me->noack = 0;	//pspoll need ack
		me->eof = 0;
		me->noenc = 0;
		/*
		 * NB: need to clear (sent, ack) because HW might refer to them while collecting
		 * MPDUs to transmit in response to Trigger frame or TRS Control
		 */
		me->sent = 0;
		me->ack = 0;
		me->more = 0;
		me->num = 0; /* # of buffer descriptors - 1 */
		me->prot = 0;	//11b 1Mhz, prate is 0
		me->prot_dur = 0;	//prate is 0
		me->spacing = 0;	//do not need spacing
		me->ts_new = 0;
		me->ts_off = 0;
		me->retry = 0;
		me->htc = 0;	//add_htc is false
		me->sw0 = (u32)m;
		me->sw1 = 0;
		me->sw2 = 0;

		idx++;
	}

	txdesc->ok = 0;
	/* assume txq->desc->fail will always be updated by hw */
	txdesc->fail = 0xF;
	txdesc->num = idx;
	dmb();

	bfzero(cmd);
	if (pkt_type == WATCHER_PS_POLL_PKT) {
		wc_mac_writel(WATCHER_PS_POLL_TX_PTR->txop_limit, REG_TX_QUEN_TXOP(hwqueue));
		slot = rand() & WATCHER_PS_POLL_TX_PTR->cw;
		bfmod_m(cmd, TX_QUEN_CMD0, BOFF_COUNT, slot);
		bfmod_m(cmd, TX_QUEN_CMD0, AIFSN, WATCHER_PS_POLL_TX_PTR->aifsn);
	}
	else if (pkt_type == WATCHER_NULL_PKT) {
		wc_mac_writel(WATCHER_NULL_TX_PTR->txop_limit, REG_TX_QUEN_TXOP(hwqueue));
		slot = rand() & WATCHER_NULL_TX_PTR->cw;
		bfmod_m(cmd, TX_QUEN_CMD0, BOFF_COUNT, slot);
		bfmod_m(cmd, TX_QUEN_CMD0, AIFSN, WATCHER_NULL_TX_PTR->aifsn);
	}
	else if (pkt_type == WATCHER_GARP_PKT) {
		wc_mac_writel(WATCHER_GARP_TX_PTR->txop_limit, REG_TX_QUEN_TXOP(hwqueue));
		slot = rand() & WATCHER_GARP_TX_PTR->cw;
		bfmod_m(cmd, TX_QUEN_CMD0, BOFF_COUNT, slot);
		bfmod_m(cmd, TX_QUEN_CMD0, AIFSN, WATCHER_GARP_TX_PTR->aifsn);
	}
	else if (pkt_type == WATCHER_UDP_HOLE_PKT) {
		wc_mac_writel(WATCHER_UDP_HOLE_TX_PTR->txop_limit, REG_TX_QUEN_TXOP(hwqueue));
		slot = rand() & WATCHER_UDP_HOLE_TX_PTR->cw;
		bfmod_m(cmd, TX_QUEN_CMD0, BOFF_COUNT, slot);
		bfmod_m(cmd, TX_QUEN_CMD0, AIFSN, WATCHER_UDP_HOLE_TX_PTR->aifsn);
	}
	else if (pkt_type == WATCHER_TCP_HOLE_PKT) {
		wc_mac_writel(WATCHER_TCP_HOLE_TX_PTR->txop_limit, REG_TX_QUEN_TXOP(hwqueue));
		slot = rand() & WATCHER_TCP_HOLE_TX_PTR->cw;
		bfmod_m(cmd, TX_QUEN_CMD0, BOFF_COUNT, slot);
		bfmod_m(cmd, TX_QUEN_CMD0, AIFSN, WATCHER_TCP_HOLE_TX_PTR->aifsn);
	}
	else if (pkt_type == WATCHER_ARP_RESP_PKT) {
		wc_mac_writel(WATCHER_ARP_RESP_TX_PTR->txop_limit, REG_TX_QUEN_TXOP(hwqueue));
		slot = rand() & WATCHER_ARP_RESP_TX_PTR->cw;
		bfmod_m(cmd, TX_QUEN_CMD0, BOFF_COUNT, slot);
		bfmod_m(cmd, TX_QUEN_CMD0, AIFSN, WATCHER_ARP_RESP_TX_PTR->aifsn);
	}
	bfmod_m(cmd, TX_QUEN_CMD0, CMD_EDCA, 1);

	wc_mac_writel(cmd, REG_TX_QUEN_CMD(hwqueue));

#ifdef WATCHER_TX_DUMP
	if (pkt_type == WATCHER_ARP_RESP_PKT || pkt_type == WATCHER_TCP_HOLE_PKT || pkt_type == WATCHER_UDP_HOLE_PKT) {
		scm2020_wc_dump_tx();
		wc_hexdump(m->data , m->len);
	}
#endif
	return;
}

/*
 * Restore MAC(Basic MAC, Connection info, RX Desc, Key)
 */
static inline void scm2020_wc_mac_restore(void)
{
	scm2020_wc_restore_init();

	scm2020_wc_pm_restore_run();
	scm2020_wc_rx_desc_restore();

	/* in advance, security key setting */
	scm2020_wc_sec_key_restore();

	/* RX Filter configure */
	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_rx_filter0, REG_VIF_RX_FILTER0(0));

	/* Fix me lator, all of filter do not need to set */
#ifdef WATCHER_WIFI_INT_TEST
	wc_mac_writel(WATCHER_MAC_REG_PTR->reg_vif_rx_filter1, REG_VIF_RX_FILTER1(0));
#else
	u32 val;

	val = RX_FILTER_WATCHER;
	wc_mac_writel(val, REG_VIF_RX_FILTER1(0));
#endif
	wc_hmac_enable(true);
}

/*
 * Set RFMODE as Standby mode
 */
static inline void scm2020_wc_set_rf_sleep(void)
{
	uint32_t val;

	/* RF directly controling is useless */
	/* Set RF Mode as Standby */

	/* set RFMODE(RFI_ACTT_CFG2) of PHY as standby mode */
	val = readl(0xf0f28008);	// read RF Mod of PHY
	val &= ~GENMASK(27, 12);
	writel(val | (0x0421 << 12), 0xf0f28008);
}

/*
 * Set RFMODE as default mode
 */
static inline void scm2020_wc_set_rf_wakeup(void)
{
	uint32_t val;

	/* RFMODE(RFI_ACTT_CFG2) of PHY set as default mode */
	val = readl(0xf0f28008);
	val &= ~GENMASK(27, 12);
	writel(val | (0x1f82 << 12), 0xf0f28008);
}

/*
 * Send a ps poll frame to the specified node.
 */
static inline int scm2020_wc_send_pspoll(void)
{
	struct watcher_pkt_hdr *m;
	u32 val;

	m = (struct watcher_pkt_hdr *)&(WATCHER_PS_POLL_TX_PTR->pkt);	/* in AON Retention mem */

	watcher_rxbuf_interrupt_cnt = 0;

	/* Enable RX interrrupt in advance */
	/* for receiving BC/MC well after beacon, run ealier DTIM parser togather */
	watcher_rxbuf_int_proc = 1;

	/* Interrupt enable */
	/* If enable other(RX_BUF.), by interrupt, working scenario will be happened some problem */
	bfzero(val);
	bfmod_m(val, INTR_ENABLE, TX_DONE, RBFM(INTR_ENABLE, TX_DONE));
	bfmod_m(val, INTR_ENABLE, RX_BUF, 1);
	wc_mac_writel(val, REG_INTR_ENABLE);

	set_wlan_irq((1));

	/* tx the PS Poll frame */
	scm2020_wc_txop_kick(m , 0 , WATCHER_PS_POLL_PKT);
	return 0;
}

static inline void scm2020_wc_send_tx(void)
{
#if defined(CONFIG_WATCHER_TX_KEEPALIVE) || defined(CONFIG_WATCHER_TX_GARP)
	struct watcher_pkt_hdr *m;

	/* tx the null frame for keepalive */
	if (watcher_tx_pkt_type & WATCHER_NULL_PKT) {
		m = (struct watcher_pkt_hdr *)&(WATCHER_NULL_TX_PTR->pkt);
		scm2020_wc_txop_kick(m , 0 , WATCHER_NULL_PKT);
		wc_printf("NULL Packet TX\n");
	}

	/* tx the null frame for garp */
	if (watcher_tx_pkt_type & WATCHER_GARP_PKT) {
		m = (struct watcher_pkt_hdr *)&(WATCHER_GARP_TX_PTR->pkt);
		scm2020_wc_txop_kick(m , 0 , WATCHER_GARP_PKT);
		wc_printf("GARP Packet TX\n");
	}
#endif

#ifdef CONFIG_WATCHER_TX_UDPHOLE
	/* tx UDP Hole punch packet */
	if (watcher_tx_pkt_type & WATCHER_UDP_HOLE_PKT) {
		m = (struct watcher_pkt_hdr *)&(WATCHER_UDP_HOLE_TX_PTR->pkt);
		scm2020_wc_txop_kick(m , 0 , WATCHER_UDP_HOLE_PKT);
		wc_printf("UDP HolePunch Packet TX\n");
	}
#endif

#ifdef CONFIG_WATCHER_TX_TCPHOLE
	/* tx TCP Hole punch packet */
	if (watcher_tx_pkt_type & WATCHER_TCP_HOLE_PKT) {
		m = (struct watcher_pkt_hdr *)&(WATCHER_TCP_HOLE_TX_PTR->pkt);
		scm2020_wc_txop_kick(m , 0 , WATCHER_TCP_HOLE_PKT);
		wc_printf("TCP HolePunch Packet TX\n");
	}
#endif

#ifdef CONFIG_WATCHER_TX_ARP_RESP
	/* If ARP Response is enabled , Send the ARP Response */
	if (watcher_tx_pkt_type & WATCHER_ARP_RESP_PKT) {
		u32 tx_start;
		m = (struct watcher_pkt_hdr *)&(WATCHER_ARP_RESP_TX_PTR->pkt);
		scm2020_wc_txop_kick(m , 0 , WATCHER_ARP_RESP_PKT);

		tx_start = wc_ktime();
		scm2020_wc_txdone(WATCHER_ARP_RESP_TX_PTR, WATCHER_ARP_RESP_PKT, tx_start);
		wc_printf("ARP RESP Packet TX\n");
	}
#endif
	return;
}

static inline void scm2020_wc_send_pkt(bool pm_bit)
{
#if defined(CONFIG_WATCHER_TX_KEEPALIVE) || defined(CONFIG_WATCHER_TX_GARP)
	u32 val;

	if (watcher_tx_pkt_type == WATCHER_TX_NO_PKT)
		return;

#ifdef CONFIG_WATCHER_TX_KEEPALIVE
	struct ieee80211_frame *nullframe;
	struct watcher_pkt_hdr *m_null;

	/* tx the null frame for keepalive */
	if (watcher_tx_pkt_type & WATCHER_NULL_PKT) {
		m_null = (struct watcher_pkt_hdr *)&(WATCHER_NULL_TX_PTR->pkt);	/* in AON Retention mem */
		nullframe = (struct ieee80211_frame *)(m_null->data + HW_TX_HDR_LEN);

		/* fill the NULL frame */
		if (pm_bit)
			nullframe->i_fc[1] |= IEEE80211_FC1_PWR_MGT;
		else
			nullframe->i_fc[1] &= ~IEEE80211_FC1_PWR_MGT;
	}
#endif

#ifdef CONFIG_WATCHER_TX_GARP
	struct watcher_pkt_hdr *m_garp;

	if (watcher_tx_pkt_type & WATCHER_GARP_PKT) {
		m_garp = (struct watcher_pkt_hdr *)&(WATCHER_GARP_TX_PTR->pkt); /* in AON Retention mem */
	}
#endif

#ifdef CONFIG_WATCHER_TX_UDPHOLE
	struct watcher_pkt_hdr *m_udphole;

	if (watcher_tx_pkt_type & WATCHER_UDP_HOLE_PKT) {
		m_udphole = (struct watcher_pkt_hdr *)&(WATCHER_UDP_HOLE_TX_PTR->pkt); /* in AON Retention mem */
	}
#endif

#ifdef CONFIG_WATCHER_TX_TCPHOLE
	struct watcher_pkt_hdr *m_tcphole;

	if (watcher_tx_pkt_type & WATCHER_TCP_HOLE_PKT) {
		m_tcphole = (struct watcher_pkt_hdr *)&(WATCHER_TCP_HOLE_TX_PTR->pkt); /* in AON Retention mem */
	}
#endif
	watcher_rxbuf_interrupt_cnt = 0;

	/* Interrupt enable */
	/* If enable other(RX_BUF.), by interrupt, working scenario will be happened some problem */
	bfzero(val);
	bfmod_m(val, INTR_ENABLE, TX_DONE, RBFM(INTR_ENABLE, TX_DONE));
	bfmod_m(val, INTR_ENABLE, RX_BUF, 1);
	wc_mac_writel(val, REG_INTR_ENABLE);

	set_wlan_irq(1);

#ifdef CONFIG_WATCHER_TX_KEEPALIVE
	/* tx the null frame */
	if (watcher_tx_pkt_type & WATCHER_NULL_PKT) {
		scm2020_wc_txop_kick(m_null , 0 , WATCHER_NULL_PKT);
		wc_printf("NULL Packet TX\n");
	}
#endif

#ifdef CONFIG_WATCHER_TX_GARP
	if (watcher_tx_pkt_type & WATCHER_GARP_PKT) {
		scm2020_wc_txop_kick(m_garp , 0 , WATCHER_GARP_PKT);
		wc_printf("GARP Packet TX\n");
	}
#endif

#ifdef CONFIG_WATCHER_TX_UDPHOLE
	if (watcher_tx_pkt_type & WATCHER_UDP_HOLE_PKT) {
		scm2020_wc_txop_kick(m_udphole , 0 , WATCHER_UDP_HOLE_PKT);
		wc_printf("UDP Hole Punch Packet TX\n");
	}
#endif

#ifdef CONFIG_WATCHER_TX_TCPHOLE
	if (watcher_tx_pkt_type & WATCHER_TCP_HOLE_PKT) {
		scm2020_wc_txop_kick(m_tcphole , 0 , WATCHER_TCP_HOLE_PKT);
		wc_printf("TCP Hole Punch Packet TX\n");
	}
#endif

	/* Interrupt disable, stop DTIM Parser */
	scm2020_wc_dtim_parser_stop();
#endif
	return;
}

#ifdef CONFIG_WATCHER_TX_ARP_RESP
/* Make ARP Response packet */
static void scm2020_ps_make_arp_resp(uint8_t *arp_header)
{
	struct ieee80211_frame *wh;
	struct watcher_pkt_hdr *pkt;
	struct watcher_arpframe *arp_resp;

	/* GARP is Enabled and TX configured */
#ifdef CONFIG_WATCHER_TX_GARP
	/* GARP TX flag --> disable */
	watcher_tx_pkt_type &= ~WATCHER_GARP_PKT;
#endif

	/* Didn't yet make ARP RESP packet, return */
	if (!WATCHER_ARP_RESP_TX_PTR->pkt.len)
		return;

	pkt = (struct watcher_pkt_hdr *)&(WATCHER_ARP_RESP_TX_PTR->pkt); /* in AON Retention mem */
	wh = (struct ieee80211_frame *)(pkt->data + HW_TX_HDR_LEN);
	arp_resp = (struct watcher_arpframe *)&wh[1];

	if (WATCHER_SEC_KEY_PTR->sec_mode) {
		arp_resp = (struct watcher_arpframe *)((uint8_t *)arp_resp + WATCHER_SEC_KEY_PTR->cipherheader);
	}

	/* DST MAC Assign in MAC Header  */
	IEEE80211_ADDR_COPY(wh->i_addr3, &arp_header[8]);

	/* DST MAC assign in ARP header */
	IEEE80211_ADDR_COPY(arp_resp->arp_hdr.target_mac, &arp_header[8]);

	/* Target IP assign in ARP header */
	memcpy (arp_resp->arp_hdr.target_ip , &arp_header[14] , 4);

	watcher_tx_pkt_type |= WATCHER_ARP_RESP_PKT;
}
#endif

static void chk_wc_beacon_window_adjust(bool beacon_rx_int)
{
	if (beacon_rx_int) /* we receive beacon by DTIM Parser */
	{
		WATCHER_BCN_WINDOW_PTR->pm_beacon_rx_count++;
		if (WATCHER_BCN_WINDOW_PTR->pm_beacon_rx_count >= PM_BCN_WINDOW_THRESHOLD) {
			if (WATCHER_BCN_WINDOW_PTR->current_beacon_window > 0) {
				/* Current Beacon window decrease */
				WATCHER_BCN_WINDOW_PTR->current_beacon_window -= 1;
			}
			WATCHER_BCN_WINDOW_PTR->pm_beacon_rx_count = 0;
		}
		WATCHER_BCN_WINDOW_PTR->pm_beacon_loss_count = 0;
		WATCHER_BCN_WINDOW_PTR->pm_bcn_loss_chk_cnt = 0;
	} else /* we do not receive beacon */
	{
		WATCHER_BCN_WINDOW_PTR->pm_beacon_loss_count++;
		WATCHER_BCN_WINDOW_PTR->pm_bcn_loss_chk_cnt++;
		if (WATCHER_BCN_WINDOW_PTR->pm_beacon_loss_count >= PM_BCN_WINDOW_THRESHOLD) {
			WATCHER_BCN_WINDOW_PTR->current_beacon_window += 1; //Current Beacon window increase
			if (WATCHER_BCN_WINDOW_PTR->current_beacon_window >= MAX_RX_BEACON_WINDOW) {
				/* set last rx beacon window */
				WATCHER_BCN_WINDOW_PTR->current_beacon_window = (MAX_RX_BEACON_WINDOW - 1);
			}
			WATCHER_BCN_WINDOW_PTR->pm_beacon_loss_count = 0;
		}
		WATCHER_BCN_WINDOW_PTR->pm_beacon_rx_count = 0;
	}
}

/* In case of important packet, can be retransmitted ack flag and tx-done interupt */
#define TX_DONE_INT_TIMEOUT	20		/* TX Done interrupt Timeout */
static void inline scm2020_wc_txdone(struct watcher_tx_pkt_ctx *tx_pkt, u8 pkt_type, u32 tx_start)
{
	volatile struct mentry *me = (struct mentry *)(&tx_pkt->me);
	watcher_txdone_interrupt_cnt = 0;

	do {
		if (watcher_txdone_interrupt_cnt) {
			if (me->sent && me->ack) {
				break;
			} else {
				watcher_txdone_interrupt_cnt = 0;
				scm2020_wc_txop_kick(&(tx_pkt->pkt), 0, pkt_type);
			}
		}
		if (wc_abs(wc_ktime() - tx_start) > MSEC_TO_RTC(TX_DONE_INT_TIMEOUT)) {
			break;
		}
	} while(1);
}

static inline u64 scm2020_wc_bcn_rx_proc(u32 waited_time)
{
	u64 sleep_duration_us;
	int val;
	u32 tx_start;
	u32 rx_start;
	bool dtim_beacon = 0;
	bool aid0_bitmap = 0;
	u32 curbcn_window_ms = beacon_window_array[WATCHER_BCN_WINDOW_PTR->current_beacon_window];

	/* rx beacon window adjusting */
	chk_wc_beacon_window_adjust(true);

	/* Read REG_DTIM_INFO and check AID Bitmap(UC), and AID0 Bitmap(BC/MC) */
	val = wc_mac_readl(REG_RX_DTIM_INFO);

	/* Detect the UC Bitmap or AID0 Bitmap or traffic indicator */
	aid0_bitmap = bfget_m(val, RX_DTIM_INFO, RX_DTIM_AID0_BITMAP_HIT);
	if (bfget_m(val, RX_DTIM_INFO, RX_DTIM_BITMAP_HIT) || aid0_bitmap \
		|| bfget_m(val, RX_DTIM_INFO, RX_TRAFFIC_INDICATOR)) {
		wc_printf("[PM WiFi] wlan active with UC/BC\n");
		sleep_duration_us = 0;

		dtim_beacon = bfget_m(val, RX_DTIM_INFO, RX_IS_DTIM);

		scm2020_wc_send_pspoll();
		tx_start = wc_ktime();
		scm2020_wc_txdone(WATCHER_PS_POLL_TX_PTR, WATCHER_PS_POLL_PKT, tx_start);

		u32 watcher_rx_timeout = RX_PKT_WAIT_TIMEOUT;
		watcher_rxbuf_interrupt_cnt = 0;

		/* For BC/MC, in case dtim beacon, let's wait double */
		if (dtim_beacon || aid0_bitmap)
			watcher_rx_timeout += RX_PKT_WAIT_TIMEOUT;

		/* In case of longer duration, let's wait more */
		if (WATCHER_BCN_WINDOW_PTR->pm_sleep_duration > BCN_BASIC_DURATION) {
			watcher_rx_timeout += (WATCHER_BCN_WINDOW_PTR->pm_sleep_duration / BCN_BASIC_DURATION) * 5;
		}
		rx_start = wc_ktime();

		/* do RX Data packet(interrupt) Timeout */
		do {
			if (watcher_rxbuf_interrupt_cnt) {
				break;
			}
		} while(wc_abs(wc_ktime() - rx_start) < MSEC_TO_RTC(watcher_rx_timeout));

		/* Watcher tx Null or Garp UDP/TCP Hole punch */
		scm2020_wc_send_tx();

		/* Interrupt disable, stop DTIM Parser */
		scm2020_wc_dtim_parser_stop();

		/* Next wakeup scheduling */
		/* required Full booting for processing RX packet */
		if (WATCHER_BCN_WINDOW_PTR->not_rx_desc_restore) {
			sleep_duration_us = 0;
		}
		else {
			/* do not need Full booting for filtering RX packet */
			sleep_duration_us = WATCHER_BCN_WINDOW_PTR->pm_sleep_duration;
		}
	}
	/* No packet and No tx packet in tx queue */
	else {
		wc_printf("No UC/BC time wait::recv (%dms::%dms)\n", curbcn_window_ms, waited_time);

		/* if waited time is under 2 ms , let's decrease a little current sleep duration */
		sleep_duration_us = WATCHER_BCN_WINDOW_PTR->pm_sleep_duration;

		/* Watcher tx Null or Garp */
		scm2020_wc_send_pkt(1);
	}
	return sleep_duration_us;
}

static inline u64 scm2020_wc_dtim_chk_timeout(void)
{
	u8 bcn_loss_threshold = CONFIG_CONS_BCN_LOSS_THRESHOLD;

	/* beacon dtim parser interrupt is not happened, so fail to rx beacon */

	/* rx beacon window adjusting */
	chk_wc_beacon_window_adjust(false);

	/* check if AP status is ok with consecutive bcn loss count */

	/* improve watcher beacon loss rate in case of longer duration */
	if (WATCHER_BCN_WINDOW_PTR->pm_sleep_duration > BCN_BASIC_DURATION) {
		/* Basic threshold - Beacon interval */
		/* 1sec : 20 - 6 */
		bcn_loss_threshold -= (WATCHER_BCN_WINDOW_PTR->pm_sleep_duration / BCN_BASIC_DURATION) * 2;
	}
	if (WATCHER_BCN_WINDOW_PTR->pm_bcn_loss_chk_cnt > bcn_loss_threshold) {
		wc_printf("[PM WiFi] Full booting by consecutive bcn loss\n");
		WATCHER_BCN_WINDOW_PTR->pm_bcn_loss_chk_cnt = 0;
		WATCHER_BCN_WINDOW_PTR->pm_wifi_active_state = 1;
		return 0;
	}
	/* Watcher tx Null or Garp */
	scm2020_wc_send_pkt(1);

	/** If current beacon window was changed by beacon_window_adjust(),
	 * decrease sleep duration, wakeup earlier than before
	 */
	return (u64)WATCHER_BCN_WINDOW_PTR->pm_sleep_duration;
}

static uint8_t wc_tx_schedule(void)
{
	uint8_t tx_pkt_type = WATCHER_TX_NO_PKT;

#ifdef CONFIG_WATCHER_TX_KEEPALIVE
	/* About Null Keepalive, watcher wakeup & check it tx_time is cleared */
	if (!WATCHER_NULL_TX_PTR->tx_time) {
		/* set keepalive start time. */
		WATCHER_NULL_TX_PTR->tx_time = wc_ktime();
	}
	else {
		/* if (tx time under keepalive start time ) */
		if (RTCDIFF_TO_MSEC(WATCHER_NULL_TX_PTR->tx_time) > WC_NULL_PERIODIC_TX_TIME) {
			/* In this watcher wakeup, TX Null time should be cleared */
			tx_pkt_type |= WATCHER_NULL_PKT;
			WATCHER_NULL_TX_PTR->tx_time = 0;
		}
	}
#endif

	/* IP address is not assigned yet, GARP/TCP HP/UDP HP do not transmit */
	if (!scm2020_ps_get_net_ipaddr()) {
		WATCHER_GARP_TX_PTR->tx_time = 0;
		WATCHER_UDP_HOLE_TX_PTR->tx_time = 0;
		WATCHER_TCP_HOLE_TX_PTR->tx_time = 0;
		return tx_pkt_type;
	}

#ifdef CONFIG_WATCHER_TX_GARP
	/* About GARP, watcher wakeup & check it tx_time is cleared */
	if (WATCHER_GARP_TX_PTR->pkt.len) {

		if (!WATCHER_GARP_TX_PTR->tx_time) {
			/* set keepalive start time. */
			WATCHER_GARP_TX_PTR->tx_time = wc_ktime();
		}
		else {
			/* if (tx time under garp start time ) */
			if (RTCDIFF_TO_MSEC(WATCHER_GARP_TX_PTR->tx_time) > WC_GARP_PERIODIC_TX_TIME) {
				/* In this watcher wakeup, TX garp time should be cleared */
				tx_pkt_type |= WATCHER_GARP_PKT;
				WATCHER_GARP_TX_PTR->tx_time = 0;
			}
		}
	}
#endif

#ifdef CONFIG_WATCHER_TX_UDPHOLE
	/* If udp filtering port is exist */
	if (WATCHER_UDP_HOLE_TX_PTR->pkt.len) {

		/* About UDP Hole punch, watcher wakeup & check it tx_time is cleared */
		if (!WATCHER_UDP_HOLE_TX_PTR->tx_time) {
			/* set keepalive start time. */
			WATCHER_UDP_HOLE_TX_PTR->tx_time = wc_ktime();
		}
		else {
			/* if (tx time under garp start time ) */
			if (RTCDIFF_TO_MSEC(WATCHER_UDP_HOLE_TX_PTR->tx_time) > WC_UDPHOLE_PERIODIC_TX_TIME) {
				/* In this watcher wakeup, TX UDP Hole punch time should be cleared */
				tx_pkt_type |= WATCHER_UDP_HOLE_PKT;
				WATCHER_UDP_HOLE_TX_PTR->tx_time = 0;
			}
		}
	}
#endif

#ifdef CONFIG_WATCHER_TX_TCPHOLE
	/* If tcp filtering port is exist */
	if (WATCHER_TCP_HOLE_TX_PTR->pkt.len) {

		/* About TCP Hole punch, watcher wakeup & check it tx_time is cleared */
		if (!WATCHER_TCP_HOLE_TX_PTR->tx_time) {
			/* set keepalive start time. */
			WATCHER_TCP_HOLE_TX_PTR->tx_time = wc_ktime();
		}
		else {
			/* if (tx time under garp start time ) */
			if (RTCDIFF_TO_MSEC(WATCHER_TCP_HOLE_TX_PTR->tx_time) > WC_TCPHOLE_PERIODIC_TX_TIME) {
				/* In this watcher wakeup, TX TCP Hole punch time should be cleared */
				tx_pkt_type |= WATCHER_TCP_HOLE_PKT;
				WATCHER_TCP_HOLE_TX_PTR->tx_time = 0;
			}
		}
	}
#endif

	return tx_pkt_type;
}

extern uint32_t watcher_start_rtc;
uint32_t scm2020_wc_dtim_parser_run(u64 *sleep_duration, u8 watcher_first, u8 hiber_mode_flag)
{
	u32 val;
	bool bcn_rx_success = false;
	u64 cur_sleep_duration = 0;
	u64 dtim_proc_time;
	u32 bcn_rx_wait_time = 0;
	u32 dtim_wait_start;

	/* In case of Hibernation and first wakeup, no sleep, return directly for next wakeup */
	if (hiber_mode_flag && watcher_first == 1) {
		watcher_start_rtc = rtc_time_get();
		WATCHER_BCN_WINDOW_PTR->pm_hi_prep_flag = 1;
	}

	wc_printf("DTIM Parser(%d) run AID(0x%08x), chan(%04d)\n", watcher_first,
		WATCHER_MAC_REG_PTR->reg_ni_associd, WATCHER_MAC_REG_PTR->reg_chanfreq);

	/* For keepalive(null, garp), tx sceduling */
	watcher_tx_pkt_type = wc_tx_schedule();

	/* ARP Response should be removed in TX Init configuration */
	watcher_tx_pkt_type &= ~WATCHER_ARP_RESP_PKT;

	watcher_dbg_dtim_ctrl(1);

#ifdef PM_SOC_QFN40_FIB
	#define WIFI_BLE_CTRLMUX_CFG1	0x7E0380 /* FIXME SYS_REG config */

	scm2020_wc_set_rf_wakeup();

	/* PTA Force as WiFi write 0xe0d0002c 1 */
	writel(0x00000001, 0xE0D0002C);

	writel(0x00010001, PHY_BASE_START + WIFI_BLE_CTRLMUX_CFG1);

	writel(0x05000106, PHY_BASE_START + PHY_RFI_OFFSET + FW_RFI_ACTT_CFG);	// TX LDO control by BB signal
#endif
	scm2020_wc_mac_restore();

	/* for receiving BC/MC well, in advance MAC restoring when DTIM Parser is enabled */
	WATCHER_BCN_WINDOW_PTR->not_rx_desc_restore = 0;

	/* set the DTIM Parser config register */
	val = wc_mac_readl(REG_RX_DTIM_CFG);

	/* mode & fcs_check clearing */
	bfclr_m(val, RX_DTIM_CFG, DTIM_MODE);
	bfclr_m(val, RX_DTIM_CFG, DTIM_FCS_CHECK);

	/* RX_DTIM_CFG register
	 * with dtim_mode==0 (INT once found TIM IE),
	 * dtim_fcs_check = 0 (TIM IE Parsing done)
	 */
	bfmod_m(val, RX_DTIM_CFG, DTIM_MODE, 0);
	bfmod_m(val, RX_DTIM_CFG, DTIM_FCS_CHECK, 0);

	/* set the DTIM Parser config register */
	wc_mac_writel(val, REG_RX_DTIM_CFG);

	watcher_macdtim_interrupt_cnt = 0;

	val = wc_mac_readl(REG_MAC_CFG);
	if (!bfget_m(val, MAC_CFG, EN)) {
		bfmod_m(val, MAC_CFG, EN, 1);
		wc_mac_writel(val, REG_MAC_CFG);
	}

	/* all of interrupt is disabled */
	bfzero(val);

	/* for checking RX interrupt enable */
	watcher_rxbuf_int_proc = 0;

	bfmod_m(val, INTR_ENABLE, RX_BUF, 1);

	/* Only DTIM Parser interrupt enable */
	bfmod_m(val, INTR_ENABLE, RX_DTIM, 1);
	wc_mac_writel(val, REG_INTR_ENABLE);

	/* wlan interrupt enable(enable_irq()) */
	set_wlan_irq(1);

	/* Checked Watcher DTIM Parser preparing time by RTC : 730us */
	watcher_dbg_dtim_int_ctrl(0);
	{
		u32 current_beacon_waiting;

		/* DTIM Parser interupt checking timeout */
		/* get the current beacon window */
		current_beacon_waiting = beacon_window_array[WATCHER_BCN_WINDOW_PTR->current_beacon_window];
		if (current_beacon_waiting > MAX_DTIM_BEACON_WINDOW_MS) {

			current_beacon_waiting = MAX_DTIM_BEACON_WINDOW_MS;

			/* Reset the Beacon window parameter */
			WATCHER_BCN_WINDOW_PTR->current_beacon_window = 0;
		}

		/* improve watcher beacon loss rate in case of longer duration */
		if (WATCHER_BCN_WINDOW_PTR->pm_sleep_duration > BCN_BASIC_DURATION) {
			/* beacon wating time = beacon window(ms) + (pm_duration / 300 ) */
			current_beacon_waiting += (WATCHER_BCN_WINDOW_PTR->pm_sleep_duration / BCN_BASIC_DURATION);
		}

		/* for log enabled testing, first watcher running from wise, wait more time */
		if (watcher_first && !WATCHER_LOG_DISFLAG)
			current_beacon_waiting += WATCHER_LOG_FIRST_WAKEUP_MORE_WAIT;

		/* workaround for blocking consecutive bcn loss */
		if (!WATCHER_LOG_DISFLAG)
			current_beacon_waiting += WATCHER_BCN_WINDOW_PTR->pm_bcn_loss_chk_cnt;

		/**  In case of Hibernation mode preparing and wakeup,
		 * We don't know this case is Hibernation or deep sleep,
		 * If Hibernation first wakeup(By HW Restoring time), wakeup early
		 * So, add more waiting. In case of log disable, add more waiting.
		 */
		if (hiber_mode_flag) {
			if (!WATCHER_LOG_DISFLAG)
				current_beacon_waiting += WATCHER_HIB_FIRST_WAKEUP_MORE_WAIT;
			else
				current_beacon_waiting += WATCHER_HIB_FIRST_WAKEUP_MORE_WAIT * 2;
		}

		dtim_wait_start = wc_ktime();

		do {
			/* check if dtim parser int cnt is increased */
			if (watcher_macdtim_interrupt_cnt) {
				bcn_rx_success = true;
				break;
			}
		}
		/* Waiting of DTIM Parser interrupt */
		while (wc_abs(wc_ktime() - dtim_wait_start) <= MSEC_TO_RTC(current_beacon_waiting));

		/* Interrupt disable, stop DTIM Parser */
		scm2020_wc_dtim_parser_stop();

		/* Bcn rx is success by DTIM Parser */
		if (bcn_rx_success) {

			WATCHER_BCN_WINDOW_PTR->bcn_rx_count++;
			bcn_rx_wait_time = RTCDIFF_TO_MSEC(dtim_wait_start);
			cur_sleep_duration = scm2020_wc_bcn_rx_proc(bcn_rx_wait_time);
		}
		else { /* Bcn rx is failed */

			WATCHER_BCN_WINDOW_PTR->bcn_loss_count++;

			wc_printf("No BCN for(%dms)\n", RTCDIFF_TO_MSEC(dtim_wait_start));
			cur_sleep_duration = scm2020_wc_dtim_chk_timeout();
		}
	}

	/* Set RF Mode as Standby */
	scm2020_wc_set_rf_sleep();

	watcher_dbg_dtim_ctrl(0);
	/** if sleep duration is zero, means UC/BC is detected in TIM IE,
	 * so need full booting directly
	 */
	if (cur_sleep_duration == 0) {
		return 1;
	}

	/* get difference of BCN Check time from watcher running time */
	dtim_proc_time = RTCDIFF_TO_USEC(watcher_start_rtc);
	if (!bcn_rx_success) {
		if (cur_sleep_duration > dtim_proc_time)
			cur_sleep_duration = WATCHER_BCN_WINDOW_PTR->pm_sleep_duration - dtim_proc_time;
		/* Tunning value, total sleep duration = beacon DTIM interval */
		/* Case of No bcn, need additional sleep time */
		if (!hiber_mode_flag)
			cur_sleep_duration += WATCHER_NOBCN_ADDITIONAL_SLEEP_TIME;
		else
			cur_sleep_duration -= WATCHER_NOBCN_ADDITIONAL_SLEEP_TIME;
	}
	else {
		/* setting bcn rx duration time = current time - waited time */
		if (!hiber_mode_flag)
			cur_sleep_duration = WATCHER_BCN_WINDOW_PTR->pm_sleep_duration - dtim_proc_time + (bcn_rx_wait_time + 1)* USEC_PER_MSEC;
		else {
			cur_sleep_duration = WATCHER_BCN_WINDOW_PTR->pm_sleep_duration - dtim_proc_time + (bcn_rx_wait_time - 1)* USEC_PER_MSEC;;
		}
	}
#ifdef PM_SOC_QFN40_FIB
	/* DS0, DS1 Case Tunn value */
	if (!hiber_mode_flag && (cur_sleep_duration > SCM2010_SOC_FIB_AWAKE_DELAY)) {
		cur_sleep_duration -= SCM2010_SOC_FIB_AWAKE_DELAY;
	}

	/* Check Hibernation preparing state */
	if (hiber_mode_flag) {
		WATCHER_BCN_WINDOW_PTR->pm_hi_prep_flag = 1;
	}
	else {
		/* In last watcher working Hibernation preparing, for sync beacon rx */
		if (WATCHER_BCN_WINDOW_PTR->pm_hi_prep_flag && (cur_sleep_duration > SCM2010_SOC_FIB_AWAKE_DELAY)) {
			cur_sleep_duration -= SCM2010_SOC_FIB_AWAKE_DELAY;
		}
		WATCHER_BCN_WINDOW_PTR->pm_hi_prep_flag = 0;
	}
#endif
	/* Last check sleep duration */
	if (cur_sleep_duration > WATCHER_BCN_WINDOW_PTR->pm_sleep_duration || cur_sleep_duration < SCM2010_SOC_FIB_AWAKE_DELAY)
		cur_sleep_duration = WATCHER_BCN_WINDOW_PTR->pm_sleep_duration;

	*sleep_duration = cur_sleep_duration;

	return 0;
}

