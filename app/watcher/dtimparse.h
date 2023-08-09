
#ifndef _DTIMPARSE_H_
#define _DTIMPARSE_H_

#include "scm2020_regs.h"

/* If you want to dump tx packet, enable */
/* #define WATCHER_TX_DUMP */

/* #define WATCHER_SLEEP_ACTIVE_TIME_DBG */
#define PM_SOC_QFN40_FIB

/* if rx more data uc, wakeup and active */
#define CONFIG_MORE_DATA_ACTIVE

/* Kind of TX Packet */
#define	WATCHER_TX_NO_PKT		0x01
#define	WATCHER_PS_POLL_PKT		0x02	/* ps poll packet */
#define	WATCHER_NULL_PKT		0x04 	/* null packet */
#define	WATCHER_GARP_PKT		0x08 	/* GARP packet */
#define	WATCHER_ARP_RESP_PKT	0x10 	/* ARP Response packet */
#define	WATCHER_UDP_HOLE_PKT	0x20 	/* UDP Hole Punch packet */
#define	WATCHER_TCP_HOLE_PKT	0x40 	/* TCP Hole Punch packet */
#define	WATCHER_ALL_PKT			0x80 	/* all packet (null, GARP, ARP Resp, UDP/TCP Hole punch) */

#if defined(CONFIG_WATCHER_TX_KEEPALIVE) || defined(CONFIG_WATCHER_TX_GARP)
#define WC_NULL_PERIODIC_TX_TIME	(CONFIG_NULL_PERIODIC_TX_TIME * 1000) /* 60 sec */
#define WC_GARP_PERIODIC_TX_TIME	(CONFIG_GARP_PERIODIC_TX_TIME * 1000) /* 60 sec */
#endif

#ifdef CONFIG_WATCHER_TX_UDPHOLE
#define WC_UDPHOLE_PERIODIC_TX_TIME	(CONFIG_UDPHOLE_PERIODIC_TX_TIME * 60 * 1000) /* 3 min */
#endif

#ifdef CONFIG_WATCHER_TX_TCPHOLE
#define WC_TCPHOLE_PERIODIC_TX_TIME	(CONFIG_TCPHOLE_PERIODIC_TX_TIME * 60 * 1000) /* 3 min */
#endif
/* Timer related definition */
#define wc_abs(x) ((x) < 0 ? -(x) : (x))
#define _32khz              (32768)

#define USEC_PER_MSEC	1000L

#define wc_ktime			rtc_time_get
#define RTCDIFF_TO_MSEC(x)  ((wc_abs(wc_ktime() - x) * 1000) / _32khz)
#define RTCDIFF_TO_USEC(x)  ((wc_abs(wc_ktime() - x) * 1000000) / _32khz)
#define MSEC_TO_RTC(y)		((y * _32khz) / 1000)
#define RTC_TO_MSEC(z)		((z * 1000) / _32khz)
#define RTC_TO_USEC(z)		((z * 1000000) / _32khz)

#define MAC_BASE_START	0xe0200000
#define PHY_BASE_START  0xf0f20000

#define MAC_REG_OFFSET		(0x00008000)
#define MAC_SHM_OFFSET		(0x00004760)

#define INTR_CLEAR_RX_DTIM_MASK		0x20000000
#define INTR_CLEAR_RX_DTIM_SHIFT    29

#define INTR_STATUS_RX_DTIM_MASK    0x20000000
#define INTR_STATUS_RX_DTIM_SHIFT   29

#define REG_INTR_STATUS             0x004
#define REG_INTR_CLEAR              0x004

#define RBFM(r, f) 		(r##_##f##_MASK) 	/* mask */
#define RBFS(r, f) 		(r##_##f##_SHIFT) 	/* shift */

#define RBFSET(v, r, f, x) 	((v) |= ((x) << RBFS(r, f)) & RBFM(r, f))
#define RBFCLR(v, r, f) 	((v) &= ~RBFM(r, f))
#define RBFMOD(v, r, f, x) 	{RBFCLR(v, r, f); RBFSET(v, r, f, x);}
#define RBFGET(v, r, f) 	(((v) & RBFM(r, f)) >> RBFS(r, f))
#define RBFZERO(v) 		((v) = 0)

#define RBFM_P(f)		(f##_MASK) 	/* mask */
#define RBFS_P(f) 		(f##_SHIFT) 	/* shift */
#define RBFSET_P(v, f, x) 	((v) |= ((x) << RBFS_P(f)) & RBFM_P(f))
#define RBFCLR_P(v, f) 		((v) &= ~RBFM_P(f))
#define RBFMOD_P(v, f, x) 	{RBFCLR_P(v, f); RBFSET_P(v, f, x);}
#define RBFGET_P(v, f) 		(((v) & RBFM_P(f)) >> RBFS_P(f))

#define bfget_m(v, r, f) 	(((v) & RBFM(r, f)) >> RBFS(r, f))
#define bfmod_m(v, r, f, x) 	{RBFCLR(v, r, f); RBFSET(v, r, f, x);}
#define bfclr_m(v, r, f) 	((v) &= ~RBFM(r, f))
#define bfzero 			RBFZERO
#define bfmod_p  		RBFMOD_P
#define bfget_p  		RBFGET_P

#define RBFSET_P(v, f, x) 	((v) |= ((x) << RBFS_P(f)) & RBFM_P(f))
#define RBFCLR_P(v, f) 		((v) &= ~RBFM_P(f))

#define bfset_p  	RBFSET_P
#define bfclr_p  	RBFCLR_P


/** Dev Key related definition start **/
#define M_OFT_N(x, n) (REG_##x + ((REG_##x##_SPACING) * (n)))
#define REG_SEC_TEMP_KEY0_SPACING 4
#define REG_SEC_TEMP_KEY4_SPACING 4
#define REG_SEC_TEMP_KEY6_SPACING 4

#define SC_CIPHER_WEP40		0
#define SC_CIPHER_WEP104	1
#define SC_CIPHER_TKIP		2
#define SC_CIPHER_CCMP		4
#define SC_CIPHER_CCMP256	5
#define SC_CIPHER_GCMP		6
#define SC_CIPHER_GCMP256	7
#define SC_CIPHER_NONE		8
#define SC_CIPHER_UNKNOWN 	9

/* Same with IEEE80211_CIPHER_X */ 
#define SC_TXCIPHER_WEP		0
#define SC_TXCIPHER_TKIP	1
#define SC_TXCIPHER_AES_CCM	3

#define scm2020_cipher_is_valid(cipher)	\
		((cipher) >= SC_CIPHER_WEP40	\
		 && (cipher) < SC_CIPHER_NONE)
	
#define SC_KEY_CMD_ADD		1
#define SC_KEY_CMD_DEL_BY_TAG		2
#define SC_KEY_CMD_DELALL	3
#define SC_KEY_CMD_READ_BY_INDEX		4
#define SC_KEY_CMD_DEL_BY_INDEX		5
#define SC_KEY_CMD_DONE		0
/** Dev Key related definition end **/

#define REG_VIF_MBSSID_MASK_L32(n)			(REG_VIF_OFFSET(MBSSID_MASK_L32, n))
#define REG_VIF_MBSSID_MASK_H16(n)			(REG_VIF_OFFSET(MBSSID_MASK_H16, n))

#define PHY_MDM_OFFSET		(0x00000000)
#define PHY_RFI_OFFSET		(0x00008000)
#define PHY_REG_BANKSZ		(0x00004000)

#define	PHY_MODEM		0
#define PHY_RFITF		1

#define	IEEE80211_NODE_HE		0x10000000      /* HE enabled */

#define IEEE80211_ADDR_LEN 	6
#define ETHER_ADDR_LEN		6

#define	ETHER_IS_MULTICAST(addr) (*(addr) & 0x01) /* is address mcast/bcast? */
#define	ETHER_IS_BROADCAST(addr) \
	(((addr)[0] & (addr)[1] & (addr)[2] & \
	  (addr)[3] & (addr)[4] & (addr)[5]) == 0xff)
#define	ETHER_IS_ZERO(addr) \
	(((addr)[0] | (addr)[1] | (addr)[2] | \
	  (addr)[3] | (addr)[4] | (addr)[5]) == 0x00)
#define ETHER_IS_FILTER_MC(addr) \
	((addr)[0] == 0x01 & (addr)[1] == 0x00 & (addr)[2] == 0x5e)
/*
 * Check that the Ethernet address (MAC) is not 00:00:00:00:00:00, is not
 * a multicast address, and is not FF:FF:FF:FF:FF:FF.
 */
#define ETHER_IS_VALID(addr) \
	(!ETHER_IS_MULTICAST(addr) && !ETHER_IS_ZERO(addr))

#define	ETHER_IS_VALID_LEN(foo)	\
	((foo) >= ETHER_MIN_LEN && (foo) <= ETHER_MAX_LEN)
static const u_char etherbroadcastaddr[ETHER_ADDR_LEN] =
	{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const u_char etherzeroaddr[ETHER_ADDR_LEN] =
	{ 0, 0, 0, 0, 0, 0 };

static const u_char llcsnapheaderaddr[ETHER_ADDR_LEN] =
	{ 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

#define bitmask(l, h) (((1 << ((h) - (l) + 1)) - 1) << (l))
#define bitmask64(l, h) ((u64)((u64)(1 << ((h) - (l) + 1)) - 1) << (l))
#define GENMASK64(h, l)		bitmask64(l, h)
#define GENMASK(h, l)		bitmask(l, h)

#define	IEEE80211_AID(b)	((b) &~ 0xc000)

#define	IEEE80211_ADDR_COPY(dst,src)	memcpy(dst,src,IEEE80211_ADDR_LEN)

/*
 * Structure of a 10Mb/s Ethernet header.
 */
struct ether_header {
	u_char	ether_dhost[ETHER_ADDR_LEN];
	u_char	ether_shost[ETHER_ADDR_LEN];
	u_short	ether_type;
} __packed;

#define LLC_TYPE_ARP		0x0806
#define LLC_TYPE_IPv4		0x0800
#define LLC_TYPE_PAE		0x888e	/* EAPOL PAE/802.1x */
#define IPv4_HLMASK			0x0f	/* Isolates headler length in VHL field */

#define ARP_OP_REQUEST		0x0001
#define ARP_OP_REPLY		0x0002

#define IP_UDP_PROTO 	17
#define IP_TCP_PROTO	6

#define IPADDR_BROADCAST    ((u_int32_t)0xffffffffUL)
#define ETHER_IS_FILTER_MC_OUI(addr) ((addr)[0] == 0x01 && (addr)[1] == 0x00 && (addr)[2] == 0x5e)
#define CONV_IP_ADDR(a,b,c,d) (unsigned long)((a << 24) | (b << 16) | (c << 8) | (d))
#define CONV_PORT_NUMBER(v,w) ((v << 8) | w)

/*
 * Structure of a 48-bit Ethernet address.
 */
struct ether_addr {
	u_char octet[ETHER_ADDR_LEN];
} __packed;

static inline uint64_t ether_addr_to_u64(const uint8_t *addr)
{
	uint64_t u = 0;
	int i;

	for (i = 0; i < ETHER_ADDR_LEN; i++)
		u = u << 8 | addr[i];

	return u;
}

/*
 * u64_to_ether_addr - Convert a u64 to an Ethernet address.
 * @u: u64 to convert to an Ethernet MAC address
 * @addr: Pointer to a six-byte array to contain the Ethernet address
 */
static inline void u64_to_ether_addr(uint64_t u, uint8_t *addr)
{
	int i;

	for (i = ETHER_ADDR_LEN - 1; i >= 0; i--) {
		addr[i] = u & 0xff;
		u = u >> 8;
	}
}

#define DMB	asm volatile ("" : : : "memory")
#define dmb()	DMB

#define bf(l, h, name) u32 name:((h) - (l) + 1)
#define mtod(m, type)	((type)((m)->m_data))

struct mentry {
	/* MT0 */
	u32 pbd_l;	/* pointer to buffer descriptor (lower 32 bits) */
	/* MT1 */
	u32 pbd_h;	/* pointer to buffer descriptor (higher 32 bits) */

	/* MT2 */
	bf( 0, 11, aid);
	bf(12, 15, tid);
	bf(16, 27, sn);
	bf(28, 28, noack);
	bf(29, 29, eof);
	bf(30, 30, sent);
	bf(31, 31, ack);

	/* MT3 */
	bf( 0, 13, len);
	bf(14, 14, more);
	bf(15, 15, noenc);
	bf(16, 20, num);
	bf(21, 22, prot);
	bf(23, 23, ts_new);
	bf(24, 30, ts_off);
	bf(31, 31, htc);

	/* MT4 */
	bf( 0, 15, prot_dur);
	bf(16, 25, spacing);
	bf(26, 31, retry);

	/* MT5 */
	u32 sw0; /* wise: m */
	/* MT6 */
	u32 sw1;
	/* MT7 */
	u32 sw2;
};

struct bdesc {
	u32 pdata_l;	/* pointer to data buffer (lower 32 bits) */
	u32 pdata_h;	/* pointer to data buffer (higher 32 bits) */
	u32 len;
};

union key_cmd {
	struct {
		bf( 0,  2, cmd);
		bf( 3,  7, rsvd1);
		bf( 8,  9, keyid);
		bf(10, 10, gtk);
		bf(11, 15, rsvd2);
		bf(16, 18, cipher);
		/* both the STA and its peer have their SPP A-MSDU Capable fields equal to 1 */
		bf(19, 19, spp);
		bf(20, 23, rsvd3);
		bf(24, 27, idx);
		bf(28, 31, rsvd4);
	};
	u32 v;
};

/* Beacon waiting window related values */
#define MAX_RX_BEACON_WINDOW	5
#define PM_BCN_WINDOW_THRESHOLD	3
#define MAX_DTIM_BEACON_WINDOW_MS	15
#define MIN_DTIM_BEACON_WINDOW_MS	3
static int beacon_window_array[MAX_RX_BEACON_WINDOW] = {3, 5, 7, 10, 15};

#define BCN_BASIC_DURATION (300 << 10) /* 3 DTIM Interval, 1 TU = 1024 microsecond */

static __inline__ u32 get_unaligned_le32(const void *ptr)
{
	const u8 *x = ptr;

	return (x[3] << 24 | x[2] << 16 | x[1] << 8 | x[0]);
}

static __inline__ u64 get_unaligned_le64(void *p)
{
	u8 *ptr = p;
	u32 v[2];
	v[0] = ptr[0] | ptr[1] << 8 | ptr[2] << 16 | ptr[3] << 24;
	v[1] = ptr[4] | ptr[5] << 8 | ptr[6] << 16 | ptr[7] << 24;
	return v[0] | (u64) v[1] << 32;
}

static __inline__ u16 get_unaligned_le16(const void *ptr)
{
	const u8 *x = ptr;

	return (x[1] << 8 | x[0]);
}

static __inline__ void put_unaligned_le32(u32 val, void *ptr)
{
	u8 *x = ptr;

	x[0] = val & 0xff;
	x[1] = (val >> 8) & 0xff;
	x[2] = (val >> 16) & 0xff;
	x[3] = (val >> 24) & 0xff;
}

static __inline__ void put_unaligned_le16(u16 val, void *ptr)
{
	u8 *x = ptr;

	x[0] = val & 0xff;
	x[1] = (val >> 8) & 0xff;
}

static __inline__ u32 get_unaligned_be32(const void *ptr)
{
	const u8 *x = ptr;

	return (x[0] << 24 | x[1] << 16 | x[2] << 8 | x[3]);
}

static __inline__ u16 get_unaligned_be16(const void *ptr)
{
	const u8 *x = ptr;
	return (x[0] << 8 | x[1]);
}

static __inline__ void put_unaligned_be32(u32 val, void *ptr)
{
	u8 *x = ptr;

	x[3] = val & 0xff;
	x[2] = (val >> 8) & 0xff;
	x[1] = (val >> 16) & 0xff;
	x[0] = (val >> 24) & 0xff;
}

static __inline__ void put_unaligned_be16(u16 val, void *ptr)
{
	u8 *x = ptr;

	x[1] = val & 0xff;
	x[0] = (val >> 8) & 0xff;
}

#define mgnt(_stype) ( (_stype) << VIF0_RX_FILTER1_MGNT_FRAME_FILTER_BITMAP_SHIFT)
#define ctrl(_stype) ( (_stype) << VIF0_RX_FILTER1_CTRL_FRAME_FILTER_BITMAP_SHIFT)

#if 0
#define RX_FILTER_WATCHER (												\
								mgnt(bit_for_subtype(DISASSOC)) |			\
								mgnt(bit_for_subtype(DEAUTH)) |				\
								mgnt(bit_for_subtype(BEACON)) |			\
								ctrl(bit_for_subtype(BAR)) |				\
								1)
#else
#define RX_FILTER_WATCHER (												\
								mgnt(bit_for_subtype(DISASSOC)) |			\
								mgnt(bit_for_subtype(DEAUTH)) |				\
								ctrl(bit_for_subtype(BAR)) |				\
								1)
#endif

#endif

