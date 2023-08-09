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
#include <stdio.h>
#include <stdlib.h>

#include "hal/types.h"
#include "hal/device.h"
#include "hal/irq.h"
#include "hal/io.h"
#include "hal/efuse.h"
#include "head.h"
#include "mmap.h"
#include "hash.h"

#include "cmsis_os.h"

#include "crc32.c"

/*
#define DEBUG
*/

#define OK	(0)

enum {
	SDIO = 0,
  	UART = 1,
	USB  = 2,
	XIP	 = 3
} BOOT_MODE;

enum {
    ERR_MAGIC       = 1,
    ERR_HCRC32      = 2,
    ERR_SIZE        = 3,
    ERR_VMA         = 4,
    ERR_DCRC32      = 5,
    ERR_SIGNATURE   = 6,
    ERR_PK          = 7,
    ERR_ETC         = 8,
    ERR_IMAGE       = 9,
    ERR_DOWNLOAD    = 10,
    ERR_VERSION     = 11,
} BOOT_ERROR;

int g_uart_boot_ret = OK;
int g_xip_boot_ret = OK;

bool g_secure_boot = false;
u8 g_efuse_hash[32];

bool g_anti_rollback = false;
u64 g_anti_rollback_ver;

/*
 * eFuse
 */

#define ANTI_ROLLBACK_FW_MASK   (1 << 5)
#define ANTI_ROLLBACK_BL_MASK   (1 << 4)
#define SECURE_BOOT_MASK	    (1 << 3)
#define FLASH_PROT_MASK		    (1 << 2)
#define HARD_KEY_MASK		    (1 << 1)
#define PARITY_MASK			    (1 << 0)

#define M(f)				f##_MASK

extern volatile uint64_t * pullMtimecmpRegister;

#ifdef CONFIG_UART_BOOT
extern int download_by_ymodem(boot_hdr_t *hdr);
#endif
#ifdef CONFIG_USB_BOOT
extern int download_by_dfu(boot_hdr_t *bhdr);
#endif
#ifdef CONFIG_SDIO_BOOT
extern int download_by_sdio(boot_hdr_t *bhdr);
#endif
extern unsigned int  crc32 (unsigned int crc, const char *p, unsigned int len);

static int boot_mode(void)
{
	return ((readl(SMU(TOP_CFG)) & (0x3 << 28)) >> 28);
}

static int hart_id(void)
{
    return __nds__mfsr(NDS_MHARTID);
}

static void stop_mtimer(void)
{
	*pullMtimecmpRegister = UINT64_MAX;
}

#ifdef DEBUG
static void hexdump(void *buf, size_t len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", ((u8 *)buf)[i]);
    }
    printf("\n");
}
#endif

#ifdef CONFIG_SECURE_BOOT

extern int ecdsa_verify(uint8_t *hash, uint8_t *sig, uint8_t *pubkey);
#define IMAGE_BUF_SIZE      1024

static int verify_public_key(boot_hdr_t *h)
{
    struct hash_ctx ctx;
    u8 hash[32];

    hash_init(&ctx, 2); /* hash 256 */
    hash_update(&ctx, h->pk, 64); /* key 64 */
    hash_finish(&ctx, hash);

    /* compare calc hash & efuse hash */
    if (memcmp(g_efuse_hash, hash, 32)) {
        /* public key verify fail */
        return -1;
    }

    return OK;
}

static int verify_image_signature(boot_hdr_t *h, u32 img_off)
{
    struct hash_ctx ctx;
    u32 size;
    u32 off;
    u32 blk_sz;
    u8 img_hash[32];
    int ret;

    size = uswap_32(h->size);

    /* make image hash */
    hash_init(&ctx, 2);
    for (off = 0; off < size; off += blk_sz) {
        blk_sz = size - off;
        if (blk_sz > IMAGE_BUF_SIZE) {
            blk_sz = IMAGE_BUF_SIZE;
        }
        hash_update(&ctx, (void *)img_off, blk_sz);
        img_off += blk_sz;
    }
    hash_finish(&ctx, img_hash);

    /* make image hash */
    ret = ecdsa_verify(img_hash, h->signature, h->pk);
    if (ret < 0) {
        return -1;
    }

    return OK;
}

static int verify_secure_boot(boot_hdr_t *h, void *body)
{
    if (verify_public_key(h)) {
        return -ERR_PK;
    }

    if (verify_image_signature(h, (u32)body)) {
        return -ERR_SIGNATURE;
    }

	return OK;
}

static bool in_secure_boot(void)
{
	struct device *dev = device_get_by_name("efuse-scm2010");
	u32 flags;

	if (!dev)
		return false;

	flags = efuse_read(dev, 4);

#ifdef DEBUG
    printf("flags 0x%08x\n", flags);
#endif

	if (flags & M(SECURE_BOOT)) {
		return true;
	}

	return false;
}

static void read_secure_boot_info(void)
{
	struct device *dev = device_get_by_name("efuse-scm2010");
    u32 v;
    int i;

    g_secure_boot = in_secure_boot();
    if (g_secure_boot) {
        for (i = 14; i >= 7; i--) {
            v = uswap_32(efuse_read(dev, i));
            memcpy(g_efuse_hash + (14 - i) * 4, &v, 4);
        }
    }
}

#endif

#ifdef CONFIG_ANTI_ROLLBACK

static int verify_ver(boot_hdr_t *h)
{
    u32 *ver;
    u64 img_ver;
    u32 v;
    u32 i;

    if (!g_anti_rollback) {
        return OK;
    }

    /* h->ver = offset to flash address which contains major/minor version */
    ver = (u32 *)uswap_32(h->ver);
    /* major: upper 32bit */
    v = *(ver);
    img_ver = ((u64)v) << 32;
    /* minor: lower 32bit */
    v = *(ver + 1);
    img_ver |= v ;

    /* find the number of 1s of efuse anti rollback version */
    for (i = 0; i < 64; i++) {
        if ((g_anti_rollback_ver & (1ULL << i)) == 0) {
            break;
        }
    }

#ifdef DEBUG
    printf("efuse:%08x %08x\n",
        (uint32_t)(g_anti_rollback_ver >> 32), (uint32_t)(g_anti_rollback_ver & 0xFFFFFFFF));
    printf("Image:%08x %08x\n",
        (uint32_t)(img_ver >> 32), (uint32_t)(img_ver & 0xFFFFFFFF));
    osDelay(50);
#endif

    /* if rollback version is not set, just try boot normally*/
    if (i == 0) {
        return OK;
    }
    i -= 1;

    if ((img_ver & (~g_anti_rollback_ver)) > 0) {
        /* if the fw version MSB is greater than that of the efuse, error */
        return -ERR_VERSION;
    } else if ((img_ver & (1ULL << i)) == 0) {
        /* if the fw version MSB does not match that of the efuse, error */
        return -ERR_VERSION;
    }

    return OK;
}

static bool use_anti_rollback(void)
{
	struct device *dev = device_get_by_name("efuse-scm2010");
	u32 flags;

	if (!dev)
		return false;

	flags = efuse_read(dev, 4);
	if (flags & M(ANTI_ROLLBACK_BL)) {
		return true;
	}

	return false;
}

static void read_anti_rollback_info(void)
{
	struct device *dev = device_get_by_name("efuse-scm2010");
    u32 v;

    g_anti_rollback = use_anti_rollback();
    if (g_anti_rollback) {
        v = efuse_read(dev, 23);
        g_anti_rollback_ver = v;
        v = efuse_read(dev, 24);
        g_anti_rollback_ver |= ((u64)v) << 32;
    }
}
#endif

static int verify_header(boot_hdr_t *h)
{
	boot_hdr_t tmp;

	memcpy(&tmp, h, sizeof(tmp));
	tmp.hcrc32 = 0;

	if (h->magic != uswap_32(0x48787031)) {
        return -ERR_MAGIC;
    }


    if (crc32(0, (char *)&tmp, sizeof(tmp)) !=  uswap_32(h->hcrc32)) {
		return -ERR_HCRC32;
	}

	return OK;
}

static int verify_body(boot_hdr_t *h, void *body)
{
#ifdef CONFIG_SECURE_BOOT

	if (g_secure_boot) {
		return verify_secure_boot(h, body);
	}

#endif

	/* Check CRC32 of the body instead. */

	if (crc32(0, (char *)body, uswap_32(h->size)) != uswap_32(h->dcrc32)) {
		return -ERR_DCRC32;
	}

	return OK;
}

#ifdef CONFIG_FLASH_PROTECTION

static void decrypt(bool on)
{
    u32 v = *SYS(CRYPTO_CFG);

	if (on) {
    	*SYS(CRYPTO_CFG) = v | (1 << 31);
	} else {
    	*SYS(CRYPTO_CFG) = v & ~(1 << 31);
	}
}

static void transfer_rootk(void)
{
    u32 v = *SYS(CRYPTO_CFG);

	/* Wait, if needed, until root_init_dn becomes 1.
	 * It's ugly, but too specific to be in efuse driver.
	 */

	while (1) {
		if (*(volatile u32 *)(EFUSE_BASE_ADDR + 0x04) & (1 << 31))
			break;
	}

	*SYS(CRYPTO_CFG) = v | (1 << 7);
}

static void set_iv(u8 *iv)
{
    u32 word;
    int i, p;

    /* Use the most significant 100 bits.
     * iv[127:0 -> 99:0, (27:0)]
     */

    *SYS(IV_CFG(3)) =  (iv[0] >> 4) & 0xf;
    for (i = 2, p = 0; i >= 0; i--, p += 4) {
        word = (iv[p] & 0xf) << 28;
        word |= iv[p + 1] << 20;
        word |= iv[p + 2] << 12;
        word |= iv[p + 3] << 4;
        word |= (iv[p + 4] & 0xf0) >> 4;
        *SYS(IV_CFG(i)) = word;
    }
#ifdef DEBUG
	for (i = 3; i >= 0; i--)
		printf("IV_CFG(%d)\t0x%x\n", i, *SYS(IV_CFG(i)));
#endif
}

static void set_key(u8 *k)
{
    u32 *word = (u32 *)k;
    int i;

    for (i = 3; i >= 0; i--, word++)
        *SYS(KEY_CFG(i)) = uswap_32(*word);
#ifdef DEBUG
	for (i = 3; i >= 0; i--)
		printf("KEY_CFG(%d)\t0x%x\n", i, *SYS(KEY_CFG(i)));
#endif
}

extern int aes_cmac_encrypt(const u8 *key, u8 *in,
							size_t len, u8 *out);

static int config_flash_key(void)
{
	struct device *dev = device_get_by_name("efuse-scm2010");
	u8 rootk[16], *rk;
	u8 custid[16], chipid[16];
	u8 iv[16], custkey[16], flkey[16];
	u32 flags, v;
	int i;

	if (!dev)
		return -1;

	flags = efuse_read(dev, 4);
	if (flags & M(FLASH_PROT)) {
		if (flags & M(HARD_KEY)) {
			transfer_rootk();
			rk = NULL;
		} else {
			/* Read out ROOT_KEY from eFuse. */
			for (i = 3; i >= 0; i--) {
				v = uswap_32(efuse_read(dev, i));
				memcpy(rootk + (3 - i) * 4, &v, 4);
			}
			rk = rootk;
		}

		/* Read IDs. */
		memset(custid, 0, sizeof(custid));
		v = uswap_32(efuse_read(dev, 5));
		memcpy(&custid[12], &v, 4);
		custid[12] = custid[13] = custid[14] = 0;

		memset(chipid, 0, sizeof(chipid));
		v = uswap_32(efuse_read(dev, 6));
		memcpy(&chipid[12], &v, 4);

		/* Derive IV.
		 * IV = AES_CMAC_ENC(ROOT_KEY, CHIP_ID)
		 */
		aes_cmac_encrypt(rk, chipid, 16, iv);

		/* Derive FLASH_KEY.
		 * CUSTOMER_KEY = AES_CMAC_ENC(ROOT_KEY, CUSTOMER_ID)
		 * FLASH_KEY = AES_CMAC_ENC(CUSTOMER_KEY, CHIP_ID)
		 */
		aes_cmac_encrypt(rk, custid, 16, custkey);
		aes_cmac_encrypt(custkey, chipid, 16, flkey);

#ifdef DEBUG
		printf("Customer ID\n");
		hexdump(custid, 16);

		printf("Chip ID\n");
		hexdump(chipid, 16);

		printf("Root key\n");
		hexdump(rk, 16);

		printf("Customer key\n");
		hexdump(custkey, 16);

		printf("Flash key\n");
		hexdump(flkey, 16);

		printf("IV\n");
		hexdump(iv, 16);
#endif

		set_iv(iv);
		set_key(flkey);

		decrypt(true);
	} else {
		decrypt(false);
	}
	return 0;
}

#endif

int main(void)
{
	unsigned long flags;
	void (*entry)(void);
    u32 v __maybe_unused;
    boot_hdr_t h = {0,};

	if (hart_id() == 0) {

		/* D25F boot sequence */

#ifdef CONFIG_SECURE_BOOT
        read_secure_boot_info();
#endif
#ifdef CONFIG_ANTI_ROLLBACK
        read_anti_rollback_info();
#endif

		switch (boot_mode()) {
		case UART:
#ifdef CONFIG_UART_BOOT

            /* Configure GPIO21, GPIO22 as RXD and TXD, respectively. */
            v = readl(IOMUX_BASE_ADDR + 0x8);
            v &= 0xf00fffff;
            writel(v, (IOMUX_BASE_ADDR + 0x8));

            g_uart_boot_ret = download_by_ymodem(&h);

            if (g_uart_boot_ret == OK) {
                g_uart_boot_ret = verify_header(&h);
            }

            if (g_uart_boot_ret == OK) {
                g_uart_boot_ret = verify_body(&h, (void *)uswap_32(h.vma));
            }

            if (g_uart_boot_ret == OK) {
                break;
            }
#endif
			/* otherwise, fall through. */
		case XIP:

            /* Configure GPIO[9-14] as SPI0 pins. */
            v = readl(IOMUX_BASE_ADDR + 0x4);
            v &= 0xf000000f;
            writel(v, (IOMUX_BASE_ADDR + 0x4));

#ifdef CONFIG_FLASH_PROTECTION
			config_flash_key();
#endif
			memcpy(&h, (void *)FLASH_BASE, sizeof(h));

            g_xip_boot_ret = verify_header(&h);
            if (g_xip_boot_ret == OK) {
                g_xip_boot_ret = verify_body(&h, (void *)(FLASH_BASE + sizeof(h)));
            }

#ifdef CONFIG_ANTI_ROLLBACK
            if (g_xip_boot_ret == OK) {
                g_xip_boot_ret = verify_ver(&h);
            }
#endif

            if (g_xip_boot_ret == OK) {
                break;
            }

			/* otherwise, fall through. */
		default:
			goto _wfi;
		}
	} else {

		/* N22 boot sequence */

		switch (boot_mode()) {
		case USB:
#ifdef CONFIG_USB_BOOT
			if (download_by_dfu(&h) == 0) {
				/* Data integrity is already
				 * guranteed by USB protocol.
				 */
				break;
			}
#endif
			goto _wfi;
		case SDIO:
#ifdef CONFIG_SDIO_BOOT
			if (download_by_sdio(&h) == 0) {
				break;
			}
#endif
			goto _wfi;
		default:
			goto _wfi;
		}

	}

	driver_deinit();
	stop_mtimer();

	local_irq_save(flags);

	entry = (void (*)(void))uswap_32(h.vma);
	(*entry)();

	/* will never be reached */
	local_irq_restore(flags);

_wfi:

	local_irq_save(flags); /* never wake up */

#ifdef DEBUG
    printf("UART boot status : %d\n", g_uart_boot_ret);
    printf("XIP boot status : %d\n", g_xip_boot_ret);
    osDelay(100);
#endif

	stop_mtimer();
	driver_deinit();
	while (1) {
		__asm volatile( "wfi" );
	}

	return 0;
}
