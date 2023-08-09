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
#include <stdint.h>

#include <cmsis_os.h>
#include <hal/kernel.h>
#include <hal/crypto.h>

#define SKE_CTRL        (*((volatile u32 *)(SKE_BASE_ADDR+0x00)))
#define SKE_CFG         (*((volatile u32 *)(SKE_BASE_ADDR+0x04)))
#define SKE_SR1         (*((volatile u32 *)(SKE_BASE_ADDR+0x08)))
#define SKE_SR2         (*((volatile u32 *)(SKE_BASE_ADDR+0x0c)))
#define SKE_KEY          (((volatile u32 *)(SKE_BASE_ADDR+0x10)))
#define SKE_DIN          (((volatile u32 *)(SKE_BASE_ADDR+0x90)))
#define SKE_DOUT         (((volatile u32 *)(SKE_BASE_ADDR+0xb0)))

#define AES_BSIZE 		 (16)
#define AES_KSIZE   	 (16)

static void ske_set_cfg(uint8_t en)
{
    if (en) {
        SKE_CFG |= (0x01 << 12);
    } else {
        SKE_CFG &= ~(0x01 << 12);
    }
}

static void ske_config_aes128_encrypt(void)
{
    SKE_CFG = \
              (0x01 << 0) | \
              (0x00 << 11) | \
              (0x02 << 24) | \
              (0x01 << 28);
}

static void ske_start_and_wait(void)
{
    while ((SKE_SR1 & 1) != 0);

    SKE_CTRL |= 1;

    while ((SKE_SR2 & 1) != 1);

    SKE_SR2 = 0;
}

static void ske_set_key(u32 *buf)
{
    int i;

	if (buf == NULL)
		return;

    for (i = 3; i >= 0; i--) {
        SKE_KEY[i] = buf[i];
    }
}

static void ske_set_input(u32 *buf)
{
    int i;
    for (i = 3; i >= 0; i--) {
        SKE_DIN[i] = buf[i];
    }
}

static void ske_get_output(u32 *buf)
{
    int i;
    for (i = 3; i >= 0; i--) {
        buf[i] = SKE_DOUT[i];
    }
}

static int ske_encrypt_block(const u8 *k, u8 *in, u8 *out)
{
    /* configure for aes128 */
    ske_config_aes128_encrypt();

    /* set key */
    ske_set_cfg(1);
    ske_set_key((u32 *)k);
    ske_start_and_wait();
    ske_set_cfg(0);

    /* set input */
    ske_set_input((u32 *)in);

    /* start for data */
    ske_start_and_wait();

    /* get output */
    ske_get_output((u32 *)out);

    return 0;
}

const unsigned char gf_wrap = 0x87;

/*
 *  assumes: out != NULL and points to a GF(2^n) value to receive the
 *            doubled value;
 *           in != NULL and points to a 16 byte GF(2^n) value
 *            to double;
 *           the in and out buffers do not overlap.
 *  effects: doubles the GF(2^n) value pointed to by "in" and places
 *           the result in the GF(2^n) value pointed to by "out."
 */
static void gf_double(u8 *out, u8 *in)
{

	/* start with low order byte */
	u8 *x = in + (AES_BSIZE - 1);

	/* if msb == 1, we need to add the gf_wrap value, otherwise add 0 */
	u8 carry = (in[0] >> 7) ? gf_wrap : 0;

	out += (AES_BSIZE - 1);
	for (;;) {
		*out-- = (*x << 1) ^ carry;
		if (x == in) {
			break;
		}
		carry = *x-- >> 7;
	}
}

/*
 AES-CMAC

 M = M_1 || M_2 || ... || M_n

 (positive multiple block Length)

 +-----+     +-----+     +-----+     
 | M_1 |     | M_2 |     | M_n |   
 +-----+     +-----+     +-----+  
    |           |           |   +--+
    |     +--->(+)    +--->(+)<-|K1|
    |     |     |     |     |   +--+
 +-----+  |  +-----+  |  +-----+   
 |AES_K|  |  |AES_K|  |  |AES_K|  
 +-----+  |  +-----+  |  +-----+ 
    |     |     |     |     |   
    +-----+     +-----+     |  
                            |  
                         +-----+
                         |  T  |
                         +-----+
*/

int aes_cmac_encrypt(const u8 *key, u8 *in, size_t len, u8 *out)
{
	static u8 iv[AES_BSIZE], k1[AES_KSIZE];
	int i;

	/* Compute K1. */

	memset(iv, 0, sizeof(iv));

	ske_encrypt_block(key, iv, iv);
	gf_double(k1, iv);

	/* Encrypt (n-1) blocks. */

	memset(iv, 0, sizeof(iv));

	while (len > AES_BSIZE) {
		for (i = 0; i < AES_BSIZE; ++i) {
			iv[i] ^= in[i];
		}
		ske_encrypt_block(key, iv, iv);
		in  += AES_BSIZE;
		len -= AES_BSIZE;
	}

	/* Encrypt n-th, i.e., the last block. */

	for (i = 0; i < AES_BSIZE; i++) {
		iv[i] ^= (in[i] ^ k1[i]);
	}
	ske_encrypt_block(key, iv, out);

	return 0;
}


