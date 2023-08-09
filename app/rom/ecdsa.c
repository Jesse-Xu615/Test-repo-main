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
#include <stdio.h>

#define N_BYTE_LEN          32
#define N_WORD_LEN          8
#define P_BYTE_LEN          32
#define P_WORD_LEN          8
#define MAX_WORD_LEN        8

#define MAX_ECCP_WORD_LEN   64

#define PKE_BASE_ADDR   (0xE0700000UL)
#define PKE_CTRL        (*((volatile uint32_t *)(PKE_BASE_ADDR)))
#define PKE_CFG         (*((volatile uint32_t *)(PKE_BASE_ADDR+0x04)))
#define PKE_MC_PTR      (*((volatile uint32_t *)(PKE_BASE_ADDR+0x08)))
#define PKE_RISR        (*((volatile uint32_t *)(PKE_BASE_ADDR+0x0C)))
#define PKE_IMCR        (*((volatile uint32_t *)(PKE_BASE_ADDR+0x10)))
#define PKE_MISR        (*((volatile uint32_t *)(PKE_BASE_ADDR+0x14)))
#define PKE_RT_CODE     (*((volatile uint32_t *)(PKE_BASE_ADDR+0x24)))
#define PKE_EXE_CONF    (*((volatile uint32_t *)(PKE_BASE_ADDR+0x50)))
#define PKE_VERSION     (*((volatile uint32_t *)(PKE_BASE_ADDR+0xFC)))
#define PKE_A(a, step)  ((volatile uint32_t *)(PKE_BASE_ADDR+0x0400+(a)*(step)))
#define PKE_B(a, step)  ((volatile uint32_t *)(PKE_BASE_ADDR+0x1000+(a)*(step)))

static uint32_t secp256r1_p[8] = {0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0x00000000,0x00000000,0x00000000,0x00000001,0xFFFFFFFF};
static uint32_t secp256r1_a[8] = {0xFFFFFFFC,0xFFFFFFFF,0xFFFFFFFF,0x00000000,0x00000000,0x00000000,0x00000001,0xFFFFFFFF};
static uint32_t secp256r1_gx[8] = {0xD898C296,0xF4A13945,0x2DEB33A0,0x77037D81,0x63A440F2,0xF8BCE6E5,0xE12C4247,0x6B17D1F2};
static uint32_t secp256r1_gy[8] = {0x37BF51F5,0xCBB64068,0x6B315ECE,0x2BCE3357,0x7C0F9E16,0x8EE7EB4A,0xFE1A7F9B,0x4FE342E2};
static uint32_t secp256r1_n[8] = {0xFC632551,0xF3B9CAC2,0xA7179E84,0xBCE6FAAD,0xFFFFFFFF,0xFFFFFFFF,0x00000000,0xFFFFFFFF};

uint32_t g_secp256_step;

static void swap_copy(uint8_t *src, uint8_t *dst, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        dst[len - 1 - i] = src[i];
    }
}

uint8_t u32_bignum_check_zero(uint32_t *a, uint32_t word_len)
{
	uint32_t i;

    for(i = 0; i< word_len; i++) {
        if (a[i]) {
            return 0;
        }
    }

    return 1;
}

uint32_t get_valid_words(uint32_t *a, uint32_t max_words)
{
	uint32_t i;

	for (i = max_words; i > 0; i--) {
		if (a[i - 1]) {
			return i;
		}
	}

	return 0;
}

int32_t u32_big_num_cmp(uint32_t *a, uint32_t a_word_len, uint32_t *b, uint32_t b_word_len)
{
	int32_t i;

	a_word_len = get_valid_words(a, a_word_len);
	b_word_len = get_valid_words(b, b_word_len);

	if(a_word_len > b_word_len) {
		return 1;
	}

	if(a_word_len < b_word_len) {
		return -1;
	}

	for (i = (a_word_len -1); i >= 0;i--) {
		if(a[i] > b[i]) {
			return 1;
		}

		if(a[i] < b[i]) {
			return -1;
		}
	}

	return 0;
}


void pke_word_copy(uint32_t *dst, uint32_t *src, uint32_t word_len)
{
    uint32_t i;

    if (src == dst) {
        return;
    }

    for (i = 0; i < word_len; i++) {
        dst[i] = src[i];
    }
}

void pke_word_read(uint32_t *src, uint32_t *dst, uint32_t word_len)
{
    uint32_t i;

    if (src == dst) {
        return;
    }

    for (i = 0; i < word_len; i++) {
        dst[i] = *((volatile uint32_t *)src);
        src++;
    }
}

void pke_word_clean(uint32_t *a, uint32_t word_len)
{
    while (word_len) {
        a[--word_len] = 0;
    }
}

uint32_t pke_sub(const uint32_t *a, const uint32_t *b, uint32_t *out, uint32_t word_len)
{
	uint32_t i, carry, tmp, tmp2;

	carry = 0;
	for (i = 0; i < word_len; i++) {
		tmp = a[i]-b[i];
		tmp2 = tmp-carry;
		if (tmp > a[i] || tmp2 > tmp) {
			carry = 1;
		}
		else {
			carry = 0;
		}
		out[i] = tmp2;
	}

	return 0;
}

uint32_t get_valid_bits(const uint32_t *a, uint32_t word_len)
{
	uint32_t i = 0;
	uint32_t j = 0;

	if(0 == word_len) {
		return 0;
	}

	for (i = word_len; i > 0; i--) {
		if (a[i - 1]) {
			break;
		}
	}

	if (0 == i) {
		return 0;
	}

	for (j = 32; j > 0; j--) {
		if (a[i - 1] & (((uint32_t)0x1) << (j - 1))) {
			break;
		}
	}

	return ((i - 1) << 5) + j;
}

void uint32_copy(uint32_t *dst, uint32_t *src, uint32_t word_len)
{
	uint32_t i;

	if(dst != src) {
		for (i=0; i < word_len; i++) {
			dst[i] = src[i];
		}
	}
}

void uint32_clear(uint32_t *a, uint32_t word_len)
{
	volatile uint32_t i = word_len;

	while(i) {
		a[--i] = 0;
	}
}

uint32_t big_div_2n(uint32_t *a, int32_t a_word_len, uint32_t n)
{
	int32_t i;
	uint32_t j;

	a_word_len = get_valid_words(a, a_word_len);
	if (0 == n) {
		return a_word_len;
	}

	if (!a_word_len) {
		return 0;
    }

	if (n <= 32) {
		for (i = 0; i < a_word_len - 1; i++) {
			a[i] >>= n;
			a[i] |= (a[i+1]<<(32-n));
		}
		a[i] >>= n;

		if(!a[i]) {
			return i;
        }
		return a_word_len;
    } else {       //general method
		j =  n >> 5; //j=n/32;
		n &= 31;  //n=n%32;
		for(i = 0; i < a_word_len - (int32_t)j-1; i++) {
			a[i] = a[i+j] >> n;
			a[i] |= (a[i+j+1] << (32-n));
		}
		a[i] = a[i+j]>>n;
		uint32_clear(a + a_word_len - j, j);

		if (!a[i]) {
			return i;
        }

		return a_word_len - j;
	}
}

int pke_run(uint32_t mc_ptr)
{
    int ret;

    PKE_MC_PTR = mc_ptr;
	if(PKE_RISR & 1) {
		PKE_RISR &= ~0x01;
	}
	PKE_CTRL |= 1;

	while(!(PKE_RISR & 1));

	ret = PKE_RT_CODE & 0x07;

    return ret;
}

int pke_modinv(const uint32_t *modulus, const uint32_t *a, uint32_t *ainv, uint32_t mod_word_len, uint32_t a_word_len)
{
    int ret;


	pke_word_copy((uint32_t *)(PKE_B(3, g_secp256_step)), (uint32_t *)modulus, mod_word_len);
    pke_word_clean((uint32_t *)(PKE_B(3, g_secp256_step)) + mod_word_len, (g_secp256_step / 4) - mod_word_len);

	pke_word_copy((uint32_t *)(PKE_B(0, g_secp256_step)), (uint32_t *)a, a_word_len);
    pke_word_clean((uint32_t *)(PKE_B(0, g_secp256_step)) + a_word_len, (g_secp256_step / 4) - a_word_len);

    ret = pke_run(0x1C);
    if (ret) {
        return ret;
    }

    pke_word_read((uint32_t *)(PKE_A(0, g_secp256_step)), ainv, mod_word_len);

    return 0;
}


uint32_t pke_pre_calc_mont(const uint32_t *modulus)
{
    uint32_t word_len = 8;
    int ret;

	pke_word_copy((uint32_t *)(PKE_B(3, g_secp256_step)), (uint32_t *)modulus, word_len);

    pke_word_clean((uint32_t *)(PKE_B(3, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);
    pke_word_clean((uint32_t *)(PKE_A(3, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);

    ret = pke_run(0x28);
    if (ret) {
        return ret;
    }

    return 0;
}

uint32_t pke_modmul_internal(const uint32_t *modulus, const uint32_t *a, const uint32_t *b, uint32_t *out,
		uint32_t word_len)
{
	uint32_t ret;

	pke_word_copy((uint32_t *)(PKE_B(3, g_secp256_step)), (uint32_t *)modulus, word_len);
    pke_word_clean((uint32_t *)(PKE_B(3, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);

	pke_word_copy((uint32_t *)(PKE_A(0, g_secp256_step)), (uint32_t *)a, word_len);
	pke_word_copy((uint32_t *)(PKE_B(0, g_secp256_step)), (uint32_t *)b, word_len);
    pke_word_clean((uint32_t *)(PKE_A(0, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);
    pke_word_clean((uint32_t *)(PKE_B(0, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);

    ret = pke_run(0x18);
    if (ret) {
        return ret;
    }

    pke_word_read((uint32_t *)(PKE_A(0, g_secp256_step)), out, word_len);

    return 0;

}

uint32_t pke_modadd(const uint32_t *modulus, const uint32_t *a, const uint32_t *b,
				   uint32_t *out, uint32_t word_len)
{
	uint32_t ret;

	pke_word_copy((uint32_t *)(PKE_B(3, g_secp256_step)), (uint32_t *)modulus, word_len);
	pke_word_copy((uint32_t *)(PKE_A(0, g_secp256_step)), (uint32_t *)a, word_len);
	pke_word_copy((uint32_t *)(PKE_B(0, g_secp256_step)), (uint32_t *)b, word_len);

    pke_word_clean((uint32_t *)(PKE_B(3, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);
    pke_word_clean((uint32_t *)(PKE_A(0, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);
    pke_word_clean((uint32_t *)(PKE_B(0, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);


    ret = pke_run(0x20);
    if (ret) {
        return ret;
    }

    pke_word_read((uint32_t *)(PKE_A(0, g_secp256_step)), out, word_len);

    return 0;
}

uint32_t pke_mod(uint32_t *a, uint32_t a_word_len, uint32_t *b, uint32_t b_word_len, uint32_t *c)
{
	int32_t flag;
	uint32_t b_bit_len, bit_len, tmp_len;
	uint32_t *A1, *B2;
	uint32_t ret;
    uint32_t g_secp256_step = 0x24;

	flag = u32_big_num_cmp(a, a_word_len, b, b_word_len);
	if (flag < 0) {
		a_word_len = get_valid_words(a, a_word_len);
		uint32_copy(c, a, a_word_len);
		uint32_clear(c + a_word_len, b_word_len - a_word_len);
		return 0;
    } else if(0 == flag) {
		uint32_clear(c, b_word_len);
		return 0;
	}

	A1 = (uint32_t *)(PKE_A(1, g_secp256_step));
	B2 = (uint32_t *)(PKE_B(2, g_secp256_step));

	b_bit_len = get_valid_bits(b, b_word_len);
	bit_len = b_bit_len & 0x1F;

	//get B2 = a high part mod b
	if (bit_len) {
		tmp_len = a_word_len - b_word_len + 1;
		uint32_copy(B2, a + b_word_len - 1, tmp_len);
		big_div_2n(B2, tmp_len, bit_len);
		if (tmp_len < b_word_len) {
			uint32_clear(B2 + tmp_len, b_word_len - tmp_len);
        } else if (u32_big_num_cmp(B2, b_word_len, b, b_word_len) >= 0) {
			ret = pke_sub(B2, b, B2, b_word_len);
			if(ret) {
				return ret;
			}
		}
    } else {
		tmp_len = a_word_len - b_word_len;
		if (u32_big_num_cmp(a + b_word_len, tmp_len, b, b_word_len) >= 0) {
			ret = pke_sub(a + b_word_len, b, B2, b_word_len);
			if (ret) {
				return ret;
			}
        } else {
			uint32_copy(B2, a + b_word_len, tmp_len);
			uint32_clear(B2 + tmp_len, b_word_len-tmp_len);
		}
	}

	//set the pre-calculated mont parameter
    ret = pke_pre_calc_mont(b);
    if (ret) {
        return ret;
    }
	//get A1 = 1000...000 mod b
	uint32_clear(A1, b_word_len);
	if (bit_len) {
		A1[b_word_len-1] = 1<<(bit_len);
	}

	ret = pke_sub(A1, b, A1, b_word_len);
	if(ret) {
		return ret;
	}

	//get B2 = a_high * 1000..000 mod b
    PKE_EXE_CONF = 0x0000;
	ret = pke_modmul_internal(b, A1, B2, B2, b_word_len);
	if (ret) {
		return ret;
	}

	//get A1 = a low part mod b
	if (bit_len) {
		uint32_copy(A1, a, b_word_len);
		A1[b_word_len - 1] &= ((1 << (bit_len)) - 1);
		if (u32_big_num_cmp(A1, b_word_len, b, b_word_len) >= 0) {
			ret = pke_sub(A1, b, A1, b_word_len);
			if (ret) {
				return ret;
			}
		}
    } else {
		if (u32_big_num_cmp(a, b_word_len, b, b_word_len) >= 0) {
			ret = pke_sub(a, b, A1, b_word_len);
			if (ret) {
				return ret;
			}
        } else {
			A1 = a;
		}
	}

	return pke_modadd(b, A1, B2, c, b_word_len);
}

uint32_t eccp_point_mul(uint32_t *k, uint32_t *Px, uint32_t *Py,
					    uint32_t *Qx, uint32_t *Qy)
{
	uint32_t word_len = 8;
	uint32_t ret;

	pke_word_copy((uint32_t *)(PKE_B(3, g_secp256_step)), secp256r1_p, word_len);
    pke_word_clean((uint32_t *)(PKE_B(3, g_secp256_step)) + word_len, (g_secp256_step / 4) - word_len);

    pke_pre_calc_mont((uint32_t *)(PKE_B(3, g_secp256_step)));

	pke_word_copy((uint32_t *)(PKE_B(0, g_secp256_step)), Px, word_len);
	pke_word_copy((uint32_t *)(PKE_B(1, g_secp256_step)), Py, word_len);
	pke_word_copy((uint32_t *)(PKE_A(5, g_secp256_step)), secp256r1_a, word_len);
	pke_word_copy((uint32_t *)(PKE_A(4, g_secp256_step)), k, word_len);
	pke_word_copy((uint32_t *)(PKE_B(5, g_secp256_step)), secp256r1_n, word_len);

    pke_word_clean((uint32_t *)(PKE_B(0, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
    pke_word_clean((uint32_t *)(PKE_B(1, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
    pke_word_clean((uint32_t *)(PKE_A(5, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
    pke_word_clean((uint32_t *)(PKE_A(4, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
    pke_word_clean((uint32_t *)(PKE_B(5, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);

	PKE_EXE_CONF = 0x16;
    ret = pke_run(0x10);
    if (ret) {
        return ret;
    }

	pke_word_read((uint32_t *)(PKE_A(0, g_secp256_step)), Qx, word_len);
	if(NULL != Qy) {
		pke_word_read((uint32_t *)(PKE_A(1, g_secp256_step)), Qy, word_len);
	}

	return 0;
}

uint32_t eccp_pointAdd(uint32_t *P1x, uint32_t *P1y, uint32_t *P2x, uint32_t *P2y,
					   uint32_t *Qx, uint32_t *Qy)
{
	uint32_t word_len = 8;
	uint32_t ret;

	pke_word_copy((uint32_t *)(PKE_B(3, g_secp256_step)), secp256r1_p, word_len);
    pke_word_clean((uint32_t *)(PKE_B(3, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);

    pke_pre_calc_mont((uint32_t *)(PKE_B(3, g_secp256_step)));

	pke_word_copy((uint32_t *)(PKE_A(0, g_secp256_step)), P1x, word_len);
	pke_word_copy((uint32_t *)(PKE_A(1, g_secp256_step)), P1y, word_len);
	pke_word_copy((uint32_t *)(PKE_B(0, g_secp256_step)), P2x, word_len);
	pke_word_copy((uint32_t *)(PKE_B(1, g_secp256_step)), P2y, word_len);
	pke_word_copy((uint32_t *)(PKE_A(5, g_secp256_step)), secp256r1_a, word_len);

	pke_word_clean((uint32_t *)(PKE_A(0, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
	pke_word_clean((uint32_t *)(PKE_A(1, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
	pke_word_clean((uint32_t *)(PKE_B(0, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
	pke_word_clean((uint32_t *)(PKE_B(1, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);
	pke_word_clean((uint32_t *)(PKE_A(5, g_secp256_step)) + word_len, (g_secp256_step / 4)-word_len);

	PKE_EXE_CONF = 0x15;
    ret = pke_run(0x08);
    if (ret) {
        return ret;
    }

	pke_word_read((uint32_t *)(PKE_A(0, g_secp256_step)), Qx, word_len);
	if(Qy != NULL) {
		pke_word_read((uint32_t *)(PKE_A(1, g_secp256_step)), Qy, word_len);
	}

	return 0;
}


int ecdsa_verify(uint8_t *hash, uint8_t *sig, uint8_t *pubkey)
{
    uint32_t e[MAX_ECCP_WORD_LEN];
    uint32_t r[MAX_ECCP_WORD_LEN];
    uint32_t s[MAX_ECCP_WORD_LEN];
    uint32_t x[MAX_ECCP_WORD_LEN];
    uint32_t tmp[MAX_ECCP_WORD_LEN];
    int ret;

    g_secp256_step = 0x24;
	PKE_CFG &= ~(0x07FFFF);
	PKE_CFG |= (2 << 16) | (256);

    swap_copy(sig, (uint8_t *)r, N_BYTE_LEN);
    if (u32_bignum_check_zero(r, N_WORD_LEN)) {
        return -1;
    } else if (u32_big_num_cmp(r, N_WORD_LEN, secp256r1_n, N_WORD_LEN) >= 0) {
        return -1;
    }


    swap_copy(sig + N_BYTE_LEN, (uint8_t *)s, N_BYTE_LEN);
    if (u32_bignum_check_zero(s, N_WORD_LEN)) {
        return -1;
    } else if (u32_big_num_cmp(s, N_WORD_LEN, secp256r1_n, N_WORD_LEN) >= 0) {
        return -1;
    }

	ret = pke_modinv(secp256r1_n, s, tmp, N_WORD_LEN, N_WORD_LEN);
    if (ret) {
        return -1;
    }

    memset((uint8_t *)e, 0, MAX_ECCP_WORD_LEN);
    swap_copy((uint8_t *)hash, (uint8_t *)e, N_BYTE_LEN);

    if (u32_big_num_cmp(e, N_WORD_LEN, secp256r1_n, N_WORD_LEN) >= 0) {
        ret = pke_sub(e, secp256r1_n, e, N_WORD_LEN);
        if (ret) {
            return -1;
        }
    }

	ret = pke_pre_calc_mont(secp256r1_n);
    if (ret) {
        return -1;
    }

    PKE_EXE_CONF = 0x0000;
    ret = pke_modmul_internal(secp256r1_n, e, tmp, x, N_WORD_LEN);
    if (ret) {
        return -1;
    }

	ret = pke_modmul_internal(secp256r1_n, r, tmp, tmp, N_WORD_LEN);
    if (ret) {
       return -1;
    }

	memset((uint8_t *)e, 0, (MAX_WORD_LEN << 2) - P_BYTE_LEN);
	memset((uint8_t *)s, 0, (MAX_WORD_LEN << 2) - P_BYTE_LEN);

	swap_copy(pubkey, (uint8_t *)e, P_BYTE_LEN);
	swap_copy(pubkey + P_BYTE_LEN, (uint8_t *)s, P_BYTE_LEN);
	ret = eccp_point_mul(tmp, e, s, e, s);
    if (ret) {
        return -1;
    }

	if(!u32_bignum_check_zero(x, N_WORD_LEN)) {
		ret = eccp_point_mul(x, secp256r1_gx, secp256r1_gy, x, tmp);
		if(ret) {
			return -1;
		}

		ret = eccp_pointAdd(e, s, x, tmp, e, s);
		if(ret) {
			return -1;
		}
	}

	ret = pke_mod(e, P_WORD_LEN, secp256r1_n, N_WORD_LEN, tmp);
    if (ret) {
        return -1;
    }

	if(u32_big_num_cmp(tmp, N_WORD_LEN, r, N_WORD_LEN)) {
        return -1;
    }

    return 0;


}
