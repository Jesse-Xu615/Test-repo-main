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


#include <hal/kernel.h>

#include "hash.h"

#define __IO     volatile

typedef struct {
    __IO unsigned int CTRL;          /* Hash control register */
    __IO unsigned int CFG;           /* Hash configuration register */
    __IO unsigned int SR1;           /* Hash status register 1 */
    __IO unsigned int SR2;           /* Hash status register 2 */
    __IO unsigned int REV1[4];       /* reserved */
    __IO unsigned int PCR_LEN[4];    /* Message length regiter */
    __IO unsigned int OUT[16];       /* Hash value output register */
    __IO unsigned int IN[16];        /* Hash value input register */
    __IO unsigned int VERSION;       /* version register */
    __IO unsigned int REV2[19];      /* reserved */
    __IO unsigned int MDIN[32];      /* Message input resgister */
    __IO unsigned int DMA_SA;        /* DMA source adddress register */
    __IO unsigned int DMA_DA;        /* DMA destinatin address register */
    __IO unsigned int DMA_RLEN;      /* DMA read data length register */
    __IO unsigned int DMA_WLEN;      /* DMA write data legnth register */
} HASH_RegDef;

#define DEV_HASH        ((HASH_RegDef *) HASH_BASE_ADDR)

uint32_t const SHA256_IV[8] = {0x67E6096A,0x85AE67BB,0x72F36E3C,0x3AF54FA5,0x7F520E51,0x8C68059B,0xABD9831F,0x19CDE05B,};

static void hash_input_msg(uint8_t *msg, uint32_t msg_words)
{
    uint32_t tmp;
    uint32_t i;

    if (((uint32_t)msg) & 3) {
        for (i = 0; i < msg_words; i++) {
            memcpy(&tmp, msg, 4);
            DEV_HASH->MDIN[i] = tmp;
            msg += 4;
        }
    } else {
        for (i = 0; i < msg_words; i++) {
            DEV_HASH->MDIN[i] = *((uint32_t *)msg);
            msg += 4;
        }
    }
}

static void hash_set_last_block(uint8_t tag)
{
    if (tag) {
        DEV_HASH->CFG |= (1 << 24);
    } else {
        DEV_HASH->CFG &= (~(1 << 24));
    }
}

static void hash_set_msg_total_byte_len(uint32_t *msg_total_bytes, uint32_t words)
{
    while (words--) {
        DEV_HASH->PCR_LEN[words] = msg_total_bytes[words];
    }
}

static void hash_start_and_wait(void)
{
    DEV_HASH->SR2 |= 1;
    DEV_HASH->CTRL |= 1;
    while ((DEV_HASH->SR2 & 1) == 0) ;
    DEV_HASH->SR2 |= 1;
}


void hash_init(struct hash_ctx *ctx, uint8_t algo)
{
    int i;

    memset(ctx, 0, sizeof(*ctx));

    /* dma off */
    DEV_HASH->CFG &= ~(1 << 16);
    /* interrupt off */
    DEV_HASH->CFG &= ~(1 << 17);
    /* not the last block */
    DEV_HASH->CFG &= ~(1 << 24);
    /* algorithm sha256 */
    DEV_HASH->CFG &= ~(0x0F << 0);
    DEV_HASH->CFG |= (algo << 0);
    /* clear meesage length */
    DEV_HASH->PCR_LEN[0] = 0;
    DEV_HASH->PCR_LEN[1] = 0;
    DEV_HASH->PCR_LEN[2] = 0;
    DEV_HASH->PCR_LEN[3] = 0;

    for (i = 0; i < 8; i++) {
        DEV_HASH->IN[i] = SHA256_IV[i];
    }
}

void hash_update(struct hash_ctx *ctx, uint8_t *m, uint32_t size)
{
    uint32_t offset = 0;

    ctx->size += size;

    while (1) {
        if (size - offset >= 64) {
            hash_input_msg(&m[offset], (64 / 4));
            hash_start_and_wait();
            offset += 64;
        } else {
            ctx->remain_size = size - offset;
            if (ctx->remain_size) {
                memcpy(ctx->remain, &m[offset], ctx->remain_size);
            }
            break;
        }
    }
}

void hash_finish(struct hash_ctx *ctx, uint8_t *digest)
{
    uint32_t *p = (uint32_t *)digest;
    int i;
    uint32_t total[4];

    hash_set_last_block(1);
    total[0] = ctx->size;
    total[1] = 0;
    total[2] = 0;
    total[3] = 0;

    hash_set_msg_total_byte_len(total, 4);
    hash_input_msg(ctx->remain, ((ctx->remain_size + 3) / 4));
    hash_start_and_wait();

    for (i = 0; i < (32/4); i++) {
        *p = DEV_HASH->OUT[i];
        p++;
    }
}
