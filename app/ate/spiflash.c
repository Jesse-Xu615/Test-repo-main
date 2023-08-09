/*
 * Copyright 2023-2024 Senscomm Semiconductor Co., Ltd.	All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifdef CONFIG_ATE_SPIFLASH

#include <stdint.h>
#include <stdio.h>
#include <soc.h>

#include "hal/io.h"
#include "mmap.h"

#include "uart.h"

#define SPI_FLASH_MAX_ID_LEN	6

#define SPI_FLASH_OP_RDID	    0x9f	/* Read JEDEC ID */

#define IdRev 		0x0
#define TransFmt 	0x10
#define DirectIO	0x14
#define TransCtrl	0x20
#define Cmd		    0x24
#define Addr		0x28
#define Data		0x2c
#define Ctrl		0x30
#define Status		0x34
#define IntrEn		0x38
#define IntrSt		0x3c
#define Timing		0x40
#define MemCtrl		0x50
#define Config		0x7c

static inline u32 atcspi200_readl(u32 oft)
{
	return readl(SPI0_BASE_ADDR + oft);
}

static inline void atcspi200_writel(u32 val, u32 oft)
{
	writel(val, SPI0_BASE_ADDR + oft);
}


#endif


#define JEDEC_MFC_ISSI              0x9D
#define JEDEC_MFC_WINBOND           0xEF
#define JEDEC_MFC_GIGADEVICE        0xC8
#define JEDEC_MFC_MACRONIX          0xC2
#define JEDEC_MFC_MICRON            0x20
#define JEDEC_MFC_MICROCHIP         0xBF
#define JEDEC_MFC_ADESTO            0x1F
#define JEDEC_MFC_EON               0x1C
#define JEDEC_MFC_XTX               0x0B
#define JEDEC_MFC_PUYA              0x85
#define JEDEC_MFC_GIANTEC           0xC4

#define FLASH_CAPACITY_256KBIT      0x09
#define FLASH_CAPACITY_512KBIT      0x10
#define FLASH_CAPACITY_1MBIT        0x11
#define FLASH_CAPACITY_2MBIT        0x12
#define FLASH_CAPACITY_4MBIT        0x13
#define FLASH_CAPACITY_8MBIT        0x14
#define FLASH_CAPACITY_16MBIT       0x15
#define FLASH_CAPACITY_32MBIT       0x16
#define FLASH_CAPACITY_64MBIT       0x17
#define FLASH_CAPACITY_128MBIT      0x18

#define EXPECTED_MFC                JEDEC_MFC_GIGADEVICE
#define EXPECTED_TYPE               0x60
#define EXPECTED_CAPACITY           FLASH_CAPACITY_32MBIT

int spiflash(void)
{
#ifdef CONFIG_ATE_SPIFLASH
    u32 v;
    u32 trans;
    u8 id[SPI_FLASH_MAX_ID_LEN];
    int i;

    uart_printf("Test spiflash\n");

    /* SPI transfer format byte unit */
	v = atcspi200_readl(TransFmt);
    v &= ~(0x80);
    atcspi200_writel(v, TransFmt);

    /* wait idle */
    do {
        v = atcspi200_readl(Status);
    } while ((v & 0x01) != 0);

    /* clear fifo */
    atcspi200_writel(0x06, Ctrl);
    do {
        v = atcspi200_readl(Ctrl);
    } while (v & 0x06);

    /* write read id command */
    trans = 1 << 30 | /* Command enable */
            2 << 24 | /* Read */
            (SPI_FLASH_MAX_ID_LEN - 1);
    atcspi200_writel(trans, TransCtrl);
    atcspi200_writel(SPI_FLASH_OP_RDID, Cmd);

    /* read id */
    for (i = 0; i < SPI_FLASH_MAX_ID_LEN; i++) {
        /* wait fifo */
        do {
            v = atcspi200_readl(Status);
        } while ((v & (1 << 14)) != 0);
        id[i] = atcspi200_readl(Data) & 0xFF;
    }

    uart_printf("flash id: %02x %02x %02x\n", id[0], id[1], id[2]);

    /* check id */

    if (id[0] == JEDEC_MFC_GIGADEVICE &&
        id[2] == FLASH_CAPACITY_16MBIT) {
        goto found;
    }

    if (id[0] == JEDEC_MFC_GIGADEVICE &&
        id[2] == FLASH_CAPACITY_32MBIT) {
        goto found;
    }

    if (id[0] == JEDEC_MFC_GIANTEC &&
        id[2] == FLASH_CAPACITY_16MBIT) {
        goto found;
    }

    if (id[0] == JEDEC_MFC_GIANTEC &&
        id[2] == FLASH_CAPACITY_32MBIT) {
        goto found;
    }

    uart_printf("Test spiflash error\n");
    return -1;

found:
    uart_printf("Test spiflash ok\n");
#endif
	return 0;
}

