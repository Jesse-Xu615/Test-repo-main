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
#include "cmsis_os.h"
#include "mutex.h"
#include "lwip/tcpip.h"
#include "lwip/pbuf.h"
#include "mbuf.h"
#include "lwip-glue.h"

#include <stdio.h>
#include <string.h>

#include <soc.h>
#include <hal/kernel.h>
#include <hal/init.h>
#include <hal/console.h>

#include <wlan.h>
#include <cli.h>

#define TEST_DATA_SIZE		(2*1024)
static uint8_t g_test_data[TEST_DATA_SIZE];

void print_raw(uint8_t *data, int len)
{
	uint8_t *p = data;
	int i;
		
	for (i = 0; i < len; i++) {
		if (0 == (i & 7))
			printf("\n%03d :", i);
		printf("%02X ", *p);
		p++;
	}
	printf("\n");
}

void print_pbuf(struct pbuf *pb)
{
	int idx = 0;
	int headroom;

	printf("\n[PBUF] tot_len:%d, addr:%p\n", pb->tot_len, pb);
	while (pb) {
		if (pb->type_internal & PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS)
			headroom = (size_t)(pb->payload - (void *)pb)
				- LWIP_MEM_ALIGN_SIZE(sizeof(struct pbuf));
		else
			headroom = 0;
		printf("\t[%d] next:%p, payload:%p, headroom:%d, tot_len:%d, len:%d, type:0x%x\n",
				idx,
				pb->next,
				pb->payload,
				headroom,
				pb->tot_len,
				pb->len,
				pb->type_internal);
		print_raw(pb->payload, pb->len);
		pb = pb->next;
		idx++;
	}
}

void print_mbuf(struct mbuf *mb)
{
	int idx = 0;

	printf("\n[MBUF] pkt_len:%d\n", mb->m_pkthdr.len);
	while (mb) {
		printf("\t[%d] m_hdr (leading:%d, len:%d, type:%d, flags:%d, next:%p, data:%p)\n",
				idx,
				M_LEADINGSPACE(mb),
				mb->m_len,
				mb->m_type,
				mb->m_flags,
				mb->m_next,
				mb->m_data);
		if (mb->m_flags & M_EXT)
			printf("\t[%d] m_ext (buf:%p, size:%d, type:0x%x)\n",
					idx,
					mb->m_ext.ext_buf,
					mb->m_ext.ext_size,
					mb->m_ext.ext_type);

		print_raw((uint8_t *)mb->m_data, mb->m_len);
		mb = mb->m_next;
		idx++;
	}
}

static struct mbuf *mbuf_test(struct mbuf *mb)
{
	int len = mb->m_pkthdr.len;
	struct mbuf *m;

	printf("\n\ntest 1. dup2\n\n");
	m = m_dup2(mb, M_NOWAIT);
	print_mbuf(m);
	m_freem(m);

	printf("\n\ntest 2. copyback\n\n");

	m_copyback(mb, (len / 4), (len / 4), (caddr_t)g_test_data, 0);
	print_mbuf(mb);

	printf("\n\ntest 3. adj(+)\n\n");

	len = mb->m_pkthdr.len;
	m_adj(mb, (len / 2));
	print_mbuf(mb);

	printf("\n\ntest 4. adj(-)\n\n");

	len = mb->m_pkthdr.len;
	m_adj(mb, -(len / 4));
	print_mbuf(mb);

	printf("\n\ntest 5. prepend\n\n");

	len = mb->m_pkthdr.len;
	M_PREPEND(mb, 48, M_NOWAIT);
	print_mbuf(mb);

	return mb;
}

int main(void)
{
	int i;
	struct pbuf *pb, *pb1;
	struct mbuf *mb, *m;

	/* 0. Fill test data */
	for (i = 0; i < TEST_DATA_SIZE; i++)
		g_test_data[i] = (uint8_t)i;


	/* 1. UL */
	printf("\n\n1. UL\n");
	/* 1.1 pbuf(PBUF_RAM) -> MBUF */
	printf("1.1 pbuf(PBUF_RAM) -> MBUF\n\n");

	pb = pbuf_alloc(PBUF_TRANSPORT, 500, PBUF_RAM);

	pbuf_take(pb, g_test_data, 500);
	print_pbuf(pb);

	mb = m_frompbuf(pb, 48);
	print_mbuf(mb);

	mb = mbuf_test(mb);

	m_freem(mb);
	pbuf_free(pb);

	/* 1.2 pbuf(PBUF_RAM + PBUF_ROM) -> MBUF */
	printf("\n\n1.2 pbuf(PBUF_RAM + PBUF_ROM) -> MBUF\n\n");

	pb = pbuf_alloc(PBUF_TRANSPORT, 0, PBUF_RAM);
	/* print_pbuf(pb); */
	pb1 = pbuf_alloc(PBUF_RAW, 256, PBUF_ROM);
	pb1->payload = &g_test_data[128];
	/* print_pbuf(pb1); */

	pbuf_cat(pb, pb1);
	print_pbuf(pb);

	mb = m_frompbuf(pb, 48);
	print_mbuf(mb);

	mb = mbuf_test(mb);

	m_freem(mb);
	pbuf_free(pb);

	/* 1.3 pbuf(PBUF_RAM + PBUF_REF) -> MBUF */
	printf("\n\n1.3 pbuf(PBUF_RAM + PBUF_REF) -> MBUF\n\n");

	pb = pbuf_alloc(PBUF_TRANSPORT, 0, PBUF_RAM);
	/* print_pbuf(pb); */
	pb1 = pbuf_alloc(PBUF_RAW, 1329, PBUF_REF);
	pb1->payload = &g_test_data[0];
	/* print_pbuf(pb1); */

	pbuf_cat(pb, pb1);
	print_pbuf(pb);

	mb = m_frompbuf(pb, 48);
	print_mbuf(mb);

	mb = mbuf_test(mb);

	m_freem(mb);
	pbuf_free(pb);

	stats_display();

	/* 2. DL */
	printf("\n\n2. DL\n");
	/* 2.1 MBUF -> pbuf(PBUF_POOL) */
	printf("\n\n2.1 MBUF -> pbuf(PBUF_POOL)\n\n");

	mb = m_gethdr(M_NOWAIT, MT_DATA);
	m_copyback(mb, 0, 1500, (caddr_t)g_test_data, 48);
	print_mbuf(mb);

	m = m_dup2(mb, M_NOWAIT);
	m_freem(mb);
	print_mbuf(m);

	pb = m_topbuf(m);
	print_pbuf(pb);

	pbuf_free(pb);

	stats_display();
	printf("\n\nEND\n\n");

	return 0;
}
