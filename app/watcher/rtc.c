#include <stdint.h>
#include <stdlib.h>

#include "soc.h"
#include "hal/io.h"
#include "hal/rtc.h"

#define OFT_RTC_32K     0x04
#define OFT_RTC_CNT     0x10
#define OFT_RTC_ALM     0x14
#define OFT_RTC_CTR     0x18
#define OFT_RTC_STS     0x1c

#define rtc_read(o)     readl(RTC_BASE_ADDR + o)
#define rtc_write(v, o) writel(v, RTC_BASE_ADDR + o)
#define rtc_sync()      do { \
                            if (readl(RTC_BASE_ADDR + OFT_RTC_STS) & (0x1 << 16)) \
                                break; \
                        } while (1);

void rtc_int_handler(void)
{
	uint32_t intst = rtc_read(OFT_RTC_STS);
	uint32_t val;

	rtc_write(intst, OFT_RTC_STS);

	val = rtc_read(OFT_RTC_CTR) & ~(1 << 2);
	rtc_write(val, OFT_RTC_CTR);
}

uint32_t rtc_time_get(void)
{
	uint32_t rtc_cnt;
	uint32_t cnt;
	uint16_t remain;

	/* To work-around hardware issue of "32K part"
	 * not synchronized with "second part"
	 */

	while (((remain = rtc_read(OFT_RTC_32K)) & 0x7fff) == 0);

	cnt = rtc_read(OFT_RTC_CNT);
	rtc_cnt = remain;
	rtc_cnt += (cnt & 0x3f) * _32khz;
	rtc_cnt += ((cnt >> 6) & 0x3f) * 60 * _32khz;
	rtc_cnt += ((cnt >> 12) & 0x1f) * 60 * 60 * _32khz;

	return rtc_cnt;
}

void rtc_alarm_set(uint32_t count)
{
	uint32_t expiry;
	uint32_t sec;
	uint32_t min;
	uint32_t hour;

	/* To work-around hardware issue of "32K part"
	 * not synchronized with "second part"
	 */

	if ((count & 0x00007fff) == 0) {
		count -= 1;
	}

	sec = count / _32khz;
	min = sec / 60;
	sec = sec % 60;
	hour = min / 60;
	min = min % 60;
	hour = hour % 24;

	expiry = count % _32khz;
	expiry |= sec << 15;
	expiry |= min << 21;
	expiry |= hour << 27;

	rtc_sync();
	rtc_write(expiry, OFT_RTC_ALM);
	rtc_write(rtc_read(OFT_RTC_CTR) | (1 << 2), OFT_RTC_CTR);

	/* sync again here
	 * as this may be just before going into the deep sleep mode
	 */

	rtc_sync();
}


