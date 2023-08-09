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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hal/kernel.h>

#include "wise_log.h"
#include "wise_wifi.h"
#include "wise_event_loop.h"

static const char *TAG = "simple wifi";

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

static osSemaphoreId_t s_scan_kick;

static void scantask(void *param)
{
	wifi_scan_config_t sconf = {
		.ssid = NULL,
		.bssid = NULL,
		.channel = 0,
		.show_hidden = false,
		.scan_type = WIFI_SCAN_TYPE_ACTIVE,
	};
	//uint8_t bssid[] = {0x00, 0x1E, 0xE5, 0x76, 0xE8, 0x32};
	//sconf.ssid = (uint8_t *)CONFIG_WIFI_SSID;
	//sconf.bssid = bssid;
	//sconf.channel = 8;
	sconf.show_hidden = true;
#if 0
	if (sconf.scan_type == WIFI_SCAN_TYPE_ACTIVE) {
		sconf.scan_time.active.min = 500;
		sconf.scan_time.active.max = 1000;
	} else
		sconf.scan_time.passive = 1000;
#endif
	while (1) {
		if (osSemaphoreAcquire(s_scan_kick, 20) == osOK) {
			WISE_LOGV(TAG, "scant kicked");
			wise_wifi_scan_start(&sconf, true, WIFI_IF_STA);
		} else
			WISE_LOGV(TAG, "scant wait");
	}
}

static void test_scan(void) __maybe_unused;
static void test_scan(void)
{
	osSemaphoreRelease(s_scan_kick);
}

static wise_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
	case SYSTEM_EVENT_STA_START:
		wise_wifi_connect(WIFI_IF_STA);
	break;
	case SYSTEM_EVENT_STA_CONNECTED:
		WISE_LOGI(TAG, "station connected");
	break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		WISE_LOGW(TAG, "station disconnected");
		wise_wifi_connect(WIFI_IF_STA);
	break;
	case SYSTEM_EVENT_STA_GOT_IP:
		WISE_LOGI(TAG, "got ip:%s",
				ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
#if 0
		test_scan();
#endif
	break;
	case SYSTEM_EVENT_STA_LOST_IP:
		WISE_LOGW(TAG, "lost ip");
	break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		WISE_LOGI(TAG, "station:"MACSTR" join, AID=%d",
				MAC2STR(event->event_info.sta_connected.mac),
				event->event_info.sta_connected.aid);
	break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		WISE_LOGI(TAG, "station:"MACSTR" leave, AID=%d",
				MAC2STR(event->event_info.sta_disconnected.mac),
				event->event_info.sta_disconnected.aid);
	break;
	case SYSTEM_EVENT_SCAN_DONE:
	{
		uint16_t ap_num; 
		wifi_ap_record_t *ap_rec;
		int i;

		WISE_ERROR_CHECK(wise_wifi_scan_get_ap_num(&ap_num));
		if (!ap_num) {
			WISE_LOGW(TAG, "No AP found in scan");
#if 0
			test_scan();
#endif
			break;
		}
		ap_rec = (wifi_ap_record_t *)zalloc(ap_num * sizeof(wifi_ap_record_t));
		WISE_ERROR_CHECK(wise_wifi_scan_get_ap_records(0, &ap_num, ap_rec));
		for (i = 0; i < ap_num; i++) {
			WISE_LOGI(TAG, "%3d.%-32s[%2d|%d][%-19s|%-9s|%-9s][%c|%c|%c]",
				    (i + 1), (const char *)ap_rec[i].ssid,
				    ap_rec[i].primary, ap_rec[i].rssi,
				    ap_rec[i].authmode == WIFI_AUTH_OPEN ? "OPEN" :
				    ap_rec[i].authmode == WIFI_AUTH_WEP ? "WEP" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA_PSK ? "WPA_PSK" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA2_PSK ? "WPA2_PSK" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA_ENTERPRISE ? "WPA_ENTERPRISE" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA2_ENTERPRISE ? "WPA2_ENTERPRISE" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA_WPA2_PSK ? "WPA_WPA2_PSK" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA_WPA2_ENTERPRISE ? "WPA_WPA2_ENTERPRISE" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA3_SAE ? "WPA3_SAE" :
				    ap_rec[i].authmode == WIFI_AUTH_WPA3_ENTERPRISE ? "WPA3_ENTERPRISE" : "NONE",
				    ap_rec[i].pairwise_cipher == WIFI_CIPHER_TYPE_WEP40 ? "WEP40" :
				    ap_rec[i].pairwise_cipher == WIFI_CIPHER_TYPE_WEP104 ? "WEP104" :
				    ap_rec[i].pairwise_cipher == WIFI_CIPHER_TYPE_TKIP ? "TKIP" :
				    ap_rec[i].pairwise_cipher == WIFI_CIPHER_TYPE_CCMP ? "CCMP" :
				    ap_rec[i].pairwise_cipher == WIFI_CIPHER_TYPE_TKIP_CCMP ? "TKIP_CCMP" : "NONE",
				    ap_rec[i].group_cipher == WIFI_CIPHER_TYPE_WEP40 ? "WEP40" :
				    ap_rec[i].group_cipher == WIFI_CIPHER_TYPE_WEP104 ? "WEP104" :
				    ap_rec[i].group_cipher == WIFI_CIPHER_TYPE_TKIP ? "TKIP" :
				    ap_rec[i].group_cipher == WIFI_CIPHER_TYPE_CCMP ? "CCMP" :
				    ap_rec[i].group_cipher == WIFI_CIPHER_TYPE_TKIP_CCMP ? "TKIP_CCMP" : "NONE",
				    ap_rec[i].phy_11b ? 'B' : ' ',
				    ap_rec[i].phy_11g ? 'G' : ' ',
				    ap_rec[i].phy_11n ? 'N' : ' ');
		}
		free(ap_rec);
#if 0
		test_scan();
#endif
	}
	break;
	default:
	break;
	}
	return WISE_OK;
}

#if CONFIG_WIFI_MODE_AP
void wifi_init_softap()
{
	wifi_config_t wifi_config = {
		.ap = {
			.ssid = CONFIG_WIFI_SSID,
			.ssid_len = strlen(CONFIG_WIFI_SSID),
			.password = CONFIG_WIFI_PASSWORD,
			.max_connection = CONFIG_MAX_STA_CONN,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
			.channel = 8,
			.ssid_hidden = 0,
			//.beacon_interval = 200
		},
	};
	wifi_ip_info_t ip_info;

	if (!wifi_config.ap.password 
			|| strlen((char *)wifi_config.ap.password) == 0) {
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	WISE_ERROR_CHECK(wise_wifi_set_mode(WIFI_MODE_AP, WIFI_IF_AP));
	WISE_ERROR_CHECK(wise_wifi_set_config(WISE_IF_WIFI_AP, &wifi_config, WIFI_IF_AP, NULL));

	memset(&ip_info, 0, sizeof(ip_info));
	IP4_ADDR(&ip_info.ip, 192, 168, 200, 1);
	IP4_ADDR(&ip_info.nm, 255, 255, 255, 0);
	WISE_ERROR_CHECK(wise_wifi_set_ip_info(WIFI_IF_AP, &ip_info));
	WISE_ERROR_CHECK(wise_wifi_start(WIFI_IF_AP));

	WISE_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
			CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
}
#else
void wifi_init_sta()
{
    wifi_config_t wifi_config;

	memset(&wifi_config, 0, sizeof(wifi_config_t));
	strncpy((char *) wifi_config.sta.ssid,
			CONFIG_WIFI_SSID, FIELD_SIZE(wifi_sta_config_t, ssid));
	if (strlen(CONFIG_WIFI_PASSWORD) > 0) {
		wifi_config.sta.proto = WIFI_PROTO_WPA2;
		wifi_config.sta.pmf_mode = WIFI_PMF_DISABLE;
		strncpy((char *) wifi_config.sta.password,
				CONFIG_WIFI_PASSWORD,
				FIELD_SIZE(wifi_sta_config_t, password));
	}

    WISE_ERROR_CHECK(wise_wifi_set_mode(WIFI_MODE_STA, WIFI_IF_STA) );
    WISE_ERROR_CHECK(wise_wifi_set_config(WISE_IF_WIFI_STA, &wifi_config, WIFI_IF_STA, NULL) );
    WISE_ERROR_CHECK(wise_wifi_start(WIFI_IF_STA) );

    WISE_LOGI(TAG, "wifi_init_sta finished.");
    WISE_LOGI(TAG, "connect to ap SSID:%s password:%s",
		    CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
}
#endif

int main(void)
{
	osThreadAttr_t attr = {
		.name		= "scant",
		.stack_size 	= 2048,
		.priority	= osPriorityNormal,
	};

	s_scan_kick = osSemaphoreNew(1, 0, NULL);
	if (!s_scan_kick) {
		WISE_LOGE(TAG, "sem creation failed!");
		return -1;
	}

	if (osThreadNew(scantask, NULL, &attr) == NULL) {
		WISE_LOGE(TAG, "task creation failed!");
		return -1;
	}

	WISE_ERROR_CHECK(wise_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	WISE_ERROR_CHECK(wise_wifi_init(&cfg));

#if CONFIG_WIFI_MODE_AP
	WISE_LOGI(TAG, "WIFI_MODE_AP");
	wifi_init_softap();
#else
	WISE_LOGI(TAG, "WIFI_MODE_STA");
	wifi_init_sta();
#endif /* CONFIG_WIFI_MODE_AP */

	return 0;
}
