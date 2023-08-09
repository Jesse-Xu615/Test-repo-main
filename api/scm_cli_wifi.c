/*
 * Copyright 2023-2025 Senscomm Semiconductor Co., Ltd.	All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <hal/kernel.h>
#include <hal/wlan.h>
#include <hal/kmem.h>
#include <cli.h>
#include <kernel.h>
#include <wise_wpas.h>
#include <bss.h>
#include <FreeRTOS.h>
#include <task.h>
#include <compat_if.h>
#include <if_media.h>
#ifdef CONFIG_SUPPORT_SCDC
#include <fweh.h>
#endif
#include <net80211/ieee80211_var.h>
#include "wise_wifi.h"
#include "wise_event_loop.h"
#include "scm_wifi.h"
#include "scm_cli.h"

#ifdef CONFIG_SUPPORT_WIFI_REPEATER
#include "scm_channel.h"
#endif

#ifdef CONFIG_CLI_WIFI_DHCP
#include "dhcps.h"
#endif

#ifdef CONFIG_CLI_WIFI

#define SCM_CLI_OK     (0)  //WISE_OK
#define SCM_CLI_FAIL   (-1) //WISE_FAIL

#ifdef SCM_CLI_DBG
#define SCM_CLI_LOG printf
#else
#define SCM_CLI_LOG(...) //printf
#endif
#define SCM_CLI_ERR printf
#define SCM_CLI_INFO printf



#define PARM_SET_INT(name, dest, src) \
{ \
	dest = src; \
	SCM_CLI_LOG("%s = %d \n", name, dest); \
}

#define IS_OK(r) (r == SCM_CLI_OK)

#define SCM_NEED_DHCP_START(event) ((event)->event_info.connected.not_en_dhcp == false)

#ifdef CONFIG_CLI_WIFI_SCMCHANNEL
#define MAX_CMD_LEN		20
#define MAC_ADDR_LEN	6
#define WIFI_STA_NETIF_NAME "wlan0"

enum {
	HOST_CMD_GET_MAC,
	HOST_CMD_GET_IP,
	HOST_CMD_SET_FILTER,
	HOST_CMD_TBTT
};

static char cmd[][MAX_CMD_LEN] = {
	"cmd_get_mac",
	"cmd_get_ip",
	"cmd_set_filter"
};

static int g_host_app_ready = 0;
#define SCM_IS_HOST_APP_READY() g_host_app_ready
#define SCM_SET_HOST_AP_READY(s) { \
	g_host_app_ready = s; \
}
#endif

wise_err_t cli_event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		SCM_CLI_INFO("\r\nSTA_START\r\n");
		break;
	case SYSTEM_EVENT_STA_STOP:
		SCM_CLI_INFO("\r\nSTA_STOP\r\n");
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		SCM_CLI_INFO("\r\nWIFI GOT IP\r\n");
#ifdef CONFIG_CLI_WIFI_SCMCHANNEL
		if (SCM_IS_HOST_APP_READY()) {
			char msg[512];
			int msg_len = 0;

			msg[0] = HOST_CMD_GET_IP;
			msg_len++;
			if (scm_channel_construct_ip_msg(WIFI_STA_NETIF_NAME, &msg[1], &msg_len)) {
				return WISE_FAIL;
			}
			scm_channel_send_to_host(msg, msg_len);
			SCM_CLI_INFO("\r\nWIFI GOT IP indicate\r\n");
		} else {
			SCM_CLI_INFO("\r\nWIFI GOT IP wait host app\r\n");
		}
#endif
		break;
	case SYSTEM_EVENT_AP_START:
		SCM_CLI_INFO("\r\nAP_START\r\n");
		break;
	case SYSTEM_EVENT_AP_STOP:
		SCM_CLI_INFO("\r\nAP_STOP\r\n");
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		SCM_CLI_INFO("\r\nAP_STACONNECTED\r\n");
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		SCM_CLI_INFO("\r\nAP_STADISCONNECTED\r\n");
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		SCM_CLI_INFO("\r\nSTA_CONNECTED\r\n");

		if (SCM_NEED_DHCP_START(event)) {
			scm_wifi_status connect_status;
			netifapi_dhcp_start(scm_wifi_get_netif(WISE_IF_WIFI_STA));
			scm_wifi_sta_get_connect_info(&connect_status);
			scm_wifi_sta_dump_ap_info(&connect_status);
		}
#ifdef CONFIG_CLI_WIFI_SCMCHANNEL
		if (scm_channel_host_ready()) {
			scm_wifi_event_send(event, sizeof(system_event_t));
			SCM_CLI_INFO("\r\nWIFI CONNECTED indicate\r\n");
		}
#endif
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		SCM_CLI_INFO("\r\nWIFI DISCONNECT\r\n");
		break;
	case SYSTEM_EVENT_SCAN_DONE:
		SCM_CLI_INFO("WiFi: Scan results available\n");
		break;

	case SYSTEM_EVENT_SCM_CHANNEL:
		SCM_CLI_INFO("WiFi: Scm channel send msg\n");
		scm_wifi_event_send(event, sizeof(system_event_t));
		break;

	default:
		break;
	}

	return SCM_CLI_OK;
}

/**
 * CMD: wifi reg_evt_cb
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_event_callback(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	if (argc > 1) {
		goto done;
	}

	scm_wifi_unregister_event();
	ret = scm_wifi_register_event_callback(cli_event_handler, NULL);

done:
	return ret;
}
SCM_CLI(reg_evt_cb, scm_cli_event_callback, "", NULL);

#ifdef CONFIG_CLI_WIFI_STA

/**
 * CMD: wifi sta_start
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_sta_start(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	char ifname[WIFI_IFNAME_MAX_SIZE + 1] = {0};
	int len = sizeof(ifname);

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_start(ifname, &len);

	if(IS_OK(ret))
		SCM_CLI_INFO("ifname: %s\n", ifname);

done:
	return ret;
}
SCM_CLI(sta_start, scm_cli_sta_start, "", NULL);

/**
 * CMD: wifi sta_stop
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_sta_stop(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_stop();

done:
	return ret;
}
SCM_CLI(sta_stop, scm_cli_sta_stop, "", NULL);

/**
 * CMD: wifi sta_reconnect
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_sta_reconnect_policy(int argc, char *argv[])
{
	/* policy <enable> <timeout> <period> <count> | ex.policy 1 10 2 3 */
	if (argc != 5) {
		return SCM_CLI_FAIL;
	}

	return scm_wifi_sta_set_reconnect_policy(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
}
SCM_CLI(sta_set_reconnect, scm_cli_sta_reconnect_policy,
		"<enable> <timeout> <period> <count>",
		"1 10 2 3\r\n""timeout: 2 - 65535\r\n""period: 1 - 65535\r\n""count: 1 - 65535\n");

/**
 * CMD: wifi sta_cfg
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_sta_set_cfg(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	char *err = NULL;
	scm_wifi_assoc_request req = {0};
	char* ssid;
	char* key;
	int ssid_para = 1;
	int auth_para = ssid_para + 1;
	int key_para = auth_para + 1;
	int bssid_para = key_para + 1;
	int pairwise_para = bssid_para + 1;
	int hidden_ap = pairwise_para + 1;

	/* <ssid> <auth> <key> <bssid> <pairwise> <hidden ap> */
	if (argc != 7) {
		err = "Incorrect number of parameters";
		goto done;
	}

	ssid = argv[ssid_para];
	if (strlen(ssid) > SCM_WIFI_MAX_SSID_LEN) {
		err = "Invalid SSID Len";
		goto done;
	}
	memcpy(req.ssid, ssid, strlen(ssid));

	key = argv[key_para];
	if (strlen(key) > SCM_WIFI_MAX_KEY_LEN) {
		err = "Invalid KEY Len";
		goto done;
	}
	memcpy(req.key, key, strlen(key));

	req.auth = atoi(argv[auth_para]);
	req.pairwise = atoi(argv[pairwise_para]);
	req.hidden_ap = atoi(argv[hidden_ap]);

	if (hwaddr_aton(argv[bssid_para], req.bssid)) {
		err = "Invalid BSSID";
		goto done;
	}

	ret = scm_wifi_sta_set_config(&req, NULL);

done:
	if (err)
		SCM_CLI_ERR("%s: %s\n", __func__, err);
	return ret;
}
SCM_CLI(sta_cfg, scm_cli_sta_set_cfg,
		"<ssid> <auth> <key> <bssid> <pairwise> <hidden ap>",
		"connAP 2 12345678 00:00:00:00:00:00 1 0"
		"\r\n""auth: 0 OPEN, 1 WPAPSK, 2 WPA2PSK, 3 SAE"
		"\r\n""pairwise: 1 AES, 2 TKIP\n");

/**
 * CMD: wifi sta_rssi
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 * -val: RSSI value, should be minus, ex. -20
 */
#define UNAVAILABLE_RSSI 255

static int scm_cli_sta_get_ap_rssi(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_get_ap_rssi();

	if (ret == UNAVAILABLE_RSSI) {
		ret = SCM_CLI_FAIL;
		goto done;
	}

	SCM_CLI_INFO("RSSI: %d\n", ret);

	return SCM_CLI_OK;

done:
	return ret;
}
SCM_CLI(sta_rssi, scm_cli_sta_get_ap_rssi, "", NULL);
/**
 * CMD: wifi sta_connect
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_sta_con(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_connect();

done:
	return ret;
}
SCM_CLI(sta_connect, scm_cli_sta_con, "", NULL);

/**
 * CMD: wifi sta_disconnect
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 */
static int scm_cli_sta_discon(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_disconnect();

done:
	return ret;
}
SCM_CLI(sta_disconnect, scm_cli_sta_discon, "", NULL);

/**
 * CMD: wifi sta_get_connect
 * Response:
 * Print the connection information.
 * AP SSID: xxx
 * AP BSSID: %02x:%02x:%02x:%02x:%02x:%02x
 * AP CH: XX
 * Status: CONNECTED/DISCONNECTED
 */
static int scm_cli_sta_get_connect(int argc, char *argv[])
{
	int ret  = SCM_CLI_FAIL;
	scm_wifi_status connect_status = {0};

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_get_connect_info(&connect_status);

	if (IS_OK(ret))
		scm_wifi_sta_dump_ap_info(&connect_status);

done:
	return ret;
}
SCM_CLI(sta_get_connect, scm_cli_sta_get_connect, "", NULL);

static int scm_cli_set_ssid(char *ssid, char *req_ssid)
{
	if (!ssid) {
		return SCM_CLI_FAIL;
	}

	if (strlen(ssid) > SCM_WIFI_MAX_SSID_LEN) {
		SCM_CLI_ERR("invalid ssid length\n");
		return SCM_CLI_FAIL;
	}
	memcpy(req_ssid, ssid, strlen(ssid));
	SCM_CLI_LOG("ssid: %s\n", req_ssid);
	return SCM_CLI_OK;
}

static int scm_cli_set_psk(char *psk, unsigned char *req_psk)
{
	int i;

	if (strlen(psk) > WIFI_STA_PSK_LEN * 2) {
		SCM_CLI_ERR("invalid psk length\n");
		return SCM_CLI_FAIL;
	}

	SCM_CLI_LOG("psk string: %s\n", psk);

	/* Check each character */
	for (i = 0; i < WIFI_STA_PSK_LEN * 2; i++) {
		if (!isxdigit((unsigned char ) psk[i])) {
			SCM_CLI_ERR("Error: Invalid character '%c' at position %d.\n", psk[i], i);
			return SCM_CLI_FAIL;
		}
	}

	/* Convert to bytes */
	for (i = 0; i < WIFI_STA_PSK_LEN; i++) {
		sscanf(psk + 2*i, "%2x", &req_psk[i]);
		SCM_CLI_LOG("Byte %d: 0x%2x\n", i, req_psk[i]);
	}
	return SCM_CLI_OK;
}

static int scm_cli_sta_fast_connect (int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	scm_wifi_fast_assoc_request fast_request = {0};
	int update_idx = 1;

	/*  argv[1]       argv[2]    argv[3] argv[4]  argv[5]      argv[6]   argv[7]    */
	/* <ssid[32]> <auth> <bssid> <pairwise> <psk[32]> <channel> <hidden ap> */
	if (argc != 8) {
		goto done;
	}

	if ((ret = scm_cli_set_ssid(argv[update_idx++], fast_request.req.ssid)) != SCM_CLI_OK) {
		goto done;
	}

	PARM_SET_INT("auth", fast_request.req.auth, atoi(argv[update_idx++]));


	if (hwaddr_aton(argv[update_idx++], fast_request.req.bssid)) {
		ret = SCM_CLI_FAIL;
		goto done;
	}

	PARM_SET_INT("pairwise", fast_request.req.pairwise, atoi(argv[update_idx++]));

	if ((ret = scm_cli_set_psk(argv[update_idx++], fast_request.psk)) != SCM_CLI_OK) {
		goto done;
	}

	PARM_SET_INT("channel", fast_request.channel , atoi(argv[update_idx++]));
	PARM_SET_INT("hidden_ap", fast_request.req.hidden_ap , atoi(argv[update_idx++]));

	ret = scm_wifi_sta_fast_connect(&fast_request);

done:
	return ret;
}
SCM_CLI(sta_fast_connect, scm_cli_sta_fast_connect,
		"<ssid[32]> <auth> <bssid> <pairwise> <psk[32]> <channel>",
		NULL);

static int scm_cli_sta_get_psk(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	u8 psk[WISE_PMK_LEN];
	int i;

	if (argc > 1) {
		goto done;
	}

	if ((ret = scm_wifi_sta_get_psk(psk, WISE_PMK_LEN)) != SCM_CLI_OK) {
		SCM_CLI_ERR("%s: scm_wifi_sta_get_psk failed\n", __func__);
		goto done;
	}

	for (i = 0; i < WISE_PMK_LEN; i++) {
		SCM_CLI_INFO("%02x", psk[i]);
	}
	SCM_CLI_INFO("\n");

done:
	return ret;
}
SCM_CLI(sta_get_psk, scm_cli_sta_get_psk, "", NULL);
#endif /* CONFIG_CLI_WIFI_STA */

#ifdef CONFIG_CLI_WIFI_SOFTAP

/**
  * CLI command for API: scm_cli_sap_start
  * Start SoftAP network
  * @param: NA
  * @cmd: wifi sap_start
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_start(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	char ifname[WIFI_IFNAME_MAX_SIZE + 1] = {0};
	int len = sizeof(ifname);

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sap_start(ifname, &len);

	if (IS_OK(ret))
		SCM_CLI_INFO("ifname: %s\n", ifname);

done:
	return ret;
}
SCM_CLI(sap_start, scm_cli_sap_start, "", NULL);

/**
  * CLI command for API: scm_cli_sap_stop
  * Stop SoftAP network
  * @param: NA
  * @cmd: wifi sap_stop
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_stop(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sap_stop();

done:
	return ret;
}
SCM_CLI(sap_stop, scm_cli_sap_stop, "", NULL);

/**
  * CLI command for API: scm_cli_sap_cfg
  * SoftAP configurations
  * @param: ssid, key, ch, hidden, auth, pairwise
  * @cmd: wifi sap_cfg <ssid> <key> <ch> <hidden> <auth> <pairwise>
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_cfg(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	scm_wifi_softap_config sap = {0};

	char* ssid;
	char* key;

	if (argc != 7) {
		goto done;
	}

	SCM_CLI_LOG("ssid:%s key:%s ch:%s auth:%d\n", argv[1], argv[2], argv[3], atoi(argv[5]));

	ssid = argv[1];

	memcpy(sap.ssid, ssid, strlen(ssid));

	key = argv[2];

	memcpy(sap.key, key, strlen(key));

	sap.channel_num = atoi(argv[3]);

	sap.ssid_hidden = atoi(argv[4]);

	sap.authmode = atoi(argv[5]);

	sap.pairwise = atoi(argv[6]);

	ret = scm_wifi_sap_set_config(&sap);

	done:
		return ret;
}
SCM_CLI(sap_cfg, scm_cli_sap_cfg,
		"<ssid> <key> <ch> <hidden> <auth> <pairwise>",
		"Ssid 12345678 6 0 2 1"
		"\r\n""auth: 0 OPEN, 2 WPA2PSK"
		"\r\n""pairwise: 1 AES\n");

/**
  * CLI command for API: scm_cli_sap_set_beacon
  * Set SoftAP beacon interval: 25 ~ 1000
  * @param: interval
  * @cmd: wifi sap_beacon [interval]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_set_beacon(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc != 2) {
		goto done;
	}

	ret = scm_wifi_sap_set_beacon_interval((uint32_t)atoi(argv[1]));

done:
	return ret;
}
SCM_CLI(sap_beacon, scm_cli_sap_set_beacon,
		"<interval>",
		"100");

/**
  * CLI command for API: scm_cli_sap_set_dtim
  * Set SoftAP DTIM period: 1 ~ 15
  * @param: period
  * @cmd: wifi sap_dtim [period]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_set_dtim(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc != 2) {
		goto done;
	}

	ret = scm_wifi_sap_set_dtim_period(atoi(argv[1]));

	done:
		return ret;
}
SCM_CLI(sap_dtim, scm_cli_sap_set_dtim, "<period>", "5");

/**
  * CLI command for API: scm_cli_sap_deauth
  * Deauth from specific MAC of STA
  * @param: sta_mac
  * @cmd: wifi sap_deauth [sta_mac]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_deauth(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc != 2 && strlen(argv[1]) != SCM_WIFI_ADDR_STR_LEN) {
		goto done;
	}

	ret = scm_wifi_sap_deauth_sta(argv[1], SCM_WIFI_ADDR_STR_LEN);

done:
	return ret;
}
SCM_CLI(sap_deauth, scm_cli_sap_deauth,
		"<sta_mac>",
		"11:22:33:44:55:66");

/**
  * CLI command for API: scm_cli_sap_show_sta
  * Show connected STA info
  * @param: NA
  * @cmd: wifi sap_showsta
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_show_sap(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	scm_wifi_softap_config sap = {0};

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sap_get_config(&sap);

	if (IS_OK(ret)) {
		SCM_CLI_INFO("SAP SSID: %s\n", sap.ssid);
		SCM_CLI_INFO("SAP KEY: %s\n", sap.key);
		SCM_CLI_INFO("SAP CH: %d\n", sap.channel_num);
		SCM_CLI_INFO("SAP SSID Hidden: %d\n", sap.ssid_hidden);
		SCM_CLI_INFO("SAP Auth Mode: %d\n", sap.authmode);
		SCM_CLI_INFO("SAP Pairwise: %d\n", sap.pairwise);
	}

done:
	return ret;
}
SCM_CLI(sap_show, scm_cli_sap_show_sap, "", NULL);

/**
  * CLI command for API: scm_cli_sap_show_sta
  * Show connected STA info
  * @param: NA
  * @cmd: wifi sap_showsta
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_sap_show_sta(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	uint8_t sta_index;
	uint8_t sta_num = WIFI_DEFAULT_MAX_NUM_STA;
	scm_wifi_ap_sta_info  sta_list[WIFI_DEFAULT_MAX_NUM_STA];
	scm_wifi_ap_sta_info *sta_list_node = NULL;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sap_get_connected_sta(sta_list, &sta_num);

	if (IS_OK(ret)) {
		sta_list_node = sta_list;

		SCM_CLI_INFO("STA num: %d\n", sta_num);

		for (sta_index = 0; sta_index < sta_num; sta_index++, sta_list_node++) {
			SCM_CLI_INFO("STA addr:" MACSTR"\n", MAC2STR(sta_list_node->mac));
			SCM_CLI_INFO("STA rssi: %d\n", sta_list_node->rssi);
			SCM_CLI_INFO("STA rate: 0x%x\n", sta_list_node->rate);
		}
	}

done:
	return ret;
}
SCM_CLI(sap_showsta, scm_cli_sap_show_sta, "", NULL);
#endif /* CONFIG_CLI_WIFI_SOFTAP */

#ifdef CONFIG_CLI_WIFI_SCAN
/**
 * CMD: wifi sta_scan
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 * 0x3002: WISE_ERR_WIFI_NOT_STARTED
 * 0x106: WISE_ERR_WIFI_NOT_SUPPORTED
 * 0x101: WISE_ERR_NO_MEM
 */
static int scm_cli_sta_scan(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_sta_scan();

done:
	return ret;
}
SCM_CLI(sta_scan, scm_cli_sta_scan, "", NULL);

/**
 * CMD: wifi sta_advance_scan <scan_type> <ssid>|<bssid>|<channel>
 * <scan_type>:
 * - [1]Specified channel scan
 * - [2]Specified SSID scan
 * - [3]Prefix SSID scan
 * - [4]Specified BSSID scan
 * <ssid>:  string parameter SSID of the AP.
 * <bssid>:  parameter MAC address of the AP.
 * <channel>: scan channel.
 * Valid[country code related]: 1-14
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 * 0x3002: WISE_ERR_WIFI_NOT_STARTED
 * 0x106: WISE_ERR_WIFI_NOT_SUPPORTED
 * 0x101: WISE_ERR_NO_MEM
 */
static int scm_cli_sta_advance_scan(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	char *pos;
	scm_wifi_scan_params sp;

	if (argc != 3) {
		goto done;
	}

	memset(&sp, 0, sizeof(sp));
	sp.scan_type = atoi(argv[1]);
	if ((sp.scan_type < SCM_WIFI_CHANNEL_SCAN) ||
			(sp.scan_type > SCM_WIFI_BSSID_SCAN)) {
		goto done;
	}

	switch (sp.scan_type) {
		case SCM_WIFI_CHANNEL_SCAN:
			sp.channel = atoi(argv[2]);
			break;
		case SCM_WIFI_SSID_SCAN:
			/* fall-through */
		case SCM_WIFI_SSID_PREFIX_SCAN:
			sp.ssid_len = strlen(argv[2]);

			if (sp.ssid_len > SCM_WIFI_MAX_SSID_LEN) {
				goto done;
			}

			memcpy(sp.ssid, argv[2], sp.ssid_len);
			sp.ssid[sp.ssid_len] = 0;
			break;
		case SCM_WIFI_BSSID_SCAN:
			pos = argv[2];
			if (hwaddr_aton(pos, sp.bssid)) {
				goto done;
			}
			break;
		default:
			goto done;
	}

	ret = scm_wifi_sta_advance_scan(&sp);

done:
	return ret;
}
SCM_CLI(sta_advance_scan, scm_cli_sta_advance_scan, "<scan_type> <channel>|<ssid>|<bssid>",
"2 Ssid"
"\r\n""scan_type 1: Channel scan"
"\r\n""scan_type 2: SSID scan"
"\r\n""scan_type 3: SSID prefix scan"
"\r\n""scan_type 4: BSSID scan\n");

const char *scm_cli_auth_mode_strings[] = {
	"OPEN",
	"WPA_PSK",
	"WPA2_PSK",
	"SAE",
	"UNKNOWN"
};

/**
 * CMD: wifi sta_scan_results <max_ap_num>
 *  max_ap_num[Valid: 1-32]: max listed AP num
 * Response:
 * Print the err code
 * 0: OK
 * -1: Fail
 * Print the scanned APs:
 * If there is no scanned results, show empty.
 * SSID: %s, BSSID: %x:%x:%x:%x:%x:%x, CH: %d, AUTH: %s, RSSI: %d
 */
static int scm_cli_sta_scan_results(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	uint16_t num;
	uint16_t max_ap_num;
	scm_wifi_ap_info *pst_results = NULL;

	if (argc != 2) {
		goto done;
	}

	max_ap_num = atoi(argv[1]);

	pst_results = malloc(sizeof(scm_wifi_ap_info) * max_ap_num);
	if (pst_results == NULL) {
		goto done;
	}

	ret = scm_wifi_sta_scan_results(pst_results, &num, max_ap_num);

	if (IS_OK(ret)) {
		for (uint16_t loop = 0; (loop < num) && (loop < max_ap_num); loop++) {
			SCM_CLI_INFO("SSID: %-20s, CH: %-2d, AUTH: %s\n", pst_results[loop].ssid, pst_results[loop].channel,
				scm_cli_auth_mode_strings[pst_results[loop].auth]);
		}
	}

done:
	if (pst_results) {
		free(pst_results);
	}

	return ret;
}
SCM_CLI(sta_scan_results, scm_cli_sta_scan_results, "<max_ap_num>", "5");
#endif /* CONFIG_CLI_WIFI_SCAN */

#ifdef CONFIG_CLI_WIFI_SCMCHANNEL

static const struct wifi_ipv4_filter ipv4_filter_def_setting[] = {
	/* DHCP */
	{
		0,							 /* remote ip */
		68,							 /* local port */
		0,							 /* localp_min */
		0,							 /* localp_max */
		0,							 /* remote_port */
		0,							 /* remotep_min */
		0,							 /* remotep_max */
		17,							 /* packet type */
		WIFI_FILTER_TO_LWIP,					 /* config_type */
		WIFI_FILTER_MASK_LOCAL_PORT | WIFI_FILTER_MASK_PROTOCOL, /* match_mask */
	},
	/* DHCP */
	{
		0,							 /* remote ip */
		67,							 /* local port */
		0,							 /* localp_min */
		0,							 /* localp_max */
		0,							 /* remote_port */
		0,							 /* remotep_min */
		0,							 /* remotep_max */
		17,							 /* packet type */
		WIFI_FILTER_TO_LWIP,					 /* config_type */
		WIFI_FILTER_MASK_LOCAL_PORT | WIFI_FILTER_MASK_PROTOCOL, /* match_mask */
	},
	/* TCP */
	{
		0,							 /* remote ip */
		6001,							 /* local port */
		0,							 /* localp_min */
		0,							 /* localp_max */
		0,							 /* remote_port */
		0,							 /* remotep_min */
		0,							 /* remotep_max */
		6,							 /* packet type */
		WIFI_FILTER_TO_LWIP,					 /* config_type */
		WIFI_FILTER_MASK_LOCAL_PORT | WIFI_FILTER_MASK_PROTOCOL, /* match_mask */
	},
	/* TCP */
	{
		0,							 /* remote ip */
		6002,							 /* local port */
		0,							 /* localp_min */
		0,							 /* localp_max */
		0,							 /* remote_port */
		0,							 /* remotep_min */
		0,							 /* remotep_max */
		6,							 /* packet type */
		WIFI_FILTER_TO_LWIP,					 /* config_type */
		WIFI_FILTER_MASK_REMOTE_PORT | WIFI_FILTER_MASK_PROTOCOL, /* match_mask */
	},
	/* UDP */
	{
		0,							 /* remote ip */
		7001,							 /* local port */
		0,							 /* localp_min */
		0,							 /* localp_max */
		0,							 /* remote_port */
		0,							 /* remotep_min */
		0,							 /* remotep_max */
		17,							 /* packet type */
		WIFI_FILTER_TO_LWIP,					 /* config_type */
		WIFI_FILTER_MASK_LOCAL_PORT | WIFI_FILTER_MASK_PROTOCOL, /* match_mask */
	},
	/* UDP */
	{
		0,							 /* remote ip */
		7002,							 /* local port */
		0,							 /* localp_min */
		0,							 /* localp_max */
		0,							 /* remote_port */
		0,							 /* remotep_min */
		0,							 /* remotep_max */
		17,							 /* packet type */
		WIFI_FILTER_TO_LWIP,					 /* config_type */
		WIFI_FILTER_MASK_REMOTE_PORT | WIFI_FILTER_MASK_PROTOCOL, /* match_mask */
	},
};

static int scm_channel_set_default_filter(void)
{
	int ret = SCM_CLI_FAIL;
	int index;

	if (ARRAY_SIZE(ipv4_filter_def_setting) > MAX_WIFI_FILTER_IPV4_CNT) {
		SCM_CLI_ERR("fail: ipv4 filter table sz over %d \n", MAX_WIFI_FILTER_IPV4_CNT);
		goto done;
	}

	if (scm_wifi_set_default_filter(WIFI_FILTER_TO_HOST))
		goto done;

	SCM_CLI_INFO("set all net packets foward to camera default.\n");

	for (index = 0; index < ARRAY_SIZE(ipv4_filter_def_setting); index++) {
		if (scm_wifi_add_filter((char *) &ipv4_filter_def_setting[index], WIFI_FILTER_TYPE_IPV4)) {
			SCM_CLI_ERR("add filter failed: duplicate element\n");
		}
	}

	ret = SCM_CLI_OK;

	done:
		return ret;
}

static int scm_channel_rx_callback (char *buf, int len)
{
	char msg[512];
	int msg_len = 0;
	int index;

	if ((buf == NULL) || (len == 0)) {
		return SCM_CLI_FAIL;
	}

	for (index = HOST_CMD_GET_MAC; index < HOST_CMD_TBTT; index ++) {
		if (memcmp(buf, cmd[index], strlen(cmd[index])) == 0) {
			break;
		}
	}

	if (index == HOST_CMD_GET_MAC) {
		uint8_t *mac_addr = NULL;

		scm_wifi_get_wlan_mac(&mac_addr, WISE_IF_WIFI_STA);
		if (mac_addr == NULL) {
			return SCM_CLI_FAIL;
		}
		msg[0] = HOST_CMD_GET_MAC;
		msg_len++;
		memcpy(&msg[1], (char *)mac_addr, MAC_ADDR_LEN);
		msg_len += MAC_ADDR_LEN;
		scm_channel_send_to_host(msg, msg_len);
		SCM_SET_HOST_AP_READY(1);

	} else if (index == HOST_CMD_GET_IP) {
		msg[0] = HOST_CMD_GET_IP;
		msg_len++;
		if (scm_channel_construct_ip_msg(WIFI_STA_NETIF_NAME, &msg[1], &msg_len)) {
			return SCM_CLI_FAIL;
		}
		scm_channel_send_to_host(msg, msg_len);
	} else if (index == HOST_CMD_SET_FILTER) {
		scm_channel_set_default_filter();
	}

	return SCM_CLI_OK;
}

static int scm_cli_scm_ch_init(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	char *err = NULL;

	if (argc > 1) {
		goto done;
	}

	if ((ret = scm_vlwip_netif_reset(WIFI_FILTER_TYPE_IPV4)) != SCM_CLI_OK) {
		err = "netif reset failed";
		goto done;
	}

	if ((ret = scm_channel_set_default_filter()) != SCM_CLI_OK) {
		err = "set_default_filter failed";
		goto done;
	}

	if ((ret = scm_channel_register_rx_cb(scm_channel_rx_callback)) != SCM_CLI_OK) {
		err = "scm_channel_register_rx_cb failed";
		goto done;
	}

done:
	if (err)
		SCM_CLI_ERR("%s: %s\n", __func__, err);
	return ret;
}
SCM_CLI(scm_ch_init , scm_cli_scm_ch_init, "", NULL);

static int scm_cli_scm_vlwip_reset (int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	wifi_filter_type type;

	if (argc != 2) {
		goto done;
	}

	type = atoi(argv[1]);

	if ((ret = scm_vlwip_netif_reset(type)) != SCM_CLI_OK) {
		SCM_CLI_ERR("%s: netif reset failed\n", __func__);
		goto done;
	}

done:
	return ret;
}
SCM_CLI(scm_vlwip_reset, scm_cli_scm_vlwip_reset, "<type>", NULL);

static int scm_cli_scm_ch_reg_cb (int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	if ((ret = scm_channel_register_rx_cb(scm_channel_rx_callback)) != SCM_CLI_OK) {
		SCM_CLI_ERR("%s: scm_channel_register_rx_cb failed\n", __func__);
		goto done;
	}

done:
	return ret;
}
SCM_CLI(scm_ch_reg_cb, scm_cli_scm_ch_reg_cb, "", NULL);


static char *scm_cli_gen_str(int len)
{
	char *buf = NULL;
	int i = 0;

	buf = malloc(len);
	if (!buf) {
		SCM_CLI_ERR("scm_cli_gen_str malloc len:%d failed\n", len);
		return NULL;
	}

	for (i = 0; i < len; i++)
		buf[i] = '0' + (i % 10);

	buf[len] = '\0';
	SCM_CLI_LOG("%s \n", buf);
	return buf;
}

static int scm_cli_scm_ch_send (int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	char *buf = NULL, *str = NULL;
	int length = 0;

	if (argc > 3) {
		goto done;
	}

	if (argv[1])
		buf = argv[1];

	if (argv[2])
		length = atoi(argv[2]);

	if (length > 100) {
		str = scm_cli_gen_str(length);
		buf = str;
	}

	if ((ret = scm_channel_send_to_host(buf, length)) != SCM_CLI_OK) {
		SCM_CLI_ERR("%s: scm_channel_send_to_host failed\n", __func__);
		goto done;
	}

done:
	if (str)
		free(str);

	return ret;
}
SCM_CLI(scm_ch_send, scm_cli_scm_ch_send, "<msg> <length>", NULL);

/* WiFi filter feature(CONFIG_SUPPORT_WIFI_REPEATER) should be defined */
/* Or def config should be included sdio feature */
/**
  * CLI command for API: scm_wifi_query_filter
  * Query all of wifi filter
  * @param: NA
  * @cmd: wifi filter_query
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_filter_query(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	struct wifi_ipv4_filter *wlan_filter;
	int filter_index;
	int num;

	if (argc > 1) {
		goto done;
	}

	ret = scm_wifi_query_filter((char **) &wlan_filter, &num, WIFI_FILTER_TYPE_IPV4);

	if (IS_OK(ret)) {
		for (filter_index = 0; filter_index < num; filter_index++) {
			if (wlan_filter->match_mask & WIFI_FILTER_MASK_LOCAL_PORT_RANGE) {
				SCM_CLI_ERR("[%d] protocol(%d) local port(%d ~ %d) remote port(%d) config_type(%d) match_mask(0x%x)\n",
				filter_index, wlan_filter->packet_type,
				wlan_filter->localp_min, wlan_filter->localp_max, wlan_filter->remote_port,
				wlan_filter->config_type,	wlan_filter->match_mask);
			}
			else if (wlan_filter->match_mask & WIFI_FILTER_MASK_REMOTE_PORT_RANGE) {
				SCM_CLI_ERR("[%d] protocol(%d) local port(%d) remote port(%d ~ %d) config_type(%d) match_mask(0x%x)\n",
				filter_index, wlan_filter->packet_type,
				wlan_filter->local_port, wlan_filter->remotep_min, wlan_filter->remotep_max,
				wlan_filter->config_type,  wlan_filter->match_mask);
			}
			else {
				SCM_CLI_ERR("[%d] protocol(%d) local port(%d) remote port(%d) config_type(%d) match_mask(0x%x)\n",
				filter_index, wlan_filter->packet_type,
				wlan_filter->local_port, wlan_filter->remote_port,
				wlan_filter->config_type,  wlan_filter->match_mask);
			}
			wlan_filter++;
		}
	}

done:
	return ret;
}
SCM_CLI(filter_query, scm_cli_filter_query, "", NULL);

/**
  * CLI command for API: scm_wifi_del_filter
  * Delete wifi filter
  * @param: struct wifi_ipv4_filter filter
  * @cmd: wifi filter_del [protocol] [min local port] [max local port] [min remote port] [max remote port] [config_type]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_filter_del(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	struct wifi_ipv4_filter filter = { 0 };

	if (argc != 7) {
		goto done;
	}

	filter.packet_type = atoi(argv[1]);
	filter.localp_min = atoi(argv[2]);
	filter.localp_max = atoi(argv[3]);
	if (filter.localp_min == filter.localp_max) {
		filter.local_port = filter.localp_min;
		filter.localp_min = 0;
		filter.localp_max = 0;
	}
	filter.remotep_min = atoi(argv[4]);
	filter.remotep_max = atoi(argv[5]);
	if (filter.remotep_min == filter.remotep_max) {
		filter.remote_port = filter.remotep_max;
		filter.remotep_min = 0;
		filter.remotep_max = 0;
	}
	filter.config_type = atoi(argv[6]);

	/* Match_mask assignment */
	if (filter.packet_type)
		filter.match_mask |= WIFI_FILTER_MASK_PROTOCOL;
	if (filter.local_port)
		filter.match_mask |= WIFI_FILTER_MASK_LOCAL_PORT;
	if (filter.remote_port)
		filter.match_mask |= WIFI_FILTER_MASK_REMOTE_PORT;
	if (filter.localp_min)
		filter.match_mask |= WIFI_FILTER_MASK_LOCAL_PORT_RANGE;
	if (filter.remotep_min)
		filter.match_mask |= WIFI_FILTER_MASK_REMOTE_PORT_RANGE;

	ret = scm_wifi_del_filter((char *) &filter, WIFI_FILTER_TYPE_IPV4);

done:
	return ret;
}
SCM_CLI(filter_del, scm_cli_filter_del,
		"<protocol> <min local port> <max local port> <min remote port> <max remote port> <config_type>",
		NULL);

/**
  * CLI command for API: scm_wifi_add_filter
  * Add wifi filter
  * @param: struct wifi_ipv4_filter filter
  * @cmd: wifi filter_add [protocol] [min local port] [max local port] [min remote port] [max remote port] [config_type]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_filter_add(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	struct wifi_ipv4_filter filter = { 0 };

	if (argc != 7) {
		goto done;
	}

	filter.packet_type = atoi(argv[1]);
	filter.localp_min = atoi(argv[2]);
	filter.localp_max = atoi(argv[3]);
	if (filter.localp_min == filter.localp_max) {
		filter.local_port = filter.localp_min;
		filter.localp_min = 0;
		filter.localp_max = 0;
	}
	filter.remotep_min = atoi(argv[4]);
	filter.remotep_max = atoi(argv[5]);
	if (filter.remotep_min == filter.remotep_max) {
		filter.remote_port = filter.remotep_max;
		filter.remotep_min = 0;
		filter.remotep_max = 0;
	}
	filter.config_type = atoi(argv[6]);

	/* Match_mask assignment */
	if (filter.packet_type)
		filter.match_mask |= WIFI_FILTER_MASK_PROTOCOL;
	if (filter.local_port)
		filter.match_mask |= WIFI_FILTER_MASK_LOCAL_PORT;
	if (filter.remote_port)
		filter.match_mask |= WIFI_FILTER_MASK_REMOTE_PORT;
	if (filter.localp_min)
		filter.match_mask |= WIFI_FILTER_MASK_LOCAL_PORT_RANGE;
	if (filter.remotep_min)
		filter.match_mask |= WIFI_FILTER_MASK_REMOTE_PORT_RANGE;

	ret = scm_wifi_add_filter((char *) &filter, WIFI_FILTER_TYPE_IPV4);

done:
	return ret;
}
SCM_CLI(filter_add, scm_cli_filter_add,
		"<protocol> <min local port> <max local port> <min remote port> <max remote port> <config_type>",
		NULL);

/**
  * CLI command for API: scm_wifi_set_default_filter
  * Set wifi filter default direction
  * @param: direction
  * @cmd: wifi filter_dir [direction]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_filter_dir(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;
	int direction;

	if (argc != 2) {
		goto done;
	}

	direction = atoi(argv[1]);

	ret = scm_wifi_set_default_filter(direction);

done:
	return ret;
}
SCM_CLI(filter_dir, scm_cli_filter_dir, "<direction>", NULL);
#endif /* CONFIG_CLI_WIFI_SCMCHANNEL */

#ifdef CONFIG_CLI_WIFI_DHCP
/**
  * CLI command for API: scm_cli_ip_set
  * Set network Address of interface
  * @param:
  *    ifname: name of interface
  *    ip: IP address
  *    nm: network mask
  *    gw: gateway
  * @cmd: wifi ip_set <ip> [nm] [gw]
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_ip_set(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc < 3 && argc > 5) {
		goto done;
	}

	ret = scm_wifi_set_ip(argv[1], argv[2], argv[3], argv[4]);

done:
	return ret;
}
SCM_CLI(ip_set, scm_cli_ip_set,
		"<wlan0/wlan1> <ip> [nm] [gw]",
		"wlan0 192.168.200.2 255.255.255.0 192.168.200.1");

/**
  * CLI command for API: scm_cli_ip_reset
  * Reset network Address of interface
  * @param:
  *    ifname: name of interface
  * @cmd: wifi ip_reset
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_ip_reset(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc != 2 || (strcmp(argv[1], "wlan0") && strcmp(argv[1], "wlan1"))) {
		goto done;
	}

	ret = scm_wifi_reset_ip(argv[1]);

done:
	return ret;
}
SCM_CLI(ip_reset, scm_cli_ip_reset, "<wlan0/wlan1>", NULL);

/**
  * CLI command for API: netifapi_dhcps_stop
  * Stop the DHCP Server
  * @param: NA
  * @cmd: wifi dhcps_stop
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_dhcps_stop(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = netifapi_dhcps_stop(scm_wifi_get_netif(WISE_IF_WIFI_AP));
	if (!IS_OK(ret)) {
		/* re-direct return value */
		ret = SCM_CLI_FAIL;
	}

done:
	return ret;
}
SCM_CLI(dhcps_stop, scm_cli_dhcps_stop, "", NULL);

/**
  * CLI command for API: netifapi_dhcps_start
  * Start the DHCP Server
  * @param: NA
  * @cmd: wifi dhcps_start
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_dhcps_start(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = netifapi_dhcps_start(scm_wifi_get_netif(WISE_IF_WIFI_AP));
	if (!IS_OK(ret)) {
		/* re-direct return value */
		ret = SCM_CLI_FAIL;
	}

done:
	return ret;
}
SCM_CLI(dhcps_start, scm_cli_dhcps_start, "", NULL);

/**
  * CLI command for API: netifapi_dhcp_stop
  * Stop the DHCP Client
  * @param: NA
  * @cmd: wifi dhcp_stop
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_dhcp_stop(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = netifapi_dhcp_stop(scm_wifi_get_netif(WISE_IF_WIFI_STA));
	if (!IS_OK(ret))  {
		/* re-direct return value */
		ret = SCM_CLI_FAIL;
	}

done:
	return ret;
}
SCM_CLI(dhcp_stop, scm_cli_dhcp_stop, "", NULL);


/**
  * CLI command for API: netifapi_dhcp_start
  * Start the DHCP Client
  * @param: NA
  * @cmd: wifi dhcp_start
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_dhcp_start(int argc, char *argv[])
{
	int ret = SCM_CLI_FAIL;

	if (argc > 1) {
		goto done;
	}

	ret = netifapi_dhcp_start(scm_wifi_get_netif(WISE_IF_WIFI_STA));
	if (!IS_OK(ret)) {
		/* re-direct return value */
		ret = SCM_CLI_FAIL;
	}

done:
	return ret;
}
SCM_CLI(dhcp_start, scm_cli_dhcp_start, "", NULL);
#endif

/**
  * Help command for CLI
  * Print CLI usage help
  * @param: NA
  * @cmd: help
  * @return
  *    - SCM_CLI_OK: succeed
  *    - SCM_CLI_FAIL: fail
  */
static int scm_cli_help(int argc, char *argv[])
{
		char * help =
#ifdef CONFIG_CLI_WIFI_STA
		"wifi sta_cfg <ssid> <auth> <key> <bssid> <pairwise> <hidden ap>" OR
		"wifi sta_connect" OR
		"wifi sta_disconnect" OR
		"wifi sta_get_connect" OR
		"wifi sta_set_reconnect" OR
		"wifi sta_fast_connect <ssid> <auth> <bssid> <pairwise> <psk> <channel> " OR
		"wifi sta_start" OR
		"wifi sta_get_psk" OR
#endif
#ifdef CONFIG_CLI_WIFI_SCAN
		"wifi sta_scan" OR
		"wifi sta_advance_scan <scan_type> <channel>|<ssid>|<bssid>" OR
		"wifi sta_scan_results <max_ap_num>" OR
#endif
#ifdef CONFIG_CLI_WIFI_SOFTAP
		"wifi sap_start" OR
		"wifi sap_stop" OR
		"wifi sap_cfg <ssid> <key> <ch> <hidden> <auth> <pairwise>" OR
		"wifi sap_beacon <interval>" OR
		"wifi sap_dtim <period>" OR
		"wifi sap_deauth <sta_mac>" OR
		"wifi sap_show" OR
		"wifi sap_showsta" OR
#endif
#ifdef CONFIG_CLI_WIFI_SCMCHANNEL
		"wifi scm_ch_init" OR
		"wifi scm_vlwip_reset <type>" OR
		"wifi scm_ch_reg_cb" OR
		"wifi scm_ch_send <msg> <len>" OR
#endif
#ifdef CONFIG_CLI_WIFI_DHCP
		"wifi ip_set <ifn> <ip> [nm] [gw]" OR
		"wifi dhcp_start/dhcp_stop" OR
		"wifi dhcps_start/dhcps_stop" OR
#endif
#ifdef CONFIG_CLI_WIFI_FILTER
		"wifi filter_query" OR
		"wifi filter_del <proto> <local port min> <local port max> <remote port min> <remote port max> <config_type>" OR
		"wifi filter_add <proto> <local port min> <local port max> <remote port min> <remote port max> <config_type>" OR
		"wifi filter_dir <direction>" OR
#endif
		"wifi reg_evt_cb"
		"\r\n";

	SCM_CLI_INFO("%s\n", help);

	return SCM_CLI_OK;
}
SCM_CLI(help, scm_cli_help, "", NULL);


const struct scm_cli *scm_cli_find_cmd(char *cmd, const struct scm_cli *table, int nr)
{
	const struct scm_cli *t;

	for (t = table; t < table + nr; t++) {
		if (strcmp(cmd, t->name) == 0 &&
			strlen(t->name) == strlen(cmd))
			return t;
	}
	return NULL;
}

static int do_scm_cli_wifi(int argc, char *argv[])
{
	int ret;
	const struct scm_cli *start, *end, *cmd;

	start = scm_cli_cmd_start();
	end = scm_cli_cmd_end();

	argc--;
	argv++;

	cmd = scm_cli_find_cmd(argv[0], start, end - start);
	if (cmd == NULL) {
		SCM_CLI_ERR("fail to find CLI command\n");
		return SCM_CLI_FAIL;
	}

	ret = cmd->ops(argc, argv);

	SCM_CLI_INFO("%s %s (%d)\n", cmd->name, IS_OK(ret) ? "OK" : "FAIL", ret);

	if (!IS_OK(ret)) {
		SCM_CLI_INFO("Usage: %s %s %s\n", "wifi", cmd->name, cmd->usage);

		if (cmd->desc != NULL)
			SCM_CLI_INFO("ex: %s %s %s\n", "wifi", cmd->name, cmd->desc);
	}

	return SCM_CLI_OK;
}

CMD(wifi, do_scm_cli_wifi,
	"CLI for wifi API test",
	"Run \"wifi help\" to get detail"
);

#endif
