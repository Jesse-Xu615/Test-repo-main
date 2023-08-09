/*
 * Copyright 2018-2019 Senscomm Semiconductor Co., Ltd.
 */
// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Inspired by ESP8266_RTOS_SDK
 * (https://github.com/espressif/ESP8266_RTOS_SDK)
 * and will provide wise Wi-Fi API as being ESP8266 style
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>

#include "compat_param.h"
#include "compat_if.h"
#include "if_dl.h"
#include "if_media.h"
#include "ethernet.h"
#include "route.h"
#include "libifconfig.h"

#include "lwip/netif.h"
#include "lwip/netifapi.h"

#include "common.h"

#include "wise_err.h"
#include "wise_log.h"
#include "wise_wifi.h"
#include "wise_wpas.h"

#include "net80211/ieee80211_ioctl.h"

#include "bss.h"
#include "rsn_supp/wpa.h"

#define FREE_MEM	1
#define RESET_MEM	2

struct wifi_config{
    char *name;
    char *value;
};

static struct wifi_config open_config [] = {
	{"ssid", NULL},
	{"key_mgmt", "NONE"},
	{"pairwise", "NONE"},
	{"scan_ssid", NULL},
};

static struct wifi_config wep_config [] = {
	{"ssid", NULL},
	{"key_mgmt", "NONE"},
	{"auth_alg", "SHARED"},
	{"wep_tx_keyidx", "0"},
	{"wep_key0", NULL},
	{"scan_ssid", NULL},
};

static struct wifi_config tkip_config [] = {
	{"ssid", NULL},
	{"key_mgmt", "WPA-PSK"},
	{"pairwise", "TKIP"},
	{"psk", NULL},
	{"proto", NULL},
	{"scan_ssid", NULL},
};

static struct wifi_config ccmp_config [] = {
	{"ssid", NULL},
	{"key_mgmt", "WPA-PSK"},
	{"pairwise", "CCMP"},
	{"psk", NULL},
	{"proto", NULL},
	{"scan_ssid", NULL},
#ifdef CONFIG_IEEE80211W
	{"ieee80211w" , NULL},
#endif
};

#ifdef CONFIG_SAE
static struct wifi_config sae_config [] = {
	{"ssid", NULL},
	{"key_mgmt", "SAE"},
	{"proto", "RSN"},
	{"sae_password", NULL},
	{"ieee80211w" , NULL},
	{"scan_ssid", NULL},
};
#endif

static struct wifi_config *sta_config_tables[] = {
	open_config,
	wep_config,
	tkip_config,
	ccmp_config,
	NULL,
	NULL,
#ifdef CONFIG_SAE
	sae_config,
#endif /* CONFIG_SAE */
};

#ifndef ARRAY_SIZE
#define ARRAY_SIZE( x ) ( sizeof( x ) / sizeof( x[ 0 ] ) )
#endif

static const int sta_config_tables_sz [] = {
	ARRAY_SIZE(open_config),
	ARRAY_SIZE(wep_config),
	ARRAY_SIZE(tkip_config),
	ARRAY_SIZE(ccmp_config),
	0,
	0,
#ifdef CONFIG_SAE
	ARRAY_SIZE(sae_config),
#endif /* CONFIG_SAE */
};

static struct wifi_config *sap_config_tables[] = {
	open_config,
	NULL,
	tkip_config,
	ccmp_config,
	NULL,
	NULL,
	NULL
};

static const int sap_config_tables_sz [] = {
	ARRAY_SIZE(open_config),
	0,
	ARRAY_SIZE(tkip_config),
	ARRAY_SIZE(ccmp_config),
	0,
	0,
	0
};

/* FIXME: use appropriate error code instead of WISE_FAIL */
wifi_mode_t s_wifi_mode[2] = {WIFI_MODE_NULL, WIFI_MODE_NULL};

#ifdef CONFIG_SUPPORT_DUAL_VIF
#define ADD_NETWORK(ret, ifc)					\
do {								\
	char *argv[3] = {"-i", NULL, "ADD_NETWORK"};			\
	argv[1] = #ifc; \
	ret = wise_wpas_cli(3, argv);				\
} while (0)

#define REMOVE_NETWORK(ifc, inum, ret)				\
do {								\
	char *argv[4] = {"-i", NULL, "REMOVE_NETWORK", NULL};		\
	argv[1] = #ifc;						\
	argv[3] = #inum;					\
	ret = wise_wpas_cli(4, argv);				\
} while (0)

#define ENABLE_NETWORK(ifc, inum, ret)				\
do {								\
	char *argv[4] = {"-i", NULL, "ENABLE_NETWORK", NULL};		\
	argv[1] = #ifc;						\
	argv[3] = #inum;					\
	ret = wise_wpas_cli(4, argv);				\
} while (0)

#define DISABLE_NETWORK(ifc, inum, ret)				\
do {								\
	char *argv[4] = {"-i", NULL, "DISABLE_NETWORK", NULL};		\
	argv[1] = #ifc;						\
	argv[3] = #inum;					\
	ret = wise_wpas_cli(4, argv);				\
} while (0)

#define SET_NETWORK(ifc, inum, name, value, ret)			\
	do {								\
		char *argv[6] = {"-i", NULL, "SET_NETWORK", NULL, NULL, NULL};	\
		argv[1] = (ifc) == 0 ? "wlan0" : "wlan1";	\
		argv[3] = #inum;					\
		argv[4] = name; 					\
		argv[5] = value;					\
		ret = wise_wpas_cli(6, argv);				\
	} while (0)

#define DISCONNECT(ifc, ret)				\
	do {								\
		char *argv[3] = {"-i", NULL, "DISCONNECT"};	\
		argv[1] = #ifc;						\
		ret = wise_wpas_cli(3, argv); 			\
	} while (0)

#define RECONNECT(ifc, ret)				\
	do {								\
		char *argv[3] = {"-i", NULL, "RECONNECT"};	\
		argv[1] = #ifc; 					\
		ret = wise_wpas_cli(3, argv); 			\
	} while (0)

#define DEAUTHENTICATE(ifc, value, ret)			\
		do {								\
			char *argv[4] = {"-i", NULL, "DEAUTHENTICATE", NULL};	\
			argv[1] = #ifc;						\
			argv[3] = value;					\
			ret = wise_wpas_cli(4, argv); 			\
		} while (0)
#else
#define ADD_NETWORK(ret, ifc)					\
do {								\
	char *argv[1] = {"ADD_NETWORK"};			\
	ret = wise_wpas_cli(1, argv);				\
} while (0)

#define REMOVE_NETWORK(ifc, inum, ret)				\
do {								\
	char *argv[2] = {"REMOVE_NETWORK", NULL};		\
	argv[1] = #inum;					\
	ret = wise_wpas_cli(2, argv);				\
} while (0)

#define ENABLE_NETWORK(ifc, inum, ret)				\
do {								\
	char *argv[2] = {"ENABLE_NETWORK", NULL};		\
	argv[1] = #inum;					\
	ret = wise_wpas_cli(2, argv);				\
} while (0)

#define DISABLE_NETWORK(ifc, inum, ret)				\
do {								\
	char *argv[2] = {"DISABLE_NETWORK", NULL};		\
	argv[1] = #inum;					\
	ret = wise_wpas_cli(2, argv);				\
} while (0)

#define SET_NETWORK(ifc, inum, name, value, ret)			\
	do {								\
		char *argv[4] = {"SET_NETWORK", NULL, NULL, NULL};	\
		argv[1] = #inum;					\
		argv[2] = name; 					\
		argv[3] = value;					\
		ret = wise_wpas_cli(4, argv);				\
	} while (0)

#define DISCONNECT(ifc, ret)				\
	do {								\
		char *argv[2] = {"DISCONNECT"}; 	\
		ret = wise_wpas_cli(1, argv);				\
	} while (0)

#define RECONNECT(ifc, ret)				\
	do {								\
		char *argv[2] = {"RECONNECT"}; 	\
		ret = wise_wpas_cli(1, argv); 			\
	} while (0)

#define DEAUTHENTICATE(ifc, value, ret)			\
		do {								\
			char *argv[2] = {"DEAUTHENTICATE", NULL};	\
			argv[1] = #value;					\
			ret = wise_wpas_cli(2, argv); 			\
		} while (0)
#endif


#define add_network(ifc)			({ int __ret; ADD_NETWORK(__ret, ifc); __ret; })
#define remove_network(ifc, inum)		({ int __ret; REMOVE_NETWORK(ifc, inum, __ret); __ret; })
#define set_network(ifc, inum, name, value) 	({ int __ret; SET_NETWORK(ifc, inum, name, value, __ret); __ret; })
#define enable_network(ifc, inum)		({ int __ret; ENABLE_NETWORK(ifc, inum, __ret); __ret; })
#define disable_network(ifc, inum)		({ int __ret; DISABLE_NETWORK(ifc, inum, __ret); __ret; })

#define disconnect(ifc)		({ int __ret; DISCONNECT(ifc, __ret); __ret; })
#define reconnect(ifc)		({ int __ret; RECONNECT(ifc, __ret); __ret; })

#define deauthenticate(ifc, value) 	({ int __ret; DEAUTHENTICATE(ifc, value, __ret); __ret; })

#define inet_addr_from_ip4addr(sockaddr, ipaddr)		\
	((sockaddr)->s_addr = ip4_addr_get_u32(ipaddr))

NETIF_DECLARE_EXT_CALLBACK(wise_netif_ext_cb)

static const char* TAG __maybe_unused = "wise_wifi";

#define err_exit(err, code)	do {			\
	err = code;					\
	WISE_LOGE(TAG, "[%s, %d] Error occurred!",	\
			__func__, __LINE__);		\
	goto exit;					\
} while (0)

osSemaphoreId_t wifi_scan_lock;

static int form_argv(char **pbuf, const char *fmt, ...)
{
	int ret;
	va_list va;
	int buflen;
	char dest[100];

	va_start(va, fmt);
	buflen = vsprintf(dest, fmt, va) + 1;
	*pbuf = zalloc(buflen);
	if (*pbuf != NULL)
		ret = vsprintf(*pbuf, fmt, va);
	else
		ret = WISE_ERR_NO_MEM;
	va_end(va);

	return ret;
}

static void
wise_netif_ext_callback(struct netif* netif, netif_nsc_reason_t reason,
		const netif_ext_callback_args_t* args)
{
	if (reason & LWIP_NSC_IPV4_SET) {
		system_event_t evt;
		ip4_addr_t old_ip;

		memset(&evt, 0, sizeof(evt));
		evt.event_id = SYSTEM_EVENT_STA_GOT_IP;
		evt.event_info.got_ip.ip_changed = false;
		ip4_addr_copy(evt.event_info.got_ip.ip_info.ip, *netif_ip4_addr(netif));
		ip4_addr_copy(evt.event_info.got_ip.ip_info.netmask, *netif_ip4_netmask(netif));
		ip4_addr_copy(evt.event_info.got_ip.ip_info.gw, *netif_ip4_gw(netif));
		if (reason & LWIP_NSC_IPV4_SETTINGS_CHANGED) {
			evt.event_info.got_ip.ip_changed = true;
			ip4_addr_copy(old_ip, *args->ipv4_changed.old_address);
			if (!ip4_addr_isany_val(old_ip)
					&& ip4_addr_isany_val(*netif_ip4_addr(netif))) {
				/* !ANY -> ANY */
				evt.event_id = SYSTEM_EVENT_STA_LOST_IP;
				wise_event_send(&evt);
				return;
			}
		}
		if (!ip4_addr_isany_val(*netif_ip4_addr(netif)))
			wise_event_send(&evt);
	}
}

wise_err_t wise_wifi_deinit(void)
{
	wise_err_t err = WISE_OK;

	if (wifi_scan_lock)
		osSemaphoreDelete(wifi_scan_lock);

	wifi_scan_lock = NULL;
	LOCK_TCPIP_CORE();
	netif_remove_ext_callback(&wise_netif_ext_cb);
	UNLOCK_TCPIP_CORE();

	wise_wpas_kill();

	return err;
}

/**
  * @brief  Init WiFi
  *         Alloc resource for WiFi driver, such as WiFi control structure, RX/TX buffer,
  *         WiFi NVS structure etc, this WiFi also start WiFi task
  *
  * @attention 1. This API must be called before all other WiFi API can be called
  * @attention 2. Always use WIFI_INIT_CONFIG_DEFAULT macro to init the config to default values, this can
  *               guarantee all the fields got correct value when more fields are added into wifi_init_config_t
  *               in future release. If you want to set your owner initial values, overwrite the default values
  *               which are set by WIFI_INIT_CONFIG_DEFAULT, please be notified that the field 'magic' of
  *               wifi_init_config_t should always be WIFI_INIT_CONFIG_MAGIC!
  *
  * @param  config pointer to WiFi init configuration structure; can point to a temporary variable.
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_NO_MEM: out of memory
  *    - others: refer to error code wise_err.h
  */
wise_err_t wise_wifi_init(const wifi_init_config_t *config)
{
	wise_err_t err = WISE_OK;
	struct sockaddr sa;
	ifconfig_handle_t *h = NULL;
	struct wpa_supplicant *wpa_s;
#ifdef CONFIG_SUPPORT_DUAL_VIF
	uint8_t			wlan1_mac[6];
#endif
	/* already initialized? we are happy with that. */
	if (wifi_scan_lock)
		return WISE_OK;

	wifi_scan_lock = osSemaphoreNew(1, 0, NULL);
	if (wifi_scan_lock == NULL)
		return -1;

	wise_event_set_default_wifi_handlers();
	LOCK_TCPIP_CORE();
	netif_add_ext_callback(&wise_netif_ext_cb, wise_netif_ext_callback);
	UNLOCK_TCPIP_CORE();

	h = ifconfig_open();
	if (h == NULL)
		err_exit(err, WISE_FAIL);

	if (wise_wpas_run("wlan0", 1) < 0)
		err_exit(err, WISE_FAIL);

	sa.sa_len = 6;
	memcpy(sa.sa_data, config->mac, 6);
#if 0
	if (ifconfig_set_hwaddr(h, "wlan0", sa) < 0)
		err_exit(err, WISE_FAIL);
#endif

	if(!(wpa_s = wise_wpas_get("wlan0")))
		err_exit(err, WISE_FAIL);

	if(!wpa_s->confname){
		if (add_network(wlan0) < 0)
			err_exit(err, WISE_FAIL);
	}

#ifdef CONFIG_SUPPORT_DUAL_VIF

	if (wise_wpas_run("wlan1", 1) < 0)
		err_exit(err, WISE_FAIL);

	memcpy(wlan1_mac, config->mac, 6);
	wlan1_mac[0] += 2;
	memcpy(sa.sa_data, wlan1_mac, 6);
	if (ifconfig_set_hwaddr(h, "wlan1", sa) < 0)
		err_exit(err, WISE_FAIL);

	if(!(wpa_s = wise_wpas_get("wlan1")))
		err_exit(err, WISE_FAIL);

	if(!wpa_s->confname){
		if (add_network(wlan1) < 0)
			err_exit(err, WISE_FAIL);
	}

#endif

exit:
	if (h)
		ifconfig_close(h);

	return err;
}

/**
  * @brief     Set the WiFi operating mode
  *
  *            Set the WiFi operating mode as station, soft-AP or station+soft-AP,
  *            The default mode is soft-AP mode.
  *
  * @param     mode  WiFi operating mode
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - others: refer to error code in wise_err.h
  */
wise_err_t wise_wifi_set_mode(wifi_mode_t mode, uint8_t wlan_if)
{
	s_wifi_mode[wlan_if] = mode;
	char *network_mode;

	if (mode == WIFI_MODE_NULL)
		return WISE_OK;

	network_mode = (mode == WIFI_MODE_STA) ? "0":"2";

	if (set_network(wlan_if, 0, "mode", network_mode) < 0)
		return WISE_FAIL;

	return WISE_OK;
}

/**
  * @brief  Get current operating mode of WiFi
  *
  * @param[out]  mode  store current WiFi mode
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  */
wise_err_t wise_wifi_get_mode(wifi_mode_t *mode, uint8_t wlan_if)
{
	*mode = s_wifi_mode[wlan_if];
	return WISE_OK;
}

/**
  * @brief     Clean the configuration of the STA or AP
  *
  * @attention 1. This API can be called only when specified interface is enabled, otherwise, API fail
  *
  * @param     interface  interface
  * @param     wlan_if  wlan interface idx
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_FAIL: failed
  *    - WISE_ERR_WIFI_MODE: invalid mode
  */
wise_err_t wise_wifi_clean_config(wifi_interface_t interface, uint8_t wlan_if)
{
	wise_err_t err = WISE_OK;

	if (interface == WIFI_IF_STA) {
		if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
			err_exit(err, WISE_ERR_WIFI_MODE);
	} else if (interface == WIFI_IF_AP) {
		if (s_wifi_mode[wlan_if] != WIFI_MODE_AP)
			err_exit(err, WISE_ERR_WIFI_MODE);
	}


	if (wlan_if == 0 ? remove_network(wlan0, 0) < 0 : remove_network(wlan1, 0) < 0)
		err_exit(err, WISE_FAIL);

	if (wlan_if == 0 ? add_network(wlan0) < 0 : add_network(wlan1) < 0)
		err_exit(err, WISE_FAIL);

	if (wise_wifi_set_mode(s_wifi_mode[wlan_if], wlan_if) != WISE_OK)
		err_exit(err, WISE_FAIL);

	exit:
		return err;
}

static char* get_ssid(wifi_interface_t interface, wifi_config_t *conf, int *free_value) {
	int len;
	char *ssid = NULL;
	uint8_t *if_ssid = (interface == WIFI_IF_STA ? conf->sta.ssid : conf->ap.ssid);

	if (if_ssid[0] == '\0')
		return NULL;
	len = strlen((char *) if_ssid);
	if (len > SSID_MAX_LEN)
		len = SSID_MAX_LEN;
	ssid = zalloc(2 * len + 1);
	if (ssid == NULL || free_value == NULL)
		return NULL;
	wpa_snprintf_hex(ssid, 2 * len + 1, (const u8 *)if_ssid, len);
	*free_value = FREE_MEM;
	return ssid;
}

static char* get_proto(wifi_interface_t interface, wifi_config_t *conf, int *free_value) {
	wifi_proto_mode_t proto = (interface == WIFI_IF_STA ? conf->sta.proto : conf->ap.proto);

	if (free_value == NULL)
		return NULL;

	*free_value = RESET_MEM;
	return proto == 1 ? "WPA" : "RSN";
}

static char* get_password(wifi_interface_t interface, wifi_config_t *conf, int *free_value) {
	int len;
	char *passphrase = NULL;
	uint8_t *password = (interface == WIFI_IF_STA ? conf->sta.password : conf->ap.password);
	len = strlen((char *) password);
	passphrase = zalloc(len + 3);
	if (passphrase == NULL || free_value == NULL)
		return NULL;

	os_snprintf(passphrase, len + 3, "\"%s\"", (char *) password);
	*free_value = FREE_MEM;
	return passphrase;
}

static char* get_pmf(wifi_interface_t interface, wifi_config_t *conf, int *free_value) {
	uint32_t pmf_mode = (interface == WIFI_IF_STA ? conf->sta.pmf_mode : 0);

	if (free_value == NULL)
		return NULL;

	*free_value = RESET_MEM;
	if (pmf_mode)
		return pmf_mode == WIFI_PMF_CAPABLE ? "1" : "2";
	else
		return "0";
}

static char* get_scan_ssid(wifi_interface_t interface, wifi_config_t *conf, int *free_value) {
	uint32_t scan_ssid = conf->sta.scan_ssid;

	*free_value = RESET_MEM;

	if (interface != WIFI_IF_STA)
		return "0";

	if (free_value == NULL)
		return NULL;

	return scan_ssid ? "1" : "0";
}

static char* get_bssid(wifi_sta_config_t *sta) {
	int len;
	char *bssid = NULL;

	/*
	 * twice larger buffer for bin2hex
	 * + 5 seperators (':')
	 * + tailing null
	 */

	len = 2 * ETH_ALEN + 5 + 1;
	bssid = zalloc(len);
	if (bssid == NULL)
		return NULL;

	wpa_snprintf_hex_sep(bssid, len, sta->bssid, ETH_ALEN, ':');
	return bssid;
}

wise_err_t wise_wpa_set_psk (uint8_t wlan_if,u8 *psk)
{
	struct wpa_ssid *ssid;
	wise_err_t err = WISE_OK;
	char *ifc = wlan_if == 0 ? "wlan0" : "wlan1";
	struct wpa_supplicant *wpa_s = wise_wpas_get(ifc);

	if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
		err_exit(err, WISE_FAIL);

	if (!wpa_s || !psk)
		err_exit(err, WISE_FAIL);

	if ((ssid = wise_wpas_get_network(wpa_s)) == NULL)
		err_exit(err, WISE_FAIL);

	memcpy(ssid->psk, psk, WISE_PMK_LEN);
	ssid->psk_set = 1;

exit:
	if (err)
		printf("err = %d \n", err);

	return err;
}


wise_err_t wise_wpa_get_psk (uint8_t wlan_if, u8 *psk, int len)
{
	struct wpa_ssid *ssid;
	wise_err_t err = WISE_OK;
	char *ifc = wlan_if == 0 ? "wlan0" : "wlan1";
	struct wpa_supplicant *wpa_s = wise_wpas_get(ifc);

	if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
		err_exit(err, WISE_FAIL);

	if (!wpa_s)
		err_exit(err, WISE_FAIL);

	if ((ssid = wise_wpas_get_network(wpa_s)) == NULL)
		err_exit(err, WISE_FAIL);

	if (!ssid->psk)
		err_exit(err, WISE_FAIL);

	if (len != WISE_PMK_LEN)
		err_exit(err, WISE_FAIL);

	memcpy(psk, ssid->psk, len);

exit:
	if (err)
		printf("err = %d \n", err);

	return err;
}

static wise_err_t wise_wifi_config_setup(wifi_interface_t interface, uint8_t wlan_if,
													wifi_config_t *conf, wifi_fast_connect_t *fast_config){
	wise_err_t err = WISE_OK;
	int alg = (interface == WIFI_IF_STA ? conf->sta.alg : conf->ap.alg);
	int max_alg = (interface == WIFI_IF_STA ? ARRAY_SIZE(sta_config_tables) : ARRAY_SIZE(sap_config_tables));
	int i = 0;
	struct wifi_config *alg_table = NULL;
	int free_value;
	int count = 0;

	if (alg >= max_alg)
		err_exit(err, WISE_FAIL);

	alg_table = (interface == WIFI_IF_STA ? sta_config_tables[alg] : sap_config_tables[alg]);
	count = (interface == WIFI_IF_STA ? sta_config_tables_sz[alg] : sap_config_tables_sz[alg]);

	if (alg_table != NULL) {
		for (i = 0 ; i < count; i++) {
			free_value = 0;
			if (alg_table[i].value == NULL)
			{
				if (strcmp(alg_table[i].name, "wep_key0") == 0 ||
					strcmp(alg_table[i].name, "psk") == 0 ||
					strcmp(alg_table[i].name, "sae_password") == 0) {
					if (fast_config && strcmp(alg_table[i].name, "psk") == 0) {
						if (wise_wpa_set_psk(wlan_if, fast_config->psk) == WISE_OK)
							continue;
					}
					alg_table[i].value = get_password(interface, conf, &free_value);
				} else if (strcmp(alg_table[i].name, "proto") == 0) {
					alg_table[i].value = get_proto(interface, conf, &free_value);
				} else if (strcmp(alg_table[i].name, "ieee80211w") == 0) {
					alg_table[i].value = get_pmf(interface, conf, &free_value);
				} else if (strcmp(alg_table[i].name, "ssid") == 0) {
					alg_table[i].value = get_ssid(interface, conf, &free_value);
				} else if (strcmp(alg_table[i].name, "scan_ssid") == 0){
					alg_table[i].value = get_scan_ssid(interface, conf, &free_value);
				} else {
					printf("invalid name %s \n", alg_table[i].name);
					err_exit(err, WISE_ERR_INVALID_ARG);
				}

				if (alg_table[i].value == NULL)
					err_exit(err, WISE_FAIL);
			}
			/* printf("alg %d, name:%s, value:%s \n", alg, alg_table[i].name, alg_table[i].value); */
			if (set_network(wlan_if, 0, alg_table[i].name, alg_table[i].value) < 0)
				err_exit(err, WISE_FAIL);

			if (free_value) {
				if (free_value == FREE_MEM)
					free(alg_table[i].value);
				alg_table[i].value = NULL;
			}

		}

	}
	else
		err_exit(err, WISE_FAIL);

exit:

	return err;
}

/**
  * @brief     Set the configuration of the STA or AP
  *
  * @attention 1. This API can be called only when specified interface is enabled, otherwise, API fail
  * @attention 2. For station configuration, bssid_set needs to be 0; and it needs to be 1 only when users need to check the MAC address of the AP.
  * @attention 3. HW is limited to only one channel, so when in the soft-AP+station mode, the soft-AP will adjust its channel automatically to be the same as
  *               the channel of the station.
  *
  * @param     interface  interface
  * @param     conf  station or soft-AP configuration
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_WIFI_IF: invalid interface
  *    - WISE_ERR_WIFI_MODE: invalid mode
  *    - WISE_ERR_WIFI_PASSWORD: invalid password
  *    - WISE_ERR_WIFI_NVS: WiFi internal NVS error
  *    - others: refer to the erro code in wise_err.h
  */
wise_err_t wise_wifi_set_config(wifi_interface_t interface,
		wifi_config_t *conf, uint8_t wlan_if, wifi_fast_connect_t *fast_config)
{
	wise_err_t err = WISE_OK;
	char *bssid = NULL;
	int freq = 0;
	char args[16] = {0};
	ifconfig_handle_t *h = NULL;
	int roam = -1;
	const char *ifc = wlan_if == 0 ? "wlan0" : "wlan1";

	if (interface == WIFI_IF_STA) {
		if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
			err_exit(err, WISE_ERR_WIFI_MODE);
	} else if (interface == WIFI_IF_AP) {
		if (s_wifi_mode[wlan_if] != WIFI_MODE_AP)
			err_exit(err, WISE_ERR_WIFI_MODE);
	}

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);
	if (ifconfig_get80211val(h, ifc,
				IEEE80211_IOC_ROAMING,
				0, &roam) < 0)
		err_exit(err, WISE_FAIL);
	if (ifconfig_set80211(h, ifc,
				IEEE80211_IOC_ROAMING,
				IEEE80211_ROAMING_MANUAL,
				0, NULL) < 0)
		err_exit(err, WISE_FAIL);


	if (wise_wifi_config_setup(interface, wlan_if, conf, fast_config) < 0)
		err_exit(err, WISE_FAIL);

	if (interface == WIFI_IF_STA) {
		wifi_sta_config_t *sta = &conf->sta;
		if (sta->bssid_set) {
			if ((bssid = get_bssid(sta)) == NULL)
				err_exit(err, WISE_FAIL);
			if (set_network(wlan_if, 0, "bssid", bssid) < 0)
				err_exit(err, WISE_FAIL);
		}

		if (fast_config) {
			if (fast_config->channel == 14)
				freq = 2484;
			if (fast_config->channel < 14)
				freq = 2407 + fast_config->channel * 5;
			else
				err_exit(err, WISE_FAIL);
		}

		os_snprintf(args, sizeof(args), "%d", freq);
		if (set_network(wlan_if, 0, "frequency", args) < 0)
			err_exit(err, WISE_FAIL);
	} else if (interface == WIFI_IF_AP) {
		wifi_ap_config_t *ap = &conf->ap;

		if (ap->channel) {
			/* FIXME: add HT40 */
			if (ap->channel == 14)
				freq = 2484;
			if (ap->channel < 14)
				freq = 2407 + ap->channel * 5;
			os_snprintf(args, sizeof(args), "%d", freq);
			if (set_network(wlan_if, 0, "frequency", args) < 0)
				err_exit(err, WISE_FAIL);
		}
		if (ifconfig_set80211(h, ifc,
					IEEE80211_IOC_HIDESSID,
					ap->ssid_hidden, 0, NULL) < 0)
			err_exit(err, WISE_FAIL);
		if (ap->beacon_interval) {
			if (ifconfig_set80211(h, ifc,
						IEEE80211_IOC_BEACON_INTERVAL,
						ap->beacon_interval, 0, NULL) < 0)
				err_exit(err, WISE_FAIL);
		}
	} else {
		WISE_LOGE(TAG, "Invalid interface(%d)", (int)interface);
		err_exit(err, WISE_ERR_INVALID_ARG);
	}

exit:
	if (roam >= IEEE80211_ROAMING_MANUAL
			&& ifconfig_set80211(h, ifc,
				IEEE80211_IOC_ROAMING,
				roam,
				0, NULL) < 0)
		err = WISE_FAIL;

	if (bssid)
		free(bssid);
	if (h)
		ifconfig_close(h);

	return err;
}

/**
  * @brief    Save wpa supplicant configuration
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_FAIL:failed
  */
wise_err_t wise_wifi_save_config(void)
{
	wise_err_t err = WISE_OK;

#ifndef CONFIG_NO_CONFIG_WRITE
	char *argv[1] = {"SAVE_CONFIG"};
	int argc = 1;
	if (wise_wpas_cli(argc, argv) < 0)
		err_exit(err, WISE_FAIL);
exit:
#endif
	return err;
}

/**
  * @brief     Set the IP configuration of the STA or AP
  *
  * @attention 1. This API can be called only when specified interface is enabled, otherwise, API fail
  * @attention 2. For station configuration, IP info might be automatically changed upon DHCP later.
  * @attention 3. For AP configuration, this should be done before wise_wifi_start().
  *
  * @param     interface  interface
  * @param     ipinfo  IP address, netmask, and gateway
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_WIFI_IF: invalid interface
  *    - others: refer to the erro code in wise_err.h
  */
wise_err_t wise_wifi_set_ip_info(wifi_interface_t interface, wifi_ip_info_t *ipinfo)
{
	ifconfig_handle_t *h;
	struct sockaddr sa;
	struct sockaddr_in *sa_in;

	char *ifc = interface == 0 ? "wlan0" : "wlan1";

	h = ifconfig_open();
	if (!h)
		return WISE_FAIL;

	sa_in = (struct sockaddr_in *)&sa;

	memset(sa_in, 0, sizeof(*sa_in));
	sa_in->sin_len = sizeof(struct sockaddr_in);
	sa_in->sin_family = AF_INET;
	sa_in->sin_port = 0;
	inet_addr_from_ip4addr(&sa_in->sin_addr, &ipinfo->ip);

	if (ifconfig_set_addr(h, ifc, sa) < 0) {
		ifconfig_close(h);
		return WISE_FAIL;
	}

	memset(sa_in, 0, sizeof(*sa_in));
	sa_in->sin_len = sizeof(struct sockaddr_in);
	sa_in->sin_family = AF_INET;
	sa_in->sin_port = 0;
	inet_addr_from_ip4addr(&sa_in->sin_addr, &ipinfo->nm);

	if (ifconfig_set_netmask(h, ifc, sa) < 0) {
		ifconfig_close(h);
		return WISE_FAIL;
	}

	memset(sa_in, 0, sizeof(*sa_in));
	sa_in->sin_len = sizeof(struct sockaddr_in);
	sa_in->sin_family = AF_INET;
	sa_in->sin_port = 0;
	inet_addr_from_ip4addr(&sa_in->sin_addr, &ipinfo->gw);

	if (ifconfig_set_gateway(h, ifc, sa) < 0) {
		ifconfig_close(h);
		return WISE_FAIL;
	}

	ifconfig_close(h);
	return WISE_OK;
}

/**
  * @brief     Get IP configuration of specified interface
  *
  * @param     interface  interface
  * @param[out]  ipinfo  IP address, netmask, and gateway
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_WIFI_IF: invalid interface
  */
wise_err_t wise_wifi_get_ip_info(wifi_interface_t interface, wifi_ip_info_t *ipinfo)
{
	int ret = WISE_OK;
	ifconfig_handle_t *h = NULL;
	struct sockaddr sa, nm, gw;
	struct sockaddr_in *addr;

	h = ifconfig_open();
	if (h == NULL)
		return WISE_FAIL;

	/* get & copy ip address */
	if (ifconfig_get_addr(h, "wlan0", &sa) < 0) {
		ret = WISE_FAIL;
		goto exit;
	}
	addr = (struct sockaddr_in *)&sa;
	memcpy(&ipinfo->ip, &(addr->sin_addr.s_addr), 4);

	/* get & copy netmask */
	if (ifconfig_get_netmask(h, "wlan0", &nm) < 0) {
		ret = WISE_FAIL;
		goto exit;
	}
	addr = (struct sockaddr_in *)&nm;
	memcpy(&ipinfo->nm, &(addr->sin_addr.s_addr), 4);

	/* get & copy gateway */
	if (ifconfig_get_gateway(h, "wlan0", &gw) < 0) {
		ret = WISE_FAIL;
		goto exit;
	}
	addr = (struct sockaddr_in *)&gw;
	memcpy(&ipinfo->gw, &(addr->sin_addr.s_addr), 4);

exit:
	ifconfig_close(h);

	return ret;
}

/**
  * @brief  Start WiFi according to current configuration
  *         If mode is WIFI_MODE_STA, it create station control block and start station
  *         If mode is WIFI_MODE_AP, it create soft-AP control block and start soft-AP
  *         If mode is WIFI_MODE_APSTA, it create soft-AP and station control block and start soft-AP and station
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_NO_MEM: out of memory
  *    - WISE_ERR_WIFI_CONN: WiFi internal error, station or soft-AP control block wrong
  *    - WISE_FAIL: other WiFi internal errors
  */
wise_err_t wise_wifi_start(uint8_t wlan_if)
{
	if (wlan_if == 0 ? enable_network(wlan0, 0) < 0 : enable_network(wlan1, 0) < 0)
		return WISE_FAIL;

	/* SYSTEM_EVENT_AP_START will send from wpas_notify_state_changed*/
	if (wlan_if == 0) {
		system_event_t evt;
		evt.event_id = SYSTEM_EVENT_STA_START;
		return wise_event_send(&evt);
	}

	return WISE_OK;
}

/**
  * @brief  Stop WiFi
  *         If mode is WIFI_MODE_STA, it stop station and free station control block
  *         If mode is WIFI_MODE_AP, it stop soft-AP and free soft-AP control block
  *         If mode is WIFI_MODE_APSTA, it stop station/soft-AP and free station/soft-AP control block
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  */
wise_err_t wise_wifi_stop(uint8_t wlan_if)
{
	if (wlan_if == 0 ? disable_network(wlan0, 0) < 0 : disable_network(wlan1, 0) < 0)
		return WISE_FAIL;

	/* SYSTEM_EVENT_AP_STOP will send from hostapd_interface_deinit*/
	if (wlan_if == 0) {
		system_event_t evt;
		evt.event_id = SYSTEM_EVENT_STA_STOP;
		return wise_event_send(&evt);
	}

	return WISE_OK;

}

/**
  * @brief     Reconnect the WiFi station to the AP.
  *
  * @attention 1. This API only impact WIFI_MODE_STA or WIFI_MODE_APSTA mode
  * @attention 2. If the station is disconnected to an AP, call wise_wifi_reconnect to reconnect with sched scan.
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_FAIL: failed
  */
wise_err_t wise_wifi_reconnect(uint8_t wlan_if)
{
	if (wlan_if == 0 ? reconnect(wlan0) < 0 : reconnect(wlan1) < 0)
		return WISE_FAIL;

	return WISE_OK;
}

/**
  * @brief     Connect the WiFi station to the AP.
  *
  * @attention 1. This API only impact WIFI_MODE_STA or WIFI_MODE_APSTA mode
  * @attention 2. If the station is connected to an AP, call wise_wifi_disconnect to disconnect.
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_WIFI_NOT_START: WiFi is not started by wise_wifi_start
  *    - WISE_ERR_WIFI_CONN: WiFi internal error, station or soft-AP control block wrong
  *    - WISE_ERR_WIFI_SSID: SSID of AP which station connects is invalid
  */
wise_err_t wise_wifi_connect(uint8_t wlan_if)
{
	if (wlan_if == 0 ? enable_network(wlan0, 0) < 0 : enable_network(wlan1, 0) < 0)
		return WISE_FAIL;

	return WISE_OK;
}

/**
  * @brief     Disconnect the WiFi station from the AP.
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi was not initialized by wise_wifi_init
  *    - WISE_ERR_WIFI_NOT_STARTED: WiFi was not started by wise_wifi_start
  *    - WISE_FAIL: other WiFi internal errors
  */
wise_err_t wise_wifi_disconnect(uint8_t wlan_if)
{
	if (wlan_if == 0 ? disable_network(wlan0, 0) < 0 : disable_network(wlan1, 0) < 0)
		return WISE_FAIL;

	return WISE_OK;
}

wise_err_t wise_wifi_deauth(uint8_t wlan_if, const char *txtaddr)
{
	if (wlan_if == 0 ? deauthenticate(wlan0, (char *)txtaddr) < 0 : deauthenticate(wlan1, (char *)txtaddr) < 0)
		return WISE_FAIL;

	return WISE_OK;
}

/**
  * @brief     Scan all available APs.
  *
  * @attention If this API is called, the found APs are stored in WiFi driver dynamic allocated memory and the
  *            will be freed in wise_wifi_scan_get_ap_records, so generally, call wise_wifi_scan_get_ap_records to cause
  *            the memory to be freed once the scan is done
  * @attention The values of maximum active scan time and passive scan time per channel are limited to 1500 milliseconds.
  *            Values above 1500ms may cause station to disconnect from AP and are not recommended.
  *
  * @param     config  configuration of scanning
  * @param     block if block is true, this API will block the caller until the scan is done, otherwise
  *                         it will return immediately
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_STARTED: WiFi was not started by wise_wifi_start
  *    - WISE_ERR_WIFI_NOT_SUPPORTED: WiFi is not set as station mode
  *    - WISE_ERR_NO_MEM: out of memory
  *    - others: refer to error code in wise_err.h
  */
wise_err_t wise_wifi_scan_start(const wifi_scan_config_t *config, bool block, uint8_t wlan_if)
{
	wise_err_t err = WISE_OK;
	/* NB: will not use scan cache (only_new=1) */
#ifdef CONFIG_SUPPORT_DUAL_VIF
#define WISE_SCAN_PARA_START 5
	char *argv[12] = {"-i", NULL,"SCAN", "TYPE=ONLY", "only_new=1", NULL, NULL, NULL,\
		NULL, NULL, NULL, NULL};
	int argc = WISE_SCAN_PARA_START;
#else
#define WISE_SCAN_PARA_START 3
	char *argv[10] = {"SCAN", "TYPE=ONLY", "only_new=1", NULL, NULL, NULL,\
		NULL, NULL, NULL, NULL};
	int argc = WISE_SCAN_PARA_START;
#endif
	int i;
	int len;
	char *ssid = NULL, *bssid = NULL;
	int freq, min, max;

	if (is_wise_wpas_run(wlan_if) == 0) {
		err_exit(err, WISE_ERR_WIFI_NOT_STARTED);
	}

	if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
		err_exit(err, WISE_ERR_NOT_SUPPORTED);

#ifdef CONFIG_SUPPORT_DUAL_VIF
	argv[1] = wlan_if == 0 ? "wlan0" : "wlan1";
#endif

	if (config->ssid) {
		len = strlen((char *)config->ssid);
		ssid = zalloc(2 * len + 1);
		if (ssid == NULL)
			err_exit(err, WISE_ERR_NO_MEM);
		wpa_snprintf_hex(ssid, 2 * len + 1, (const u8 *)config->ssid, len);
		if (form_argv(&argv[argc++], "ssid %s", ssid) <= 0)
			err_exit(err, WISE_ERR_NO_MEM);
	}
	if (config->bssid) {
		if (form_argv(&argv[argc++], "bssid=%s", config->bssid) <= 0)
			err_exit(err, WISE_ERR_NO_MEM);
	}
	if (config->channel) {
		/* FIXME: add HT40 */
		if (config->channel == 14)
			freq = 2484;
		if (config->channel < 14)
			freq = 2407 + config->channel * 5;
		if (form_argv(&argv[argc++], "freq=%d", freq) <= 0)
			err_exit(err, WISE_ERR_NO_MEM);
	}
	if (form_argv(&argv[argc++], "show_hidden=%d",
				!!config->show_hidden) <= 0)
		err_exit(err, WISE_ERR_NO_MEM);

	if (form_argv(&argv[argc++], "spec_ssid=%d",
				!!config->spec_ssid) <= 0)
		err_exit(err, WISE_ERR_NO_MEM);

	if (form_argv(&argv[argc++], "passive=%d",
				!!(config->scan_type == WIFI_SCAN_TYPE_PASSIVE)) <= 0)
		err_exit(err, WISE_ERR_NO_MEM);

	if (config->scan_type == WIFI_SCAN_TYPE_ACTIVE) {
		min = config->scan_time.active.min;
		max = config->scan_time.active.max;
	} else {
		min = max = config->scan_time.passive;
	}
	if (form_argv(&argv[argc++], "dwell_min=%d", min) <= 0)
		err_exit(err, WISE_ERR_NO_MEM);

	if (form_argv(&argv[argc++], "dwell_max=%d", max) <= 0)
		err_exit(err, WISE_ERR_NO_MEM);

	if (wise_wpas_cli(argc, argv) < 0)
		err_exit(err, WISE_FAIL);

	/* XXX: not prepared for mixed blocking, non-blocking calls */
	if (block)
		osSemaphoreAcquire(wifi_scan_lock, osWaitForever);
exit:

	for (i = WISE_SCAN_PARA_START; i < argc && argv[i]; i++)
		free(argv[i]);
	if (ssid)
		free(ssid);
	if (bssid)
		free(bssid);

#undef WISE_SCAN_PARA_START
	return err;
}

/**
  * @brief     Stop the scan in process
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_WIFI_NOT_STARTED: WiFi is not started by wise_wifi_start
  */
wise_err_t wise_wifi_scan_stop(void)
{
	ifconfig_handle_t *h;

	h = ifconfig_open();
	if (!h)
		return WISE_FAIL;
	if (ifconfig_set80211(h, "wlan0",
				104 /*IEEE80211_IOC_SCAN_CANCEL*/,
				0, 0, NULL) < 0) {
		ifconfig_close(h);
		goto fail;
	}

	ifconfig_close(h);
	return WISE_OK;

fail:
	return WISE_FAIL;
}

/**
  * @brief     Get number of APs found in last scan
  *
  * @param[out] number  store number of APIs found in last scan
  *
  * @attention This API can only be called when the scan is completed, otherwise it may get wrong value.
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_WIFI_NOT_STARTED: WiFi is not started by wise_wifi_start
  *    - WISE_ERR_INVALID_ARG: invalid argument
  */
wise_err_t wise_wifi_scan_get_ap_num(uint16_t *number)
{
	struct wpa_supplicant *wpa_s = wise_wpas_get("wlan0");

	if (!wpa_s)
		goto fail;

	osMutexAcquire(wpa_s->bss_lock, osWaitForever);

	*number = wpa_s->last_scan_res_used;

	osMutexRelease(wpa_s->bss_lock);

	return WISE_OK;

fail:
	return WISE_FAIL;
}

void wise_wifi_flush_bss(void) {
	struct wpa_supplicant *wpa_s = wise_wpas_get("wlan0");
	wpa_bss_flush(wpa_s);
}


/**
  * @brief     Get AP list found in last scan
  *
  * @param[inout]  number As input param, it stores max AP number ap_records can hold.
  *                As output param, it receives the actual AP number this API returns.
  * @param         ap_records  wifi_ap_record_t array to hold the found APs
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_WIFI_NOT_STARTED: WiFi is not started by wise_wifi_start
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_NO_MEM: out of memory
  */
wise_err_t wise_wifi_scan_get_ap_records(uint16_t ap_idx, uint16_t *number, wifi_ap_record_t *ap_records)
{
	struct wpa_supplicant *wpa_s = wise_wpas_get("wlan0");
	struct wpa_bss *bss;
	int i, num_res;
	const u8 *elem;

	if (!wpa_s)
		goto fail;

	/* NB: ap_records is assumed to be zero-ed */

	osMutexAcquire(wpa_s->bss_lock, osWaitForever);

	if (ap_idx + *number > wpa_s->last_scan_res_used)
		num_res = wpa_s->last_scan_res_used - ap_idx;
	else
		num_res = *number;

	for (i = 0; i < num_res; i++) {
		const u8 *wpa, *rsn, *ie;
		struct wpa_ie_data wpa_ie;
		bool ieee8021x;
		int maxrate, maxbrate;
		wifi_ap_record_t *record = &ap_records[i];

		bss = wpa_s->last_scan_res[i + ap_idx];
		assert(bss);
		memcpy(record->bssid, bss->bssid, ETH_ALEN);
		memcpy(record->ssid, bss->ssid, bss->ssid_len);
		/* NB: simple conversion assuming only B, G and HT20 */
		if (bss->freq == 2484)
			record->primary = 14;
		else if (bss->freq >= 2407 && bss->freq < 2484)
			record->primary = (bss->freq - 2407) / 5;
		record->second = WIFI_SECOND_CHAN_NONE;
		record->rssi = bss->qual;
		if ((bss->caps & IEEE80211_CAP_PRIVACY) == 0)
			record->authmode = WIFI_AUTH_OPEN;
		else {
			wpa = wpa_bss_get_vendor_ie(bss, WPA_IE_VENDOR_TYPE);
			rsn = wpa_bss_get_ie(bss, WLAN_EID_RSN);
			if (!wpa && !rsn)
				record->authmode = WIFI_AUTH_WEP;
			else {
				int cipher;
				ie = wpa ? wpa : rsn;
				if (wpa_parse_wpa_ie(ie, 2 + ie[1], &wpa_ie) < 0) {
					WISE_LOGE(TAG, "Cannot parse WPA/RSN IE");
					continue;
				}
				cipher = wpa_ie.pairwise_cipher & (WPA_CIPHER_TKIP |
						WPA_CIPHER_CCMP | WPA_CIPHER_WEP40 | WPA_CIPHER_WEP104);
				if (cipher == (WPA_CIPHER_TKIP | WPA_CIPHER_CCMP))
					record->pairwise_cipher = WIFI_CIPHER_TYPE_TKIP_CCMP;
				else if (cipher == WPA_CIPHER_TKIP)
					record->pairwise_cipher = WIFI_CIPHER_TYPE_TKIP;
				else if (cipher == WPA_CIPHER_CCMP)
					record->pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
				else if (cipher == WPA_CIPHER_WEP40)
					record->pairwise_cipher = WIFI_CIPHER_TYPE_WEP40;
				else if (cipher == WPA_CIPHER_WEP104)
					record->pairwise_cipher = WIFI_CIPHER_TYPE_WEP104;
				else
					record->pairwise_cipher = WIFI_CIPHER_TYPE_NONE;
				cipher = wpa_ie.group_cipher & (WPA_CIPHER_TKIP |
						WPA_CIPHER_CCMP | WPA_CIPHER_WEP40 | WPA_CIPHER_WEP104);
				if (cipher == (WPA_CIPHER_TKIP | WPA_CIPHER_CCMP))
					record->group_cipher = WIFI_CIPHER_TYPE_TKIP_CCMP;
				else if (cipher == WPA_CIPHER_TKIP)
					record->group_cipher = WIFI_CIPHER_TYPE_TKIP;
				else if (cipher == WPA_CIPHER_CCMP)
					record->group_cipher = WIFI_CIPHER_TYPE_CCMP;
				else if (cipher == WPA_CIPHER_WEP40)
					record->group_cipher = WIFI_CIPHER_TYPE_WEP40;
				else if (cipher == WPA_CIPHER_WEP104)
					record->group_cipher = WIFI_CIPHER_TYPE_WEP104;
				else
					record->group_cipher = WIFI_CIPHER_TYPE_NONE;
				ieee8021x = wpa_key_mgmt_wpa_ieee8021x(wpa_ie.key_mgmt);
				if (wpa && !rsn) {
					if (ieee8021x)
						record->authmode = WIFI_AUTH_WPA_ENTERPRISE;
					else
						record->authmode = WIFI_AUTH_WPA_PSK;
				} else if (!wpa && rsn) {
					if (ieee8021x)
						record->authmode = WIFI_AUTH_WPA2_ENTERPRISE;
					else
						record->authmode = WIFI_AUTH_WPA2_PSK;
				} else {
					if (ieee8021x)
						record->authmode = WIFI_AUTH_WPA_WPA2_ENTERPRISE;
					else
						record->authmode = WIFI_AUTH_WPA_WPA2_PSK;
				}
				if (wpa_ie.key_mgmt & WPA_KEY_MGMT_WPS)
					record->wps = 1;
			}
		}
		record->ant = WIFI_ANT_ANT0;
		elem = wpa_bss_get_ie(bss, WLAN_EID_COUNTRY);
		if (elem && elem[1] >= 6) {
			memcpy(record->country.cc, (const char *)(elem + 2), 3);
			record->country.schan = *(u8 *)(elem + 5);
			record->country.nchan = *(u8 *)(elem + 6);
			record->country.max_tx_power = *(s8 *)(elem + 7);
		}

		ie = wpa_bss_get_ext_tag_ie(bss, WLAN_EID_EXT_HE_OPERATION);
		if (ie)
			record->phy_11ax = 1;
		else
			record->phy_11ax = 0;

		/* XXX might need to handle N-Ony mode (= GF mode), but how? */
		ie = wpa_bss_get_ie(bss, WLAN_EID_HT_CAP);
		if (ie)
			record->phy_11n = 1;
		else
			record->phy_11n = 0;
		maxrate = wpa_bss_get_max_rate(bss);
		maxbrate = wpa_bss_get_max_basic_rate(bss);
		if (maxrate > 22)
			record->phy_11g = 1;
		else
			record->phy_11g = 0;
		if (maxbrate <= 22)
			record->phy_11b = 1;
		else
			record->phy_11b = 0;
	}

	osMutexRelease(wpa_s->bss_lock);

	*number = num_res;

	return WISE_OK;

fail:
	*number = 0;

	return WISE_FAIL;
}

/**
  * @brief     Get information of AP which the station is associated with
  *
  * @param     ap_info  the wifi_ap_record_t to hold AP information
  *            sta can get the connected ap's phy mode info through the struct member
  *            phy_11b，phy_11g，phy_11n，phy_lr in the wifi_ap_record_t struct.
  *            For example, phy_11b = 1 imply that ap support 802.11b mode
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_CONN: The station interface don't initialized
  *    - WISE_ERR_WIFI_NOT_CONNECT: The station is in disconnect status
  */
wise_err_t wise_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info, uint8_t wlan_if)
{
	struct wpa_bss *bss;
	const u8 *elem;
	const u8 *wpa, *rsn, *ie;
	struct wpa_ie_data wpa_ie;
	bool ieee8021x;
	int maxrate, maxbrate;
	wise_err_t err = WISE_OK;
	int len;
	ifconfig_handle_t *h;
	char *ifc = wlan_if == 0 ? "wlan0" : "wlan1";
	struct wpa_supplicant *wpa_s = wise_wpas_get(ifc);

	if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
		err_exit(err, WISE_FAIL);

	union {
		struct ieee80211req_sta_req req;
		uint8_t buf[/*24*/1*1024];
	} *u;

	if (!wpa_s)
		err_exit(err, WISE_FAIL);

	if (s_wifi_mode[wlan_if] != WIFI_MODE_STA)
		err_exit(err, WISE_ERR_WIFI_MODE);

	osMutexAcquire(wpa_s->bss_lock, osWaitForever);

	bss = wpa_s->current_bss;
	if (bss && wpa_s->wpa_state >= WPA_ASSOCIATED) {
		memcpy(ap_info->bssid, bss->bssid, ETH_ALEN);
		memcpy(ap_info->ssid, bss->ssid, bss->ssid_len);
		/* NB: simple conversion assuming only B, G and HT20 */
		if (bss->freq == 2484)
			ap_info->primary = 14;
		else if (bss->freq >= 2407 && bss->freq < 2484)
			ap_info->primary = (bss->freq - 2407) / 5;
		ap_info->second = WIFI_SECOND_CHAN_NONE;
		//ap_info->rssi = bss->qual;
		if ((bss->caps & IEEE80211_CAP_PRIVACY) == 0)
			ap_info->authmode = WIFI_AUTH_OPEN;
		else {
			wpa = wpa_bss_get_vendor_ie(bss, WPA_IE_VENDOR_TYPE);
			rsn = wpa_bss_get_ie(bss, WLAN_EID_RSN);
			if (!wpa && !rsn)
				ap_info->authmode = WIFI_AUTH_WEP;
			else {
				int cipher;
				ie = wpa ? wpa : rsn;
				if (wpa_parse_wpa_ie(ie, 2 + ie[1], &wpa_ie) == 0) {
					cipher = wpa_ie.pairwise_cipher & (WPA_CIPHER_TKIP |
							WPA_CIPHER_CCMP | WPA_CIPHER_WEP40 | WPA_CIPHER_WEP104);
					if (cipher == (WPA_CIPHER_TKIP | WPA_CIPHER_CCMP))
						ap_info->pairwise_cipher = WIFI_CIPHER_TYPE_TKIP_CCMP;
					else if (cipher == WPA_CIPHER_TKIP)
						ap_info->pairwise_cipher = WIFI_CIPHER_TYPE_TKIP;
					else if (cipher == WPA_CIPHER_CCMP)
						ap_info->pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
					else if (cipher == WPA_CIPHER_WEP40)
						ap_info->pairwise_cipher = WIFI_CIPHER_TYPE_WEP40;
					else if (cipher == WPA_CIPHER_WEP104)
						ap_info->pairwise_cipher = WIFI_CIPHER_TYPE_WEP104;
					else
						ap_info->pairwise_cipher = WIFI_CIPHER_TYPE_NONE;
					cipher = wpa_ie.group_cipher & (WPA_CIPHER_TKIP |
							WPA_CIPHER_CCMP | WPA_CIPHER_WEP40 | WPA_CIPHER_WEP104);
					if (cipher == (WPA_CIPHER_TKIP | WPA_CIPHER_CCMP))
						ap_info->group_cipher = WIFI_CIPHER_TYPE_TKIP_CCMP;
					else if (cipher == WPA_CIPHER_TKIP)
						ap_info->group_cipher = WIFI_CIPHER_TYPE_TKIP;
					else if (cipher == WPA_CIPHER_CCMP)
						ap_info->group_cipher = WIFI_CIPHER_TYPE_CCMP;
					else if (cipher == WPA_CIPHER_WEP40)
						ap_info->group_cipher = WIFI_CIPHER_TYPE_WEP40;
					else if (cipher == WPA_CIPHER_WEP104)
						ap_info->group_cipher = WIFI_CIPHER_TYPE_WEP104;
					else
						ap_info->group_cipher = WIFI_CIPHER_TYPE_NONE;
					ieee8021x = wpa_key_mgmt_wpa_ieee8021x(wpa_ie.key_mgmt);
					if (wpa && !rsn) {
						if (ieee8021x)
							ap_info->authmode = WIFI_AUTH_WPA_ENTERPRISE;
						else
							ap_info->authmode = WIFI_AUTH_WPA_PSK;
					} else if (!wpa && rsn) {
						if (ieee8021x)
							ap_info->authmode = WIFI_AUTH_WPA2_ENTERPRISE;
						else
							ap_info->authmode = WIFI_AUTH_WPA2_PSK;
					} else {
						if (ieee8021x)
							ap_info->authmode = WIFI_AUTH_WPA_WPA2_ENTERPRISE;
						else
							ap_info->authmode = WIFI_AUTH_WPA_WPA2_PSK;
					}
					if (wpa_ie.key_mgmt & WPA_KEY_MGMT_WPS)
						ap_info->wps = 1;
				} else
					WISE_LOGE(TAG, "Cannot parse WPA/RSN IE");
			}
		}
		ap_info->ant = WIFI_ANT_ANT0;
		elem = wpa_bss_get_ie(bss, WLAN_EID_COUNTRY);
		if (elem && elem[1] >= 6) {
			memcpy(ap_info->country.cc, (const char *)(elem + 2), 3);
			ap_info->country.schan = *(u8 *)(elem + 5);
			ap_info->country.nchan = *(u8 *)(elem + 6);
			ap_info->country.max_tx_power = *(s8 *)(elem + 7);
		}

		ie = wpa_bss_get_ext_tag_ie(bss, WLAN_EID_EXT_HE_OPERATION);
		if (ie)
			ap_info->phy_11ax = 1;
		else
			ap_info->phy_11ax = 0;

		/* XXX might need to handle N-Ony mode (= GF mode), but how? */
		ie = wpa_bss_get_ie(bss, WLAN_EID_HT_CAP);
		if (ie)
			ap_info->phy_11n = 1;
		else
			ap_info->phy_11n = 0;
		maxrate = wpa_bss_get_max_rate(bss);
		maxbrate = wpa_bss_get_max_basic_rate(bss);
		if (maxrate > 22)
			ap_info->phy_11g = 1;
		else
			ap_info->phy_11g = 0;
		if (maxbrate <= 22)
			ap_info->phy_11b = 1;
		else
			ap_info->phy_11b = 0;

	} else
		err = WISE_ERR_WIFI_NOT_CONNECT;

	osMutexRelease(wpa_s->bss_lock);

	u = malloc(sizeof(*u));
	if(!u)
		err_exit(err, WISE_ERR_NO_MEM);

	memset(u->req.is_u.macaddr,0xff, IEEE80211_ADDR_LEN);

	h = ifconfig_open();
	if (!h){
		free(u);
		err_exit(err, WISE_FAIL);
	}

	if (ifconfig_get80211(h, "wlan0", IEEE80211_IOC_BSSID,  u->req.is_u.macaddr, IEEE80211_ADDR_LEN) < 0) {
		free(u);
		ifconfig_close(h);
		err_exit(err, WISE_FAIL);
	}

	if (ifconfig_get80211len(h, "wlan0", IEEE80211_IOC_STA_INFO, u, sizeof(*u), &len) < 0) {
		free(u);
		ifconfig_close(h);
		err_exit(err, WISE_FAIL);
	}

	const struct ieee80211req_sta_info *si = u->req.info;
	ap_info->rssi = si->isi_rssi;
	ifconfig_close(h);
	free(u);

exit:
	return err;

}

/**
  * @brief     Set MAC address of the WiFi station or the soft-AP interface.
  *
  * @attention 1. This API can only be called when the interface is disabled
  * @attention 2. Soft-AP and station have different MAC addresses, do not set them to be the same.
  * @attention 3. The bit 0 of the first byte of the MAC address can not be 1. For example, the MAC address
  *      can set to be "1a:XX:XX:XX:XX:XX", but can not be "15:XX:XX:XX:XX:XX".
  *
  * @param     ifx  interface
  * @param     mac  the MAC address
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_WIFI_IF: invalid interface
  *    - WISE_ERR_WIFI_MAC: invalid mac address
  *    - WISE_ERR_WIFI_MODE: WiFi mode is wrong
  *    - others: refer to error codes in wise_err.h
  */
wise_err_t wise_wifi_set_mac(const uint8_t mac[6], uint8_t wlan_if)
{
	struct sockaddr sa;
	ifconfig_handle_t *h;

	h = ifconfig_open();
	if (!h)
		return WISE_FAIL;
	sa.sa_len = 6;
	memcpy(sa.sa_data, mac, 6);

	if (ifconfig_set_hwaddr(h, wlan_if == 0 ? "wlan0" : "wlan1", sa) < 0) {
		ifconfig_close(h);
		goto fail;
	}

	ifconfig_close(h);
	return WISE_OK;

fail:
	return WISE_FAIL;
}

/**
  * @brief     Get mac of specified interface
  *
  * @param      ifx  interface
  * @param[out] mac  store mac of the interface ifx
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_WIFI_IF: invalid interface
  */
wise_err_t wise_wifi_get_mac(uint8_t mac[6], uint8_t wlan_if)
{
	struct sockaddr sa;
	ifconfig_handle_t *h;

	h = ifconfig_open();
	if (!h)
		return WISE_FAIL;

	if (ifconfig_get_hwaddr(h, wlan_if == 0 ? "wlan0" : "wlan1", &sa) < 0) {
		ifconfig_close(h);
		goto fail;
	}

	memcpy(mac, sa.sa_data, 6);

	return WISE_OK;

fail:
	return WISE_FAIL;
}

/**
  * @brief     Get STAs associated with soft-AP
  *
  * @attention SSC only API
  *
  * @param[out] sta  station list
  *             ap can get the connected sta's phy mode info through the struct member
  *             phy_11b，phy_11g，phy_11n，phy_lr in the wifi_sta_info_t struct.
  *             For example, phy_11b = 1 imply that sta support 802.11b mode
  *
  * @return
  *    - WISE_OK: succeed
  *    - WISE_ERR_WIFI_NOT_INIT: WiFi is not initialized by wise_wifi_init
  *    - WISE_ERR_INVALID_ARG: invalid argument
  *    - WISE_ERR_WIFI_MODE: WiFi mode is wrong
  *    - WISE_ERR_WIFI_CONN: WiFi internal error, the station/soft-AP control block is invalid
  */
wise_err_t wise_wifi_ap_get_sta_list(wifi_sta_list_t *sta)
{
	struct wpa_supplicant *wpa_s = wise_wpas_get("wlan0");
	struct hostapd_data *hapd = NULL;
	struct sta_info *sta_list = NULL;
	u8 i = 0;

	if (!wpa_s)
		return WISE_FAIL;

	if (!wpa_s->ap_iface)
		return WISE_FAIL;

	hapd = wpa_s->ap_iface->bss[0];
	if(!hapd)
		return WISE_FAIL;

	sta_list = hapd->sta_list;
	while(sta_list)
	{
		memcpy(sta->sta[i++].mac, sta_list->addr,6);
		sta->num = i;
		sta_list = sta_list->next;
	}

	return WISE_OK;
}

wise_err_t wise_wifi_set_country(const wifi_country_t *country)
{
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;
	struct ieee80211_regdomain regdomain;

	regdomain.regdomain = 0x30;
	regdomain.location = ' ';
	regdomain.ecm = (country->nchan - 11);
	if (strncmp(country->cc, "US", 2) == 0) {
		regdomain.isocc[0] = 'U';
		regdomain.isocc[1] = 'S';
		regdomain.country = CTRY_UNITED_STATES;
	} else if (strncmp(country->cc, "CA", 2) == 0) {
		regdomain.isocc[0] = 'C';
		regdomain.isocc[1] = 'A';
		regdomain.country = CTRY_CANADA;
	} else if (strncmp(country->cc, "JP", 2) == 0) {
		regdomain.isocc[0] = 'J';
		regdomain.isocc[1] = 'P';
		regdomain.country = CTRY_JAPAN;
	} else if (strncmp(country->cc, "KR", 2) == 0) {
		regdomain.isocc[0] = 'K';
		regdomain.isocc[1] = 'R';
		regdomain.country = CTRY_KOREA_ROC;
	} else {
		regdomain.isocc[0] = 'C';
		regdomain.isocc[1] = 'N';
		regdomain.country = CTRY_CHINA;
	}

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (ifconfig_set80211(h, "wlan0",
				IEEE80211_IOC_COUNTRY,
				0,
				sizeof(regdomain), &regdomain) < 0)
		err_exit(err, WISE_FAIL);

exit:
	if (h)
		ifconfig_close(h);

	return err;
}

wise_err_t wise_wifi_get_country(wifi_country_t *country)
{
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;
	struct ieee80211_regdomain regdomain;

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (ifconfig_get80211(h, "wlan0",
				IEEE80211_IOC_COUNTRY,
				&regdomain,
				sizeof(regdomain)) < 0)
		err_exit(err, WISE_FAIL);

	country->nchan = regdomain.ecm + 11;
	country->cc[0] = regdomain.isocc[0];
	country->cc[1] = regdomain.isocc[1];

exit:
	if (h)
		ifconfig_close(h);

	return err;
}

wise_err_t wise_wifi_set_ps(wifi_ps_type_t type)
{
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;
	uint8_t ps = IEEE80211_POWERSAVE_OFF;

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (type == WIFI_PS_NONE)
		ps = IEEE80211_POWERSAVE_OFF;
	else if (type == WIFI_PS_MAX_MODEM)
		ps = IEEE80211_POWERSAVE_ON;
	else if (type == WIFI_PS_MIN_MODEM)
		ps = IEEE80211_POWERSAVE_ON;

	if (ifconfig_set80211(h, "wlan0",
				IEEE80211_IOC_POWERSAVE,
				ps,
				0, NULL) < 0)
		err_exit(err, WISE_FAIL);

exit:
	if (h)
		ifconfig_close(h);


	return err;
}

wise_err_t wise_wifi_get_ps(wifi_ps_type_t *type) {

	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;
	int ps = -1;

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (ifconfig_get80211val(h, "wlan0",
				IEEE80211_IOC_POWERSAVE,
				0,
				&ps) < 0)
		err_exit(err, WISE_FAIL);

	*type = WIFI_PS_NONE;
	if ((ps == IEEE80211_POWERSAVE_CAM) || (ps == IEEE80211_POWERSAVE_PSP) || (ps == IEEE80211_POWERSAVE_PSP_CAM))
		*type = WIFI_PS_MIN_MODEM;
exit:
	if (h)
		ifconfig_close(h);

	return err;
}

wise_err_t wise_wifi_set_max_tx_power(int8_t power, uint8_t wlan_if) {
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;
	struct ieee80211req_sta_txpow txpow;
	wifi_ap_record_t ap_info;
	wifi_sta_list_t sta_list;
	int i = 0;

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (s_wifi_mode[wlan_if] == WIFI_MODE_STA) {

		memset(&ap_info, 0, sizeof(wifi_ap_record_t));
		err = wise_wifi_sta_get_ap_info(&ap_info, wlan_if);

		if (err == WISE_ERR_WIFI_NOT_CONNECT) {
				printf("No AP\r\n");
				return WISE_ERR_INVALID_STATE;
			} else if (err != WISE_OK)
				return WISE_FAIL;

		memcpy(txpow.it_macaddr, ap_info.bssid, 6);
		txpow.it_txpow = power;

		if (ifconfig_set80211(h, "wlan0",
			IEEE80211_IOC_STA_TXPOW,
			0,
			sizeof(txpow), &txpow) < 0)
				err_exit(err, WISE_FAIL);
	}
	else if (s_wifi_mode[wlan_if] == WIFI_MODE_AP){
		wise_wifi_ap_get_sta_list(&sta_list);
		for (i = 0; i < sta_list.num; i++) {
			memcpy(&txpow.it_macaddr[0], &sta_list.sta[i].mac[0], 6);
			txpow.it_txpow = power;
			if (ifconfig_set80211(h, "wlan0",
				IEEE80211_IOC_STA_TXPOW,
				0,
				sizeof(txpow), &txpow) < 0)
					err_exit(err, WISE_FAIL);
		}
	}

exit:
	if (h)
		ifconfig_close(h);

	return err;
}

wise_err_t wise_wifi_get_max_tx_power(int8_t *power, uint8_t wlan_if) {
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;
	struct ieee80211req_sta_txpow txpow;
	wifi_sta_list_t sta_list;
	wifi_ap_record_t ap_info;
	int i;

	h = ifconfig_open();

	if (s_wifi_mode[wlan_if] == WIFI_MODE_STA) {
		memset(&ap_info, 0, sizeof(wifi_ap_record_t));
		err = wise_wifi_sta_get_ap_info(&ap_info, wlan_if);

		if (err == WISE_ERR_WIFI_NOT_CONNECT) {
			printf("No AP\r\n");
			err_exit(err, WISE_ERR_INVALID_STATE);
		} else if (err != WISE_OK)
			err_exit(err, WISE_FAIL);

		memcpy(txpow.it_macaddr, ap_info.bssid, 6);

		if (ifconfig_get80211(h, "wlan0",
				IEEE80211_IOC_STA_TXPOW,
				&txpow,
				sizeof(txpow)) < 0)
				err_exit(err, WISE_FAIL);

		*power = txpow.it_txpow;
	}
	else if (s_wifi_mode[wlan_if] == WIFI_MODE_AP){
		wise_wifi_ap_get_sta_list(&sta_list);
		for (i = 0; i < sta_list.num; i++) {
			memcpy(&txpow.it_macaddr[0], &sta_list.sta[i].mac[0], 6);

			if (ifconfig_get80211(h, "wlan0",
				IEEE80211_IOC_STA_TXPOW,
				&txpow,
				sizeof(txpow)) < 0)
				err_exit(err, WISE_FAIL);

			*power = txpow.it_txpow;
		}
	}


exit:
	if (h)
		ifconfig_close(h);

	return err;
}

wise_err_t wise_wifi_set_beacon_interval(uint32_t beacon_interval)
{
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (ifconfig_set80211(h, "wlan1",
				IEEE80211_IOC_BEACON_INTERVAL,
				beacon_interval, 0, NULL) < 0)
		err_exit(err, WISE_FAIL);

exit:
	if (h)
		ifconfig_close(h);

	return err;
}

wise_err_t wise_wifi_set_dtim_period(uint8_t dtim_period)
{
	wise_err_t err = WISE_OK;
	ifconfig_handle_t *h = NULL;

	h = ifconfig_open();
	if (!h)
		err_exit(err, WISE_FAIL);

	if (ifconfig_set80211(h, "wlan1",
				IEEE80211_IOC_DTIM_PERIOD,
				dtim_period, 0, NULL) < 0)
		err_exit(err, WISE_FAIL);

exit:
	if (h)
		ifconfig_close(h);

	return err;
}
