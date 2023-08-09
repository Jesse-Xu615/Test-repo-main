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

#ifndef __WIFI_API_H__
#define __WIFI_API_H__

#define WPA_FLAG_ON     1
#define WPA_FLAG_OFF    0

#define WPA_MIN_KEY_LEN                 8
#define WPA_MAX_KEY_LEN                 64

#define WPA_MAX_SSID_LEN                32
#define WPA_MAX_ESSID_LEN               WPA_MAX_SSID_LEN

#define WPA_AP_MIN_BEACON               25
#define WPA_AP_MAX_BEACON               1000

#define WPA_AP_MAX_DTIM                 15
#define WPA_AP_MIN_DTIM                 1

#define WPA_MAX_NUM_STA                 CONFIG_SUPPORT_STA_NUM /* only support 1 sta */

#define WPA_24G_CHANNEL_NUMS            14

#define WPA_COUNTRY_CODE_LEN            3
#define WPA_COUNTRY_CODE_USA            "US"
#define WPA_COUNTRY_CODE_JAPAN          "JP"
#define WPA_CHANNEL_MAX_USA             11
#define WPA_CHANNEL_MAX_JAPAN           14
#define WPA_CHANNEL_MAX_OTHERS          13

/**
 * @ingroup scm_wifi_basic
 *
 * default max num of station.
 */
#define WIFI_DEFAULT_MAX_NUM_STA         CONFIG_SUPPORT_STA_NUM


/**
 * @ingroup hi_wifi_basic
 *
 * Length of wpa ssid psk
 */
#define WIFI_STA_PSK_LEN                 32

/**
 * @ingroup scm_wifi_basic
 *
 * max interiface name length.
 */
#define WIFI_IFNAME_MAX_SIZE             16

/**
 * @ingroup scm_wifi_basic
 *
 * The minimum timeout of a single reconnection.
 */
#define WIFI_MIN_RECONNECT_TIMEOUT   2

/**
 * @ingroup scm_wifi_basic
 *
 * The maximum timeout of a single reconnection, representing an infinite number of loop reconnections.
 */
#define WIFI_MAX_RECONNECT_TIMEOUT   65535

/**
 * @ingroup scm_wifi_basic
 *
 * The minimum auto reconnect interval.
 */
#define WIFI_MIN_RECONNECT_PERIOD    1

/**
 * @ingroup scm_wifi_basic
 *
 * The maximum auto reconnect interval.
 */
#define WIFI_MAX_RECONNECT_PERIOD   65535

/**
 * @ingroup scm_wifi_basic
 *
 * The minmum times of auto reconnect.
 */
#define WIFI_MIN_RECONNECT_TIMES    1

/**
 * @ingroup scm_wifi_basic
 *
 * The maximum times of auto reconnect.
 */
#define WIFI_MAX_RECONNECT_TIMES   65535

/**
 * @ingroup scm_wifi_basic
 *
 * max scan number of ap.
 */
#define WIFI_SCAN_AP_LIMIT               32

/**
 * @ingroup scm_wifi_basic
 *
 * Max length of SSID.
 */
#define SCM_WIFI_MAX_SSID_LEN  32

/**
 * @ingroup scm_wifi_basic
 *
 * Length of MAC address.
 */
#define SCM_WIFI_MAC_LEN        6

/**
 * @ingroup scm_wifi_basic
 *
 * Maximum  length of Key.
 */
#define SCM_WIFI_MAX_KEY_LEN    64

/**
 * @ingroup scm_wifi_basic
 *
 * String length of bssid, eg. 00:00:00:00:00:00.
 */
#define SCM_WIFI_ADDR_STR_LEN   17

/**
 * @ingroup scm_wifi_basic
 *
 * Return value of invalid channel.
 */
#define SCM_WIFI_INVALID_CHANNEL 0xFF


struct wifi_reconnect_set {
	int enable;
	unsigned int timeout;
	unsigned int period;
	unsigned int max_try_count;
	unsigned int try_count;
	unsigned int try_freq_scan_count;
	int pending_flag;
	struct wpa_ssid *current_ssid;
};

/**
 * @ingroup scm_wifi_basic
 *
 * Scan type enum.
 */
typedef enum {
    SCM_WIFI_BASIC_SCAN,             /* Common and all channel scan. */
    SCM_WIFI_CHANNEL_SCAN,           /* Specified channel scan. */
    SCM_WIFI_SSID_SCAN,              /* Specified SSID scan. */
    SCM_WIFI_SSID_PREFIX_SCAN,       /* Prefix SSID scan. */
    SCM_WIFI_BSSID_SCAN,             /* Specified BSSID scan. */
} scm_wifi_scan_type;

/**
 * @ingroup scm_wifi_basic
 *
 * Authentification type enum.
 */
typedef enum {
    SCM_WIFI_SECURITY_OPEN,                  /* OPEN. */
    SCM_WIFI_SECURITY_WPAPSK,                /* WPA-PSK. */
    SCM_WIFI_SECURITY_WPA2PSK,               /* WPA2-PSK. */
    SCM_WIFI_SECURITY_SAE,                   /* SAE. */
    SCM_WIFI_SECURITY_UNKNOWN                /* UNKNOWN. */
} scm_wifi_auth_mode;

/**
 * @ingroup scm_wifi_basic
 *
 * Encryption type enum.
 *
 */
typedef enum {
    SCM_WIFI_PARIWISE_UNKNOWN,               /* UNKNOWN.  */
    SCM_WIFI_PAIRWISE_AES,                   /* AES. */
    SCM_WIFI_PAIRWISE_TKIP,                  /* TKIP. */
    SCM_WIFI_PAIRWISE_MAX,                   /* MAX. */
} scm_wifi_pairwise;

/**
 * @ingroup scm_wifi_basic
 *
 * Struct of connect parameters.
 */
typedef struct {
    char ssid[SCM_WIFI_MAX_SSID_LEN + 1];    /* SSID. */
    unsigned int hidden_ap;                  /* Ap is hidden AP */
    scm_wifi_auth_mode auth;                 /* Authentication mode. */
    char key[SCM_WIFI_MAX_KEY_LEN + 1];      /* Secret key. */
    unsigned char bssid[SCM_WIFI_MAC_LEN];   /* BSSID. */
    scm_wifi_pairwise pairwise;              /* Encryption type. */
} scm_wifi_assoc_request;

/**
 * @ingroup scm_wifi_basic
 *
 * Struct of connect parameters.
 */
typedef struct {
    scm_wifi_assoc_request req;              /* Association request */
    unsigned char channel;                  /* AP Channel number  */
    unsigned char psk[WIFI_STA_PSK_LEN]; /* PSK. */
    unsigned int resv;
} scm_wifi_fast_assoc_request;

/**
 * @ingroup scm_wifi_basic
 *
 * parameters of scan.
 */
typedef struct {
    char ssid[SCM_WIFI_MAX_SSID_LEN + 1];    /* SSID. */
    unsigned char bssid[SCM_WIFI_MAC_LEN];   /* BSSID. */
    unsigned char ssid_len;                  /* SSID length. */
    unsigned char channel;                    /* Channel number. */
    scm_wifi_scan_type scan_type;            /* Scan type. */
} scm_wifi_scan_params;

/**
 * @ingroup scm_wifi_basic
 *
 * Type of connect's status.
 */
typedef enum {
    SCM_WIFI_DISCONNECTED,   /* Disconnected. */
    SCM_WIFI_CONNECTED,      /* Connected. */
} scm_wifi_conn_status;

/**
 * @ingroup scm_wifi_basic
 *
 * Status of sta's connection.
 */
typedef struct {
    char ssid[SCM_WIFI_MAX_SSID_LEN + 1];    /* SSID. */
    unsigned char bssid[SCM_WIFI_MAC_LEN];   /* BSSID. */
    int channel;                   /* Channel number. */
    scm_wifi_conn_status status;             /* Connect status. */
} scm_wifi_status;

/**
 * @ingroup scm_wifi_basic
 *
 * Struct of scan result.
 */
typedef struct {
    char ssid[SCM_WIFI_MAX_SSID_LEN + 1];    /* SSID. */
    unsigned char bssid[SCM_WIFI_MAC_LEN];   /* BSSID. */
    unsigned int channel;                    /* Channel number. */
    scm_wifi_auth_mode auth;                 /* Authentication type. */
    int rssi;                                /* Signal Strength. */
    unsigned char resv1 : 1;                 /* Reserved. */
    unsigned char resv2 : 1;                 /* Reserved. */
    unsigned char resv3 : 1;                 /* Reserved. */
    unsigned char resv4 : 1;                 /* Reserved. */
    unsigned char resv5 : 1;                 /* Reserved. */
} scm_wifi_ap_info;

/**
 * @ingroup scm_wifi_basic
 *
 * Struct of softap's basic config.
 *
 */
typedef struct {
    char ssid[SCM_WIFI_MAX_SSID_LEN + 1];    /* SSID. */
    char key[SCM_WIFI_MAX_KEY_LEN + 1];       /* Secret key. */
    unsigned char channel_num;              /* Channel number. */
    int ssid_hidden;                        /* Hidden ssid. */
    scm_wifi_auth_mode authmode;             /* Authentication mode. */
    scm_wifi_pairwise pairwise;              /* Encryption type. */
} scm_wifi_softap_config;

/**
 * @ingroup scm_wifi_basic
 *
 * rate of packet.
 *
 */
typedef union {
        unsigned char   mcs        : 4;      /* rate code. */
        unsigned char   protocol_mode : 4;   /* protocol mode 0x8:mcs 0x4:ax 0x2:ac. */
} scm_wifi_rate_union;

/**
 * @ingroup scm_wifi_basic
 *
 * information of softap's user.
 *
 */
typedef struct {
    unsigned char   mac[SCM_WIFI_MAC_LEN];     /* MAC address. */
    int             rssi;                      /* rssi. */
    scm_wifi_rate_union rate;                  /* data rate code. */
} scm_wifi_ap_sta_info;

int scm_wifi_register_event_callback(system_event_cb_t event_cb, void *priv);

int scm_wifi_system_event_send(system_event_t *evt);

int scm_wifi_unregister_event(void);

void scm_wifi_event_send(void *event, size_t size);

void scm_wifi_get_wlan_mac(uint8_t **mac_addr, int idx);

struct netif * scm_wifi_get_netif(int idx);

int scm_wifi_set_ip(const char *ifname, const char *ip,
                    const char *nm, const char *gw);

int scm_wifi_reset_ip(const char *ifname);

int scm_wifi_sta_start(char *ifname, int *len);

int scm_wifi_sta_stop(void);

int scm_wifi_sta_set_reconnect_policy(int enable, unsigned int seconds,
                                     unsigned int period, unsigned int max_try_count);

#ifdef CONFIG_API_SCAN
int scm_wifi_sta_scan(void);

int scm_wifi_sta_advance_scan(scm_wifi_scan_params *sp);

int scm_wifi_sta_scan_results(scm_wifi_ap_info *ap_list, uint16_t *ap_num, uint16_t max_ap_num);
#endif

int scm_wifi_sta_set_config(scm_wifi_assoc_request *req,  wifi_fast_connect_t *fast_config);

int scm_wifi_sta_connect(void);

int scm_wifi_sta_disconnect(void);

int scm_wifi_sta_get_ap_rssi(void);

int scm_wifi_sta_get_connect_info(scm_wifi_status *connect_status);

int scm_wifi_sta_restore_psk(u8 *psk);

int scm_wifi_sta_fast_connect(scm_wifi_fast_assoc_request *fast_request);

void scm_wifi_sta_dump_ap_info(scm_wifi_status *connect_status);

int scm_wifi_sta_get_psk(u8 *psk, int len);

#ifdef CONFIG_API_SOFTAP
int scm_wifi_sap_start(char *ifname, int *len);

int scm_wifi_sap_stop(void);

int scm_wifi_sap_set_config(scm_wifi_softap_config *sap);

int scm_wifi_sap_get_config(scm_wifi_softap_config *sap);

int scm_wifi_sap_set_beacon_interval(uint32_t interval);

int scm_wifi_sap_set_dtim_period(uint8_t period);

int scm_wifi_sap_get_connected_sta(scm_wifi_ap_sta_info *sta_list, uint8_t *sta_num);

int scm_wifi_sap_deauth_sta (const char *txtaddr, unsigned char addr_len);
#endif

#ifdef __NOT_YET__

int scm_wifi_raw_scan(const char *ifname,
                         scm_wifi_scan_params *custom_scan_param, scm_wifi_scan_no_save_cb cb);

int scm_wifi_enable_intrf_mode(const char* ifname, unsigned char enable, unsigned short flag);

int scm_wifi_enable_anti_microwave_intrf(unsigned char enable);
#endif

#endif /* __WIFI_API_H__ */
