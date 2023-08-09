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

#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include <wise_err.h>
#include <wise_log.h>
#include <wise_wifi.h>
#include <wise_event_loop.h>
#include <common.h>

#include <scm_wifi.h>

wise_err_t event_handler(void *ctx, system_event_t * event)
{
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		break;
	case SYSTEM_EVENT_STA_STOP:
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		printf("\r\nWIFI GOT IP\r\n");
		break;
	case SYSTEM_EVENT_AP_START:
		printf("\r\nSYSTEM_EVENT_AP_START\r\n");
		break;
	case SYSTEM_EVENT_AP_STOP:
		printf("\r\nSYSTEM_EVENT_AP_STOP\r\n");
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		printf("\r\nSYSTEM_EVENT_AP_STACONNECTED\r\n");
		printf("Connected STA:" MACSTR "\r\n", MAC2STR(event->event_info.sta_connected.mac));
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		printf("\r\nSYSTEM_EVENT_AP_STADISCONNECTED\r\n");
		printf("Disconnected STA:" MACSTR "\r\n", MAC2STR(event->event_info.sta_disconnected.mac));
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		{
			scm_wifi_status connect_status;

			printf("\r\nWIFI CONNECTED\r\n");

			netifapi_dhcp_start(scm_wifi_get_netif(WISE_IF_WIFI_STA));
			scm_wifi_sta_get_connect_info(&connect_status);

			scm_wifi_sta_dump_ap_info(&connect_status);
		}
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		printf("\r\nWIFI DISCONNECT\r\n");
		break;
	case SYSTEM_EVENT_SCAN_DONE:
		printf("WiFi: Scan results available\n");

		break;
	case SYSTEM_EVENT_SCM_CHANNEL:
		printf("WiFi: Scm channel send msg\n");
		scm_wifi_event_send(event, sizeof(system_event_t));
		break;
	default:
		break;
	}

	return WISE_OK;
}

int main(void)
{
	printf("Hello world!\n");

	scm_wifi_register_event_callback(event_handler, NULL);

	return 0;
}
