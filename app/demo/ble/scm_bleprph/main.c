/****************************************************************************
 * apps/examples/scm_bleprph/src/main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_CMDLINE

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include "syscfg/syscfg.h"
#include "nimble/nimble_port.h"
#include "console/console.h"
#if MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0
#include "bootutil/image.h"
#include "imgmgr/imgmgr.h"
#include "services/dis/ble_svc_dis.h"
#endif

/* BLE */

#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

/* Application-specified header. */

#include "bleprph.h"
#include "hal/cmsis/cmsis_os2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t g_dev_index;
static char *g_peer_dev_name;
static bool g_exit;

/**
 * Logs information about a connection to the console.
 */

static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
  MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
              desc->conn_handle, desc->our_ota_addr.type);
  print_addr(desc->our_ota_addr.val);
  MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
              desc->our_id_addr.type);
  print_addr(desc->our_id_addr.val);
  MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
              desc->peer_ota_addr.type);
  print_addr(desc->peer_ota_addr.val);
  MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
              desc->peer_id_addr.type);
  print_addr(desc->peer_id_addr.val);
  MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
              "encrypted=%d authenticated=%d bonded=%d\n",
              desc->conn_itvl, desc->conn_latency,
              desc->supervision_timeout,
              desc->sec_state.encrypted,
              desc->sec_state.authenticated,
              desc->sec_state.bonded);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */

static void
bleprph_advertise(void)
{
  uint8_t own_addr_type;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name;
  int rc;

  /* Figure out address to use while advertising (no privacy for now) */

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0)
    {
      MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
      return;
    }

  /* force the address type */
  own_addr_type = BLE_OWN_ADDR_PUBLIC;

  /**
   *  Set the advertisement data included in our advertisements:
   *     o Flags (indicates advertisement type and other general info).
   *     o Advertising tx power.
   *     o Device name.
   *     o 16-bit service UUIDs (alert notifications).
   */

  memset(&fields, 0, sizeof fields);

  /* Advertise two flags:
   *     o Discoverability in forthcoming advertisement (general)
   *     o BLE-only (BR/EDR unsupported).
   */

  fields.flags = BLE_HS_ADV_F_DISC_GEN |
    BLE_HS_ADV_F_BREDR_UNSUP;

  /* Indicate that the TX power level field should be included; have the
   * stack fill this value automatically.  This is done by assiging the
   * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
   */

  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  name = ble_svc_gap_device_name();
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;

  fields.uuids16 = (ble_uuid16_t[])
    {
      BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    };

  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0)
    {
      MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
      return;
    }

  memset(&fields, 0, sizeof fields);

  fields.mfg_data = (const uint8_t *)"ABCD";
  fields.mfg_data_len = 4;

  rc = ble_gap_adv_rsp_set_fields(&fields);
  if (rc != 0)
    {
      MODLOG_DFLT(ERROR, "error setting scan response data; rc=%d\n", rc);
      return;
    }

  /* Begin advertising. */

  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                         &adv_params, bleprph_gap_event, NULL);
  if (rc != 0)
    {
      MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
      return;
    }

  MODLOG_DFLT(INFO, "advertising started\n");
}

/**
 * The nimble host executes this callback when a GAP event occurs.
 * The application associates a GAP event callback with each connection
 * that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                              bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                              event; nonzero on failure.  The semantics
 *                              of the return code is specific to the
 *                              particular GAP event being signalled.
 */

static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
  struct ble_gap_conn_desc desc;
  int rc;

  switch (event->type)
    {
      case BLE_GAP_EVENT_CONNECT:

        /* A new connection was established or a connection attempt failed. */

        MODLOG_DFLT(INFO, "connection %s; status=%d ",
            event->connect.status == 0 ? "established" : "failed",
            event->connect.status);
        if (event->connect.status == 0)
          {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);

#if MYNEWT_VAL(BLEPRPH_LE_PHY_SUPPORT)
            phy_conn_changed(event->connect.conn_handle);
#endif
          }

        MODLOG_DFLT(INFO, "\n");

        if (event->connect.status != 0)
          {
            /* Connection failed; resume advertising. */

            bleprph_advertise();
          }

        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d ",
                   event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

#if MYNEWT_VAL(BLEPRPH_LE_PHY_SUPPORT)
        phy_conn_changed(CONN_HANDLE_INVALID);
#endif

        /* Connection terminated; resume advertising. */

        bleprph_advertise();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:

        /* The central has updated the connection parameters. */

        MODLOG_DFLT(INFO, "connection updated; status=%d ",
            event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

      case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
            event->adv_complete.reason);
        bleprph_advertise();
        return 0;

      case BLE_GAP_EVENT_ENC_CHANGE:

        /* Encryption has been enabled or disabled for this connection. */

        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
            event->enc_change.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

      case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
            "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
            event->subscribe.conn_handle,
            event->subscribe.attr_handle,
            event->subscribe.reason,
            event->subscribe.prev_notify,
            event->subscribe.cur_notify,
            event->subscribe.prev_indicate,
            event->subscribe.cur_indicate);
        return 0;

      case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
            event->mtu.conn_handle,
            event->mtu.channel_id,
            event->mtu.value);
        return 0;

      case BLE_GAP_EVENT_REPEAT_PAIRING:

        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */

        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host
         * should continue with the pairing operation.
         */

        return BLE_GAP_REPEAT_PAIRING_RETRY;

      case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        MODLOG_DFLT(INFO, "PHY updated; status=%d\n",
            event->phy_updated.status);
        if (event->phy_updated.status == 0)
          {
            MODLOG_DFLT(INFO, "TX PHY %d, RX PHY %d\n",
                        event->phy_updated.tx_phy,
                        event->phy_updated.rx_phy);
          }
        return 0;
    }

  return 0;
}

static void
bleprph_on_reset(int reason)
{
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
bleprph_on_sync(void)
{
  int rc;
  ble_addr_t addr;
  int fd;

  fd = os_open("/ble_hs/rand_addr", O_RDONLY);
  if (fd > 0) {
    printf("random address stored\n");
    addr.type = BLE_ADDR_RANDOM;
    os_read(fd, addr.val, 6);
    os_close(fd);
  } else {

    /* generate new random address */

    rc = ble_hs_id_gen_rnd(0, &addr);
    assert(rc == 0);

    printf("random address generated\n");
    fd = os_open("/ble_hs/rand_addr", O_CREAT | O_WRONLY);
    if (fd > 0) {
      printf("random address saved\n");
      os_write(fd, addr.val, 6);
      os_close(fd);
    }
  }

  /* set generated address */

  rc = ble_hs_id_set_rnd(addr.val);
  assert(rc == 0);

  /* Make sure we have proper identity address set (public preferred) */

  rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);

  MODLOG_DFLT(INFO, "random addr ");
  print_addr(addr.val);
  MODLOG_DFLT(INFO, "\n");

  ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr.val, NULL);

  MODLOG_DFLT(INFO, "public addr ");
  print_addr(addr.val);
  MODLOG_DFLT(INFO, "\n");

  ble_svc_gap_device_name_set((const char *)g_peer_dev_name);

  /* Begin advertising. */

  bleprph_advertise();
}

static struct ble_npl_task s_task_host;
void ble_store_config_init(void);
static void *ble_host_task(void *param)
{
  nimble_port_run();
  return NULL;
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE:\n");
  fprintf(stderr, "\t%s <-n advname] <-d hciindex>\n", progname);
  fprintf(stderr, "where:\n:");
  fprintf(stderr, "\t\tadvname: advertising name (default: xiaohu)\n");
  fprintf(stderr, "\t\thciindex: index of HCI device (default: 0)\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * main
 *
 * The main task for the project. This function initializes the packages,
 * then starts serving events from default event queue.
 *
 * @return int NOTE: this function should never return!
 */

int
bleprph_main(int argc, char *argv[])
{
#if MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0
  struct image_version ver;
  static char ver_str[IMGMGR_NMGR_MAX_VER];
#endif
  int rc;
  int opt;
  bool badarg = false;

  g_dev_index = 0;
  g_peer_dev_name = "xiaohu";
  g_exit = 0;

  while ((opt = getopt(argc, argv, "n:d:")) != -1)
    {
      switch (opt)
        {
        case 'n':
          g_peer_dev_name = optarg;
          break;
        case 'd':
          g_dev_index = atoi(optarg);
          break;
        default:
          fprintf(stderr, "<unknown parameter '-%c'>\n\n", opt);

          /* fall through */

        case '?':
        case ':':
          badarg = true;
        }
    }

  if (badarg)
    {
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

  /* Initialize the NimBLE host configuration. */

  ble_hs_cfg.reset_cb = bleprph_on_reset;
  ble_hs_cfg.sync_cb = bleprph_on_sync;
  ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  rc = gatt_svr_init();
  assert(rc == 0);

#if MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0
  /* Set firmware version in DIS */

  imgr_my_version(&ver);
  imgr_ver_str(&ver, ver_str);
  ble_svc_dis_firmware_revision_set(ver_str);
#endif

  ble_store_config_init();

  ble_npl_task_init(&s_task_host, "ble_host", ble_host_task,
      NULL, 0, 0,
      NULL, 0);

  return 0;
}

#include <cli.h>

CMD(bleprph, bleprph_main, "CLI command for BLE Peripheral", "");

#endif
