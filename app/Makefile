ccflags-y += -I$(srctree)/include/freebsd -I$(srctree)/lib/net80211 -I$(srctree)/lib/lwip/src/include -I$(srctree)/lib/lwip/ports/freertos/include
ccflags-y += -I$(srctree)/include/FreeRTOS
ccflags-$(CONFIG_DEMO_SIMPLE_WIFI) += -I$(srctree)/api/include
ccflags-$(CONFIG_WATCHER) += -I$(srctree)/hal/drivers/wlan/senscomm
ccflags-$(CONFIG_IRQ_DISPATCHER) += -I$(srctree)/hal/drivers/sdio

obj-$(CONFIG_BOOTLOADER)   += boot/main.o
obj-$(CONFIG_WATCHER)      += watcher/main.o watcher/trap.o watcher/start.o watcher/reset.o watcher/uart.o watcher/rtc.o watcher/dtimparse.o watcher/systimer.o
obj-$(CONFIG_IRQ_DISPATCHER)   += irqd/main.o irqd/trap.o irqd/start.o irqd/reset.o irqd/uart.o irqd/sdio.o
obj-$(CONFIG_DEMO_CMDLINE) += demo/cli/main.o
obj-$(CONFIG_DEMO_SIMPLE_WIFI) += demo/simple_wifi/main.o
obj-$(CONFIG_TEST_PBUF)    += test/pbuf-test.o
obj-$(CONFIG_TEST_CALLOUT) += test/callout-test.o
obj-$(CONFIG_TEST_TASKQ)   += test/taskq-test.o
obj-$(CONFIG_MCUBOOT_BL)   += mcuboot/main.o
obj-$(CONFIG_ATE) 		   += ate/

obj-$(CONFIG_BLE_NIMBLE_HOST) += demo/ble/

obj-$(CONFIG_BOOTROM) += rom/
