ccflags-y += -I$(srctree)/include/FreeRTOS
ccflags-y += -I$(srctree)/lib/tinyusb
ccflags-y += -I$(src)

obj-y += main.o
obj-y += hash.o
obj-y += ecdsa.o
obj-$(CONFIG_UART_BOOT) 		+= uart-download.o
obj-$(CONFIG_USB_BOOT)  		+= dfu-download.o
obj-$(CONFIG_SDIO_BOOT)  		+= sdio-download.o sdio-sdvt-br.o

obj-$(CONFIG_FLASH_PROTECTION) 	+= aes-cmac.o
