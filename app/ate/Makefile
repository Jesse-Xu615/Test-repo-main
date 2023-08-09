obj-y 		   			   += head.o start.o trap.o main.o uart.o timer.o
obj-$(CONFIG_ATE_PORBOR)   += porbor.o
obj-$(CONFIG_ATE_EFUSE)    += efuse.o
obj-$(CONFIG_ATE_AUXADC)   += auxadc.o
obj-$(CONFIG_ATE_SYSMEM)   += sysmem.o
obj-$(CONFIG_ATE_32KRC)    += osc32krc.o
obj-$(CONFIG_ATE_USBPHY)   += usbphy.o
obj-$(CONFIG_ATE_LOWPOWER) += lowpower.o
obj-$(CONFIG_ATE_SPIFLASH) += spiflash.o
obj-$(CONFIG_ATE_IOLEAKAGE)+= ioleakage.o

hostprogs-y    			   := mkimage
mkimage-objs			   := ../../hal/soc/scm2010/mkimage.o ../../hal/soc/scm2010/crc32.o
always 		               := $(hostprogs-y)

HOST_EXTRACFLAGS += -g
