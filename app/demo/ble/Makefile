libble=lib/ble/nimble
ccflags-y += -I$(srctree)/include/FreeRTOS
ccflags-y += -Dstatic_assert=_Static_assert
ccflags-y += -Wno-maybe-uninitialized
ccflags-y += -Wno-unused-function -Wno-unused-const-variable -Wno-unused-variable

ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/nimble/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/porting/npl/wise/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/porting/nimble/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/porting/nimble/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/nimble/host/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/nimble/transport/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/nimble/host/util/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/nimble/host/services/gap/include
ccflags-y += -I$(srctree)/$(libble)/mynewt-nimble/nimble/host/services/gatt/include

obj-y 	   += scm_blecent/main.o scm_blecent/misc.o scm_blecent/peer.o
obj-y 	   += scm_bleprph/main.o scm_bleprph/gatt_svr.o scm_bleprph/misc.o
