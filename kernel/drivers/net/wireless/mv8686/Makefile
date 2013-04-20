#libertas-objs := main.o wext.o rx.o tx.o cmd.o cmdresp.o scan.o 11d.o	assoc.o

#usb8xxx-objs += if_usb.o
#libertas_cs-objs += if_cs.o
#libertas_sdio-objs += if_sdio.o 
#libertas_spi-objs += if_spi.o

#obj-$(CONFIG_LIBERTAS)     += libertas.o
#obj-$(CONFIG_LIBERTAS_USB) += usb8xxx.o
#obj-$(CONFIG_LIBERTAS_CS)  += libertas_cs.o
#obj-$(CONFIG_LIBERTAS_SDIO) += libertas_sdio.o wifi_power.o
#obj-$(CONFIG_LIBERTAS_SPI) += libertas_spi.o

mv8686-objs := main.o wext.o rx.o tx.o cmd.o cmdresp.o \
               scan.o 11d.o	assoc.o if_sdio.o ../wifi_power/wifi_power_ops.o mv8686_cis.o

obj-$(CONFIG_MV8686)     += mv8686.o ../wifi_power/wifi_power.o ../wifi_power/wifi_config.o
