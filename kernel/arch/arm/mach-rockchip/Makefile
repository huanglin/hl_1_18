# Makefile for the linux kernel
# arch/arm/mach-rockchip/Makefile
# copyright	: ROCKCHIP Inc
# author	: WQQ
# date 		: 2009-04-19
# log			:
#		CONFIG_ARCH_ROCKCHIP : define rockchip arch
#		CONFIG_CHIP_RK2818	 : define chip rk281x.
#		CONFIG_RK2818_A7		 : define product A7.

#Object file lists.
obj-y	:= gpio.o iomux.o rk28_irq.o rk28_timer.o intc.o rk28_debug_uart.o
obj-y += adc.o
obj-y += rk28_dma.o rk28_sdram.o
obj-y += rk28_ddr.o
obj-y += lm.o
obj-$(CONFIG_RK2818_A7) +=   rk28_devices.o 
obj-$(CONFIG_RK2818_A7_V1_1) +=   rk28_devices.o
obj-$(CONFIG_RK2818_SDK) +=   rk28_devices.o
obj-$(CONFIG_CHIP_RK2818) += rk28_pm.o board-rk28.o
obj-$(CONFIG_CPU_FREQ_RK2818)       += cpufreq.o
#$(obj)/rk28_ddr.o: $(obj)/rk28_ddr.uu
#	uudecode $(obj)/rk28_ddr.uu -o $(obj)/rk28_ddr.o
