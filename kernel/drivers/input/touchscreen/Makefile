#
# Makefile for the touchscreen drivers.
#

## Each configuration option enables a list of files.

#obj-y   += calib_iface.o

obj-$(CONFIG_RK28_GPIO_TS)		+= rk28_gpio_ts.o
obj-$(CONFIG_RK28_SPI_TS)		+= rk28_spi_ts.o
obj-$(CONFIG_RK28_I2C4310_TS)		+= rk28_i2c_ts4310.o
obj-$(CONFIG_TOUCHSCREEN_BITSY)		+= h3600_ts_input.o
obj-$(CONFIG_RK28_I2C_TS_SIS809)      += rk28_i2c_ts_sis809_7.0_v2.o
obj-$(CONFIG_TS_SPI_XPT2046)               += ts_spi_xpt2046.o largenum.o calibration.o calib_iface.o
obj-$(CONFIG_TS_SPI_XPT2046_A7)               += ts_spi_xpt2046_a7.o largenum.o calibration.o calib_iface.o
obj-$(CONFIG_TOUCHSCREEN_ELAN) += elan_touch_i2c.o
