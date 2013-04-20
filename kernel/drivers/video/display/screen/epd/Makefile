obj-$(CONFIG_LCD_RK_EINK)+= eink_s.o 
obj-$(CONFIG_LCD_RK_EINK)+= wfzip.o
obj-$(CONFIG_LCD_RK_EINK)+= rgb565_2_luma0.o
obj-$(CONFIG_EINK_BATTERY_CHECK)+=eink_load_init.o
obj-$(CONFIG_EPD_VOLTAGE_CTL) += epd_voltage.o
obj-$(CONFIG_EPD_TI_65185) += tps65185.o
obj-$(CONFIG_LCD_RK_EINK)+= rk28_gpiolib.o 
obj-$(CONFIG_EPD_LM75) += epd_lm75_sensor.o
obj-$(CONFIG_EPD_SN1086) += epd_sn1086_sensor.o
obj-$(CONFIG_EPD_NTC) += ntc_sensor.o
obj-$(CONFIG_WAVEFORM_FROM_EEPROM) += spi_flash.o
$(obj)/eink_s.o: $(obj)/eink_s.uu
	uudecode $(obj)/eink_s.uu -o $(obj)/eink_s.o
$(obj)/wfzip.o: $(obj)/wfzip.uu
	uudecode $(obj)/wfzip.uu -o $(obj)/wfzip.o