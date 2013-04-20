#
# linux/drivers/rknand/Makefile
#
# $Id: Makefile,v 1.3 2011/01/21 10:12:56 Administrator Exp $
#
obj-$(CONFIG_MTD_NAND_RK28XX)		+= rknand_base_ko.o 
obj-$(CONFIG_MTD_RKNAND_BUFFER)		+= rk28xxnand_ko.o

$(obj)/rk28xxnand_ko.o: $(obj)/rk28xxnand_ko.uu
	uudecode $(obj)/rk28xxnand_ko.uu -o $(obj)/rk28xxnand_ko.o
