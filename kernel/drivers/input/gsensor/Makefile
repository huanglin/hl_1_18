# gsensor drivers

#ifeq ($(CONFIG_MACH_RK2808SDK),y)
ifdef CONFIG_RK2818_SDK
obj-$(CONFIG_GS_MMA7660) 	+= mma7660.o
obj-$(CONFIG_GS_MXC6225) 	+= mxc6225.o
endif
ifeq ($(CONFIG_MACH_PWS700AA),y)
obj-$(CONFIG_GS_MMA7660) 	+= mma7660.o
endif
ifeq ($(CONFIG_MACH_LANMO_W7),y)
obj-$(CONFIG_GS_MMA7660) 	+= lanmow7_gsensor.o
endif

