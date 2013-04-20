/*
 * Definitions for MXC6225 memsic chip.
 */
#ifndef MXC6225_H
#define MXC6225_H

#include <linux/ioctl.h>


/* Default register settings */
#define RBUFF_SIZE		12	/* Rx buffer size */

#define MXC6225_REG_X_OUT      0x0
#define MXC6225_REG_Y_OUT      0x1
#define MXC6225_REG_STATU      0x2
#define MXC6225_REG_TILT       0x3
#define MXC6225_REG_DETECTION  0x4

#define MMAIO				0xA1

/* IOCTLs for MMA7660 library */
#define ECS_IOCTL_INIT                  _IO(MMAIO, 0x01)
#define ECS_IOCTL_RESET      	          _IO(MMAIO, 0x04)
#define ECS_IOCTL_CLOSE		           _IO(MMAIO, 0x02)
#define ECS_IOCTL_START		             _IO(MMAIO, 0x03)
#define ECS_IOCTL_GETDATA               _IOR(MMAIO, 0x08, char[RBUFF_SIZE+1])

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_RATE		_IOW(MMAIO, 0x10, char)



/*rate*/
#define MXC6225_RATE_1          1
#define MXC6225_RATE_2          2
#define MXC6225_RATE_4          4
#define MXC6225_RATE_8          8
#define MXC6225_RATE_16         16
#define MXC6225_RATE_32         32
#define MXC6225_RATE_64         64
#define MXC6225_RATE_120        128

/*status*/
#define MXC6225_OPEN           1
#define MXC6225_CLOSE          0


#define MXC6225_IIC_ADDR 	    0x2a  
#define MXC6225_REG_LEN         11
#define MXC6225_GRAVITY_STEP    23
#define MXC6225_PRECISION       6
#define MXC6225_BOUNDARY        (0x1 << (MXC6225_PRECISION - 1))
#define MXC6225_TOTAL_TIME      10



struct mxc6225_platform_data {
	int reset;
	int clk_on;
	int intr;
};


struct mxc6225_data {
    char  status;
    char  curr_tate;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct work_struct work;
	struct delayed_work delaywork;	/*report second event*/
};

struct mxc6225_axis {
	int x;
	int y;
	int z;
};

#endif

