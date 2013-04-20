/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)   2004 BY ROCK-CHIP FUZHOU
				--  ALL RIGHTS RESERVED  --

File Name:  flash.h
Author:     XUESHAN LIN
Created:    1st Dec 2008
Modified:
Revision:   1.00
NANDC版本:  V1----RK27XXA(8bit/512B ECC)
            V2----RK27XXB、RK280X(8/14bit/512B ECC)
            V3----RK281X(16/24bit/1KB ECC)
********************************************************************************
********************************************************************************/
#ifndef _FLASH_H
#define _FLASH_H

#ifndef LINUX
#define PACKED1 __packed
#define PACKED2
#else
#define PACKED1
#define PACKED2 __attribute__((packed))
#endif


//1可配置参数
//#define     READ_2PLANE         //定义是否使能DUAL-PLANE读操作
//#define     CACHE_PROG        //定义是否使能CACHE编程
#define     DUAL_PLANE          //定义是否使能DUAL PLANE
//#define     INTERLEAVE          //定义是否使能Interleave
//#define     MULTI_CH            //定义是否使能多通道操作

#define     MAX_FLASH_NUM       4                       /*最大支持的FLASH数*/
#define     MAX_REFLESH_LOG     10                      //定义需要更新的地址记录数
#define     DATA_LEN            (8192*2/4)              //数据块单位word
#define     SPARE_LEN           (256*2/4)               //校验数据长度
#define     PAGE_LEN            (DATA_LEN+SPARE_LEN)    //每个数据单位的长度

#define     CHIP_SIGN           0x38324B52              //RK28
#define     FLASH_MAGIC         0x4E414E44

/*******************************************************************
宏常数定义
*******************************************************************/
#define     MIN(x,y) ((x) < (y) ? (x) : (y))
#define     MAX(x,y) ((x) > (y) ? (x) : (y))


/*******************************************************************
厂商ID编码
*******************************************************************/
#define     SAMSUNG             0x00		//三星SAMSUNG
#define     TOSHIBA             0x01		//东芝TOSHIBA
#define     HYNIX               0x02		//海力士HYNIX
#define     INFINEON            0x03		//英飞凌INFINEON
#define     MICRON              0x04		//美光MICRON
#define     RENESAS             0x05		//瑞萨RENESAS
#define     ST                  0x06		//意法半导体ST
#define     INTEL               0x07		//英特尔intel

/*******************************************************************
器件ID编码
*******************************************************************/
#define     Small32M            0x00
#define     Small64M            (Small32M+0x01)
#define     Small128M           (Small32M+0x02)
#define     Small128M_2         (Small32M+0x03)
#define     Small256M           (Small32M+0x04)
#define     SmallPageEnd        Small256M
#define     Large128M           (Small32M+0x05)
#define     Large256M           (Small32M+0x06)
#define     Large512M           (Small32M+0x07)
#define     Large1G             (Small32M+0x08)
#define     Large2G             (Small32M+0x09)


/*******************************************************************
FLASH通过操作命令集(三星)
*******************************************************************/
#define     RESET_CMD               0xff
#define     READ_ID_CMD             0x90
#define     READ_UNIQUE_ID_CMD      0xed
#define     READ_STATUS_CMD         0x70
#define     READ_STATUS_CMD_TOSHIBA 0x71
#define     READ_STATUS_CMD_MICRON  0x78
#define     CHIP1_STATUS_CMD        0xf1
#define     CHIP2_STATUS_CMD        0xf2
#define     PAGE_PROG_CMD           0x8010
#define     CACHE_PROG_CMD          0x8015
#define     PLANE2_PAGE_PROG_CMD    0x8111
#define     INTERLEAVE_PROG_CMD     0x8080
#define     BLOCK_ERASE_CMD         0x60d0

#define     READ_MODE_CMD           0x0057
#define     READ_NORMAL_MODE        0xA1
#define     READ_FASTER_CMD         0xA2
#define     READ_PREREAD_CMD        0xA3

/*******************************************************************
LARGE PAGE FLASH操作命令集(三星)
*******************************************************************/
#define     READ_CMD                0x0030
#define     DUALPLANE_READ_CMD      0x6030
#define     READ_COPY_CMD           0x0035
#define     COPY_PROG_CMD           0x8510
#define     RAND_DATAIN_CMD         0x85
#define     RAND_DATAOUT_CMD        0x05e0
#define     PLANE1_DATAOUT_CMD      0x06e0
/*******************************************************************
SMALL PAGE FLASH操作命令集(三星)
*******************************************************************/
#define     READ0_CMD               0x00
#define     READ1_CMD               0x01
#define     READ_SPARE_CMD          0x50
#define     SMALL_COPY_PROG_CMD     0x8A10

//BCHCTL寄存器
#define     BCH_WR                  0x0002
#define     BCH_RST                 0x0001
//FLCTL寄存器
#define     FL_RDY                  0x1000
#define     FL_COR_EN               0x0800
#define     FL_INT_EN               0x0400
#define     FL_XFER_EN              0x0200
#define     FL_INTCLR_EN            0x0100
#define     FL_START                0x0004
#define     FL_WR                   0x0002
#define     FL_RST                  0x0001

#define     FLASH_PROTECT_ON()      //write_XDATA32(FMCTL, ReadNandReg(FMCTL) & (~0x10))
#define     FLASH_PROTECT_OFF()     //write_XDATA32(FMCTL, ReadNandReg(FMCTL) | 0x10)

/*******************************************************************
FLASH读写接口
*******************************************************************/

//1结构定义
//3寄存器位结构定义
typedef union tagFM_CTL
{
	uint32 d32;
	struct 
	{
		unsigned cs : 4;
		unsigned wp : 1;
		unsigned rdy : 1;
		unsigned reserved6_31 : 26;
	}V2;
	struct 
	{
		unsigned cs : 8;
		unsigned wp : 1;
		unsigned rdy : 1;
		unsigned rdyIntEn : 1;
		unsigned rdyIntClr : 1;
		unsigned reserved12_31 : 20;
	}V3;
}FM_CTL;

typedef union tagFM_WAIT
{
	uint32 d32;
	struct 
	{
		unsigned csrw : 5;
		unsigned rwpw : 6;
		unsigned rdy : 1;
		unsigned rwcs : 4;
		unsigned reserved16_31 : 16;
	}V2;
}FM_WAIT;

typedef union tagFL_CTL
{
	uint32 d32;
	struct 
	{
		unsigned rst : 1;
		unsigned rdn : 1;
		unsigned start : 1;
		unsigned stAddr : 2;
		unsigned trCount : 3;
		unsigned intClr : 1;
		unsigned xfer : 1;
		unsigned intEn : 1;
		unsigned corEn : 1;
		unsigned trRdy : 1;
		unsigned reserved13_31 : 19;
	}V2;
	struct 
	{
		unsigned rst : 1;
		unsigned rdn : 1;
		unsigned start : 1;
		unsigned xfer : 1;
		unsigned stAddr : 1;
		unsigned trCount : 2;
		unsigned rdyIgnore : 1;
		unsigned intClr : 1;
		unsigned intEn : 1;
		unsigned corEn : 1;
		unsigned lbaEn : 1;
		unsigned spareSize : 6;
		unsigned reserved18_19 : 2;
		unsigned trRdy : 1;
		unsigned reserved21_31 : 11;
	}V3;
}FL_CTL;

typedef union tagBCH_CTL
{
	uint32 d32;
	struct 
	{
		unsigned rst : 1;
		unsigned rdn : 1;
		unsigned addrDontCare : 1;
		unsigned addr : 7;
		unsigned region : 2;
		unsigned powerDown : 1;
		unsigned bchMode : 1;       //0-8bit/512B, 1-14bit/512B
		unsigned reserved14_31 : 18;
	}V2;
	struct 
	{
		unsigned rst : 1;
		unsigned rdn : 1;
		unsigned addrDontCare : 1;
		unsigned powerDown : 1;
		unsigned bchMode : 1;       //0-16bit/1KB, 1-24bit/1KB
		unsigned region : 3;
		unsigned addr : 8;
		unsigned reserved16_31 : 16;
	}V3;
}BCH_CTL;

typedef  union tagBCH_ST
{
	uint32 d32;
	struct 
	{
		unsigned errf0 : 1;
		unsigned done0 : 1;
		unsigned fail0 : 1;
		unsigned errBits0 : 4;
		unsigned errf1 : 1;
		unsigned done1 : 1;
		unsigned fail1 : 1;
		unsigned errBits1 : 4;
		unsigned errf2 : 1;
		unsigned done2 : 1;
		unsigned fail2 : 1;
		unsigned errBits2 : 4;
		unsigned errf3 : 1;
		unsigned done3 : 1;
		unsigned fail3 : 1;
		unsigned errBits3 : 4;
		unsigned rdy : 1;
		unsigned cnt : 2;
		unsigned reserved31_31 : 1;
	}V2;
	struct 
	{
		unsigned errf0 : 1;
		unsigned done0 : 1;
		unsigned fail0 : 1;
		unsigned errBits0 : 5;
		unsigned errBitsLow0 : 5;
		unsigned errf1 : 1;
		unsigned done1 : 1;
		unsigned fail1 : 1;
		unsigned errBits1 : 5;
		unsigned errBitsLow1 : 5;
		unsigned rdy : 1;
		unsigned cnt : 1;
		unsigned reserved28_31 : 4;
	}V3;
}BCH_ST;

typedef volatile struct tagCHIP_IF
{
    uint32 data;
    uint32 addr;
    uint32 cmd;
    uint32 RESERVED[0x3d];
}CHIP_IF, *pCHIP_IF;

//NANDC Registers
typedef  union tagNANDC
{
    volatile struct
    {
        uint32 FMCTL;
        uint32 FMWAIT;
        uint32 FLCTL;
        uint32 BCHCTL;
        uint32 RESERVED1[(0xd0-0x10)/4];
        uint32 BCHST;
        uint32 RESERVED2[(0x200-0xd4)/4];
        CHIP_IF chip[8];
        uint32 buf[0x800/4];
        uint32 spare[0x80/4];
    }V2;

    volatile struct 
    {
        uint32 FMCTL; 
        uint32 FMWAIT;
        uint32 FLCTL;
        uint32 BCHCTL; 
        uint32 BCHST; 
        uint32 RESERVED1[(0x200-0x14)/4];   //FLR
        uint32 spare[0x200/4]; 
        uint32 RESERVED2[0x400/4]; 
        CHIP_IF chip[8];
        uint32 buf[0x800/4]; 
    }V3;
}NANDC, *pNANDC;

typedef PACKED1 struct tagFLASH_CMD
{
    uint16  pageRead;        //读命令           0x0030
    uint16  pageProg;        //编程命令         0x8010
    uint16  blockErase;      //擦除命令 
    uint16  status;          //读数据时，读状态寄存器命令

    uint16  dualPlaneRead;   //duplane 读命令   0x6030
    uint16 	dualPlaneProg;   //duplane 编程命令 0x8111
    uint16  dualPlaneErase;  //目前应该没有普通擦除一样的，可以填充-1 
    uint16  dualPlaneStatus; //编程时，读状态寄存器命令

    uint16  interleaveProg;  //Interleave 编程命令 0x8080
    uint16  chip1Status;     //Interleave编程时，读第一片状态寄存器命令 
    uint16  chip2Status;     //Interleave编程时，读第二片状态寄存器命令 

    uint16  cacheProg;       //Cache 编程命令   0x8015    

    uint16  randDataIn;      //0x85
    uint16  randDataOut;     //0x05e0
    uint16  Plane1DataOut;
    uint16  copyProg;
    uint16  readCopy;
}PACKED2 FLASH_CMD;

typedef PACKED1 struct tagFLASH_SPEC
{
    uint8   MulPlane;           //是否支持MultiPlane
    uint8   Interleave;         //是否支持Interleave
    uint8   CacheProg;          //是否支持cache program
    uint8   read2PlaneEn;       //是否使能DUAL-PLANE读操作

    uint8   Large;              //是否LargeBlock
    uint8   Five;               //是否有3个行地址
    uint8 	fstPlaneAddr;       //=0表示PLANE0地址为0, 否则为实际地址
    uint8 	dualPlaneAddrMap;   //=0表示相邻2BLOCK为DUAL-PLANE, 否则为前后一半的容量为DUAL-PLANE

    uint8   ch;                 //同时进行擦写的CHANLE数
    uint8   MLC;                //是否MLC
    uint8 	EccBits;			//ECC比特数
    uint8   AccessTime;         //cycle time

    uint8   Vonder;             //厂商
    uint8   SecPerPage;         //FLASH DualPlane每PAGE扇区数
    uint8   SecPerPageRaw;		//FLASH原始每PAGE扇区数
    uint8   CurReadStatusCmd;   //read starus函数调用时的 flash read status 命令，编程，读写时命令不同
    
    uint32	SecPerBlock;
    uint32	SecPerBlockRaw;
    uint32	PagePerBlock;
    uint32	Die2PageAddr;
    uint32  TotalPhySec;
    FLASH_CMD cmd;
    uint32	phySecs[8];//MAX_FLASH_NUM
    uint32 (*pReadRetrial)(uint32 sec, void *pData, void *pSpare, uint8 nSec);
}PACKED2 FLASH_SPEC, *pFLASH_SPEC;

typedef struct _FLASH_DISK_INFO
{
    uint32 	diskCap[2];     //系统程序/数据盘容量
    uint16 	rsvdBlks;
    uint32  sysProtMagic;
    uint32 	sysProtAddr;
}FLASH_DISK_INFO, *pFLASH_DISK_INFO;

typedef struct _FLASH_PEND_CMD
{
    uint8   Valid;          //指示是否有效
    uint16  contProgPages;  //连续编程PAGE数计数
    uint16  Cmd;            //FLASH命令码
    uint32  nextReadSec;    //预读物理扇区地址
    uint32  lastSec;        //上一次擦写扇区
    uint8*  Buf;            //缓冲区
    uint16  Len;            //扇区数
}FLASH_PEND_CMD, *pFLASH_PEND_CMD;

typedef struct _NFLASH_INFO
{
    uint32 ver;     //NANDC version
    pNANDC nandc;       //NANDC寄存器
    FLASH_SPEC spec;
    FLASH_PEND_CMD pendCmd;
    FLASH_DISK_INFO sysInfo;
    uint32 	refleshSec[MAX_REFLESH_LOG];
}NFLASH_INFO, *pNFLASH_INFO;

/*ID BLOCK SECTOR 0 INFO*/
typedef PACKED1 struct tagIDSEC0
{
    uint32  magic;              //0x0ff0aa55, MASKROM限定不能更改
    uint8   reserved[8];
    uint16  bootCodeOffset1;    //中间件扇区偏移, MASKROM限定不能更改
    uint16  bootCodeOffset2;    //第二份中间件偏移
    uint8   reserved1[508-16];
    uint16  bootCodeSize;       //以SEC表示的中间件大小, MASKROM限定不能更改
    uint16  crc16;              //整个ID SEC的前512－2字节数据的CRC16
}PACKED2 IDSEC0, *pIDSEC0;

/*ID BLOCK SECTOR 1 INFO*/
typedef PACKED1 struct tagIDSEC1
{
    uint16  sysAreaBlockRaw;        //系统保留块, 以原始Block为单位
    uint16  sysProgDiskCapacity;    //系统固件盘容量, 以MB为单位
    uint16  sysDataDiskCapacity;    //系统参数盘容量, 以MB为单位
    uint16  reserved0[2];           //保留部分填0
    uint16  chipSignL;              // 28
    uint16  chipSignH;              // RK
    uint16  MachineUIDL;            //机型UID,升级时匹配固件用
    uint16  MachineUIDH;
    uint16  year;                   //中间件生成日期年
    uint16  data;                   //中间件生成日期月日
    uint16  mainVer;                //主版本号
    uint8   reserved1[96-24];      //保留部分填0
    uint16  flashInfoSec;
    uint16  flashInfoSize;
    uint8   reserved2[484-100];      //保留部分填0
    uint32  flashSize;
    uint8   reserved3;
    uint8   accessTime;             //读写cycle时间, ns
    uint16  blockSize;              //以SEC表示的BLOCK SIZE
    uint8   pageSize;               //以SEC表示的PAGE SIZE
    uint8   eccBits;                //需要的ECC BIT数, eg. 8/14
    uint16  bStartEIB;
    uint16  bEndEIB;
    uint16  bStartRB;
    uint16  bEndRB;
    uint16  idBlk[5];                //ID块位置
}PACKED2 IDSEC1, *pIDSEC1;

typedef PACKED1  struct  _FLASH_INFO//需要加__packed或着声明时4对齐不然程序可能在有判断的时候出现异常
{
//和原来系统兼容，提供这些信息，大小为16个byte，实际使用11个byte
    int32  FlashSize;          //（Sector为单位）   4Byte
    int16  BlockSize;          //（Sector为单位）   2Byte
    int8   PageSize;           // (Sector为单位）    1Byte
    int8   ECCBits;            //（bits为单位）    1Byte
    int8   AccessTime;
    int8   ManufacturerName;   // 1Byte
    int8   FlashMask;          // 每一bit代表那个片选是否有FLASH
    int8   reserved0[5];        //保留扩展用，填充 0
// flash 参数  起始16
    uint32   magic;              //NAND  0x4E414E44
    uint16   ver;                //版本  1.00
    uint16   lenth;              //有效数据长度
    FLASH_SPEC spec;
    uint32  validDataEnd;        //用了判断有效数据长度
    uint8   reserved1[256];      //保留扩展用，填充 0
}PACKED2 FLASH_INFO, *pFLASH_INFO;

//1全局变量
#undef	EXT
#ifdef	IN_FLASH
#define	EXT
#else
#define	EXT		extern
#endif
EXT		NFLASH_INFO gFlashInfo;
EXT     FLASH_INFO gFlashIDInfo;
EXT     unsigned long gIdDataBuf[512];

//1函数原型声明
//flash.c
extern	void    FlashTimingCfg(uint32 AHBnKHz);
extern  void    FlashSysProtSet(uint32);
extern	uint32  FlashWaitBusy(uint32 sec, uint32 *secReplace);
extern	uint32  FlashInit(uint32 nandcBaseAddr);
extern	uint32  FlashReadEnhanced(uint32 sec, void *pData, void *pSpare, uint8 nSec, uint32 nextSec);
extern	uint32  FlashProgPage(uint32 sec, void *pData, void *pSpare, uint8 nSec);
extern	uint32  FlashProgEnhanced(uint32 sec, void* buf, void* spare, uint8 mod);
extern	uint32  FlashBlockErase(uint32 sec, uint32 *replaceSec);
extern	uint32  FlashCopyProg(uint32 srcSec, uint32 destSec, uint16 nSec, void *pSpare);
extern  uint32 SamsungReadRetrial(uint32 sec, void *pData, void *pSpare, uint8 nSec);
extern  uint32 HynixReadRetrial(uint32 sec, void *pData, void *pSpare, uint8 nSec);
extern  void HynixGetReadRetryDefault(void);


//1表格定义
#ifdef	IN_FLASH
/*******************************
厂商ID表
********************************/
uint8 ManufactureIDTbl[]=
{
    0xec,					//三星SAMSUNG
    0x98,					//东芝TOSHIBA
    0xad,					//海力士HYNIX
    0xc1,					//英飞凌INFINEON
    0x2c,					//美光MICRON
    0x07,					//瑞萨RENESAS
    0x20,					//意法半导体ST
    0x89					//英特尔intel
};

/*******************************
器件ID表
********************************/
uint8 DeviceCode[]=
{
    0x75,                   //small 8bit-32MB
    0x76,					//small 8bit-64MB
    0x78,                   //small 8bit-128M
    0x79,					//small 8bit-128MB
    0x71,                   //small 8bit-256MB
    0xF1,					//large 8bit-128M
    0xD1,                   //large 8bit-128M
    0xDA,					//large 8bit-256M
    0xDC,					//large 8bit-512M
    0xD3,					//large 8bit-1G
    0xD5,					//large 8bit-2G
    0xAA,                   //large 8bit-256M 1.8V
    0xAC,                   //large 8bit-512M 1.8V
    0xA3,                   //large 8bit-1G 1.8V
    0xA5,					//large 8bit-2G 1.8V
    0xD7,					//large 8bit-4G
    0xD9,					//large 8bit-8G
    0x48,					//large 8bit-2G
    0x68,                   //large 8bit-4G
    0x88,                   //large 8bit-8G
    0xA8,					//large 8bit-16G
    0xDE,                   //PBA 8bit-4GB/CE ,samsung and hynix 8GB
    0x3A,                   //PBA 8bit-8GB/CE
    0x3C,                   //PBA 8bit-16GB/CE
};

/*******************************
器件信息表
********************************/
uint32 DeviceInfo[]=		//phisical sectors
{
    0x10000, 					// DI_32M,	small page
    0x20000, 					// DI_64M,	small page
    0x40000, 					// DI_128M,	small page
    0x40000, 					// DI_128M,	small page
    0x80000,  					// DI_256M,	small page
    0x40000, 					// DI_128M,	large page
    0x40000, 					// DI_128M,	large page
    0x80000,  					// DI_256M,	large page
    0x100000,					// DI_512M,	large page
    0x200000,					// DI_1G,   large page
    0x400000,					// DI_2G,   large page
    0x80000,                    // DI_256M, large page 1.8V
    0x100000,                   // DI_512M, large page
    0x200000,                   // DI_1G,   large page
    0x400000,                   // DI_2G,   large page
    0x800000,					// DI_4G,   large page
    0x1000000,					// DI_8G,   large page
    0x400000,					// DI_2G,   large page
    0x800000,					// DI_4G,   large page
    0x1000000,					// DI_8G,   large page
    0x2000000,					// DI_16G,  large page
    0x1000000,					// DI_4G,   large page
    0x1000000,					// DI_8G,   large page
    0x2000000,					// DI_8G,   large page
};

#if 0
const char* VonderDesc[]=
{
    "Samsung",
    "Toshiba",
    "Hynix",
    "Infineon",
    "Micron",
    "Renesas",
    "ST",
    "Intel"
};
#endif
#else
extern	uint8 	ManufactureIDTbl[];
extern	uint8 	DeviceCode[];
extern	uint32	DeviceInfo[];
#endif
#endif

