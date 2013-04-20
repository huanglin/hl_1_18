/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)   2004 BY ROCK-CHIP FUZHOU
				--  ALL RIGHTS RESERVED  --

File Name:  flash.h
Author:     XUESHAN LIN
Created:    1st Dec 2008
Modified:
Revision:   1.00
NANDC�汾:  V1----RK27XXA(8bit/512B ECC)
            V2----RK27XXB��RK280X(8/14bit/512B ECC)
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


//1�����ò���
//#define     READ_2PLANE         //�����Ƿ�ʹ��DUAL-PLANE������
//#define     CACHE_PROG        //�����Ƿ�ʹ��CACHE���
#define     DUAL_PLANE          //�����Ƿ�ʹ��DUAL PLANE
//#define     INTERLEAVE          //�����Ƿ�ʹ��Interleave
//#define     MULTI_CH            //�����Ƿ�ʹ�ܶ�ͨ������

#define     MAX_FLASH_NUM       4                       /*���֧�ֵ�FLASH��*/
#define     MAX_REFLESH_LOG     10                      //������Ҫ���µĵ�ַ��¼��
#define     DATA_LEN            (8192*2/4)              //���ݿ鵥λword
#define     SPARE_LEN           (256*2/4)               //У�����ݳ���
#define     PAGE_LEN            (DATA_LEN+SPARE_LEN)    //ÿ�����ݵ�λ�ĳ���

#define     CHIP_SIGN           0x38324B52              //RK28
#define     FLASH_MAGIC         0x4E414E44

/*******************************************************************
�곣������
*******************************************************************/
#define     MIN(x,y) ((x) < (y) ? (x) : (y))
#define     MAX(x,y) ((x) > (y) ? (x) : (y))


/*******************************************************************
����ID����
*******************************************************************/
#define     SAMSUNG             0x00		//����SAMSUNG
#define     TOSHIBA             0x01		//��֥TOSHIBA
#define     HYNIX               0x02		//����ʿHYNIX
#define     INFINEON            0x03		//Ӣ����INFINEON
#define     MICRON              0x04		//����MICRON
#define     RENESAS             0x05		//����RENESAS
#define     ST                  0x06		//�ⷨ�뵼��ST
#define     INTEL               0x07		//Ӣ�ض�intel

/*******************************************************************
����ID����
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
FLASHͨ���������(����)
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
LARGE PAGE FLASH�������(����)
*******************************************************************/
#define     READ_CMD                0x0030
#define     DUALPLANE_READ_CMD      0x6030
#define     READ_COPY_CMD           0x0035
#define     COPY_PROG_CMD           0x8510
#define     RAND_DATAIN_CMD         0x85
#define     RAND_DATAOUT_CMD        0x05e0
#define     PLANE1_DATAOUT_CMD      0x06e0
/*******************************************************************
SMALL PAGE FLASH�������(����)
*******************************************************************/
#define     READ0_CMD               0x00
#define     READ1_CMD               0x01
#define     READ_SPARE_CMD          0x50
#define     SMALL_COPY_PROG_CMD     0x8A10

//BCHCTL�Ĵ���
#define     BCH_WR                  0x0002
#define     BCH_RST                 0x0001
//FLCTL�Ĵ���
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
FLASH��д�ӿ�
*******************************************************************/

//1�ṹ����
//3�Ĵ���λ�ṹ����
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
    uint16  pageRead;        //������           0x0030
    uint16  pageProg;        //�������         0x8010
    uint16  blockErase;      //�������� 
    uint16  status;          //������ʱ����״̬�Ĵ�������

    uint16  dualPlaneRead;   //duplane ������   0x6030
    uint16 	dualPlaneProg;   //duplane ������� 0x8111
    uint16  dualPlaneErase;  //ĿǰӦ��û����ͨ����һ���ģ��������-1 
    uint16  dualPlaneStatus; //���ʱ����״̬�Ĵ�������

    uint16  interleaveProg;  //Interleave ������� 0x8080
    uint16  chip1Status;     //Interleave���ʱ������һƬ״̬�Ĵ������� 
    uint16  chip2Status;     //Interleave���ʱ�����ڶ�Ƭ״̬�Ĵ������� 

    uint16  cacheProg;       //Cache �������   0x8015    

    uint16  randDataIn;      //0x85
    uint16  randDataOut;     //0x05e0
    uint16  Plane1DataOut;
    uint16  copyProg;
    uint16  readCopy;
}PACKED2 FLASH_CMD;

typedef PACKED1 struct tagFLASH_SPEC
{
    uint8   MulPlane;           //�Ƿ�֧��MultiPlane
    uint8   Interleave;         //�Ƿ�֧��Interleave
    uint8   CacheProg;          //�Ƿ�֧��cache program
    uint8   read2PlaneEn;       //�Ƿ�ʹ��DUAL-PLANE������

    uint8   Large;              //�Ƿ�LargeBlock
    uint8   Five;               //�Ƿ���3���е�ַ
    uint8 	fstPlaneAddr;       //=0��ʾPLANE0��ַΪ0, ����Ϊʵ�ʵ�ַ
    uint8 	dualPlaneAddrMap;   //=0��ʾ����2BLOCKΪDUAL-PLANE, ����Ϊǰ��һ�������ΪDUAL-PLANE

    uint8   ch;                 //ͬʱ���в�д��CHANLE��
    uint8   MLC;                //�Ƿ�MLC
    uint8 	EccBits;			//ECC������
    uint8   AccessTime;         //cycle time

    uint8   Vonder;             //����
    uint8   SecPerPage;         //FLASH DualPlaneÿPAGE������
    uint8   SecPerPageRaw;		//FLASHԭʼÿPAGE������
    uint8   CurReadStatusCmd;   //read starus��������ʱ�� flash read status �����̣���дʱ���ͬ
    
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
    uint32 	diskCap[2];     //ϵͳ����/����������
    uint16 	rsvdBlks;
    uint32  sysProtMagic;
    uint32 	sysProtAddr;
}FLASH_DISK_INFO, *pFLASH_DISK_INFO;

typedef struct _FLASH_PEND_CMD
{
    uint8   Valid;          //ָʾ�Ƿ���Ч
    uint16  contProgPages;  //�������PAGE������
    uint16  Cmd;            //FLASH������
    uint32  nextReadSec;    //Ԥ������������ַ
    uint32  lastSec;        //��һ�β�д����
    uint8*  Buf;            //������
    uint16  Len;            //������
}FLASH_PEND_CMD, *pFLASH_PEND_CMD;

typedef struct _NFLASH_INFO
{
    uint32 ver;     //NANDC version
    pNANDC nandc;       //NANDC�Ĵ���
    FLASH_SPEC spec;
    FLASH_PEND_CMD pendCmd;
    FLASH_DISK_INFO sysInfo;
    uint32 	refleshSec[MAX_REFLESH_LOG];
}NFLASH_INFO, *pNFLASH_INFO;

/*ID BLOCK SECTOR 0 INFO*/
typedef PACKED1 struct tagIDSEC0
{
    uint32  magic;              //0x0ff0aa55, MASKROM�޶����ܸ���
    uint8   reserved[8];
    uint16  bootCodeOffset1;    //�м������ƫ��, MASKROM�޶����ܸ���
    uint16  bootCodeOffset2;    //�ڶ����м��ƫ��
    uint8   reserved1[508-16];
    uint16  bootCodeSize;       //��SEC��ʾ���м����С, MASKROM�޶����ܸ���
    uint16  crc16;              //����ID SEC��ǰ512��2�ֽ����ݵ�CRC16
}PACKED2 IDSEC0, *pIDSEC0;

/*ID BLOCK SECTOR 1 INFO*/
typedef PACKED1 struct tagIDSEC1
{
    uint16  sysAreaBlockRaw;        //ϵͳ������, ��ԭʼBlockΪ��λ
    uint16  sysProgDiskCapacity;    //ϵͳ�̼�������, ��MBΪ��λ
    uint16  sysDataDiskCapacity;    //ϵͳ����������, ��MBΪ��λ
    uint16  reserved0[2];           //����������0
    uint16  chipSignL;              // 28
    uint16  chipSignH;              // RK
    uint16  MachineUIDL;            //����UID,����ʱƥ��̼���
    uint16  MachineUIDH;
    uint16  year;                   //�м������������
    uint16  data;                   //�м��������������
    uint16  mainVer;                //���汾��
    uint8   reserved1[96-24];      //����������0
    uint16  flashInfoSec;
    uint16  flashInfoSize;
    uint8   reserved2[484-100];      //����������0
    uint32  flashSize;
    uint8   reserved3;
    uint8   accessTime;             //��дcycleʱ��, ns
    uint16  blockSize;              //��SEC��ʾ��BLOCK SIZE
    uint8   pageSize;               //��SEC��ʾ��PAGE SIZE
    uint8   eccBits;                //��Ҫ��ECC BIT��, eg. 8/14
    uint16  bStartEIB;
    uint16  bEndEIB;
    uint16  bStartRB;
    uint16  bEndRB;
    uint16  idBlk[5];                //ID��λ��
}PACKED2 IDSEC1, *pIDSEC1;

typedef PACKED1  struct  _FLASH_INFO//��Ҫ��__packed��������ʱ4���벻Ȼ������������жϵ�ʱ������쳣
{
//��ԭ��ϵͳ���ݣ��ṩ��Щ��Ϣ����СΪ16��byte��ʵ��ʹ��11��byte
    int32  FlashSize;          //��SectorΪ��λ��   4Byte
    int16  BlockSize;          //��SectorΪ��λ��   2Byte
    int8   PageSize;           // (SectorΪ��λ��    1Byte
    int8   ECCBits;            //��bitsΪ��λ��    1Byte
    int8   AccessTime;
    int8   ManufacturerName;   // 1Byte
    int8   FlashMask;          // ÿһbit�����Ǹ�Ƭѡ�Ƿ���FLASH
    int8   reserved0[5];        //������չ�ã���� 0
// flash ����  ��ʼ16
    uint32   magic;              //NAND  0x4E414E44
    uint16   ver;                //�汾  1.00
    uint16   lenth;              //��Ч���ݳ���
    FLASH_SPEC spec;
    uint32  validDataEnd;        //�����ж���Ч���ݳ���
    uint8   reserved1[256];      //������չ�ã���� 0
}PACKED2 FLASH_INFO, *pFLASH_INFO;

//1ȫ�ֱ���
#undef	EXT
#ifdef	IN_FLASH
#define	EXT
#else
#define	EXT		extern
#endif
EXT		NFLASH_INFO gFlashInfo;
EXT     FLASH_INFO gFlashIDInfo;
EXT     unsigned long gIdDataBuf[512];

//1����ԭ������
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


//1�����
#ifdef	IN_FLASH
/*******************************
����ID��
********************************/
uint8 ManufactureIDTbl[]=
{
    0xec,					//����SAMSUNG
    0x98,					//��֥TOSHIBA
    0xad,					//����ʿHYNIX
    0xc1,					//Ӣ����INFINEON
    0x2c,					//����MICRON
    0x07,					//����RENESAS
    0x20,					//�ⷨ�뵼��ST
    0x89					//Ӣ�ض�intel
};

/*******************************
����ID��
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
������Ϣ��
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

