/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)   2004 BY ROCK-CHIP FUZHOU
				--  ALL RIGHTS RESERVED  --

File Name:  ftl.h
Author:     XUESHAN LIN
Created:    1st Dec 2008
Modified:
Revision:   1.00
********************************************************************************
********************************************************************************/
#ifndef _FTL_H
#define _FTL_H


//1可配置参数
//#define     IN_LOADER                       //定义编译loader时宏开关, 没定义是系统驱动
#ifndef IN_LOADER
#define     SYS_PROTECT                     //定义系统区写保护使能
#endif
#define     FTL_READ_CACHE
#define     FLASH_READ_REFRESH
//#define     FLASH_READ_REFRESH_DEBUG
//#define     MALLOC_DISABLE
#define     WL_ENABLE
//#define     WL_DEBUG
#define     FTL_POWER_ON_FAST
#define     PAGE_REMAP
#define     EXCH_PAGE_REMAP

#define     MAX_READ_REFLESH_COUNT      10000         //40000 当flash被读了30万次后，检查是否需要refresh
#define     MAX_READ_CHECK_SETP         500          //当读N次后，检查flash是否需要refresh
#define     MAX_READ_CHECK_PAGES        16           //一次检查page数   

#define     FTL_DATE                "20110420"
#define     FTL_VERSION             0x0422  //FTL版本号, 0x100表示ver1.00, 需要低格时修改主版本号, 否则只需修改次版本号 300
#define     MAX_REMAP_TBL           (32768)//最大的映射表, 字节单位, 必须>=1024
#define     MAX_EC_TBL              MAX_REMAP_TBL
#define     MAX_BAD_BLK_NUM         2048    //最大的的坏块交替块数
#define     MAX_CACHE_BLK_NUM       16      //用来作CACHE的块数
#define     MAX_EXCH_BLK_NUM        32       //交换块数
#define     MAX_FREE_BLK_NUM        1024    //用来作扩展块的数量最大限制
#define     MIN_FREE_BLK_NUM        128      //用来作扩展块的数量最小限制
#define     MAX_RSVD_BLK_NUM        4       //保留块数(包含2个坏块表记录块)

#ifdef      PAGE_REMAP
#define     MAX_PAGE_CACHE            8
#define     MAX_PAGE_REMAP_SIZE       128      //page remap size MB
#define     MAX_SMALLPAGE_REMAP_SIZE  48      //page remap size MB
#define     MAX_PAGE_REMAP_PAGE_NUM  (MAX_PAGE_REMAP_SIZE * 256)
#define     MAX_PAGE_REMAP_BLOCK     512 //page remap blk num
#define     MAX_PAGE_REMAP_RECOVERY_BLOCK   MAX_FREE_BLK_NUM
#endif

#define     DISK_NAND_CODE          0
#define     DISK_NAND_DATA          1
#define     DISK_NAND_USER          2
#define     DISK_NAND_TOTAL         0xff

#define     MAX_CACHE               32      //至少要6条
#define     MAX_READ_CACHE          16
#define     MAX_CACHE_CONTINUE_LEN  3

#define     FLASH_PROT_MAGIC        0x444e414e  //NAND

/*******************************************************************
宏常数定义
*******************************************************************/
#define     FTL_OK                  0
#define     FTL_ERROR               -1

#define     FTL_CACHE_NEED_WRITE     0
#define     FTL_CACHE_WRITED         1

#define     SIGN_BAD_BLK            0xf000
#define     SIGN_CACHE_BLK          0xf100
#define     SIGN_DATA_BLK           0xf200
#define     SIGN_RCV_BLK            0xf300
#define     SIGN_REMAP_BLK          0xF400
#define     SIGN_EXCH_PAGE_BLK      0xF500
#define     SIGN_PAGE_BLK           0xf800

#define     WL_MIN_FREE_BLK        16
#define     WL_THRESHOLD           64 

typedef enum tagWL_DATA_STS
{
    WL_HOT_DATA = 0,
    WL_NORMAL_DATA,
    WL_COOL_DATA
}WL_DATA_STS;

//1结构定义
typedef struct tagEXCH_BLK_INFO
{
    uint32	Start;
    uint32	End;
    uint32	SrcAddr;		//源BLOCK地址, 用于关闭或恢复
    uint32  ver;            //版本号, 表示最新数据
    uint8	Count;
    uint8	Valid;
    uint16	LBA;
    uint8   pageRemapFlag;
    uint16	*pageRemap;
}EXCH_BLK_INFO, *pEXCH_BLK_INFO;

typedef struct tagFTL_CACHE
{
    uint32 LBA;
    uint32 Mask;
    uint8 count;
    uint8 Valid;
    uint16  flag;
    uint16 blk;//记录这条cache最后写在哪个block中
    uint32 *Buf;
}FTL_CACHE, *pFTL_CACHE;

typedef struct tagREMAP_INFO
{
    uint16 page[64];
    uint8  num[64];
    uint32 write[2];
    uint16 max;
    uint16 blkAddr;
    uint16 pageAddr;
    uint16 rcvRemapPageAddr; //保留块中记录 page位置，指向下一个用来保存的page(空page)
    uint16 rcvRemapValid;    //保留块中记录的remap数据有效
    uint16 tbl[MAX_REMAP_TBL/2];
}REMAP_INFO, *pREMAP_INFO;

typedef struct tagBAD_BLK_INFO
{
    uint16 max;
    uint16 cnt;
    uint16 offset;
    uint16 blkAddr[4];
    uint16 tbl[MAX_BAD_BLK_NUM];
}BAD_BLK_INFO, *pBAD_BLK_INFO;

typedef struct tagCACHE_BLK_INFO
{
    uint32 curPageAddr;
    uint32 curBlkAddr;
    uint32 ver;
    uint16 tbl[MAX_CACHE_BLK_NUM];
}CACHE_BLK_INFO, *pCACHE_BLK_INFO;

typedef struct tagCIR_QUEUE
{
    uint16 max;
	uint16 front;
	uint16 rear;
	uint16 count;
	uint16 restCount; //休息的blk数
	uint16 arr[MAX_FREE_BLK_NUM];
}CIR_QUEUE, *pCIR_QUEUE;

typedef struct tagREAD_LOG
{
    uint32  RefreshSec;                 //当需要refresh时，记录当前正在检查的sec地址
    uint32  RefreshOffset;              //当需要refresh时，记录当前正在检查的sec地址
    uint32  ReadCount;                  //计数用
    uint16  CountTbl[MAX_REMAP_TBL/2];
}READ_LOG, *pREAD_LOG;

typedef struct tagWL_INFO
{
    uint16 IsWLEn;              //WL enable
    uint16 isNeedSwap;          //true时，需要替换
    uint16 min;                 //最小EC
    uint16 max;                 //最大EC
    uint16 minLevel;            //最小EC门限值
    uint16 maxLevel;            //最大EC门限值
    //uint16 avg;               //平均值
    uint32 total;               //总的EC
    WL_DATA_STS QoutDataSts;         
    uint16 lastWriteLbaBlk;
    uint32 overHead;            //开销，测试用
    uint16 startBlk;            //动态数据起始block
    uint16 endBlk;              //动态数据结束block，即物理结束block
    uint16 totBlk;              //参与计算的block数
    uint16 startLbaBlk;         //LBA起始block
    uint32 LbaTotleCount;
    uint32 LbaAvgCount;
    uint16 blkAddr;   //保存page info的blk地址
    uint16 pageAddr;  //保存page info的page地址
    uint16 maxPage;
    uint16 *tbl;//Erase Count Table  
}WL_INFO, *pWL_INFO;
#ifdef PAGE_REMAP
typedef struct taPR_PAGE_INFO
{
    uint16 blk;
    uint16 page;
}PR_PAGE_INFO, *pPR_PAGE_INFO;

typedef struct tagPR_BLK_INFO
{
    uint16 blk; //blk号，0为没有分配
    uint16 validPages; //这个blk中有效的page数
    uint32 ver;
}PR_BLK_INFO, *pPR_BLK_INFO;


typedef struct taPAGE_REMAP_INFO
{
    uint16 enable;
    uint16 secPerPage;   //FLASH的page size
    uint16 secPerBlk;    //FLASH的block size
    uint16 curPageAddr;   //当前正在编程的page 地址
    uint16 curBlkIndex;  //当前正在编程 的 blk 在blk中index
    uint16 maxBlk;
    uint32 maxPageNum;
    uint32 ver;          //当前blk 的version
    uint32 totleSec;
    PR_BLK_INFO  blk[MAX_PAGE_REMAP_RECOVERY_BLOCK];
    PR_PAGE_INFO page[MAX_PAGE_REMAP_PAGE_NUM];
}PAGE_REMAP_INFO, *pPAGE_REMAP_INFO;
#endif

typedef struct tagEXCH_PAGE_SAVE_INFO
{
    uint16 BlkAddr;   //保存page info的blk地址
    uint16 PageAddr;  //保存page info的page地址
    uint32 Ver;
}EXCH_PAGE_SAVE_INFO;

typedef struct tagFTL_INFO
{
    uint8 valid;    //FTL有效
    uint8 writeCacheEn; //FTL延迟写CACHE是否使能
    uint8 delayWriteCacheEn; //所有cache延时写
    
    uint16 ftlVer;  //FTL版本号
    uint16 ftlSoftVer;  //FTL版本号
        
    uint16 secPerPage;   //FLASH的page size
    uint16 secPerBlk;   //FLASH的block size
    uint16 startPhyBlk; //FLASH作为数据区的起始块
    uint16 maxPhyBlk;   //FLASH作为数据区的最大块
    uint16 totalLogicBlk;
    uint16 totalLogicBlkRaw; 
    
#ifdef PAGE_REMAP
    uint16 pageSaveBlkAddr;   //保存page info的blk地址
    uint16 pageSavePageAddr;  //保存page info的page地址
#endif

#ifdef WL_ENABLE
    WL_INFO  WLInfo;
#endif

#ifdef EXCH_PAGE_REMAP
    EXCH_PAGE_SAVE_INFO ExchPageInfo;
#endif

    CACHE_BLK_INFO cacheBlkInfo;    //CACHE块信息
    EXCH_BLK_INFO exchBlk[MAX_EXCH_BLK_NUM];//交换块
    CIR_QUEUE freeBlkInfo;          //空块表, 当磁盘空间满时至少保证有64块可以滚动
    REMAP_INFO remapInfo;           //块映射表, 随容量大小而变, e.g. 32GB/(1MB/blk)=64KB
    BAD_BLK_INFO badBlkInfo;        //坏块信息

    uint8 cacheSortList[MAX_CACHE+1];
    pFTL_CACHE curCache;
    FTL_CACHE cache[MAX_CACHE];
    
#ifdef FTL_READ_CACHE    
    FTL_CACHE readCache[MAX_READ_CACHE];
#endif

#ifdef FLASH_READ_REFRESH
    READ_LOG readCountInfo;
#endif

#ifdef PAGE_REMAP
    PAGE_REMAP_INFO pageInfo;
#endif
}FTL_INFO, *pFTL_INFO;

//1全局变量
#undef	EXT
#ifdef	IN_FTL
#define	EXT
#else
#define	EXT		extern
#endif
EXT     FTL_INFO gFtlInfo;
EXT     uint32 gCacheBuf[MAX_CACHE][PAGE_LEN];
#ifdef FTL_READ_CACHE    
EXT     uint32 gReadCacheBuf[MAX_READ_CACHE][DATA_LEN/2]; //raw page
#endif

#ifdef EXCH_PAGE_REMAP
EXT    uint32 gExchPageRemapBuf[MAX_EXCH_BLK_NUM*256];//PAGE_LEN
#endif

EXT     uint32 CurEraseBlock;

EXT     uint16 EcTbl[MAX_EC_TBL];//Erase Count Table  
//EXT     uint32 LBACount[MAX_REMAP_TBL/2]; //记录每个lba block擦除次数，调试时用


//1函数原型声明
//FTL.c
extern  int32 FtlInit(uint32 nandcBaseAddr,uint8 pageRemapEn);
extern	int32   FTLInit(void);
extern  uint32  ReadCacheInit(void);
extern	uint16  GetRemapBlk(uint16 LBA, uint16 PBA);
extern	uint32  GetRemap(uint32 LBA, uint8 mod);
extern  uint32  GetUnremap(uint32 PBA);

extern	uint32 	FlashProgError(uint32 secAddr, void *DataBuf, uint16 nSec);

extern	uint32  FtlGetCapacity(uint8 LUN);
extern	int32   FtlRead(uint8 LUN, uint32 Index, uint32 nSec, void *buf);
extern  int32   FtlReadRaw(uint8 LUN, uint32 Index, uint32 nSec, void *buf);
extern	uint32  FtlWritePage(uint32 Index, void *pData, uint16 pages);
extern	int32   FtlWrite(uint8 LUN, uint32 Index, uint32 nSec, void *buf);
extern	void    FtlClose(void);
extern  void    FtlFlashSysProtSetOn(void);
extern  void    FtlFlashSysProtSetOff(void);
extern  void    FtlLowFormatSysDisk(void);
extern  void    PwrOffSaveRemapTbl(uint32 mod);

extern  uint32  FtlBlockCopyProg(uint32 srcSec, uint32 destSec, uint16 nSec, void *pSpare);
extern  uint32  Ftl2FlashProg(uint32 sec, void* buf, void *Spare, uint8 mod);
extern  uint16 GetBlkEcCount(uint16 PBA, uint16 mod);
extern uint32 ExchBlkMatchClose(uint32 LBA);


//1表格定义
#ifdef	IN_FTL
#else
#endif
#endif

