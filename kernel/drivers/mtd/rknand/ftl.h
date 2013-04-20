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


//1�����ò���
//#define     IN_LOADER                       //�������loaderʱ�꿪��, û������ϵͳ����
#ifndef IN_LOADER
#define     SYS_PROTECT                     //����ϵͳ��д����ʹ��
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

#define     MAX_READ_REFLESH_COUNT      10000         //40000 ��flash������30��κ󣬼���Ƿ���Ҫrefresh
#define     MAX_READ_CHECK_SETP         500          //����N�κ󣬼��flash�Ƿ���Ҫrefresh
#define     MAX_READ_CHECK_PAGES        16           //һ�μ��page��   

#define     FTL_DATE                "20110420"
#define     FTL_VERSION             0x0422  //FTL�汾��, 0x100��ʾver1.00, ��Ҫ�͸�ʱ�޸����汾��, ����ֻ���޸Ĵΰ汾�� 300
#define     MAX_REMAP_TBL           (32768)//����ӳ���, �ֽڵ�λ, ����>=1024
#define     MAX_EC_TBL              MAX_REMAP_TBL
#define     MAX_BAD_BLK_NUM         2048    //���ĵĻ��齻�����
#define     MAX_CACHE_BLK_NUM       16      //������CACHE�Ŀ���
#define     MAX_EXCH_BLK_NUM        32       //��������
#define     MAX_FREE_BLK_NUM        1024    //��������չ��������������
#define     MIN_FREE_BLK_NUM        128      //��������չ���������С����
#define     MAX_RSVD_BLK_NUM        4       //��������(����2��������¼��)

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

#define     MAX_CACHE               32      //����Ҫ6��
#define     MAX_READ_CACHE          16
#define     MAX_CACHE_CONTINUE_LEN  3

#define     FLASH_PROT_MAGIC        0x444e414e  //NAND

/*******************************************************************
�곣������
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

//1�ṹ����
typedef struct tagEXCH_BLK_INFO
{
    uint32	Start;
    uint32	End;
    uint32	SrcAddr;		//ԴBLOCK��ַ, ���ڹرջ�ָ�
    uint32  ver;            //�汾��, ��ʾ��������
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
    uint16 blk;//��¼����cache���д���ĸ�block��
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
    uint16 rcvRemapPageAddr; //�������м�¼ pageλ�ã�ָ����һ�����������page(��page)
    uint16 rcvRemapValid;    //�������м�¼��remap������Ч
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
	uint16 restCount; //��Ϣ��blk��
	uint16 arr[MAX_FREE_BLK_NUM];
}CIR_QUEUE, *pCIR_QUEUE;

typedef struct tagREAD_LOG
{
    uint32  RefreshSec;                 //����Ҫrefreshʱ����¼��ǰ���ڼ���sec��ַ
    uint32  RefreshOffset;              //����Ҫrefreshʱ����¼��ǰ���ڼ���sec��ַ
    uint32  ReadCount;                  //������
    uint16  CountTbl[MAX_REMAP_TBL/2];
}READ_LOG, *pREAD_LOG;

typedef struct tagWL_INFO
{
    uint16 IsWLEn;              //WL enable
    uint16 isNeedSwap;          //trueʱ����Ҫ�滻
    uint16 min;                 //��СEC
    uint16 max;                 //���EC
    uint16 minLevel;            //��СEC����ֵ
    uint16 maxLevel;            //���EC����ֵ
    //uint16 avg;               //ƽ��ֵ
    uint32 total;               //�ܵ�EC
    WL_DATA_STS QoutDataSts;         
    uint16 lastWriteLbaBlk;
    uint32 overHead;            //������������
    uint16 startBlk;            //��̬������ʼblock
    uint16 endBlk;              //��̬���ݽ���block�����������block
    uint16 totBlk;              //��������block��
    uint16 startLbaBlk;         //LBA��ʼblock
    uint32 LbaTotleCount;
    uint32 LbaAvgCount;
    uint16 blkAddr;   //����page info��blk��ַ
    uint16 pageAddr;  //����page info��page��ַ
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
    uint16 blk; //blk�ţ�0Ϊû�з���
    uint16 validPages; //���blk����Ч��page��
    uint32 ver;
}PR_BLK_INFO, *pPR_BLK_INFO;


typedef struct taPAGE_REMAP_INFO
{
    uint16 enable;
    uint16 secPerPage;   //FLASH��page size
    uint16 secPerBlk;    //FLASH��block size
    uint16 curPageAddr;   //��ǰ���ڱ�̵�page ��ַ
    uint16 curBlkIndex;  //��ǰ���ڱ�� �� blk ��blk��index
    uint16 maxBlk;
    uint32 maxPageNum;
    uint32 ver;          //��ǰblk ��version
    uint32 totleSec;
    PR_BLK_INFO  blk[MAX_PAGE_REMAP_RECOVERY_BLOCK];
    PR_PAGE_INFO page[MAX_PAGE_REMAP_PAGE_NUM];
}PAGE_REMAP_INFO, *pPAGE_REMAP_INFO;
#endif

typedef struct tagEXCH_PAGE_SAVE_INFO
{
    uint16 BlkAddr;   //����page info��blk��ַ
    uint16 PageAddr;  //����page info��page��ַ
    uint32 Ver;
}EXCH_PAGE_SAVE_INFO;

typedef struct tagFTL_INFO
{
    uint8 valid;    //FTL��Ч
    uint8 writeCacheEn; //FTL�ӳ�дCACHE�Ƿ�ʹ��
    uint8 delayWriteCacheEn; //����cache��ʱд
    
    uint16 ftlVer;  //FTL�汾��
    uint16 ftlSoftVer;  //FTL�汾��
        
    uint16 secPerPage;   //FLASH��page size
    uint16 secPerBlk;   //FLASH��block size
    uint16 startPhyBlk; //FLASH��Ϊ����������ʼ��
    uint16 maxPhyBlk;   //FLASH��Ϊ������������
    uint16 totalLogicBlk;
    uint16 totalLogicBlkRaw; 
    
#ifdef PAGE_REMAP
    uint16 pageSaveBlkAddr;   //����page info��blk��ַ
    uint16 pageSavePageAddr;  //����page info��page��ַ
#endif

#ifdef WL_ENABLE
    WL_INFO  WLInfo;
#endif

#ifdef EXCH_PAGE_REMAP
    EXCH_PAGE_SAVE_INFO ExchPageInfo;
#endif

    CACHE_BLK_INFO cacheBlkInfo;    //CACHE����Ϣ
    EXCH_BLK_INFO exchBlk[MAX_EXCH_BLK_NUM];//������
    CIR_QUEUE freeBlkInfo;          //�տ��, �����̿ռ���ʱ���ٱ�֤��64����Թ���
    REMAP_INFO remapInfo;           //��ӳ���, ��������С����, e.g. 32GB/(1MB/blk)=64KB
    BAD_BLK_INFO badBlkInfo;        //������Ϣ

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

//1ȫ�ֱ���
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
//EXT     uint32 LBACount[MAX_REMAP_TBL/2]; //��¼ÿ��lba block��������������ʱ��


//1����ԭ������
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


//1�����
#ifdef	IN_FTL
#else
#endif
#endif

