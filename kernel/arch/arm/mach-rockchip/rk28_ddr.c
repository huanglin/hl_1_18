/****************************************************************
//    CopyRight(C) 2008 by Rock-Chip Fuzhou
//      All Rights Reserved
//文件名:hw_sdram.c
//描述:sdram driver implement
//作者:hcy
//创建日期:2008-11-08
//更改记录:
$Log: hw_sdram.c,v $
Revision 1.1.1.1  2009/12/15 01:46:27  zjd
20091215 杨永忠提交初始版本

Revision 1.1.1.1  2009/09/25 08:00:32  zjd
20090925 黄德胜提交 V1.3.1

Revision 1.1.1.1  2009/08/18 06:43:26  Administrator
no message

Revision 1.1.1.1  2009/08/14 08:02:00  Administrator
no message

Revision 1.4  2009/04/02 03:03:37  hcy
更新SDRAM时序配置，只考虑-6，-75系列SDRAM

Revision 1.3  2009/03/19 13:38:39  hxy
hcy去掉SDRAM保守时序,保守时序不对,导致MP3播放时界面抖动

Revision 1.2  2009/03/19 12:21:18  hxy
hcy增加SDRAM保守时序供测试

Revision 1.1.1.1  2009/03/16 01:34:06  zjd
20090316 邓训金提供初始SDK版本

Revision 1.2  2009/03/07 07:30:18  yk
(yk)更新SCU模块各频率设置，完成所有函数及代码，更新初始化设置，
更新遥控器代码，删除FPGA_BOARD宏。
(hcy)SDRAM驱动改成28的

//当前版本:1.00
****************************************************************/
#define  DRIVERS_DDRAM


#ifdef DRIVERS_DDRAM
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/irqflags.h>
#include <linux/string.h>

#include <asm/tcm.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_scu.h>
#include <asm/delay.h>
#include <asm/cacheflush.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_debug.h>

typedef volatile struct tagSCU_REG
{
    unsigned int SCU_APLL_CON;//[3];//0:arm 1:dsp 2:codec
    unsigned int SCU_DPLL_CON;
    unsigned int SCU_CPLL_CON;
    unsigned int SCU_MODE_CON;
    unsigned int SCU_PMU_CON;
    unsigned int SCU_CLKSEL0_CON;
    unsigned int SCU_CLKSEL1_CON;
    unsigned int SCU_CLKGATE0_CON;
    unsigned int SCU_CLKGATE1_CON;
    unsigned int SCU_CLKGATE2_CON;
    unsigned int SCU_SOFTRST_CON;
    unsigned int SCU_CHIPCFG_CON;
    unsigned int SCU_CPUPD;
	unsigned int SCU_CLKSEL2_CON;
}SCU_REG,*pSCU_REG;


#define SDRAM_REG_BASE     (SDRAMC_BASE_ADDR_VA)
#define DDR_REG_BASE       (SDRAMC_BASE_ADDR_VA)

/* CPU_APB_REG4 */
#define MSDR_1_8V_ENABLE  (0x1 << 24)
#define READ_PIPE_ENABLE  (0x1 << 22)
#define EXIT_SELF_REFRESH (0x1 << 21)

/* CPU_APB_REG5 */
#define MEMTYPEMASK   (0x3 << 11) 
#define SDRAM         (0x0 << 11)
#define Mobile_SDRAM  (0x1 << 11)
#define DDRII         (0x2 << 11)
#define Mobile_DDR    (0x3 << 11)

/* SDRAM Config Register */
#define DATA_WIDTH_16     (0x0 << 13)
#define DATA_WIDTH_32     (0x1 << 13)
#define DATA_WIDTH_64     (0x2 << 13)
#define DATA_WIDTH_128    (0x3 << 13)

#define COL(n)            ((n-1) << 9)
#define ROW(n)            ((n-1) << 5)

#define BANK_2            (0 << 3)
#define BANK_4            (1 << 3)
#define BANK_8            (2 << 3)
#define BANK_16           (3 << 3) 

/* SDRAM Timing Register0 */
#define T_RC_SHIFT        (22)
#define T_RC_MAX         (0xF)
#define T_XSR_MSB_SHIFT   (27)
#define T_XSR_MSB_MASK    (0x1F)
#define T_XSR_LSB_SHIFT   (18)
#define T_XSR_LSB_MASK    (0xF)
#define T_RCAR_SHIFT      (14)
#define T_RCAR_MAX       (0xF)
#define T_WR_SHIFT        (12)
#define T_WR_MAX         (0x3)
#define T_RP_SHIFT        (9)
#define T_RP_MAX         (0x7)
#define T_RCD_SHIFT       (6)
#define T_RCD_MAX        (0x7)
#define T_RAS_SHIFT       (2)
#define T_RAS_MASK        (0xF)

#define CL_1              (0)
#define CL_2              (1)
#define CL_3              (2)
#define CL_4              (3)

/* SDRAM Timing Register1 */
#define AR_COUNT_SHIFT    (16)

/* SDRAM Control Regitster */
#define MSD_DEEP_POWERDOWN (1 << 20)
#define UPDATE_EMRS    (1 << 18)
#define OPEN_BANK_COUNT_SHIFT  (12)
#define SR_MODE            (1 << 11)
#define UPDATE_MRS         (1 << 9)
#define READ_PIPE_SHIFT    (6)
#define REFRESH_ALL_ROW_A  (1 << 5)
#define REFRESH_ALL_ROW_B  (1 << 4)
#define DELAY_PRECHARGE    (1 << 3)
#define SDR_POWERDOWN      (1 << 2)
#define ENTER_SELF_REFRESH (1 << 1)
#define SDR_INIT           (1 << 0)

/* Extended Mode Register */
#define DS_FULL            (0 << 5)
#define DS_1_2             (1 << 5)
#define DS_1_4             (2 << 5)
#define DS_1_8             (3 << 5)

#define TCSR_70            (0 << 3)
#define TCSR_45            (1 << 3)
#define TCSR_15            (2 << 3)
#define TCSR_85            (3 << 3)

#define PASR_4_BANK        (0)
#define PASR_2_BANK        (1)
#define PASR_1_BANK        (2)
#define PASR_1_2_BANK      (5)
#define PASR_1_4_BANK      (6)

/* SDRAM Controller register struct */
typedef volatile struct TagSDRAMC_REG
{
    volatile uint32 MSDR_SCONR;         //SDRAM configuration register
    volatile uint32 MSDR_STMG0R;        //SDRAM timing register0
    volatile uint32 MSDR_STMG1R;        //SDRAM timing register1
    volatile uint32 MSDR_SCTLR;         //SDRAM control register
    volatile uint32 MSDR_SREFR;         //SDRAM refresh register
    volatile uint32 MSDR_SCSLR0_LOW;    //Chip select register0(lower 32bits)
    volatile uint32 MSDR_SCSLR1_LOW;    //Chip select register1(lower 32bits)
    volatile uint32 MSDR_SCSLR2_LOW;    //Chip select register2(lower 32bits)
    uint32 reserved0[(0x54-0x1c)/4 - 1];
    volatile uint32 MSDR_SMSKR0;        //Mask register 0
    volatile uint32 MSDR_SMSKR1;        //Mask register 1
    volatile uint32 MSDR_SMSKR2;        //Mask register 2
    uint32 reserved1[(0x84-0x5c)/4 - 1];
    volatile uint32 MSDR_CSREMAP0_LOW;  //Remap register for chip select0(lower 32 bits)
    uint32 reserved2[(0x94-0x84)/4 - 1];
    volatile uint32 MSDR_SMTMGR_SET0;   //Static memory timing register Set0
    volatile uint32 MSDR_SMTMGR_SET1;   //Static memory timing register Set1
    volatile uint32 MSDR_SMTMGR_SET2;   //Static memory timing register Set2
    volatile uint32 MSDR_FLASH_TRPDR;   //FLASH memory tRPD timing register
    volatile uint32 MSDR_SMCTLR;        //Static memory control register
    uint32 reserved4;
    volatile uint32 MSDR_EXN_MODE_REG;  //Extended Mode Register
}SDRAMC_REG_T,*pSDRAMC_REG_T;

#define ODT_DIS          (0x0)
#define ODT_75           (0x8)
#define ODT_150          (0x40)
#define ODT_50           (0x48)

/* CTRL_REG_01 */
#define EXP_BDW_OVFLOW   (0x1 << 24)  //port 3
#define LCDC_BDW_OVFLOW  (0x1 << 16)  //port 2

/* CTRL_REG_02 */
#define VIDEO_BDW_OVFLOW (0x1 << 24) //port 7
#define CEVA_BDW_OVFLOW  (0x1 << 16) //port 6
#define ARMI_BDW_OVFLOW  (0x1 << 8)  //port 5
#define ARMD_BDW_OVFLOW  (0x1 << 0)  //port 4

/* CTR_REG_03 */
#define CONCURRENTAP     (0x1 << 16)

/* CTRL_REG_04 */
#define SINGLE_ENDED_DQS  (0x0 << 24)
#define DIFFERENTIAL_DQS  (0x1 << 24)
#define DLL_BYPASS_EN     (0x1 << 16)

/* CTR_REG_05 */
#define CS1_BANK_4        (0x0 << 16)
#define CS1_BANK_8        (0x1 << 16)
#define CS0_BANK_4        (0x0 << 8)
#define CS0_BANK_8        (0x1 << 8)

/* CTR_REG_07 */
#define ODT_CL_3          (0x1 << 8)

/* CTR_REG_08 */
#define BUS_16BIT         (0x1 << 16)
#define BUS_32BIT         (0x0 << 16)

/* CTR_REG_10 */
#define EN_TRAS_LOCKOUT   (0x1 << 24)
#define DIS_TRAS_LOCKOUT  (0x0 << 24)

/* CTR_REG_12 */
#define AXI_MC_ASYNC      (0x0)
#define AXI_MC_2_1        (0x1)
#define AXI_MC_1_2        (0x2)
#define AXI_MC_SYNC       (0x3)

/* CTR_REG_13 */
#define LCDC_WR_PRIO(n)    ((n) << 24)
#define LCDC_RD_PRIO(n)    ((n) << 16)

/* CTR_REG_14 */
#define EXP_WR_PRIO(n)     ((n) << 16)
#define EXP_RD_PRIO(n)     ((n) << 8)

/* CTR_REG_15 */
#define ARMD_WR_PRIO(n)     ((n) << 8)
#define ARMD_RD_PRIO(n)     (n)
#define ARMI_RD_PRIO(n)     ((n) << 24)

/* CTR_REG_16 */
#define ARMI_WR_PRIO(n)     (n)
#define CEVA_WR_PRIO(n)     ((n) << 24)
#define CEVA_RD_PRIO(n)     ((n) << 16)

/* CTR_REG_17 */
#define VIDEO_WR_PRIO(n)     ((n) << 16)
#define VIDEO_RD_PRIO(n)     ((n) << 8)
#define CS_MAP(n)            ((n) << 24)

/* CTR_REG_18 */
#define CS0_RD_ODT_MASK      (0x3 << 24)
#define CS0_RD_ODT(n)        (0x1 << (24+(n)))
#define CS0_LOW_POWER_REF_EN (0x0 << 8)
#define CS0_LOW_POWER_REF_DIS (0x1 << 8)
#define CS1_LOW_POWER_REF_EN (0x0 << 9)
#define CS1_LOW_POWER_REF_DIS (0x1 << 9)

/* CTR_REG_19 */
#define CS0_ROW(n)           ((n) << 24)
#define CS1_WR_ODT_MASK      (0x3 << 16)
#define CS1_WR_ODT(n)        (0x1 << (16+(n)))
#define CS0_WR_ODT_MASK      (0x3 << 8)
#define CS0_WR_ODT(n)        (0x1 << (8+(n)))
#define CS1_RD_ODT_MASK      (0x3)
#define CS1_RD_ODT(n)        (0x1 << (n))

/* CTR_REG_20 */
#define CS1_ROW(n)           (n)
#define CL(n)                (((n)&0x7) << 16)

/* CTR_REG_21 */
#define CS0_COL(n)           (n)
#define CS1_COL(n)           ((n) << 8)

/* CTR_REG_23 */
#define TRRD(n)              (((n)&0x7) << 24)
#define TCKE(n)              (((n)&0x7) << 8)

/* CTR_REG_24 */
#define TWTR_CK(n)           (((n)&0x7) << 16)
#define TRTP(n)              ((n)&0x7)

/* CTR_REG_29 */
//CAS latency linear value
#define CL_L_1_0             (2)
#define CL_L_1_5             (3)
#define CL_L_2_0             (4)
#define CL_L_2_5             (5)
#define CL_L_3_0             (6)
#define CL_L_3_5             (7)
#define CL_L_4_0             (8)
#define CL_L_4_5             (9)
#define CL_L_5_0             (0xA)
#define CL_L_5_5             (0xB)
#define CL_L_6_0             (0xC)
#define CL_L_6_5             (0xD)
#define CL_L_7_0             (0xE)
#define CL_L_7_5             (0xF)

/* CTR_REG_34 */
#define CS0_TRP_ALL(n)       (((n)&0xF) << 24)
#define TRP(n)               (((n)&0xF) << 16)

/* CTR_REG_35 */
#define WL(n)                ((((n)&0xF) << 16) | (((n)&0xF) << 24))
#define TWTR(n)              (((n)&0xF) << 8)
#define CS1_TRP_ALL(n)       ((n)&0xF)

/* CTR_REG_37 */
#define TMRD(n)              ((n) << 24)
#define TDAL(n)              ((n) << 16)
#define TCKESR(n)            ((n) << 8)
#define TCCD(n)              (n)

/* CTR_REG_38 */
#define TRC(n)               ((n) << 24)
#define TFAW(n)              ((n) << 16)
#define TWR(n)               (n)

/* CTR_REG_40 */
#define EXP_BW_PER(n)        ((n) << 16)
#define LCDC_BW_PER(n)       (n)

/* CTR_REG_41 */
#define ARMI_BW_PER(n)       ((n) << 16)
#define ARMD_BW_PER(n)       (n)

/* CTR_REG_42 */
#define VIDEO_BW_PER(n)      ((n) << 16)
#define CEVA_BW_PER(n)       (n)

/* CTR_REG_43 */
#define TMOD(n)              (((n)&0xFF) << 24)

/* CTR_REG_44 */
#define TRFC(n)              ((n) << 16)
#define TRCD(n)              ((n) << 8)
#define TRAS_MIN(n)          (n)

/* CTR_REG_48 */
#define TPHYUPD_RESP(n)      (((n)&0x3FFF) << 16)
#define TCTRLUPD_MAX(n)      ((n)&0x3FFF)

/* CTR_REG_51 */
#define CS0_MR(n)            ((((n) & 0xFEF0) | 0x2) << 16)
#define TREF(n)              (n)

/* CTR_REG_52 */
#define CS0_EMRS_1(n)        (((n)&0xFFC7) << 16)
#define CS1_MR(n)            (((n) & 0xFEF0) | 0x2)

/* CTR_REG_53 */
#define CS0_EMRS_2(n)        ((n) << 16)
#define CS0_EMR(n)           ((n) << 16)
#define CS1_EMRS_1(n)        ((n)&0xFFC7)

/* CTR_REG_54 */
#define CS0_EMRS_3(n)        ((n) << 16)
#define CS1_EMRS_2(n)        (n)
#define CS1_EMR(n)           (n)

/* CTR_REG_55 */
#define CS1_EMRS_3(n)        (n)

/* CTR_REG_59 */
#define CS_MSK_0(n)          (((n)&0xFFFF) << 16)

/* CTR_REG_60 */
#define CS_VAL_0(n)          (((n)&0xFFFF) << 16)
#define CS_MSK_1(n)          ((n)&0xFFFF)

/* CTR_REG_61 */
#define CS_VAL_1(n)          ((n)&0xFFFF)

/* CTR_REG_62 */
#define MODE5_CNT(n)         (((n)&0xFFFF) << 16)
#define MODE4_CNT(n)         ((n)&0xFFFF)

/* CTR_REG_63 */
#define MODE1_2_CNT(n)        ((n)&0xFFFF)

/* CTR_REG_64 */
#define TCPD(n)              ((n) << 16)
#define MODE3_CNT(n)         (n)

/* CTR_REG_65 */
#define TPDEX(n)             ((n) << 16)
#define TDLL(n)              (n)

/* CTR_REG_66 */
#define TXSNR(n)             ((n) << 16)
#define TRAS_MAX(n)          (n)

/* CTR_REG_67 */
#define TXSR(n)              (n)

/* CTR_REG_68 */
#define TINIT(n)             (n)


/* DDR Controller register struct */
typedef volatile struct TagDDRC_REG
{
    volatile uint32 CTRL_REG_00;
    volatile uint32 CTRL_REG_01;
    volatile uint32 CTRL_REG_02;
    volatile uint32 CTRL_REG_03;
    volatile uint32 CTRL_REG_04;
    volatile uint32 CTRL_REG_05;
    volatile uint32 CTRL_REG_06;
    volatile uint32 CTRL_REG_07;
    volatile uint32 CTRL_REG_08;
    volatile uint32 CTRL_REG_09;
    volatile uint32 CTRL_REG_10;
    volatile uint32 CTRL_REG_11;
    volatile uint32 CTRL_REG_12;
    volatile uint32 CTRL_REG_13;
    volatile uint32 CTRL_REG_14;
    volatile uint32 CTRL_REG_15;
    volatile uint32 CTRL_REG_16;
    volatile uint32 CTRL_REG_17;
    volatile uint32 CTRL_REG_18;
    volatile uint32 CTRL_REG_19;
    volatile uint32 CTRL_REG_20;
    volatile uint32 CTRL_REG_21;
    volatile uint32 CTRL_REG_22;
    volatile uint32 CTRL_REG_23;
    volatile uint32 CTRL_REG_24;
    volatile uint32 CTRL_REG_25;
    volatile uint32 CTRL_REG_26;
    volatile uint32 CTRL_REG_27;
    volatile uint32 CTRL_REG_28;
    volatile uint32 CTRL_REG_29;
    volatile uint32 CTRL_REG_30;
    volatile uint32 CTRL_REG_31;
    volatile uint32 CTRL_REG_32;
    volatile uint32 CTRL_REG_33;
    volatile uint32 CTRL_REG_34;
    volatile uint32 CTRL_REG_35;
    volatile uint32 CTRL_REG_36;
    volatile uint32 CTRL_REG_37;
    volatile uint32 CTRL_REG_38;
    volatile uint32 CTRL_REG_39;
    volatile uint32 CTRL_REG_40;
    volatile uint32 CTRL_REG_41;
    volatile uint32 CTRL_REG_42;
    volatile uint32 CTRL_REG_43;
    volatile uint32 CTRL_REG_44;
    volatile uint32 CTRL_REG_45;
    volatile uint32 CTRL_REG_46;
    volatile uint32 CTRL_REG_47;
    volatile uint32 CTRL_REG_48;
    volatile uint32 CTRL_REG_49;
    volatile uint32 CTRL_REG_50;
    volatile uint32 CTRL_REG_51;
    volatile uint32 CTRL_REG_52;
    volatile uint32 CTRL_REG_53;
    volatile uint32 CTRL_REG_54;
    volatile uint32 CTRL_REG_55;
    volatile uint32 CTRL_REG_56;
    volatile uint32 CTRL_REG_57;
    volatile uint32 CTRL_REG_58;
    volatile uint32 CTRL_REG_59;
    volatile uint32 CTRL_REG_60;
    volatile uint32 CTRL_REG_61;
    volatile uint32 CTRL_REG_62;
    volatile uint32 CTRL_REG_63;
    volatile uint32 CTRL_REG_64;
    volatile uint32 CTRL_REG_65;
    volatile uint32 CTRL_REG_66;
    volatile uint32 CTRL_REG_67;
    volatile uint32 CTRL_REG_68;
    volatile uint32 CTRL_REG_69;
    volatile uint32 CTRL_REG_70;
    volatile uint32 CTRL_REG_71;
    volatile uint32 CTRL_REG_72;
    volatile uint32 CTRL_REG_73;
    volatile uint32 CTRL_REG_74;
    volatile uint32 CTRL_REG_75;
    volatile uint32 CTRL_REG_76;
    volatile uint32 CTRL_REG_77;
    volatile uint32 CTRL_REG_78;
    volatile uint32 CTRL_REG_79;
    volatile uint32 CTRL_REG_80;
    volatile uint32 CTRL_REG_81;
    volatile uint32 CTRL_REG_82;
    volatile uint32 CTRL_REG_83;
    volatile uint32 CTRL_REG_84;
    volatile uint32 CTRL_REG_85;
    volatile uint32 CTRL_REG_86;
    volatile uint32 CTRL_REG_87;
    volatile uint32 CTRL_REG_88;
    volatile uint32 CTRL_REG_89;
    volatile uint32 CTRL_REG_90;
    volatile uint32 CTRL_REG_91;
    volatile uint32 CTRL_REG_92;
    volatile uint32 CTRL_REG_93;
    volatile uint32 CTRL_REG_94;
    volatile uint32 CTRL_REG_95;
    volatile uint32 CTRL_REG_96;
    volatile uint32 CTRL_REG_97;
    volatile uint32 CTRL_REG_98;
    volatile uint32 CTRL_REG_99;
    volatile uint32 CTRL_REG_100;
    volatile uint32 CTRL_REG_101;
    volatile uint32 CTRL_REG_102;
    volatile uint32 CTRL_REG_103;
    volatile uint32 CTRL_REG_104;
    volatile uint32 CTRL_REG_105;
    volatile uint32 CTRL_REG_106;
    volatile uint32 CTRL_REG_107;
    volatile uint32 CTRL_REG_108;
    volatile uint32 CTRL_REG_109;
    volatile uint32 CTRL_REG_110;
    volatile uint32 CTRL_REG_111;
    volatile uint32 CTRL_REG_112;
    volatile uint32 CTRL_REG_113;
    volatile uint32 CTRL_REG_114;
    volatile uint32 CTRL_REG_115;
    volatile uint32 CTRL_REG_116;
    volatile uint32 CTRL_REG_117;
    volatile uint32 CTRL_REG_118;
    volatile uint32 CTRL_REG_119;
    volatile uint32 CTRL_REG_120;
    volatile uint32 CTRL_REG_121;
    volatile uint32 CTRL_REG_122;
    volatile uint32 CTRL_REG_123;
    volatile uint32 CTRL_REG_124;
    volatile uint32 CTRL_REG_125;
    volatile uint32 CTRL_REG_126;
    volatile uint32 CTRL_REG_127;
    volatile uint32 CTRL_REG_128;
    volatile uint32 CTRL_REG_129;
    volatile uint32 CTRL_REG_130;
    volatile uint32 CTRL_REG_131;
    volatile uint32 CTRL_REG_132;
    volatile uint32 CTRL_REG_133;
    volatile uint32 CTRL_REG_134;
    volatile uint32 CTRL_REG_135;
    volatile uint32 CTRL_REG_136;
    volatile uint32 CTRL_REG_137;
    volatile uint32 CTRL_REG_138;
    volatile uint32 CTRL_REG_139;
    volatile uint32 CTRL_REG_140;
    volatile uint32 CTRL_REG_141;
    volatile uint32 CTRL_REG_142;
    volatile uint32 CTRL_REG_143;
    volatile uint32 CTRL_REG_144;
    volatile uint32 CTRL_REG_145;
    volatile uint32 CTRL_REG_146;
    volatile uint32 CTRL_REG_147;
    volatile uint32 CTRL_REG_148;
    volatile uint32 CTRL_REG_149;
    volatile uint32 CTRL_REG_150;
    volatile uint32 CTRL_REG_151;
    volatile uint32 CTRL_REG_152;
    volatile uint32 CTRL_REG_153;
    volatile uint32 CTRL_REG_154;
    volatile uint32 CTRL_REG_155;
    volatile uint32 CTRL_REG_156;
    volatile uint32 CTRL_REG_157;
    volatile uint32 CTRL_REG_158;
    volatile uint32 CTRL_REG_159;
}DDRC_REG_T,*pDDRC_REG_T;

typedef volatile struct tagUART_STRUCT
{
    unsigned int UART_RBR;
    unsigned int UART_DLH;
    unsigned int UART_IIR;
    unsigned int UART_LCR;
    unsigned int UART_MCR;
    unsigned int UART_LSR;
    unsigned int UART_MSR;
    unsigned int UART_SCR;
    unsigned int RESERVED1[(0x30-0x20)/4];
    unsigned int UART_SRBR[(0x70-0x30)/4];
    unsigned int UART_FAR;
    unsigned int UART_TFR;
    unsigned int UART_RFW;
    unsigned int UART_USR;
    unsigned int UART_TFL;
    unsigned int UART_RFL;
    unsigned int UART_SRR;
    unsigned int UART_SRTS;
    unsigned int UART_SBCR;
    unsigned int UART_SDMAM;
    unsigned int UART_SFE;
    unsigned int UART_SRT;
    unsigned int UART_STET;
    unsigned int UART_HTX;
    unsigned int UART_DMASA;
    unsigned int RESERVED2[(0xf4-0xac)/4];
    unsigned int UART_CPR;
    unsigned int UART_UCV;
    unsigned int UART_CTR;
} UART_REG, *pUART_REG;

#define pSDR_Reg       ((pSDRAMC_REG_T)SDRAM_REG_BASE)
#define pDDR_Reg       ((pDDRC_REG_T)DDR_REG_BASE)
#define pSCU_Reg       ((pSCU_REG)SCU_BASE_ADDR_VA)
#define pGRF_Reg       ((pGRF_REG)REG_FILE_BASE_ADDR_VA)
#define pGPIO0_Reg		((pGPIO_REG)GPIO0_BASE_ADDR_VA)
#define pGPIO1_Reg		((pGPIO_REG)GPIO1_BASE_ADDR_VA)

#define DDR_MEM_TYPE()	        (pGRF_Reg->CPU_APB_REG0 & MEMTYPEMASK)
#define DDR_ENABLE_SLEEP()     do{pDDR_Reg->CTRL_REG_36 = 0x1818;}while(0)

#define UART_IER UART_DLH
#define UART_DLL UART_RBR
#define UART_THR UART_RBR

#define  read32(address)           (*((uint32 volatile*)(address)))
#define  write32(address, value)   (*((uint32 volatile*)(address)) = value)

extern volatile int  rk28_debugs;

 uint32 __tcmdata telement;
 uint32 __tcmdata capability;  //单个CS的容量
 uint32 __tcmdata SDRAMnewKHz = 150000;
// uint32 __tcmdata DDRnewKHz = 400000 ; //266000;
 uint32 __tcmdata SDRAMoldKHz = 66000;
 //uint32 __tcmdata DDRoldKHz = 200000;
 unsigned int __tcmdata ddr_reg[8] ;
 
__tcmdata uint32 bFreqRaise;
__tcmdata uint32 elementCnt;
__tcmdata uint32 ddrSREF;
__tcmdata uint32 ddrRASMAX;
__tcmdata uint32 ddrCTRL_REG_23;
__tcmdata uint32 ddrCTRL_REG_24;
__tcmdata uint32 ddrCTRL_REG_34;
__tcmdata uint32 ddrCTRL_REG_35;
__tcmdata uint32 ddrCTRL_REG_37;
__tcmdata uint32 ddrCTRL_REG_38;
__tcmdata uint32 ddrCTRL_REG_43;
__tcmdata uint32 ddrCTRL_REG_44;
__tcmdata uint32 ddrCTRL_REG_64;
__tcmdata uint32 ddrCTRL_REG_65;
__tcmdata uint32 ddrCTRL_REG_66;
__tcmdata uint32 ddrCTRL_REG_67;
__tcmdata uint32 ddrCTRL_REG_68;



//经常有报出错在变频的点，想通过这2个宏的设置，看看是否有改善
//变频是要求RK28 DDR控制器和DDR存储器都锁住频率，才能正常工作
//可以排列组合一下开启哪个，理论上两个都打开效果最好
#define FORCE_CTRL_RELOCK                //可以注释与不注释，来开启或关闭这个功能，用于强制RK28 DDR控制器内部重新锁定频率
//#define FORCE_DDR_RELOCK                 //可以注释与不注释，来开启或关闭这个功能，用于强制DDR存储器内部重新锁定频率

static void __tcmfunc DLLBypass(void)
{

    volatile uint32 value = 0;
    
    value = pDDR_Reg->CTRL_REG_04;
    pDDR_Reg->CTRL_REG_04 = value | DLL_BYPASS_EN;

    
    pDDR_Reg->CTRL_REG_70 = 0x10002117 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_71 = 0x10002117 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_72 = 0x10002117 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_73 = 0x10002117 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_74 = 0x00002104 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_75 = 0x00002104 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_76 = 0x00002104 | (elementCnt << 15);
    pDDR_Reg->CTRL_REG_77 = 0x00002104 | (elementCnt << 15);
}

void SDRAMUpdateRef(uint32 kHz)
{
    if(capability <= 0x2000000) // <= SDRAM_2x32x4
    {
        // 64Mb and 128Mb SDRAM's auto refresh cycle 15.6us or a burst of 4096 auto refresh cycles once in 64ms
        pSDR_Reg->MSDR_SREFR = (((125*kHz)/1000) >> 3) & 0xFFFF;  // 125/8 = 15.625us
    }
    else
    {
        // 256Mb and 512Mb SDRAM's auto refresh cycle 7.8us or a burst of 8192 auto refresh cycles once in 64ms
        pSDR_Reg->MSDR_SREFR = (((62*kHz)/1000) >> 3) & 0xFFFF;  // 62/8 = 7.75us
    }
}

void SDRAMUpdateTiming(uint32 kHz)
{
    uint32 value =0;
    uint32 tmp = 0;
    
    value = pSDR_Reg->MSDR_STMG0R;
    value &= 0xFC3C003F;
    //t_rc =  70ns
    tmp = (70*kHz/1000000) + ((((70*kHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    tmp = (tmp > T_RC_MAX) ? T_RC_MAX : tmp;
    value |= tmp << T_RC_SHIFT;
    //t_rcar = 80ns
    tmp = (80*kHz/1000000) + ((((80*kHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    tmp = (tmp > T_RCAR_MAX) ? T_RCAR_MAX : tmp;
    value |= tmp << T_RCAR_SHIFT;
    //t_wr 固定为2个clk
    value |= 1 << T_WR_SHIFT;
    //t_rp =  20ns
    tmp = (20*kHz/1000000) + ((((20*kHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    tmp = (tmp > T_RP_MAX) ? T_RP_MAX : tmp;
    value |= tmp << T_RP_SHIFT;
    //t_rcd = 20ns
    tmp = (20*kHz/1000000) + ((((20*kHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    tmp = (tmp > T_RCD_MAX) ? T_RCD_MAX : tmp;
    value |= tmp << T_RCD_SHIFT;
    pSDR_Reg->MSDR_STMG0R = value;
    #if 0
    if(newKHz >= 90000)  //大于90M，开启read pipe
    {
        value = *pSDR_SUB_Reg;
        value |= READ_PIPE_ENABLE;
        *pSDR_SUB_Reg = value;
        value = pSDR_Reg->MSDR_SCTLR;
        value &= ~(0x7 << READ_PIPE_SHIFT);
        value |= (0x1 << READ_PIPE_SHIFT);
        pSDR_Reg->MSDR_SCTLR = value;
    }
    else
    {
        value = *pSDR_SUB_Reg;
        value &= ~(READ_PIPE_ENABLE);
        *pSDR_SUB_Reg = value;
    }
    #endif
}
 
#if 0
static uint32 __tcmfunc DDR_debug_byte(char byte)
{
	uint32 uartTimeOut;
#define UART_THR            UART_RBR
	pUART_REG g_UART =((pUART_REG)(( UART1_BASE_ADDR_VA)));
    if( byte == '\n' )
	  	DDR_debug_byte(0x0d);          
	uartTimeOut = 0xffff;
	while((g_UART->UART_USR & (1<<1)) != (1<<1))
	{
		if(uartTimeOut == 0)
		{
			return (1);
		}
		uartTimeOut--;
	}
	g_UART->UART_THR = byte;         
	return (0);
}
static uint32 __tcmfunc DDR_debug_string(char *s)
{
	 while(*s){
                DDR_debug_byte(*s);
                s++;
        }
        return 0;
}
#else
#define DDR_debug_string( a ) do{ }while(0)
#endif

static void DDRPreUpdateRef(uint32 MHz)
{
    uint32 tmp;
    
    ddrSREF = ((59*MHz) >> 3) & 0x3FFF;  // 62/8 = 7.75us
    
    ddrRASMAX = (70*MHz);
    ddrRASMAX = (ddrRASMAX > 0xFFFF) ? 0xFFFF : ddrRASMAX;

    //tMOD = 12ns
    tmp = (12*MHz/1000);
    tmp = (tmp > 0xFF) ? 0xFF : tmp;
    ddrCTRL_REG_43 = TMOD(tmp);
}

static void __tcmfunc DDRUpdateRef(void)
{
    volatile uint32 value = 0;
	
    value = pDDR_Reg->CTRL_REG_51;
    value &= ~(0x3FFF);
    pDDR_Reg->CTRL_REG_51 = value|ddrSREF;
    pDDR_Reg->CTRL_REG_48 = TPHYUPD_RESP(ddrSREF) | TCTRLUPD_MAX(ddrSREF);

    //tXSNR =没这个名字的时间，按DDRII，也让其等于 =  tRFC + 10ns, tRAS_max = 70000ns
    value = pDDR_Reg->CTRL_REG_66;
    value &= ~(0xFFFF);
    pDDR_Reg->CTRL_REG_66 = value | TRAS_MAX(ddrRASMAX);

    pDDR_Reg->CTRL_REG_43 = ddrCTRL_REG_43;
}

 
static  uint32 PLLGetDDRFreq(void)
{

#define DIV1  (0x00 << 30)
#define DIV2  (0x01 << 30)
#define DIV4  (0x02 << 30)
#define DIV8  (0x03 << 30)
#define EXT_OSC_CLK		24
#define LOADER_DDR_FREQ_MHZ  133
	uint32 codecfrqMHz,ddrfrqMHz,codecpll_nf,codecpll_nr,codecpll_od,ddrclk_div;
	codecpll_nr = ((pSCU_Reg->SCU_CPLL_CON & 0x003f0000) >> 16 ) + 1; /*NR = CLKR[5:0] + 1*/
	codecpll_nf = ((pSCU_Reg->SCU_CPLL_CON & 0x0000fff0) >> 4 ) + 1;	/*NF = CLKF[11:0] + 1*/
	codecpll_od = ((pSCU_Reg->SCU_CPLL_CON & 0x0000000e) >> 1 ) + 1;	/*OD = CLKOD[2:0] + 1*/
	ddrclk_div =(pSCU_Reg->SCU_CLKSEL0_CON & 0xc0000000);
	switch(ddrclk_div)
	{
		case DIV1 :
			ddrclk_div = 1;
			//printk("%s-ddr clk div =%d->%d\n",__FUNCTION__,ddrclk_div,__LINE__);
			break;
		case DIV2 :
			ddrclk_div = 2;
			//printk("%s-ddr clk div =%d->%d\n",__FUNCTION__,ddrclk_div,__LINE__);
			break;
		case DIV4 :
			ddrclk_div = 4;
			//printk("%s-ddr clk div =%d->%d\n",__FUNCTION__,ddrclk_div,__LINE__);
			break;
		case DIV8 :
			ddrclk_div = 8;
			//printk("%s-ddr clk div =%d->%d\n",__FUNCTION__,ddrclk_div,__LINE__);
			break;
		default :
			ddrclk_div = 1;
			break;
	}
	#if 0
	if((armpll_nr == 0)||(armpll_od == 0))
		//printk("codec current clk error !! flag armpll_nr ==%d armpll_od==%d\n",armpll_nr,armpll_od);
	if(((EXT_OSC_CLK/armpll_nr) < 9 ) && ((EXT_OSC_CLK/armpll_nr) > 800))
		//printk(" codec current freq/nr range error\n");
	if(((EXT_OSC_CLK/armpll_nr)*armpll_nf < 160 ) && ((EXT_OSC_CLK/armpll_nr)*armpll_nf > 800))
		//printk(" codec current freq/nr*nf range error\n");
	#endif
	/*the output frequency  Fout = EXT_OSC_CLK * armpll_nf/armpll_nr/armpll_od */
	codecfrqMHz = EXT_OSC_CLK * codecpll_nf;
	codecfrqMHz = codecfrqMHz/codecpll_nr;
	codecfrqMHz = codecfrqMHz/codecpll_od;
	ddrfrqMHz = codecfrqMHz/ddrclk_div;	/*calculate DDR current frquency*/
	//printk("SCU_CPLL_CON ==%x codecpll_nr == %d codecpll_nf == %d codecpll_od ==%d\n",pSCU_Reg->SCU_CPLL_CON,armpll_nr,armpll_nf,armpll_od);
	//printk("get current freq codecfrqkhz==%d ddfrqkHz==%d\n",armfrqkHz,ddrfrqkHz);
	if(ddrfrqMHz == 0)
		return LOADER_DDR_FREQ_MHZ;
	return ddrfrqMHz;
}

static uint32 GetDDRCL(uint32 newMHz)
{
    uint32 memType = DDR_MEM_TYPE();

    if(memType == DDRII)
    {
        if(newMHz <= 266)
        {
            return 4;
        }
        else if((newMHz > 266) && (newMHz <= 333))
        {
            return 5;
        }
        else if((newMHz > 333) && (newMHz <= 400))
        {
            return 6;
        }
        else // > 400MHz
        {
            return 6;
        }
    }
    else
    {
        return 3;
    }
}

static void DDRPreUpdateTiming(uint32 MHz)
{
    uint32 tmp;
    uint32 tmp2;
    uint32 tmp3;
    uint32 cl;
    uint32 memType = DDR_MEM_TYPE();// (read32(CPU_APB_REG0) >> 11) & 0x3;

    cl = GetDDRCL(MHz);
    //时序
    if(memType == DDRII)
    {
        // tRRD = 10ns, tCKE = 3 tCK
        tmp = (10*MHz/1000) + ((((10*MHz)%1000) > 0) ? 1:0);
        tmp = (tmp > 7) ? 7 : tmp;
        ddrCTRL_REG_23 = TRRD(tmp) | TCKE(3) | 0x2;
        // tRTP = 7.5ns
        tmp = (8*MHz/1000) + ((((8*MHz)%1000) > 0) ? 1:0);
        tmp = (tmp > 7) ? 7 : tmp;
        ddrCTRL_REG_24 = TRTP(tmp) | TWTR_CK(1) | 0x01000100;
        //tRP_ALL = tRP + 1 tCK, tWTR = 10ns(DDR2-400), 7.5ns (DDR2-533/667/800)
        if(MHz <= 200)
        {
            tmp2 = (10*MHz/1000) + ((((10*MHz)%1000) > 0) ? 1:0);
        }
        else
        {
            tmp2 = (8*MHz/1000) + ((((8*MHz)%1000) > 0) ? 1:0);
        }
        tmp2 = (tmp2 > 0xF) ? 0xF : tmp2;
        ddrCTRL_REG_35 = CS1_TRP_ALL(cl+1) | TWTR(tmp2);
        //tRP_ALL = tRP + 1 tCK, tRP = CL
        ddrCTRL_REG_34 = CS0_TRP_ALL(cl+1) | TRP(cl) | 0x200;
        // tMRD = 2 tCK, tDAL = tWR + tRP = 15ns + CL, tCCD = 2 tCK, DDR2: tCKESR=tCKE=3 tCK, mobile DDR: tCKESR=tRFC
        tmp = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
        tmp += cl;
        tmp = (tmp > 0x1F) ? 0x1F : tmp;
        ddrCTRL_REG_37 = TMRD(2) | TDAL(tmp) | TCKESR(3) | TCCD(2);
        if(MHz <= 200)  //tRC = 65ns
        {
            tmp = (65*MHz/1000) + ((((65*MHz)%1000) > 0) ? 1:0);
        }
        else //tRC = 60ns
        {
            tmp = (60*MHz/1000) + ((((60*MHz)%1000) > 0) ? 1:0);
        }
        //tFAW = 50ns, tWR = 15ns
        tmp = (tmp > 0x3F) ? 0x3F : tmp;
        tmp2 = (50*MHz/1000) + ((((50*MHz)%1000) > 0) ? 1:0);
        tmp2 = (tmp2 > 0x3F) ? 0x3F : tmp2;
        tmp3 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
        tmp3 = (tmp3 > 0x1F) ? 0x1F : tmp3;
        ddrCTRL_REG_38 = TRC(tmp) | TFAW(tmp2) | TWR(tmp3);
        if(capability <= 0x2000000)  // 256Mb
        {
            tmp = (75*MHz/1000) + ((((75*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (85*MHz/1000) + ((((85*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        else if(capability <= 0x4000000) // 512Mb
        {
            tmp = (105*MHz/1000) + ((((105*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (115*MHz/1000) + ((((115*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        else if(capability <= 0x8000000)  // 1Gb
        {
            tmp = (128*MHz/1000) + ((((128*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (138*MHz/1000) + ((((138*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        else  // 4Gb
        {
            tmp = (328*MHz/1000) + ((((328*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (338*MHz/1000) + ((((338*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        //tRFC = 75ns(256Mb)/105ns(512Mb)/127.5ns(1Gb)/327.5ns(4Gb), tRCD = CL, tRAS_min = 45ns
        tmp3 = (45*MHz/1000) + ((((45*MHz)%1000) > 0) ? 1:0);
        tmp3 = (tmp3 > 0xFF) ? 0xFF : tmp3;
        ddrCTRL_REG_44 = TRFC(tmp) | TRCD(cl) | TRAS_MIN(tmp3);
        //tPDEX=没找到，应该等于=tXP和tXARD中的最大者
        ddrCTRL_REG_65 = TPDEX(0x6) | TDLL(200);
        //tXSR = tXSRD = 200 tCK
        ddrCTRL_REG_67 = TXSR(200);
    }
    else
    {
        // tRTP = 0ns
        ddrCTRL_REG_24 = TRTP(0) | TWTR_CK(1) | 0x01000100;
        //tWTR = 1 tCK
        ddrCTRL_REG_35 = CS1_TRP_ALL(cl) | TWTR(1);
        //tRP_ALL = 没找到 = tRP, tRP = CL
        ddrCTRL_REG_34 = CS0_TRP_ALL(cl) | TRP(cl) | 0x200;
        if(MHz <= 100)  //tRC = 80ns
        {
            tmp = (80*MHz/1000) + ((((80*MHz)%1000) > 0) ? 1:0);
        }
        else //tRC = 75ns
        {
            tmp = (75*MHz/1000) + ((((75*MHz)%1000) > 0) ? 1:0);
        }
        //tFAW = 没有这个时间, tWR = 15ns
        tmp = (tmp > 0x3F) ? 0x3F : tmp;
        tmp3 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
        tmp3 = (tmp3 > 0x1F) ? 0x1F : tmp3;
        ddrCTRL_REG_38 = TRC(tmp) | TFAW(0) | TWR(tmp3);
        if(capability <= 0x2000000)  // 128Mb,256Mb
        {
            // 128Mb,256Mb  tRFC=80ns
            tmp = (80*MHz/1000) + ((((80*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0x1F) ? 0x1F : tmp;
            // tMRD = 2 tCK, tDAL = tWR + tRP = 15ns + CL, tCCD = 2 tCK, DDR2: tCKESR=tCKE=3 tCK, mobile DDR: tCKESR=tRFC
            tmp2 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
            tmp2 += cl;
            tmp2 = (tmp2 > 0x1F) ? 0x1F : tmp2;
            ddrCTRL_REG_37 = TMRD(2) | TDAL(tmp2) | TCKESR(tmp) | TCCD(2);
            tmp = (80*MHz/1000) + ((((80*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (90*MHz/1000) + ((((90*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        else if(capability <= 0x4000000) // 512Mb
        {
            // 512Mb  tRFC=110ns
            tmp = (110*MHz/1000) + ((((110*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0x1F) ? 0x1F : tmp;
            // tMRD = 2 tCK, tDAL = tWR + tRP = 15ns + CL, tCCD = 2 tCK, DDR2: tCKESR=tCKE=3 tCK, mobile DDR: tCKESR=tRFC
            tmp2 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
            tmp2 += cl;
            tmp2 = (tmp2 > 0x1F) ? 0x1F : tmp2;
            ddrCTRL_REG_37 = TMRD(2) | TDAL(tmp2) | TCKESR(tmp) | TCCD(2);
            tmp = (110*MHz/1000) + ((((110*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (120*MHz/1000) + ((((120*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        else if(capability <= 0x8000000)  // 1Gb
        {
            // 1Gb tRFC=140ns
            tmp = (140*MHz/1000) + ((((140*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0x1F) ? 0x1F : tmp;
            // tMRD = 2 tCK, tDAL = tWR + tRP = 15ns + CL, tCCD = 2 tCK, DDR2: tCKESR=tCKE=3 tCK, mobile DDR: tCKESR=tRFC
            tmp2 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
            tmp2 += cl;
            tmp2 = (tmp2 > 0x1F) ? 0x1F : tmp2;
            ddrCTRL_REG_37 = TMRD(2) | TDAL(tmp2) | TCKESR(tmp) | TCCD(2);
            tmp = (140*MHz/1000) + ((((140*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (150*MHz/1000) + ((((150*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        else  // 4Gb
        {
            // 大于1Gb没找到，按DDR2的 tRFC=328ns
            tmp = (328*MHz/1000) + ((((328*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0x1F) ? 0x1F : tmp;
            // tMRD = 2 tCK, tDAL = tWR + tRP = 15ns + CL, tCCD = 2 tCK, DDR2: tCKESR=tCKE=3 tCK, mobile DDR: tCKESR=tRFC
            tmp2 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
            tmp2 += cl;
            tmp2 = (tmp2 > 0x1F) ? 0x1F : tmp2;
            ddrCTRL_REG_37 = TMRD(2) | TDAL(tmp2) | TCKESR(tmp) | TCCD(2);
            tmp = (328*MHz/1000) + ((((328*MHz)%1000) > 0) ? 1:0);
            tmp = (tmp > 0xFF) ? 0xFF : tmp;
            //tXSNR = tRFC + 10ns, tRAS_max = 70000ns
            tmp3 = (338*MHz/1000) + ((((338*MHz)%1000) > 0) ? 1:0);
            tmp3 = (tmp3 > 0xFFFF) ? 0xFFFF : tmp3;
            ddrCTRL_REG_66 = TXSNR(tmp3);
        }
        //tRFC = 80ns(128Mb,256Mb)/110ns(512Mb)/140ns(1Gb), tRCD = CL
        if(MHz <= 100)  //tRAS_min = 50ns
        {
            tmp3 = (50*MHz/1000) + ((((50*MHz)%1000) > 0) ? 1:0);
        }
        else //tRAS_min = 45ns
        {
            tmp3 = (45*MHz/1000) + ((((45*MHz)%1000) > 0) ? 1:0);
        }
        tmp3 = (tmp3 > 0xFF) ? 0xFF : tmp3;
        ddrCTRL_REG_44 = TRFC(tmp) | TRCD(cl) | TRAS_MIN(tmp3);
        //tPDEX=没找到，应该等于=tXP=25ns
        tmp = (25*MHz/1000) + ((((25*MHz)%1000) > 0) ? 1:0);
        tmp = (tmp > 0xFFFF) ? 0xFFFF : tmp;
        ddrCTRL_REG_65 = TPDEX(tmp) | TDLL(200);
        // tRRD = 15ns, tCKE = 2 tCK
        tmp2 = (15*MHz/1000) + ((((15*MHz)%1000) > 0) ? 1:0);
        tmp2 = (tmp2 > 7) ? 7 : tmp2;
        tmp = (tmp <= 1) ? 2 : (tmp + 1);
        tmp = (tmp > 7) ? 7 : tmp;
        ddrCTRL_REG_23 = TRRD(tmp2) | TCKE(tmp) | 0x2;
        //tXSR = 200 ns
        tmp = (200*MHz/1000) + ((((200*MHz)%1000) > 0) ? 1:0);
        tmp = (tmp > 0xFFFF) ? 0xFFFF : tmp;
        ddrCTRL_REG_67 = TXSR(tmp);
    }
    //tCPD = 400ns
    tmp = (400*MHz/1000) + ((((400*MHz)%1000) > 0) ? 1:0);
    tmp = (tmp > 0xFFFF) ? 0xFFFF : tmp;
    ddrCTRL_REG_64 = TCPD(tmp) | MODE3_CNT(0x1FF);
    //tINIT = 200us
    tmp = (200000*MHz/1000) + ((((200000*MHz)%1000) > 0) ? 1:0);
    tmp = (tmp > 0xFFFFFF) ? 0xFFFFFF : tmp;
    ddrCTRL_REG_68 = TINIT(tmp);
}

static void __tcmfunc DDRUpdateTiming(void)
{
    uint32 value;
    value = pDDR_Reg->CTRL_REG_35;
    value &= ~(0xF0F);
    pDDR_Reg->CTRL_REG_35 = value | ddrCTRL_REG_35;
    pDDR_Reg->CTRL_REG_23 = ddrCTRL_REG_23;
    pDDR_Reg->CTRL_REG_24 = ddrCTRL_REG_24;
    pDDR_Reg->CTRL_REG_37 = ddrCTRL_REG_37;
    pDDR_Reg->CTRL_REG_34 = ddrCTRL_REG_34;
    pDDR_Reg->CTRL_REG_38 = ddrCTRL_REG_38;
    pDDR_Reg->CTRL_REG_44 = ddrCTRL_REG_44;
    pDDR_Reg->CTRL_REG_64 = ddrCTRL_REG_64;
    pDDR_Reg->CTRL_REG_65 = ddrCTRL_REG_65;
    value = pDDR_Reg->CTRL_REG_66;
    value &= ~(0xFFFF0000);
    pDDR_Reg->CTRL_REG_66 = ddrCTRL_REG_66 | value;
    pDDR_Reg->CTRL_REG_67 = ddrCTRL_REG_67;
    pDDR_Reg->CTRL_REG_68 = ddrCTRL_REG_68;
}

/* we avoid to use stack and global var. six instr in one cycle.*/
#if 0
volatile uint32 __tcmdata for_ddr_delay;
void __tcmfunc ddr_pll_delay( int loops ) 
{
        for_ddr_delay = loops;
        for( ; for_ddr_delay > 0 ; for_ddr_delay-- ){
                ;
        }
}
#else
/* 65ns in one loop when CPU=156MHz.*/
asm(	
"	.section \".tcm.text\",\"ax\"\n"	
"	.align\n"
"	.type	ddr_pll_delay, #function\n"
"               .global ddr_pll_delay\n"
"ddr_pll_delay:\n"
"               cmp r0,#0\n"
"               movle  pc , lr\n"
"1:	sub  r0,r0,#1\n"	
"	mov r1,r1\n"	
"	mov r1,r1\n"
"	mov r1,r1\n"
"	mov r2,r2\n"	
"	mov r2,r2\n"
"               cmp r0,#0\n"
"               bgt     1b\n"
"	mov pc,lr\n"
"	.previous"
);
#endif

static void __tcmfunc  disable_DDR_Sleep(void);

/****************************************************************/
//函数名:SDRAM_BeforeUpdateFreq
//描述:调整SDRAM/DDR频率前调用的函数，用于调整SDRAM/DDR时序参数
//参数说明:SDRAMnewKHz   输入参数   SDRAM将要调整到的频率，单位KHz
//         DDRnewKHz     输入参数   DDR将要调整到的频率，单位KHz
//返回值:
//相关全局变量:
//注意:这个函数只修改MSDR_STMG0R，MSDR_SREFR，MSDR_SCTLR的值
/****************************************************************/
void __tcmfunc SDRAM_BeforeUpdateFreq(uint32 SDRAMnewKHz, uint32 DDRnewMHz)
{
    uint32 MHz;
    uint32 memType = DDR_MEM_TYPE();// (pGRF_Reg->CPU_APB_REG0) & MEMTYPEMASK;
    uint32 tmp;
    uint32 cl;
	
	ddr_reg[0] = pGRF_Reg->CPU_APB_REG0;
	ddr_reg[1] = pGRF_Reg->CPU_APB_REG1;
	
	
	ddr_reg[4] = pDDR_Reg->CTRL_REG_10;
	
	ddr_reg[6] = pDDR_Reg->CTRL_REG_78;
    switch(memType)
    {
        case Mobile_SDRAM:
        case SDRAM:
		printk("%s:erroe memtype=0x%lx\n" ,__func__, memType);
#if 0
		KHz = PLLGetAHBFreq();
		if(KHz < SDRAMnewKHz)  //升频
		{
		SDRAMUpdateTiming(SDRAMnewKHz);
		bFreqRaise = 1;
		}
		else //降频
		{
		SDRAMUpdateRef(SDRAMnewKHz);
		bFreqRaise = 0;
		}
#endif
            break;
        case DDRII:
        case Mobile_DDR:
            MHz = PLLGetDDRFreq();
            if(telement)
            {
                elementCnt = 250000/(DDRnewMHz*telement);
                if(elementCnt > 156)
                {
                    elementCnt = 156;
                }
            }
            else
            {
                elementCnt = 2778/DDRnewMHz;  // 90ps算
                if(elementCnt > 156)
                {
                    elementCnt = 156;
                }
            }
            DDRPreUpdateRef(DDRnewMHz);
            DDRPreUpdateTiming(DDRnewMHz);
            
            disable_DDR_Sleep();
            //printk("%s::just befor ddr refresh.new ddr=%ld\n" , __func__ , DDRnewKHz);
            //WAIT_ME();
            if(memType == DDRII)
            {
                cl = GetDDRCL(DDRnewMHz);
        		pDDR_Reg->CTRL_REG_00 |= (0x1 << 16);  //refresh to close all bank
                //set mode register cl
                pDDR_Reg->CTRL_REG_51 = (pDDR_Reg->CTRL_REG_51 & (~(0x7FFF << 16))) | (((cl << 4) | ((cl -1) << 9) | 0x2) << 16);
                tmp = pDDR_Reg->CTRL_REG_52;
                tmp &= 0xFFFF0000;
                tmp |= (((cl << 4) | ((cl -1) << 9) | 0x2));
                pDDR_Reg->CTRL_REG_52 = tmp;
                pDDR_Reg->CTRL_REG_11 |= (0x1 << 16);
                //set controller register cl
                pDDR_Reg->CTRL_REG_20 = (pDDR_Reg->CTRL_REG_20 & (~(0x7 << 16))) | (cl << 16);
                pDDR_Reg->CTRL_REG_29 = (pDDR_Reg->CTRL_REG_29 & (~(0xF << 24))) | ((cl << 1) << 24);
                pDDR_Reg->CTRL_REG_30 = (pDDR_Reg->CTRL_REG_30 & (~0xF)) | (cl << 1);
                pDDR_Reg->CTRL_REG_32 = (pDDR_Reg->CTRL_REG_32 & (~(0xF << 8))) | (cl << 8);
                pDDR_Reg->CTRL_REG_35 = (pDDR_Reg->CTRL_REG_35 & (~(0xF0F << 16))) | ((cl - 1) << 16) | ((cl-1) << 24);
            }
            pDDR_Reg->CTRL_REG_09 |= (0x1 << 24);	// after this on,ddr enter refresh,no access valid!!
            //WAIT_ME();
            while(!(pDDR_Reg->CTRL_REG_03 & 0x100));
            pDDR_Reg->CTRL_REG_10 &= ~(0x1);   
            if(MHz < DDRnewMHz)  //升频
            {
                DDRUpdateTiming();
                bFreqRaise = 1;
            }
            else //降频
            {
                DDRUpdateRef();
                bFreqRaise = 0;
            }
            break;
        default:
            printk("%s:unkown memtype=0x%lx\n" ,__func__, memType);
            break;
    }
}

/****************************************************************/
//函数名:SDRAM_AfterUpdateFreq
//描述:SDRAM/DDR频率调整后，调用这个函数，完成调频后的善后工作
//参数说明:SDRAMoldKHz   输入参数   SDRAM调整前的频率，单位KHz
//         DDRoldKHz     输入参数   SDRAM调整前的频率，单位KHz
//返回值:
//相关全局变量:
//注意:这个函数只修改MSDR_STMG0R，MSDR_SREFR，MSDR_SCTLR的值
/****************************************************************/

void __tcmfunc SDRAM_AfterUpdateFreq(uint32 SDRAMoldKHz, uint32 DDRnewMHz)
{
    uint32 value =0;
    //uint32 tmp = 0;
    uint32 ddrMHz;
    //uint32 ahbKHz;
    uint32 memType = DDR_MEM_TYPE();// (pGRF_Reg->CPU_APB_REG0) & MEMTYPEMASK;
        //	DDR_debug_string("21\n");
    switch(memType)
    {
        case Mobile_SDRAM:
        case SDRAM:
		printk("%s:erroe memtype=0x%lx\n" ,__func__, memType);
#if 0
		// ahbKHz = PLLGetAHBFreq();
		if(ahbKHz > SDRAMoldKHz)  //升频
		{
		// SDRAMUpdateRef(ahbKHz);
		}
		else //降频
		{
		//  SDRAMUpdateTiming(ahbKHz);
		}
#endif
            break;
        case DDRII:
        case Mobile_DDR:
            if(bFreqRaise)  //升频
            {
                DDRUpdateRef();
            }
            else //降频
            {
                DDRUpdateTiming();
            }
            ddrMHz = DDRnewMHz;
            pDDR_Reg->CTRL_REG_10 |= 0x1;
            value = 1000;
            while(value)
            {
                if(pDDR_Reg->CTRL_REG_04 & 0x100)
                {
                    break;
                }
                value--;
            }
            if(!(value))
            {
                DLLBypass();
                pDDR_Reg->CTRL_REG_10 |= 0x1;
                while(!(pDDR_Reg->CTRL_REG_04 & 0x100));
            }
            #ifdef FORCE_CTRL_RELOCK
            pDDR_Reg->CTRL_REG_09 |= 1;  //force controller relock
            #endif
            pDDR_Reg->CTRL_REG_09 &= ~(0x1 << 24);
            while(pDDR_Reg->CTRL_REG_03 & 0x100); // exit 
            //    printk("exit ddr refresh,");
            //退出自刷新后，再算element的值
            //ddrKHz = PLLGetDDRFreq();
            ddrMHz = DDRnewMHz;
            //printk("new ddr kHz=%ld\n" , ddrKHz);
            if(110 < ddrMHz)
            {
                value = pDDR_Reg->CTRL_REG_78;
                if(value & 0x1)
                {
                    value = value >> 1;
                    if( value == 0 ){
                        S_WARN("%s::value = 0!\n" , __func__ );
                    } else 
                        telement = 1000000/(ddrMHz*value);
                }
            }
            // 20100714,HSL@RK,use DDR_ENABLE_SLEEP instead!!
            //pDDR_Reg->CTRL_REG_36 = (0x1F1F); // 20100608,YK@RK.
            DDR_ENABLE_SLEEP();
            break;
        default:
            printk("%s:unkown memtype=0x%lx\n" ,__func__, memType);
            break;
    }
}

/* XXX:HSL@RK,use phy addr,only be call after mmu disable,for debug reboot.
  * if use at kernel,must change to va addr.
*/
#if 1
static void __tcmfunc ddr_put_one_char(char byte)
{
	//pUART_REG   pReg  = ((pUART_REG)UART1_BASE_ADDR);
	pUART_REG   pReg  = ((pUART_REG)UART1_BASE_ADDR_VA);

	while((pReg->UART_USR & (1<<1)) != (1<<1));
	pReg->UART_THR = byte;
	while(!(pReg->UART_USR & (1<<2))); /* wait for empty */
}

void __tcmfunc ddr_put_char( char c )
{
	ddr_put_one_char( c );
	ddr_put_one_char(0x0d);
	ddr_put_one_char( '\n' );
}
#else
#define ddr_put_char( c )
#endif

asm(	
"	.section \".tcm.text\",\"ax\"\n"	
"	.align\n"
"	.type	ddr_save_sp, #function\n"
"               .global ddr_save_sp\n"
"ddr_save_sp:\n"
"	mov r1,sp\n"	
"	mov sp,r0\n"	
"	mov r0,r1\n"	
"	mov pc,lr\n"
"	.previous"
);

#define PLL_TEST        (0x01u<<25)
#define PLL_SAT         (0x01u<<24)
#define PLL_FAST        (0x01u<<23)
#define PLL_PD          (0x01u<<22)
#define PLL_CLKR(i)     (((i)&0x3f)<<16)
#define PLL_CLKF(i)     (((i)&0x0fff)<<4)
#define PLL_CLKOD(i)    (((i)&0x07)<<1)
#define PLL_BYPASS      (0X01)

#define DDR_CLK_NORMAL          300
#define DDR_CLK_LOWPW            128

static void __tcmfunc  disable_DDR_Sleep(void)
{
    volatile uint32 *p_ddr = (volatile uint32 *)0xc0080000;
    unsigned int tmp;

    //printk("disable auto power down\n");
    do
    {
	/* 20100910,HSL@RK,1.不能使用物理地址，会 dataabort.
	 * 2.使用 p_ddr++,防止 p_ddr 已经被CACHE,无法唤醒 DDR.
	*/
        //tmp = *(volatile uint32 *)SDRAM_ADDR;  //read to wakeup
        tmp = *p_ddr;  //read to wakeup
        pDDR_Reg->CTRL_REG_36 &= ~(0x1F1F); //disable auto power down
        p_ddr += 0x40000;                   //read to next 1MB, so cache will miss, and read from extend mem 
	    if((uint32)p_ddr > (0xc0080000 + 0x1000000))
	    {
		    p_ddr = (volatile uint32 *)0xc0080000;
	    }
	    ddr_pll_delay(10);  //wait write buffer clean
    }while(((pDDR_Reg->CTRL_REG_67 & 0xFFFF0000) != 0x20440000)
           || ((pDDR_Reg->CTRL_REG_31 & 0x0F0F0000) == 0x0)
           ||  (pDDR_Reg->CTRL_REG_36 & 0x1f1f)); 
    //printk("confirm wakeup, p_ddr=0x%x\n", (uint32)p_ddr);
    p_ddr = (volatile uint32 *)0xc0080000;
    while(pDDR_Reg->CTRL_REG_03 & 0x100)
    {
        //tmp = *(volatile uint32 *)SDRAM_ADDR;  //read to wakeup
        tmp = *p_ddr;  //read to wakeup
        p_ddr += 0x40000;                   //read to next 1MB, so cache will miss, and read from extend mem 
	    if((uint32)p_ddr > (0xc0080000 + 0x1000000))
	    {
		    p_ddr = (volatile uint32 *)0xc0080000;
	    }
    }
    while(pGRF_Reg->CPU_APB_REG1 & 0x100); //wait controller idle
}

void __tcmfunc rk28_ddr_enter_self_refresh(void)
{
	uint32 memType = DDR_MEM_TYPE();
	volatile uint32 value;
	
	switch(memType) {
	case DDRII:
	case Mobile_DDR:
		disable_DDR_Sleep();
		//while((pDDR_Reg->CTRL_REG_67 & 0xFFFF0000) == 0x20440000); 
		pDDR_Reg->CTRL_REG_09 |= (0x1 << 24);	// after this on,ddr enter refresh,no access valid!!
		//WAIT_ME();
		while(!(pDDR_Reg->CTRL_REG_03 & 0x100));
		pDDR_Reg->CTRL_REG_10 &= ~(0x1);
		pGRF_Reg->CPU_APB_REG7 |= (0x1 << 29); //keep cke low
		pSCU_Reg->SCU_PMU_CON |= (0x1 << 2); //DDR PHY power down
		value = 1000;
		while(value){
			if(pSCU_Reg->SCU_PMU_CON & (0x1 << 7))
				break;
			value--;
		}

		ddr_pll_delay( 2000 );
		pSCU_Reg->SCU_CLKGATE1_CON |= 0x00c00001;
		ddr_pll_delay( 10 );
		pSCU_Reg->SCU_CPLL_CON |= ((1<<22)|1);
		ddr_pll_delay( 10 );
		break;
	default:
		break;
	}
}

void __tcmfunc rk28_ddr_exit_self_refresh(void)
{
	uint32 memType = DDR_MEM_TYPE();
	volatile uint32 value;

	switch(memType) {
	case DDRII:
	case Mobile_DDR:
		pSCU_Reg->SCU_CPLL_CON &= ~((1<<22)|1);
		ddr_pll_delay( 6000 );
		pSCU_Reg->SCU_CLKGATE1_CON &= ~0x00c00000;
		ddr_pll_delay( 100 );
		pSCU_Reg->SCU_PMU_CON &= ~(0x1 << 2); //DDR PHY power resume
		
		value = 1000;
		while(value){
			if(!(pSCU_Reg->SCU_PMU_CON & (0x1 << 7)))
				break;
			value--;
		}
		pGRF_Reg->CPU_APB_REG7 &= ~(0x1 << 29); //keep cke original level
		pDDR_Reg->CTRL_REG_10 |= 0x1;
		
		value = 1000;
		while(value) {
			if(pDDR_Reg->CTRL_REG_04 & 0x100)
				break;
			value--;
		}
		pDDR_Reg->CTRL_REG_09 &= ~(0x1 << 24);	// after this on,ddr exit refresh,no access valid!!		
		while(pDDR_Reg->CTRL_REG_03 & 0x100);   //exit self refresh
		DDR_ENABLE_SLEEP();
		break;
	default:
		break;
	}
}
void __tcmfunc  GPIOSetPinLevel_sram(int  GPIOPinNum,int level)
{
	uint8 gpioPortNum;
	uint8 gpioPinNum;
	pGPIO_REG pheadGpio;
	if(GPIOPinNum >= 64)
		return ;
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio= (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	gpioPortNum = GPIOPinNum/8;
	gpioPinNum	= GPIOPinNum%8;
	switch ( gpioPortNum ) {
	case 0:
	pheadGpio->GPIO_SWPORTA_DR = (pheadGpio->GPIO_SWPORTA_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	pheadGpio->GPIO_SWPORTA_DDR = (pheadGpio->GPIO_SWPORTA_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
	pheadGpio->GPIO_SWPORTA_DR = (pheadGpio->GPIO_SWPORTA_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	break;
	case 1:
	pheadGpio->GPIO_SWPORTB_DR = (pheadGpio->GPIO_SWPORTB_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	pheadGpio->GPIO_SWPORTB_DDR = (pheadGpio->GPIO_SWPORTB_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
	pheadGpio->GPIO_SWPORTB_DR = (pheadGpio->GPIO_SWPORTB_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	break;
	case 2:
	pheadGpio->GPIO_SWPORTC_DR = (pheadGpio->GPIO_SWPORTC_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	pheadGpio->GPIO_SWPORTC_DDR = (pheadGpio->GPIO_SWPORTC_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
	pheadGpio->GPIO_SWPORTC_DR = (pheadGpio->GPIO_SWPORTC_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	break;
	case 3:
	pheadGpio->GPIO_SWPORTD_DR = (pheadGpio->GPIO_SWPORTD_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	pheadGpio->GPIO_SWPORTD_DDR = (pheadGpio->GPIO_SWPORTD_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
	pheadGpio->GPIO_SWPORTD_DR = (pheadGpio->GPIO_SWPORTD_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
	break;
	}
}

void __tcmfunc rk28_lowpower_suspend(void)
{
	
	GPIOSetPinLevel_sram(DDR_CTL_IOPIN,1);
	GPIOSetPinLevel_sram(SLEEP_CTL_IOPIN,0);
	ddr_pll_delay( 6000 );//delay for debug print ABC
	pGRF_Reg->IOMUX_A_CON = (pGRF_Reg->IOMUX_A_CON&(~(3ul<<26))) |(0 << 26);//uart1 tx to PF1
	pGPIO1_Reg->GPIO_SWPORTB_DDR = (pGPIO1_Reg->GPIO_SWPORTB_DDR&(~(1ul<<1))) |(1ul << 1);//PF1 direc OUT
	pGPIO1_Reg->GPIO_SWPORTB_DR = (pGPIO1_Reg->GPIO_SWPORTB_DR&(~(1ul<<1))) |(1ul << 1);//PF1 level high
	pGRF_Reg->GPIO1_AB_PU_CON = (pGRF_Reg->GPIO1_AB_PU_CON&(~(3ul<<18))) |(1ul << 18);//PF1  pull up
	//Set the uart iomux gpio
	pGRF_Reg->IOMUX_A_CON = (pGRF_Reg->IOMUX_A_CON&(~(3ul<<24))) |(0 << 24);//uart1 rx to PF0
	pGPIO1_Reg->GPIO_SWPORTB_DDR = (pGPIO1_Reg->GPIO_SWPORTB_DDR&(~(1ul<<1))) |(1ul << 1);//PF0 direc OUT
	pGRF_Reg->GPIO1_AB_PU_CON = (pGRF_Reg->GPIO1_AB_PU_CON&(~(3ul<<16))) |(0 << 16);//PF0  pull normal

	//Disable uart clock
	pSCU_Reg->SCU_CLKGATE0_CON |= (1ul << 19);
	//Set arm_pll_clk to 24M, divide to 16
	pSCU_Reg->SCU_CLKSEL2_CON |= 0xf;

}

void __tcmfunc rk28_lowpower_resume(void)
{
	
	//Set arm_pll_clk to 24M, divide to 1
	pSCU_Reg->SCU_CLKSEL2_CON &= ~(0xf);
	//Enable uart clock
	pSCU_Reg->SCU_CLKGATE0_CON &= ~(1ul << 19);

	//Set the uart iomux gpio
	pGRF_Reg->IOMUX_A_CON = (pGRF_Reg->IOMUX_A_CON&(~(3ul<<24))) |(1ul << 24);//uart1 rx
	pGPIO1_Reg->GPIO_SWPORTB_DDR = (pGPIO1_Reg->GPIO_SWPORTB_DDR&(~(1ul<<1))) |(1ul << 1);//PF0 direc IN
	pGRF_Reg->GPIO1_AB_PU_CON = (pGRF_Reg->GPIO1_AB_PU_CON&(~(3ul<<16))) |(1ul << 16);//PF0  pull down
	pGRF_Reg->IOMUX_A_CON = (pGRF_Reg->IOMUX_A_CON&(~(3ul<<26))) |(1ul << 26);//uart1 tx
	pGPIO1_Reg->GPIO_SWPORTB_DDR = (pGPIO1_Reg->GPIO_SWPORTB_DDR&(~(1ul<<1))) |(1ul << 1);//PF1 direc IN
	pGRF_Reg->GPIO1_AB_PU_CON = (pGRF_Reg->GPIO1_AB_PU_CON&(~(3ul<<18))) |(1ul << 18);//PF1  pull down
	GPIOSetPinLevel_sram(SLEEP_CTL_IOPIN,1);
	GPIOSetPinLevel_sram(DDR_CTL_IOPIN,0);

}

uint32 scu_clk_gate[3];
uint32 scu_pmu_con;
uint32 scu_mode_con;
extern void __rk28_halt_here( void );

void scu_reg_save(void)
{
	pGRF_Reg->OTGPHY_CON1 |= 0x80000000;// bit 31: 1.1host phy. bit 2: 2.0host.
	scu_clk_gate[0] = pSCU_Reg->SCU_CLKGATE0_CON;
	scu_clk_gate[1] = pSCU_Reg->SCU_CLKGATE1_CON;
	scu_clk_gate[2] = pSCU_Reg->SCU_CLKGATE2_CON;
	pSCU_Reg->SCU_CLKGATE0_CON = 0xfcf0bff6;//0xfff4fff6;//0xfcf0bff6;
	pSCU_Reg->SCU_CLKGATE1_CON = 0xff3f8001;    //sdram clock 0xe7fff
	pSCU_Reg->SCU_CLKGATE2_CON = 0x24;       //ahb, apb clock
	scu_pmu_con = pSCU_Reg->SCU_PMU_CON;
	pSCU_Reg->SCU_PMU_CON |= 0x09;//power down lcdc & dsp power domain
	scu_mode_con = pSCU_Reg->SCU_MODE_CON;
	pSCU_Reg->SCU_MODE_CON |= 0x11;
}
void scu_reg_restore(void)
{
	pSCU_Reg->SCU_MODE_CON = scu_mode_con;
	pSCU_Reg->SCU_PMU_CON = scu_pmu_con;
	pSCU_Reg->SCU_CLKGATE0_CON = scu_clk_gate[0]; 
	pSCU_Reg->SCU_CLKGATE1_CON = scu_clk_gate[1]; 
	pSCU_Reg->SCU_CLKGATE2_CON = scu_clk_gate[2]; 
	pGRF_Reg->OTGPHY_CON1 &= ~0x80000000;
}

void __tcmfunc rk28_do_halt( void )
{
	ddr_put_char('A');
	scu_reg_save();
	ddr_disabled = 1;
	ddr_put_char('B');
	rk28_ddr_enter_self_refresh();
	ddr_put_char('C');
	rk28_lowpower_suspend();
	__rk28_halt_here();
	rk28_lowpower_resume();
	ddr_put_char('C');
	rk28_ddr_exit_self_refresh();
	ddr_put_char('B');
	scu_reg_restore();
	ddr_disabled = 0;
	ddr_put_char('A');
}
void __tcmfunc rk28_halt_at_tcm( void )
{
        flush_cache_all();
        __cpuc_flush_user_all();
        DDR_SAVE_SP;	
        rk28_do_halt();
        DDR_RESTORE_SP;
}

static void  SDRAM_DDR_Init(void)
{
	uint32 value;
	uint32 MHz;
	uint32 memType = DDR_MEM_TYPE();// (pGRF_Reg->CPU_APB_REG0) & MEMTYPEMASK;
	uint32 capTmp;

	telement = 0;
	if(memType == Mobile_DDR)
	{
		pDDR_Reg->CTRL_REG_11 = 0x101;
	}
	/* 20100609,HSL@RK,disable ddr interrupt .*/
	pDDR_Reg->CTRL_REG_45 = 0x07ff03ff; 
	//tPDEX=没找到，应该等于=tXP=25ns 
#if 0 //by Vincent @2010 07 30 // scrn failed
	pDDR_Reg->CTRL_REG_63 = 200 ; // 0x64;  // can not wakeup ddr ctrler for setting if too small.
	pDDR_Reg->CTRL_REG_62 = 0x00100008; 
	//pDDR_Reg->CTRL_REG_36 = 0x1f1f; // use DDR_ENABLE_SLEEP instead!!
#else
	// pDDR_Reg->CTRL_REG_63 &= 0x0000ffff; // scrn ok HSL@RK,no use.
#endif 
	//读操作时端口优先级是:LCDC(port 2) > CEVA(port 6) > VIDEO(port 7) > ARMI(port 5) > ARMD(port 4) > EXP(port3)
	//写操作时端口优先级是:CEVA(port 6) > ARMD(port 4) > EXP(port3) > LCDC(port 2) > VIDEO(port 7) > ARMI(port 5)
	pDDR_Reg->CTRL_REG_13 = (AXI_MC_ASYNC << 8) | LCDC_WR_PRIO(3) | LCDC_RD_PRIO(0);
	pDDR_Reg->CTRL_REG_14 = (AXI_MC_ASYNC << 24) | (AXI_MC_ASYNC) | EXP_WR_PRIO(2) | EXP_RD_PRIO(3);
	pDDR_Reg->CTRL_REG_15 = (AXI_MC_ASYNC << 16) | ARMD_WR_PRIO(1) | ARMD_RD_PRIO(3) | ARMI_RD_PRIO(3);
	pDDR_Reg->CTRL_REG_16 = (AXI_MC_ASYNC << 8) | CEVA_WR_PRIO(0) | CEVA_RD_PRIO(1) | ARMI_WR_PRIO(3);
	value = pDDR_Reg->CTRL_REG_17;
	value &= ~(0x3FFFF);
	value |= (AXI_MC_ASYNC) | VIDEO_RD_PRIO(2) | VIDEO_WR_PRIO(3);
	pDDR_Reg->CTRL_REG_17 = value;
	capability = (0x1 << ((0x2 >> ((pDDR_Reg->CTRL_REG_08 >> 16) & 0x1))  //bus width
	    + (13 - (pDDR_Reg->CTRL_REG_21 & 0x7))  //col
	    + (2 + ((pDDR_Reg->CTRL_REG_05 >> 8) & 0x1))  //bank
	    + (15 - ((pDDR_Reg->CTRL_REG_19 >> 24) & 0x7))));  //row
	printk("CS0, ROW=%ld, COL=%ld, Bank=%ld, BusWidth=%ld, Capability=%ldMB\n", (long unsigned int)(15 - ((pDDR_Reg->CTRL_REG_19 >> 24) & 0x7))
	                                                             ,(long unsigned int)(13 - (pDDR_Reg->CTRL_REG_21 & 0x7))
	                                                             ,(long unsigned int)(4<<((pDDR_Reg->CTRL_REG_05 >> 8) & 0x1))
	                                                             ,(long unsigned int)(32 >> ((pDDR_Reg->CTRL_REG_08 >> 16) & 0x1))
	                                                             ,(long unsigned int)(capability>>20));
	if(((pDDR_Reg->CTRL_REG_17 >> 24) & 0x3) == 0x3)
	{
		capTmp = (0x1 << ((0x2 >> ((pDDR_Reg->CTRL_REG_08 >> 16) & 0x1))  //bus width
		    + (13 - ((pDDR_Reg->CTRL_REG_21 >> 8) & 0x7))  //col
		    + (2 + ((pDDR_Reg->CTRL_REG_05 >> 16) & 0x1))  //bank
		    + (15 - (pDDR_Reg->CTRL_REG_20 & 0x7))));      //row
		printk("CS1, ROW=%ld, COL=%ld, Bank=%ld, BusWidth=%ld, Capability=%ldMB\n", (long unsigned int)(15 - (pDDR_Reg->CTRL_REG_20 & 0x7))
	                                                             ,(long unsigned int)(13 - ((pDDR_Reg->CTRL_REG_21 >> 8) & 0x7))
	                                                             ,(long unsigned int)(4<<((pDDR_Reg->CTRL_REG_05 >> 16) & 0x1))
	                                                             ,(long unsigned int)(32 >> ((pDDR_Reg->CTRL_REG_08 >> 16) & 0x1))
	                                                             ,(long unsigned int)(capTmp>>20));
		if(capability < capTmp) 
		{
			capability = capTmp;   //这里capability只登记最大容量，用于计算tRFC，tRFC只跟最大容量有关系
		}
	}
	MHz = PLLGetDDRFreq();
	printk("PLLGetDDRFreq=%ld MHZ\n" , MHz);
	if(110 > MHz)
	{
		if(telement)
		{
			elementCnt = 250000/(MHz*telement);
			if(elementCnt > 156)
			{
				elementCnt = 156;
			}
		}
		else
		{
			elementCnt = 2778/MHz;  // 90ps算
			if(elementCnt > 156)
			{
				elementCnt = 156;
			}
		}
		DLLBypass();
	}
	else
	{
		value = pDDR_Reg->CTRL_REG_78;
		if(value & 0x1)
		{
			value = value >> 1;
			telement = 1000000/(MHz*value);
		}
	}
	DDRPreUpdateRef(MHz);
	DDRPreUpdateTiming(MHz);
	DDRUpdateRef();
	DDRUpdateTiming();
	DDR_ENABLE_SLEEP();

	//change_ddr_freq( 150 );
}

void ddr_change_mode( int performance )
{
	/* change ddr to mode :1:performance , 0:power save*/
        if( performance ){
                pDDR_Reg->CTRL_REG_63 = 0x0000ffff; 
         } else {
                pDDR_Reg->CTRL_REG_63 = 200 ; //  0x64; 
        }
}

 void __tcmfunc rk281x_reboot( void )
{
#if 0
	g_intcReg->FIQ_INTEN &= 0x0;
	g_intcReg->IRQ_INTEN_H &= 0x0;
	g_intcReg->IRQ_INTEN_L &= 0x0;
	g_intcReg->IRQ_INTMASK_L &= 0x0;
	g_intcReg->IRQ_INTMASK_H &= 0x0;	
#endif		
	//rk2818_reduce_armfrq();
	printk("disable DDR sleep!!!\n");
	disable_DDR_Sleep();
	printk("start reboot!!!\n");
#if 0
	uint32 i;
	uint32 *p;
	for(i=0;i<93; i++)
	{
		p = (uint32 *)(DDR_REG_BASE + (4*i));
		printk("DDR Reg[%d]=0x%x\n", i, *p);
	}
#else
	//WAIT_ME();
#endif
	asm(    
	"ldr r6,=0x18018000      @ get scu reg base first.\n"
	"ldr r7,=0x10002FF8     @load sram for stack.\n"
	"mov r0,#0\n"
	"MCR  p15, 0, r0, c7, c7, 0    @flush I-cache & D-cache\n"
	"MRC p15,0,r0,c1,c0,0\n"
	"BIC r0,r0,#(1<<2)	@ disable Dcache \n"
	"BIC r0,r0,#(1<<12)     @ disable Icache \n"
	"MCR p15,0,r0,c1,c0,0\n"

	"MRC p15,0,r0,c1,c0,0\n"
	"BIC r0,r0,#(1<<0)	   @disable mmu\n"
	"BIC r0,r0,#(1<<13)    @set vector to 0x00000000\n"
	"MCR p15,0,r0,c1,c0,0\n"
	"mov r1,r1\n"
	"mov r1,r1\n" 
	"mov r1,r1\n"

	"mov sp , r7\n"     // set SP for call other functions at itcm.
	// print char '0'
	//"mov r0,#0x30\n"
	//"bl ddr_put_char\n"

	//"ldr r2,=0x18018000      @ enable sram arm dsp clock\n"
	//"mov    r2,r6\n"
	"ldr r3,[r6,#0x1c]\n"
	"bic r3,r3,#(1<<3)\n"
	"bic r3,r3,#(1<<4)\n"
	"str r3,[r6,#0x1c]\n"

	// print char 'A'
	//"mov r0,#0x41\n"
	//"bl ddr_put_char\n"

	"ldr r3,[r6,#0x1c]         @ITCM not map to address 0, it is right in linux system\n"
	"bic r3,r3,#(1<<15)      @maskrom clock enable\n"
	"bic r3,r3,#(1<<8)       @nandc clock enable\n"
	"bic r3,r3,#(1<<22)      @spi0 clock enable\n"
	"bic r3,r3,#(1<<18)      @uart0 clock enable\n"
	"bic r3,r3,#(1<<6)       @usb clock enable\n"
	"bic r3,r3,#(1<<7)\n"
	"bic r3,r3,#(1<<9)       @intc clock enable\n"
	"bic r3,r3,#(1<<5)       @hif clock enable\n"
	"str r3,[r6,#0x1c]\n"

	// print char '1'
	//"mov r0,#0x31\n"
	//"bl ddr_put_char\n"

	"ldr r2,=0x10040804        @usb soft disconnect.\n"
	" mov r3,#2\n"
	"str r3,[r2,#0]\n"

	"ldr r2,=0x100AE00C       @ BCH reset.\n"
	" mov r3,#1\n"
	"str r3,[r2,#0]\n"

	"ldr r3,[r6,#0x14]       @ARM:AHB=1:1, AHB:APB=2:1\n"
	"bic r3,r3,#0xF\n"
	"orr r3,r3,#0x4\n"
	"str r3,[r6,#0x14]\n"

	"ldr r3,[r6,#0x0c]       @CPU slow mode\n"
	"bic r3,r3,#(3<<2)\n"
	"str r3,[r6,#0x0c]\n"

	// print char 'B'
	//"mov r0,#0x42\n"
	//"bl ddr_put_char\n"

	"ldr r3,[r6,#0x28]\n"
	"orr r3,r3,#(1<<3)      @reset nandc\n"
	"orr r3,r3,#(1<<4)      @reset DSP core\n"
	"orr r3,r3,#(1<<0)      @reset USB OTG\n"
	"orr r3,r3,#(1<<11)     @reset USB controller\n"
	"orr r3,r3,#(1<<12)     @reset USB PHY\n"
	//"orr r3,r3,#(1<<14)  @reset uart1\n"
	"orr r3,r3,#(1<<15)     @reset SPI0\n"
	"orr r3,r3,#(1<<30)     @reset DDR core\n"
	"orr r3,r3,#(1<<31)     @reset DDR bus\n"
	"str r3,[r6,#0x28]\n"

	"mov r0,#1000\n"
	"bl ddr_pll_delay\n"

	//"ldr r2,=0x18018000\n"
	"ldr r3,[r6,#0x28]\n"
	"bic r3,r3,#(1<<3)      @De-reset nandc\n"
	"bic r3,r3,#(1<<0)      @De-reset USB OTG\n"
	"bic r3,r3,#(1<<11)     @De-reset USB controller\n"
	"bic r3,r3,#(1<<12)     @De-reset USB PHY\n"
	"bic r3,r3,#(1<<14)     @De-reset uart1\n"
	"bic r3,r3,#(1<<15)     @De-reset SPI0\n"
	"bic r3,r3,#(1<<30)     @reset DDR core\n"
	"bic r3,r3,#(1<<31)     @reset DDR bus\n"
	"str r3,[r6,#0x28]\n"

	"mov r0,#1000\n"
	"bl ddr_pll_delay\n"

	// print char '2'
	//"mov r0,#0x32\n"
	//"bl ddr_put_char\n"

	//"ldr r2,=0x18018000\n"
	"ldr r3,[r6,#0x18]      @uart0 use 24MHz clock\n"
	"bic r3,r3,#(1<<31)\n"
	"str r3,[r6,#0x18]\n"

	"ldr r6,=0x18019000        @ DisableRemap\n"
	"ldr r3,[r6,#0x14]\n"
	"bic r3,r3,#(1<<0)\n"
	"bic r3,r3,#(1<<1)\n"
	"str r3,[r6,#0x14]\n"

	"ldr r3,[r6,#0x10]   @hif interface enable\n"
	"orr r3,r3,#(1<<27)\n"
	"str r3,[r6,#0x10]\n"

	// print char '3'
	//"mov r0,#0x33\n"
	//"bl ddr_put_char\n"
#if 0
	"ldr r2,=0x18009000       @rk2818_reduce_corevoltage\n"
	"ldr r3,[r2,#0x24]\n"
	"bic r3,#(1<<6)\n"
	"str r3,[r2,#0x24]\n"
	"ldr r3,[r2,#0x28]\n"
	"bic r3,#(1<<6)\n"
	"str r3,[r2,#0x28]\n"
#endif			
	"mov r12,#0\n"
	"mov pc ,r12\n"
	);
}


void(*rk28_restart_mmu)(void )= (void(*)(void ))rk281x_reboot;

 int __init update_frq(void)
{
#if 0
	ddr_reg[0] = pGRF_Reg->CPU_APB_REG0;
	ddr_reg[1] = pGRF_Reg->CPU_APB_REG1;
	ddr_reg[2] = pDDR_Reg->CTRL_REG_03;
	ddr_reg[3] = pDDR_Reg->CTRL_REG_09;
	ddr_reg[4] = pDDR_Reg->CTRL_REG_10;
	ddr_reg[5] = pDDR_Reg->CTRL_REG_36;
	ddr_reg[6] = pDDR_Reg->CTRL_REG_78;
	printk(" before 0x%08x 0x%08x 0x%08x 0x%08x\n"
                            "0x%08x 0x%08x 0x%08x \n"
                          //  "0x%08x 0x%08x 0x%08x 0x%08x\n" ,
                            ,ddr_reg[0],ddr_reg[1],ddr_reg[2],ddr_reg[3],
                            ddr_reg[4],ddr_reg[5],ddr_reg[6]);
#endif
	//local_irq_disable();
	//printk("wait for jtag...\n");
	//WAIT_ME();
#if 0
	rockchip_mux_api_set(GPIOG_MMC1_SEL_NAME,IOMUXA_GPIO1_C237);
	GPIOSetPinLevel(GPIOPortG_Pin7,GPIO_HIGH);
	while(!(chip_type = GPIOGetPinLevel(GPIOPortG_Pin7)));
#endif
#if 0
	int chip_type;
	printk("chip_type is %d,orbiden\n",chip_type);
	*p = 'n';
#endif
	SDRAM_DDR_Init();
#if 0
	printk("after SDRAM_DDR_Init\n");
	//local_irq_enable();		
	ddr_reg[0] = pGRF_Reg->CPU_APB_REG0;
	ddr_reg[1] = pGRF_Reg->CPU_APB_REG1;
	ddr_reg[2] = pDDR_Reg->CTRL_REG_03;
	ddr_reg[3] = pDDR_Reg->CTRL_REG_09;
	ddr_reg[4] = pDDR_Reg->CTRL_REG_10;
	ddr_reg[5] = pDDR_Reg->CTRL_REG_36;
	ddr_reg[6] = pDDR_Reg->CTRL_REG_78;
	printk("after 0x%08x 0x%08x 0x%08x 0x%08x\n"
                            "0x%08x 0x%08x 0x%08x \n"
                          //  "0x%08x 0x%08x 0x%08x 0x%08x\n" ,
                            ,ddr_reg[0],ddr_reg[1],ddr_reg[2],ddr_reg[3],
                            ddr_reg[4],ddr_reg[5],ddr_reg[6]);
#endif
#if 0
	int *reg = (int *)(SCU_BASE_ADDR_VA);
	S_INFO("scu after frq%s::\n0x%08x 0x%08x 0x%08x 0x%08x\n"
                            "0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n"
                            "0x%08x 0x%08x 0x%08x 0x%08x\n"
                          //  "0x%08x 0x%08x 0x%08x 0x%08x\n" ,
                            ,__func__,
                            reg[0],reg[1],reg[2],reg[3],
                            reg[4],reg[5],reg[6],reg[7], reg[8],
                            reg[9], reg[10],reg[11], reg[12]);
#endif
	return 0;	
}
core_initcall_sync(update_frq);

#endif //endi of #ifdef DRIVERS_SDRAM

