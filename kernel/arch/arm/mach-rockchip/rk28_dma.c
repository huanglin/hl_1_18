/*
 *  linux/arch/arm/mach-rockchip/dma.c
 *
 *  rk28 DMA registration and IRQ dispatching
 *
 *
 *  2009-06-20 nizhenyu 
 *             initial version heavily inspired by
 *             linux/arch/arm/mach-rockchip/dma.c
 *
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/types.h>
#include <asm/arch/rk28_dma.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>

//#define RK28_PRINT 1
#include <asm/arch/rk28_debug.h>

#define DMA_DEBUG  0
//#define DMA_MEMCPY_TEST

#if DMA_DEBUG
#define dmaprintk(msg...) printk(msg)
#else
#define dmaprintk(msg...) 
#endif


#define write_dma_reg(regbase, addr, val)        __raw_writel(val, addr+regbase) 
#define read_dma_reg(regbase, addr)              __raw_readl(addr+regbase)    
#define mask_dma_reg(regbase, addr, msk, val)    write_dma_reg(regbase, addr, (val)|((~(msk))&read_dma_reg(regbase, addr)))

   /* clear interrupt */
#define CLR_DWDMA_INTR(regbase, dma_ch)            write_dma_reg(regbase, DWDMA_ClearTfr, 0x101<<dma_ch)

   /* Unmask interrupt */
#define UN_MASK_DWDMA_ALL_TRF_INTR(regbase)        write_dma_reg(regbase, DWDMA_MaskTfr, 0x3f3f)//mask_dma_reg(DWDMA_MaskTfr, 0x101<<dma_ch, 0x101<<dma_ch)

   /* Mask interrupt */
#define MASK_DWDMA_ALL_TRF_INTR(regbase)           write_dma_reg(regbase, DWDMA_MaskTfr, 0x3f00)//mask_dma_reg(DWDMA_MaskTfr, 0x101<<dma_ch, 0x100<<dma_ch)

   /* Unmask interrupt */
#define UN_MASK_DWDMA_INTR(regbase, dma_ch)        mask_dma_reg(regbase, DWDMA_MaskTfr, 0x101<<dma_ch, 0x101<<dma_ch)

   /* Mask interrupt */
#define MASK_DWDMA_INTR(regbase, dma_ch)           mask_dma_reg(regbase, DWDMA_MaskTfr, 0x101<<dma_ch, 0x100<<dma_ch)

   /* Enable channel */
#define ENABLE_DWDMA(regbase, dma_ch)              mask_dma_reg(regbase, DWDMA_ChEnReg, 0x101<<dma_ch, 0x101<<dma_ch)

   /* Disable channel */
#define DISABLE_DWDMA(regbase, dma_ch)             mask_dma_reg(regbase, DWDMA_ChEnReg, 0x101<<dma_ch, 0x100<<dma_ch)

   /* Whether channel is free */
#define WHETHER_DWDMA_FREE(regbase, dma_ch)        read_dma_reg(regbase, DWDMA_ChEnReg) & (0x001<<dma_ch)


static struct rk28_dma_channel rk28_dma_channels[RK28_DMA_MAX];

static struct tasklet_struct rk2818_dma_tasklet;
static int last_ch;

static struct rk28_dma_infor rk28_dma_info;
static struct rk28_dma_dev  rk28_dev_info[RK28_DMA_MAX] = {
	[RK28_DMA_SD_MMC0] = {
		.name		 = "sd_mmc_0",
		.hd_if       = RK28_DMA_SD_MMC0,
		.dev_addr	 = SDMMC0_BASE_ADDR + 0x100,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH0,
	},
#if 0
	[RK28_DMA_LCDC_R0] = {
		.name		 = "lcdc_req_0",
	    .hd_if       = RK28_DMA_LCDC_R0,
		.dev_addr	 = LCDC_BASE_ADDR + 0x08,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH2,
	},
	[RK28_DMA_LCDC_R1] = {
		.name		 = "lcdc_req_1",
		.hd_if       = RK28_DMA_LCDC_R1,
		.dev_addr	 = LCDC_BASE_ADDR + 0x20,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH2,
	},
	[RK28_DMA_LCDC_R2] = {
		.name		 = "lcdc_req_2",
		.hd_if       = RK28_DMA_LCDC_R2,
		.dev_addr	 = LCDC_BASE_ADDR + 0x24,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH2,
	},
	[RK28_DMA_LCDC_R3] = {
		.name		 = "lcdc_req_3",
		.hd_if       = RK28_DMA_LCDC_R3,
		.dev_addr	 = LCDC_BASE_ADDR + 0x28,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH2,
	},
#else
	[RK28_DMA_URAT2_TXD] = {
		.name		 = "uart_2_txd",
	    .hd_if       = RK28_DMA_URAT2_TXD,
		.dev_addr	 = UART2_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_URAT2_RXD] = {
		.name		 = "uart_2_rxd",
		.hd_if       = RK28_DMA_URAT2_RXD,
		.dev_addr	 = UART2_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_URAT3_TXD] = {
		.name		 = "uart_3_txd",
		.hd_if       = RK28_DMA_URAT3_TXD,
		.dev_addr	 = UART3_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_URAT3_RXD] = {
		.name		 = "uart_3_rxd",
		.hd_if       = RK28_DMA_URAT3_RXD,
		.dev_addr	 = UART3_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
#endif
	
	[RK28_DMA_SD_MMC1] = {
		.name		 = "sd_mmc_1",
		.hd_if       = RK28_DMA_SD_MMC1,
		.dev_addr	 = SDMMC1_BASE_ADDR + 0x100,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH1,
	},
	[RK28_DMA_I2S_TXD] = {
		.name		 = "i2s_txd",
		.hd_if       = RK28_DMA_I2S_TXD,
		.dev_addr	 = I2S_BASE_ADDR + 0x04,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH2,
	},
	[RK28_DMA_I2S_RXD] = {
		.name		 = "i2s_rxd",
		.hd_if       = RK28_DMA_I2S_RXD,
		.dev_addr	 = I2S_BASE_ADDR + 0x08,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH0,  ////RK28_DMA_CH2,
	},
	[RK28_DMA_SPI_M_TXD] = {
		.name		 = "spi_m_txd",
		.hd_if       = RK28_DMA_SPI_M_TXD,
		.dev_addr	 = SPI_MASTER_BASE_ADDR + 0x60,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH3,
	},
	[RK28_DMA_SPI_M_RXD] = {
		.name		 = "spi_m_rxd",
		.hd_if       = RK28_DMA_SPI_M_RXD,
		.dev_addr	 = SPI_MASTER_BASE_ADDR + 0x60,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH3,
	},
	[RK28_DMA_SPI_S_TXD] = {
		.name		 = "spi_s_txd",
		.hd_if       = RK28_DMA_SPI_S_TXD,
		.dev_addr	 = SPI_SLAVE_BASE_ADDR + 0x60,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH4,
	},
	[RK28_DMA_SPI_S_RXD] = {
		.name		 = "spi_s_rxd",
		.hd_if       = RK28_DMA_SPI_S_RXD,
		.dev_addr	 = SPI_SLAVE_BASE_ADDR + 0x60,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH4,
	},
	[RK28_DMA_URAT0_TXD] = {
		.name		 = "uart_0_txd",
		.hd_if       = RK28_DMA_URAT0_TXD,
		.dev_addr	 = UART0_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_URAT0_RXD] = {
		.name		 = "uart_0_rxd",
		.hd_if       = RK28_DMA_URAT0_RXD,
		.dev_addr	 = UART0_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_URAT1_TXD] = {
		.name		 = "uart_1_txd",
		.hd_if       = RK28_DMA_URAT1_TXD,
		.dev_addr	 = UART1_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_URAT1_RXD] = {
		.name		 = "uart_1_rxd",
		.hd_if       = RK28_DMA_URAT1_RXD,
		.dev_addr	 = UART1_BASE_ADDR,
		.fifo_width  = 8,
		.dma_dft_ch  = RK28_DMA_CH5,
	},
	[RK28_DMA_SDRAM] = {
		.name		 = "sdram_wr",
		.hd_if       = RK28_DMA_SDRAM,
		.dev_addr	 = 0x60800000,
		.fifo_width  = 32,
		.dma_dft_ch  = RK28_DMA_CH1,
	},		
};



void show_dma_regs(int phy_ch) {

	u32 regbase = rk28_dma_info.reg_vir_base;

	dmaprintk(">>>>>show dma registers(channel %d):\n",phy_ch);
	dmaprintk("DWDMA_SAR: \t0x%x\n",read_dma_reg(regbase, DWDMA_SAR(phy_ch)));
	dmaprintk("DWDMA_DAR: \t0x%x\n",read_dma_reg(regbase, DWDMA_DAR(phy_ch)));
	dmaprintk("DWDMA_LLP: \t0x%x\n",read_dma_reg(regbase, DWDMA_LLP(phy_ch)));
	dmaprintk("DWDMA_CTLL: \t0x%x\n",read_dma_reg(regbase, DWDMA_CTLL(phy_ch)));
	dmaprintk("DWDMA_CTLH: \t0x%x\n",read_dma_reg(regbase, DWDMA_CTLH(phy_ch)));
//	dmaprintk("DWDMA_SSTAT: \t0x%x\n",read_dma_reg(regbase, DWDMA_SSTAT(phy_ch)));
//	dmaprintk("DWDMA_DSTAT: \t0x%x\n",read_dma_reg(regbase, DWDMA_DSTAT(phy_ch)));
//	dmaprintk("DWDMA_SSTATAR: \t0x%x\n",read_dma_reg(regbase, DWDMA_SSTATAR(phy_ch)));
//	dmaprintk("DWDMA_DSTATAR: \t0x%x\n",read_dma_reg(regbase, DWDMA_DSTATAR(phy_ch)));
	dmaprintk("DWDMA_CFGL: \t0x%x\n",read_dma_reg(regbase, DWDMA_CFGL(phy_ch)));
	dmaprintk("DWDMA_CFGH: \t0x%x\n",read_dma_reg(regbase, DWDMA_CFGH(phy_ch)));
//	dmaprintk("DWDMA_SGR: \t0x%x\n",read_dma_reg(regbase, DWDMA_SGR(phy_ch)));
//	dmaprintk("DWDMA_DSR: \t0x%x\n",read_dma_reg(regbase, DWDMA_DSR(phy_ch)));
	dmaprintk("DWDMA_RawTfr: \t0x%x\n",read_dma_reg(regbase, DWDMA_RawTfr));
	dmaprintk("DWDMA_RawBlock: \t0x%x\n",read_dma_reg(regbase, DWDMA_RawBlock));
	dmaprintk("DWDMA_RawSrcTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_RawSrcTran));
	dmaprintk("DWDMA_RawDstTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_RawDstTran));
	dmaprintk("DWDMA_RawErr: \t0x%x\n",read_dma_reg(regbase, DWDMA_RawErr));
	dmaprintk("DWDMA_StatusTfr: \t0x%x\n",read_dma_reg(regbase, DWDMA_StatusTfr));
	dmaprintk("DWDMA_StatusBlock: \t0x%x\n",read_dma_reg(regbase, DWDMA_StatusBlock));
	dmaprintk("DWDMA_StatusSrcTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_StatusSrcTran));
	dmaprintk("DWDMA_StatusDstTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_StatusDstTran));
	dmaprintk("DWDMA_StatusErr: \t0x%x\n",read_dma_reg(regbase, DWDMA_StatusErr));
	dmaprintk("DWDMA_MaskTfr: \t0x%x\n",read_dma_reg(regbase, DWDMA_MaskTfr));
	dmaprintk("DWDMA_MaskBlock: \t0x%x\n",read_dma_reg(regbase, DWDMA_MaskBlock));
	dmaprintk("DWDMA_MaskSrcTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_MaskSrcTran));
	dmaprintk("DWDMA_MaskDstTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_MaskDstTran));
	dmaprintk("DWDMA_MaskErr: \t0x%x\n",read_dma_reg(regbase, DWDMA_MaskErr));
//	dmaprintk("DWDMA_ClearTfr: \t0x%x\n",read_dma_reg(regbase, DWDMA_ClearTfr));
//	dmaprintk("DWDMA_ClearBlock: \t0x%x\n",read_dma_reg(regbase, DWDMA_ClearBlock));
//	dmaprintk("DWDMA_ClearSrcTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_ClearSrcTran));
//	dmaprintk("DWDMA_ClearDstTran: \t0x%x\n",read_dma_reg(regbase, DWDMA_ClearDstTran));
//	dmaprintk("DWDMA_ClearErr: \t0x%x\n",read_dma_reg(regbase, DWDMA_ClearErr));
	dmaprintk("DWDMA_StatusInt: \t0x%x\n",read_dma_reg(regbase, DWDMA_StatusInt));
	dmaprintk("DWDMA_ReqSrcReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_ReqSrcReg));
	dmaprintk("DWDMA_ReqDstReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_ReqDstReg));
	dmaprintk("DWDMA_LstSrcReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_LstSrcReg));
	dmaprintk("DWDMA_LstDstReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_LstDstReg));
	dmaprintk("DWDMA_DmaCfgReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_DmaCfgReg));
	dmaprintk("DWDMA_ChEnReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_ChEnReg));
	dmaprintk("DWDMA_DmaIdReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_DmaIdReg));
	dmaprintk("DWDMA_DmaTestReg: \t0x%x\n",read_dma_reg(regbase, DWDMA_DmaTestReg));
	//if()
	//UN_MASK_DWDMA_ALL_TRF_INTR(regbase);

}

void show_dma_virtual_channel() {

	dmaprintk("DWdma virtual channel %d\n",rk28_dma_info.dma_flag[0]);
	dmaprintk("DWdma virtual channel %d\n",rk28_dma_info.dma_flag[1]);
	dmaprintk("DWdma virtual channel %d\n",rk28_dma_info.dma_flag[2]);
	dmaprintk("DWdma virtual channel %d\n",rk28_dma_info.dma_flag[3]);
	dmaprintk("DWdma virtual channel %d\n",rk28_dma_info.dma_flag[4]);
	dmaprintk("DWdma virtual channel %d\n",rk28_dma_info.dma_flag[5]);
}


/**
 * rk28_dma_control_for_write - set dma control register for writing mode   
 *
 */
static u32 rk28_dma_control_for_write( struct rk28_dma_dev *dev_info)
{    
    u32 dev_mode = (dev_info->hd_if == RK28_DMA_SDRAM) ?  B_CTLL_MEM2MEM_DMAC : B_CTLL_MEM2PER_DMAC;
    u32 inc_mode = (dev_info->hd_if == RK28_DMA_SDRAM) ?  B_CTLL_DINC_INC : B_CTLL_DINC_UNC;
	
	#if 0
    u32 llp_mode = (dev_info->dma_phy_ch  != RK28_DMA_CH2)   ?  (B_CTLL_LLP_DST_EN | B_CTLL_LLP_SRC_EN) : 0;
	u32 int_mode = (dev_info->dma_phy_ch  == RK28_DMA_CH2)   ?  B_CTLL_INT_EN : 0;
	#else /*[xjh] rk2818*/
	u32 llp_mode = (dev_info->dma_phy_ch  < RK28_DMA_CH2)   ?  (B_CTLL_LLP_DST_EN | B_CTLL_LLP_SRC_EN) : 0;
	u32 int_mode = (dev_info->dma_phy_ch  >= RK28_DMA_CH2)   ?  B_CTLL_INT_EN : 0;
	#endif

    u32 ctll = B_CTLL_SRC_TR_WIDTH_32 | B_CTLL_DST_TR_WIDTH(dev_info->fifo_width >> 4) |
               B_CTLL_SINC_INC | inc_mode |
               B_CTLL_DMS_ARMD | B_CTLL_SMS_EXP | dev_mode |
               B_CTLL_SRC_MSIZE_4 | B_CTLL_DST_MSIZE_4 |
               llp_mode | int_mode;
               
    return ctll;               
}

/**
 * rk28_dma_control_for_write - set dma control register for reading mode   
 *
 */
static u32 rk28_dma_control_for_read(struct rk28_dma_dev *dev_info)
{    
    u32 dev_mode = (dev_info->hd_if == RK28_DMA_SDRAM) ?  B_CTLL_MEM2MEM_DMAC : B_CTLL_PER2MEM_DMAC;
    u32 inc_mode = (dev_info->hd_if == RK28_DMA_SDRAM) ?  B_CTLL_SINC_INC : B_CTLL_SINC_UNC;
	
	#if 0
    u32 llp_mode = (dev_info->dma_phy_ch  != RK28_DMA_CH2)   ?  (B_CTLL_LLP_DST_EN | B_CTLL_LLP_SRC_EN) : 0;
    u32 int_mode = (dev_info->dma_phy_ch  == RK28_DMA_CH2)   ?  B_CTLL_INT_EN : 0;
    #else	/*[xjh] rk2818*/
    u32 llp_mode = (dev_info->dma_phy_ch  < RK28_DMA_CH2)   ?  (B_CTLL_LLP_DST_EN | B_CTLL_LLP_SRC_EN) : 0;
    u32 int_mode = (dev_info->dma_phy_ch  >= RK28_DMA_CH2)   ?  B_CTLL_INT_EN : 0;
	#endif
	
    
    u32 ctll = B_CTLL_SRC_TR_WIDTH(dev_info->fifo_width>> 4) | B_CTLL_DST_TR_WIDTH_32 |
               inc_mode | B_CTLL_DINC_INC |
               B_CTLL_DMS_EXP | B_CTLL_SMS_ARMD | dev_mode |
               B_CTLL_SRC_MSIZE_4 | B_CTLL_DST_MSIZE_4 | 
               llp_mode | int_mode;
               
    return ctll;               
}

/**
 * rk28_dma_setup_reg - set dma registers  
 *
 */
static void rk28_dma_setup_reg(rk28_dmach_t dma_ch,
                                              u32 sar, 
                                              u32 dar, 
                                              u32 ctll,
                                              struct rk28_dma_llp *llp,
                                              u32 size)
{    
	struct rk28_dma_channel *rk28dma = &rk28_dma_channels[dma_ch];
	u32 phy_ch = rk28dma->dev_info->dma_phy_ch;
	u32 dma_if = rk28dma->dev_info->hd_if;
	u32 regbase = rk28_dma_info.reg_vir_base;

    write_dma_reg(regbase, DWDMA_SAR(phy_ch), sar); 
    write_dma_reg(regbase, DWDMA_DAR(phy_ch), dar);         
    write_dma_reg(regbase, DWDMA_LLP(phy_ch), (u32)llp);        
    write_dma_reg(regbase, DWDMA_CTLL(phy_ch), ctll);
    write_dma_reg(regbase, DWDMA_CTLH(phy_ch), size);
    write_dma_reg(regbase, DWDMA_CFGL(phy_ch),  B_CFGL_CH_PRIOR(7) | 
                                                B_CFGL_H_SEL_DST | B_CFGL_H_SEL_SRC |
                                                B_CFGL_DST_HS_POL_H | B_CFGL_SRC_HS_POL_H);
    write_dma_reg(regbase, DWDMA_CFGH(phy_ch), B_CFGH_SRC_PER(dma_if & 0xf) | 
                                               B_CFGH_DST_PER(dma_if & 0xf) |
                                               B_CFGH_PROTCTL);
}

/**
 * rk28_dma_setup_reg - set linked list content  
 *
 */
static void rk28_dma_setup_llp(u32 sar, 
                                         u32 dar, 
                                         struct rk28_dma_llp *curllp, 
                                         struct rk28_dma_llp *nexllp, 
                                         u32 ctll,
                                         u32 size)
{            
    curllp->sar  = sar; //pa
    curllp->dar  = dar; //pa
    curllp->ctll = ctll;	
    curllp->llp  = nexllp; //physical next linked list pointer
    curllp->size = size;
}

/**
 * rk28_dma_end_llp - set linked list end  
 *
 */
static void rk28_dma_end_llp(struct rk28_dma_llp *curllp)
{
    curllp->llp = 0; 
    curllp->ctll &= (~B_CTLL_LLP_DST_EN) & (~B_CTLL_LLP_SRC_EN);
    curllp->ctll |= B_CTLL_INT_EN;
}

/**
 * rk28_dma_setup_sg - setup rk28 DMA channel SG list to/from device transfer
 * @dma_ch: rk28 device ID which using DMA, device id list is showed in dma.h
 * @sg: pointer to the scatter-gather list/vector
 * @sgcount: scatter-gather list hungs count
 * @dmamode: DMA transfer mode, 
                %DMA_MODE_READ from the device to the memory
 *              %DMA_MODE_WRITE from memory to the device
 *
 * The function sets up DMA channel state and registers to be ready for transfer
 * specified by provided parameters. The scatter-gather emulation is set up
 * according to the parameters.
 *
 * enbale dma should be called after setup sg
 *
 * Return value: negative if incorrect parameters
 * Zero indicates success.
 */
s32 rk28_dma_setup_sg(rk28_dmach_t dma_ch,
		                      struct scatterlist *sg, 
		                      u32 sgcount,
		                      u32 dmamode)
{
    u32 ret;
    u32 wid_off;
    u32 sg_length_tmp;
    u32 ctll_r, ctll_w, dev_addr_tmp, dev_addr_bak;
    u32 sgcount_tmp, i, bk_count, bk_res;
    struct rk28_dma_channel *rk28dma;
    struct rk28_dma_llp * rk28llp_vir;
    struct rk28_dma_llp * rk28llp_phy;
    struct scatterlist *sg_tmp;

	if (dma_ch >= RK28_DMA_MAX) {
		printk(KERN_CRIT "%s: called for non-existed channel %d\n",
		       __func__, dma_ch);
		return -ENODEV;
	}
	
    rk28dma = &rk28_dma_channels[dma_ch];
    
	if (!rk28dma->dev_info) {
		printk(KERN_CRIT "%s: trying to setup channel %d which is already freed\n",
		       __func__, dma_ch);
		return -ENODEV;
	}

	if (rk28dma->dma_status == RK28_DMA_BUSY) {
		printk(KERN_CRIT "%s channel %d is using\n",
		       __func__, dma_ch);
		return -EBUSY;
	}
	
	if (!sg) {
		printk(KERN_ERR "%s: illegal sg list be used by channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}

	if (!sgcount || !sg->length) {
		printk(KERN_ERR "%s: epty sg list be used by channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}
	
	
    wid_off = rk28dma->dev_info->fifo_width >> 4;
    dev_addr_tmp = rk28dma->dev_info->dev_addr;
	rk28dma->sg = sg;
	rk28dma->sgcount = sgcount;
	rk28dma->dma_mode = dmamode;

    if ((dmamode & DMA_MODE_MASK) == DMA_MODE_READ) {            
        ctll_r = rk28_dma_control_for_read(rk28dma->dev_info);
    } else {//DMA_MODE_WRITE            
        ctll_w = rk28_dma_control_for_write(rk28dma->dev_info);
    }

	#if 0
    if (rk28dma->dev_info->dma_phy_ch != RK28_DMA_CH2){
	#else /*[xjh] rk2818 */
	if (rk28dma->dev_info->dma_phy_ch < RK28_DMA_CH2){
	#endif

        rk28llp_vir = rk28dma->dma_llp_vir;
        rk28llp_phy = (struct rk28_dma_llp *)rk28dma->dma_llp_phy;
        sg_tmp = sg;
        /*setup linked list table start*/        
        if ((dmamode & DMA_MODE_MASK) == DMA_MODE_READ) {          
            for (sgcount_tmp = sgcount; sgcount_tmp >= 1; sgcount_tmp--, sg_tmp++) { 
                bk_count = (sg_tmp->length >> wid_off) / RK28_DMA_CH0A1_MAX_LEN;
                bk_res = (sg_tmp->length >> wid_off) % RK28_DMA_CH0A1_MAX_LEN;
				if(bk_count > RK28_DMA_LLPS -1 )
				{
					printk("Error: DMA transfer data size is too big!!!\n");
					return -EINVAL;
				}
				
				dmaprintk("[xjh] %s sgcount_tmp 0x%x bk_count 0x%x bk_res 0x%x\n", 
								__FUNCTION__, sgcount_tmp, bk_count, bk_res);
                for (i = bk_count; i >= 1; i--) { 
					dmaprintk("[xjh] %s dma_llp_vir 0x%x \n", __FUNCTION__, rk28llp_vir);
					dev_addr_bak = ( rk28dma->dev_info->hd_if == RK28_DMA_SDRAM) ?  
									dev_addr_tmp + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off): dev_addr_tmp;
                    rk28_dma_setup_llp(dev_addr_bak /*dev_addr_tmp*/, 
                                   sg_tmp->dma_address + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off), 
                                   rk28llp_vir++,
                                   ++rk28llp_phy,
                                   ctll_r,
                                   RK28_DMA_CH0A1_MAX_LEN);
                }
                if (bk_res > 0) {
					dev_addr_bak = ( rk28dma->dev_info->hd_if == RK28_DMA_SDRAM) ?  
									dev_addr_tmp + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off): dev_addr_tmp;
					dmaprintk("[xjh] %s cur_dma_addr 0x%x dev_addr_tmp 0x%x bk_count 0x%x i 0x%x rk28llp_phy 0x%x\n", 
									__FUNCTION__, sg_tmp->dma_address, dev_addr_tmp, bk_count,i,rk28llp_phy);
                    rk28_dma_setup_llp(dev_addr_bak/*dev_addr_tmp*/, 
                                   sg_tmp->dma_address + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off), 
                                   rk28llp_vir++,
                                   ++rk28llp_phy,
                                   ctll_r,
                                   bk_res);
                }
            }
        } else {//DMA_MODE_WRITE 
            for (sgcount_tmp = sgcount; sgcount_tmp >= 1; sgcount_tmp--, sg_tmp++) { 
                bk_count = (sg_tmp->length >> wid_off) / RK28_DMA_CH0A1_MAX_LEN;
                bk_res = (sg_tmp->length >> wid_off) % RK28_DMA_CH0A1_MAX_LEN;
				if(bk_count > RK28_DMA_LLPS -1 )
				{
					printk("Error: DMA transfer data size(block count=0x%x) is too big!!!\n",bk_count);
					return -EINVAL;
				}
				dmaprintk("[xjh] %s sgcount_tmp 0x%x bk_count 0x%x bk_res 0x%x\n", 
								__FUNCTION__, sgcount_tmp, bk_count, bk_res);
                for (i = bk_count; i >= 1; i--) { 
					dmaprintk("[xjh] %s dma_llp_vir 0x%x \n", __FUNCTION__, rk28llp_vir);
					dev_addr_bak = ( rk28dma->dev_info->hd_if == RK28_DMA_SDRAM) ?  
									dev_addr_tmp + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off): dev_addr_tmp;
                    rk28_dma_setup_llp(sg_tmp->dma_address + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off), 
                                   dev_addr_bak/*dev_addr_tmp*/,
                                   rk28llp_vir++,
                                   ++rk28llp_phy,
                                   ctll_w,
                                   RK28_DMA_CH0A1_MAX_LEN);
                }
                if (bk_res > 0) {
					dev_addr_bak = ( rk28dma->dev_info->hd_if == RK28_DMA_SDRAM) ?  
									dev_addr_tmp + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off): dev_addr_tmp;
                    rk28_dma_setup_llp(sg_tmp->dma_address + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off), 
                                   dev_addr_bak/*dev_addr_tmp*/,
                                   rk28llp_vir++,
                                   ++rk28llp_phy,
                                   ctll_w,
                                   bk_res);
                }
            }
        }
        rk28_dma_end_llp(rk28llp_vir - 1);

        /*setup linked list table end*/
		dmaprintk("[xjh] %s setup linked list table end \n", __FUNCTION__);
        rk28printk("\n %s..%s..%d    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__);
        rk28printk("\n %s..%d..%d..0x%08x    ******** nzy ********* \n",__FUNCTION__, bk_count, bk_res, (--sg_tmp)->dma_address + (bk_count - i) * (RK28_DMA_CH0A1_MAX_LEN << wid_off));
    }

    bk_count = ((sg->length >> wid_off) >= RK28_DMA_CH2_MAX_LEN) ? RK28_DMA_CH2_MAX_LEN :(sg->length >> wid_off);
    bk_res = (rk28dma->sg->length >> wid_off) - bk_count; 
    rk28dma->dma_block = bk_count;
    rk28dma->res_block = bk_res;
    rk28dma->cur_dma_addr = rk28dma->sg->dma_address;
    
    rk28printk("\n %s..%s..%d    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__);
    rk28printk("\n dma_block=%d..res_block=%d..wid_off=%d    ******** nzy ********* \n",bk_count, bk_res, wid_off);
	
	dmaprintk("[xjh] %s cur_dma_addr 0x%x dev_addr_tmp 0x%x bk_count 0x%x rk28dma->dma_llp_phy 0x%x\n", 
									__FUNCTION__, rk28dma->cur_dma_addr, dev_addr_tmp, bk_count,rk28dma->dma_llp_phy);

    /*setup dma register start*/
    if ((dmamode & DMA_MODE_MASK) == DMA_MODE_READ) {
        rk28_dma_setup_reg(dma_ch,
                           dev_addr_tmp, 
                           sg->dma_address, 
                           ctll_r,
                           (struct rk28_dma_llp *)rk28dma->dma_llp_phy,
                           bk_count);
    } else {//DMA_MODE_WRITE
        rk28_dma_setup_reg(dma_ch,
                           sg->dma_address, 
                           dev_addr_tmp, 
                           ctll_w,
                           (struct rk28_dma_llp *)rk28dma->dma_llp_phy,
                           bk_count);
    }
    /*setup dma register end*/

       
    rk28printk("\n %s..%s..%d    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__);
	
	return 0;
}

/**
 * rk28_dma_enable - function to start rk28 DMA channel operation
 * @dma_ch: rk28 device ID which using DMA, device id list is showed in dma.h
 *
 * The channel has to be allocated by driver through rk28_dma_request()
 * The transfer parameters has to be set to the channel registers through
 * call of the rk28_dma_setup_sg() function.

 */
s32 rk28_dma_enable(rk28_dmach_t dma_ch)
{	
	rk28_flags_t flags;
    struct rk28_dma_channel *rk28dma;

	if (dma_ch >= RK28_DMA_MAX) {
		printk(KERN_CRIT "%s: called for  non-existed channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}

    rk28dma = &rk28_dma_channels[dma_ch];
    
	if (!rk28dma->dev_info) {
		printk(KERN_CRIT "%s: trying to enable channel %d which is already freed %d\n",
		       __func__, dma_ch);
		return -ENODEV;
	}	

	if (rk28dma->dma_status == RK28_DMA_BUSY) {
		printk(KERN_CRIT "%s: channel %d is using\n",
		       __func__, dma_ch);
		return -EBUSY;
	}
	
	local_irq_save(flags);
    rk28dma->dma_status = RK28_DMA_BUSY;
	ENABLE_DWDMA(rk28_dma_info.reg_vir_base, rk28dma->dev_info->dma_phy_ch);
    rk28printk("\n %s..%s..%d..DWDMA_ChEnReg=0x%08x    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__, read_dma_reg(rk28_dma_info.reg_vir_base, DWDMA_ChEnReg));
	local_irq_restore(flags);
    return 0;	
}

/**
 * rk28_dma_disable - stop, finish rk28 DMA channel operatin
 * @dma_ch: rk28 device ID which using DMA, device id list is showed in dma.h
 *
 * dma transfer will be force into suspend state whether dma have completed current transfer
 */
s32 rk28_dma_disable(rk28_dmach_t dma_ch)
{	
	rk28_flags_t flags;
    struct rk28_dma_channel *rk28dma;
    int ret = 0;

	if (dma_ch >= RK28_DMA_MAX) {
		printk(KERN_CRIT "%s: called for  non-existed channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}
	
    rk28dma = &rk28_dma_channels[dma_ch];
    
	if (!rk28dma->dev_info) {
		printk(KERN_CRIT "%s: trying to disable channel %d which is already freed %d\n",
		       __func__, dma_ch);
		return -ENODEV;
	}
	local_irq_save(flags);
    rk28dma->dma_status = RK28_DMA_IDLE;
	DISABLE_DWDMA(rk28_dma_info.reg_vir_base, rk28dma->dev_info->dma_phy_ch);
	while (WHETHER_DWDMA_FREE(rk28_dma_info.reg_vir_base, rk28dma->dev_info->dma_phy_ch)) {
        cpu_relax();  
	};//wait until channel is free
	local_irq_restore(flags);
    return ret;	
}

/**
 * rk28_dma_request - request/allocate specified channel number
 * @dma_ch: rk28 device ID which using DMA, device id list is showed in dma.h
 * @irq_handler: pointer to function which is called after 
 *               dma have completed transfer
 * @data: pointer to data which input irq_handler
 *
 * requesting dma channel if device need dma transfer 
 * but just called one time in one event, and channle should be 
 * free after this event  
 */
s32 rk28_dma_request(rk28_dmach_t dma_ch,
                            void (*irq_handler) (s32, void *),
		                    void *data)
{  
	struct rk28_dma_channel *rk28dma;
	u32 i, regbase, phy_ch;
	rk28_flags_t flags;
	int ret;

	if (dma_ch >= RK28_DMA_MAX) {
		printk(KERN_CRIT "%s: called for  non-existed channel %d\n",
		       __func__, dma_ch);
		ret = -ENODEV;
		goto err;
	}
	
	rk28dma = &rk28_dma_channels[dma_ch];
	regbase = rk28_dma_info.reg_vir_base;
	
	local_irq_save(flags);
	
	if (rk28dma->dev_info) {
		printk(KERN_CRIT "%s: trying to request virtual channel %d which is already requested %d\n",
		       __func__, dma_ch);			
		local_irq_restore(flags);
		ret = -EMFILE;
		goto err;
	}
	
    rk28dma->dev_info = &rk28_dev_info[dma_ch];
    rk28dma->dev_info->dma_phy_ch = rk28dma->dev_info->dma_dft_ch;
	phy_ch = rk28dma->dev_info->dma_phy_ch;
#if 1	
	if (rk28_dma_info.dma_flag[phy_ch] != RK28_DMA_NULL) {
        printk(KERN_WARNING "%s: no default physical channel can be requested by %d\n",
               __func__, dma_ch);           
        for (i = 0; i < RK28_DMA_CHANNELS; i++) {
            if (rk28_dma_info.dma_flag[i] == RK28_DMA_NULL) {
                rk28dma->dev_info->dma_phy_ch = i;
                phy_ch = i;
                goto next;
            } 
        }
        local_irq_restore(flags);
        printk(KERN_CRIT "%s: no physical channel can be requested by %d\n",
               __func__, dma_ch);  
		ret = -EBUSY;
		goto err1;
    }
next:  

#else    
	if (rk28_dma_info.dma_flag[phy_ch] != RK28_DMA_NULL) {
        printk(KERN_CRIT "%s: trying to request physical channel %d which is already requested %d\n",
               __func__, phy_ch);           
        local_irq_restore(flags);
		return -EBUSY;
	}
#endif
    rk28_dma_info.dma_flag[phy_ch] = dma_ch;
	rk28dma->irq_handler = irq_handler;
	rk28dma->data = data;
    rk28dma->dma_status = RK28_DMA_IDLE;

	dmaprintk("[xjh] %s DMA phy channel %d is requested\n", __FUNCTION__, phy_ch);

	#if 0
	if (phy_ch != RK28_DMA_CH2) {
	#else /*[xjh] rk2818*/
	if (phy_ch < RK28_DMA_CH2) {
	#endif	
        rk28dma->dma_llp_vir = (struct rk28_dma_llp *)dma_alloc_coherent(NULL, 
                                                                         RK28_DMA_LLPS*sizeof(struct rk28_dma_llp), 
                                                                         &rk28dma->dma_llp_phy,
                                                                         GFP_KERNEL);
        if (!rk28dma->dma_llp_vir) {
            printk(KERN_CRIT "%s: no dma space can be request by virtual channel %d\n",
                   __func__, dma_ch); 
            local_irq_restore(flags);
            ret = -ENOMEM;
            goto err2;
        }
		else
		{
			dmaprintk("[xjh] %s sdma_llp_vir 0x%x dma_llp_phy 0x%x\n", __FUNCTION__, rk28dma->dma_llp_vir,rk28dma->dma_llp_phy);
		}
	}

    local_irq_restore(flags);

    rk28printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
    rk28printk("\n vir=0x%08x phy=0x%08x ******** nzy ********* \n", (u32)rk28dma->dma_llp_vir, rk28dma->dma_llp_phy);
	
	/* clear interrupt */
    CLR_DWDMA_INTR(regbase, phy_ch);

	/* Unmask interrupt */
    UN_MASK_DWDMA_INTR(regbase, phy_ch);

    rk28printk("\n %s..%s..%d..%d    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__,dma_ch);
	return 0;

err2:
    rk28_dma_info.dma_flag[phy_ch] = RK28_DMA_NULL;
	rk28dma->irq_handler = NULL;
	rk28dma->data = NULL;
err1:
    rk28dma->dev_info = NULL;
err:
	return ret;
}

/**
 * rk28_dma_free - release previously acquired channel
 * @dma_ch: rk28 device ID which using DMA, device id list is showed in dma.h
 *
 * request dam should be prior free dma
 */
s32 rk28_dma_free(rk28_dmach_t dma_ch)
{
	u32 regbase, phy_ch;
	rk28_flags_t flags;
	struct rk28_dma_channel *rk28dma;
    int ret =0;
    
	if (dma_ch >= RK28_DMA_MAX) {
		printk(KERN_CRIT "%s: called for  non-existed channel %d\n",
		       __func__, dma_ch);
		return -ENODEV;
	}

	rk28dma = &rk28_dma_channels[dma_ch];

	if (!rk28dma->dev_info) {
		printk(KERN_CRIT
		       "%s: trying to free channel %d which is already freed\n",
		       __func__, dma_ch);
		return -ENODEV;       
	}
	
	regbase = rk28_dma_info.reg_vir_base;
	phy_ch = rk28dma->dev_info->dma_phy_ch;


    /* Disable chn */
	DISABLE_DWDMA(regbase, phy_ch);
	while (WHETHER_DWDMA_FREE(regbase, phy_ch)) {
        cpu_relax();  
	};//wait until channel is free

	
	/* clear interrupt */
    CLR_DWDMA_INTR(regbase, phy_ch);

	/* Mask interrupt */
    MASK_DWDMA_INTR(regbase, phy_ch);

	#if 0
	if (phy_ch != RK28_DMA_CH2) {
	#else /*[xjh] rk2818*/
	if (phy_ch < RK28_DMA_CH2) {
	#endif
        if (!rk28dma->dma_llp_vir) {
            printk(KERN_CRIT "%s: no dma space can be free by virtual channel %d\n",
                   __func__, dma_ch);  
            ret = -ENOMEM;       
            goto free;       
        }
        dma_free_coherent(NULL, 
                          RK28_DMA_LLPS*sizeof(struct rk28_dma_llp), 
                          (void *)rk28dma->dma_llp_vir,
                          rk28dma->dma_llp_phy);
        rk28printk("\n %s..%s..%d    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__);
    }

	
	dmaprintk("[xjh] %s DMA phy channel %d is free\n", __FUNCTION__, phy_ch);
    
free:   
	local_irq_save(flags);
	
    rk28_dma_info.dma_flag[phy_ch] = RK28_DMA_NULL;
	rk28dma->dev_info = NULL;
	rk28dma->irq_handler = NULL;
	rk28dma->data = NULL;
	rk28dma->sg = NULL;
	rk28dma->sgcount = 0;
	rk28dma->dma_block = 0;
	rk28dma->res_block = 0;
    rk28dma->dma_status = RK28_DMA_IDLE;
    
	local_irq_restore(flags);

    rk28printk("\n %s..%s..%d    ******** nzy ********* \n",__FUNCTION__,__FILE__,__LINE__);
	return ret;
}

/**
 * rk28_dma_next - set dma regiters and start of next transfer if using channel 2   
 * @dma_ch: rk28 device ID which using DMA, device id list is showed in dma.h
 *
 * just be applied to channel 2
 */
static int rk28_dma_next(rk28_dmach_t dma_ch)
{
	struct rk28_dma_channel *rk28dma = &rk28_dma_channels[dma_ch];

	if (!rk28dma->dev_info) {
		printk(KERN_CRIT "%s: called for  not allocated channel %d\n",
		       __func__, dma_ch);
		return -ENODEV;
	}

	if (!rk28dma->sg) {
		printk(KERN_ERR "%s: illegal sg list be used by channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}

	if (!rk28dma->sg->length) {
		printk(KERN_ERR "%s: epty sg list be used by channel %d\n",
		       __func__, dma_ch);
		return -EINVAL;
	}
	
	u32 nextlength;
	u32 nextaddr;
    u32 wid_off = rk28dma->dev_info->fifo_width >> 4;
    u32 ctll_r, ctll_w;
	
	if (rk28dma->res_block > 0) {
        rk28dma->cur_dma_addr += (rk28dma->dma_block << wid_off);
        nextaddr = rk28dma->cur_dma_addr;
        nextlength = (rk28dma->res_block >= RK28_DMA_CH2_MAX_LEN) ? RK28_DMA_CH2_MAX_LEN : rk28dma->res_block;
        rk28dma->res_block -= nextlength; 
		if( rk28dma->dev_info->hd_if == RK28_DMA_SDRAM)
			rk28dma->dev_info->dev_addr += (rk28dma->dma_block << wid_off);			
	} else {
        rk28dma->sgcount--;
	    if (rk28dma->sgcount == 0) {
            return 0; // completed
	    } else {
		    if( rk28dma->dev_info->hd_if == RK28_DMA_SDRAM)
		    {
				printk("Error:DMA memory transactions do not support sgcount>1\n");
				return 0;
		    }
            rk28dma->sg++;
            rk28dma->cur_dma_addr = rk28dma->sg->dma_address;
            nextaddr = rk28dma->sg->dma_address;
            nextlength = (rk28dma->sg->length >= RK28_DMA_CH2_MAX_LEN) ? RK28_DMA_CH2_MAX_LEN : rk28dma->sg->length;
            rk28dma->res_block = (rk28dma->sg->length >> wid_off) - nextlength; 
	    }
	}
    rk28dma->dma_block = nextlength;

    rk28printk("\n dma_block=%d..res_block=%d..wid_off=%d..nextaddr=0x%08x    ******** nzy ********* \n",
            rk28dma->dma_block, rk28dma->res_block, wid_off, nextaddr);

    if ((rk28dma->dma_mode & DMA_MODE_MASK) == DMA_MODE_READ) {
        ctll_r = rk28_dma_control_for_read(rk28dma->dev_info);
        rk28_dma_setup_reg(dma_ch,
                           rk28dma->dev_info->dev_addr, 
                           nextaddr, 
                           ctll_r,
                           NULL,
                           nextlength);
    } else {//DMA_MODE_WRITE
        ctll_w = rk28_dma_control_for_write(rk28dma->dev_info);
        rk28_dma_setup_reg(dma_ch,
                           nextaddr, 
                           rk28dma->dev_info->dev_addr, 
                           ctll_w,
                           NULL,
                           nextlength);
    }
    
    ENABLE_DWDMA(rk28_dma_info.reg_vir_base, rk28dma->dev_info->dma_phy_ch);

	return rk28dma->sgcount;
}

/**
 * rk28_dma_irq_handler - irq callback function   
 *
 */
static irqreturn_t rk28_dma_irq_handler(s32 irq, void *dev_id)
{
	/*
	 * Just disable the interrupts. We'll turn them back on in the
	 * softirq handler.
	 */
	
	dmaprintk("[xjh] %s dma interrupt occur\n", __FUNCTION__);
	s32 i, raw_status;
    u32 dma_ch;
	u32 regbase = rk28_dma_info.reg_vir_base;
    struct rk28_dma_channel *rk28dma;

	dmaprintk("%s DMA done!!!\n",__FUNCTION__);
    raw_status = read_dma_reg(regbase, DWDMA_RawTfr);
    
	for (i = 0; i < RK28_DMA_CHANNELS; i++) {
        dma_ch = rk28_dma_info.dma_flag[i];
        if (dma_ch < RK28_DMA_MAX) { // already allocated
            if (raw_status & (1 << i)) {
				last_ch = dma_ch;
                CLR_DWDMA_INTR(regbase, i);
		#if 0
                if ((i == RK28_DMA_CH2) && (rk28_dma_next(dma_ch))){
		#else
				if ((i >= RK28_DMA_CH2) && (rk28_dma_next(dma_ch))){
		#endif
                    dmaprintk(KERN_WARNING
                           "don't finish  for channel %d\n", dma_ch);
                    continue;
                }  
                rk28dma = &rk28_dma_channels[dma_ch];
                if (rk28dma->irq_handler) {
                    rk28dma->irq_handler(dma_ch, rk28dma->data);
                    rk28dma->dma_status = RK28_DMA_IDLE;
                } else {
                    printk(KERN_WARNING
                           "no IRQ handleer for DMA channel %d\n", dma_ch);
                }
            }
        } else {
            rk28printk(KERN_WARNING
                   "spurious IRQ for DMA channel %d\n", i);
        }
	}

	return IRQ_HANDLED;
}

/*
 * 功能:用于修改虚拟通道的地址，在rk28_dma_request()之后调用
 * 参数:
 *		dma_ch:	虚拟通道，0~16
 *		addr  :	DMA地址
 *				对于DMA_MODE_READ是src addr
 *				对于DMA_MODE_WRITE是dest addr	
 */
void rk28_dma_set_addr(rk28_dmach_t dma_ch,void *addr)
{
	struct rk28_dma_channel *rk28dma;
	rk28dma = &rk28_dma_channels[dma_ch];
	//rk28dma->dev_info->dev_addr = (u32)(virt_to_dma(NULL, addr));
	rk28dma->dev_info->dev_addr = (u32)(addr);
}


/**
 * rk28_dma_init - dma information initialize   
 *
 */
static s32 __init rk28_dma_init(void)
{
	s32 ret;
	s32 i;
	u32 regbase;

	for (i = 0; i < RK28_DMA_MAX; i++) {
        rk28_dma_channels[i].dev_info = NULL;
        rk28_dma_channels[i].dma_llp_vir = NULL;
        rk28_dma_channels[i].dma_llp_phy = NULL;
		rk28_dma_channels[i].irq_handler = NULL;
		rk28_dma_channels[i].data = NULL;
		rk28_dma_channels[i].sg = NULL;
		rk28_dma_channels[i].sgcount = 0;
	}
	
    rk28_dma_info.reg_vir_base = DW_DMA_BASE_ADDR_VA;
    rk28_dma_info.dma_irq_num= RK28_DMA_IRQ_NUM;
    
	for (i = 0; i < RK28_DMA_CHANNELS; i++) {
        rk28_dma_info.dma_flag[i] = RK28_DMA_NULL;
    }
    
	regbase = rk28_dma_info.reg_vir_base;



	/* disable DMA module first, then enable*/
	write_dma_reg(regbase, DWDMA_DmaCfgReg, 0x0);
	while(read_dma_reg(regbase, DWDMA_DmaCfgReg) & 0x01)
		printk("wait for dma disable...\n");
	
	/* enable DMA module */
	write_dma_reg(regbase, DWDMA_DmaCfgReg, 0x01);
	
	for ( i = 0; i < RK28_DMA_CHANNELS; i++)
		DISABLE_DWDMA(regbase, i); 
	
	/*init dma reg for hot reboot*/
	for ( i = 0; i < RK28_DMA_CHANNELS; i++) {
		printk("write dma reg channel %d\n",i);
		write_dma_reg(regbase, DWDMA_SAR(i), 0x0);
		write_dma_reg(regbase, DWDMA_DAR(i), 0x0);
		write_dma_reg(regbase, DWDMA_LLP(i), 0x0);
		write_dma_reg(regbase, DWDMA_CTLL(i), 0x0);
		write_dma_reg(regbase, DWDMA_CTLH(i), 0x0);
		//write_dma_reg(regbase, DWDMA_SSTAT(i), 0x0);
		//write_dma_reg(regbase, DWDMA_DSTAT(i), 0x0);
		//write_dma_reg(regbase, DWDMA_SSTATAR(i), 0x0);
		//write_dma_reg(regbase, DWDMA_DSTATAR(i), 0x0);
		write_dma_reg(regbase, DWDMA_CFGL(i), 0x0);
		write_dma_reg(regbase, DWDMA_CFGH(i), 0x0);
		//write_dma_reg(regbase, DWDMA_SGR(i), 0x0);
		//write_dma_reg(regbase, DWDMA_DSR(i), 0x0);
		show_dma_regs(i);
	}


	/* clear all interrupts */
	#if 0 /*rk2808*/
	write_dma_reg(regbase, DWDMA_ClearBlock, 0x707);
	#else
	/*[xjh] for rk2818, 6 DMA channels*/
	write_dma_reg(regbase, DWDMA_ClearBlock, 0x3f3f);
	write_dma_reg(regbase, DWDMA_ClearTfr, 0x3f3f);
	write_dma_reg(regbase, DWDMA_ClearSrcTran, 0x3f3f);
	write_dma_reg(regbase, DWDMA_ClearDstTran, 0x3f3f);
	write_dma_reg(regbase, DWDMA_ClearErr, 0x3f3f);

    /*mask all interrupts*/
	write_dma_reg(regbase, DWDMA_MaskBlock, 0x3f00);
	write_dma_reg(regbase, DWDMA_MaskSrcTran, 0x3f00);
	write_dma_reg(regbase, DWDMA_MaskDstTran, 0x3f00);
	write_dma_reg(regbase, DWDMA_MaskErr, 0x3f00);

    /*unmask transfer completion interrupt*/
	UN_MASK_DWDMA_ALL_TRF_INTR(regbase);
	#endif

    /* 20100303,HSL@RK,enable dma irq after clear all unwant interrupts 
     *  for soft reboot!!.
     */
    ret = request_irq(RK28_DMA_IRQ_NUM, rk28_dma_irq_handler, 0, "DMA", NULL);
	if (ret < 0) {
		printk(KERN_CRIT "Can't register IRQ for DMA\n");
		return ret;
	}
	
	return 0;
}
int rockchip_dma_getposition(rk28_dmach_t channel, rk28_dmach_t *src, rk28_dmach_t *dst)
{
	/*struct rk28_dma_channel *rk28dma = &rk28_dma_channels[channel];
	u32 chnbase = 0;  ///(u32)rk28dma->reg_vir_base;
	if (src != NULL)
 		*src = read_dma_reg(chnbase,DWDMA_SAR);  
 	if (dst != NULL)
 		*dst = read_dma_reg(chnbase,DWDMA_DAR);  
*/
    struct rk28_dma_channel *rk28dma = &rk28_dma_channels[channel];
	u32 phy_ch = rk28dma->dev_info->dma_phy_ch;
	u32 dma_if = rk28dma->dev_info->hd_if;
	u32 regbase = rk28_dma_info.reg_vir_base;
    if (src != NULL)
        *src = read_dma_reg(regbase, DWDMA_SAR(phy_ch));
    if (dst != NULL)     
        *dst = read_dma_reg(regbase, DWDMA_DAR(phy_ch)); 
    
 	return 0;
}
arch_initcall(rk28_dma_init);

EXPORT_SYMBOL(rockchip_dma_getposition);
EXPORT_SYMBOL(rk28_dma_setup_sg);
EXPORT_SYMBOL(rk28_dma_enable);
EXPORT_SYMBOL(rk28_dma_disable);
EXPORT_SYMBOL(rk28_dma_request);
EXPORT_SYMBOL(rk28_dma_free);
EXPORT_SYMBOL(rk28_dma_channels);
EXPORT_SYMBOL(rk28_dma_set_addr);

