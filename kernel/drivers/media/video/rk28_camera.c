/*
 * V4L2 Driver for RK28 VIP (camera interface)
 *
 * Copyright (C) 2008 Rockchip Inc.
 *
 * Based on V4L2 Driver for PXA camera host - "pxa_camera.c",
 *
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <media/videobuf-dma-contig.h>

/* register offsets for VIP */
#define RK28_VIP_AHBR_CTRL				0x00
#define RK28_VIP_INT_MASK				0x04
#define RK28_VIP_INT_STS				0x08
#define RK28_VIP_STS					0x0c
#define RK28_VIP_CTRL					0x10
#define RK28_VIP_CAPTURE_F1SA_Y 		0x14
#define RK28_VIP_CAPTURE_F1SA_UV		0x18
#define RK28_VIP_CAPTURE_F1SA_Cr		0x1c
#define RK28_VIP_CAPTURE_F2SA_Y 		0x20
#define RK28_VIP_CAPTURE_F2SA_UV		0x24
#define RK28_VIP_CAPTURE_F2SA_Cr		0x28
#define RK28_VIP_FB_SR					0x2c
#define RK28_VIP_FS						0x30
#define RK28_VIP_VIPRESERVED			0x34
#define RK28_VIP_CROP					0x38
#define RK28_VIP_CRM					0x3c
#define RK28_VIP_RESET					0x40
#define RK28_VIP_L_SFT					0x44

/* 
 * VIP Vsync polarity (bit 7 of register CPU_APB_REG5),
 * compatible with rk280x. for rk281x, there's alreday a 
 * Vsync sentive bit in RK28_VIP_CTRL register also for this.
 */
#define RK28_CPU_API_REG                  		(REG_FILE_BASE_ADDR_VA+0x14)
#define VSY_POL_CTRL					0x07

/* bit offsets for RK28_VIP_CTRL register */
#define CAPTURE_SET						0x00
#define HREF_SENTIVE					0x01
#define SENSOR_OR_656					0x02
#define YUV_IN_ORDER					0x03
#define YUV_OR_RAW						0x04
#define ONE_FRAME_EN					0x05
#define YUV422_OUT_EN					0x06
#define CCIR_FILED						0x07
#define PING_PONG_EN				 	0x08
#define PCLK_EDGE						0x09
#define CCIR656_FMT				       	0x10
#ifdef RK281X_VIP	/* for rk281x only */
#define END_SET							0x11
#define VSYNC_SENTIVE					0x12
#define RAW_INPUT_EDGE					0x13	/* 2 bits, 13&14 */
#define VIP_CTRL_RESERVED				0x15	/* 15~31 bits reserved */
#endif

/* bit definitions for RK28_VIP_CTRL register */
#define DISABLE_CAPTURE 				(0 << CAPTURE_SET)
#define ENABLE_CAPTURE 					(1 << CAPTURE_SET)

#define HSY_HIGH_ACTIVE 				(0 << HREF_SENTIVE)
#define HSY_LOW_ACTIVE					(1 << HREF_SENTIVE)

#define CCIR656							(0 << SENSOR_OR_656)
#define SENSOR							(1 << SENSOR_OR_656)

#define SENSOR_UYVY		 				(0 << YUV_IN_ORDER)
#define SENSOR_YUYV 					(1 << YUV_IN_ORDER)

#define VIPREGYUV 						(0 << YUV_OR_RAW)
#define VIPREGRAW 						(1 << YUV_OR_RAW)

#define CON_OR_PIN		 				(0 << ONE_FRAME_EN)
#define ONEFRAME	 					(1 << ONE_FRAME_EN)

#define VIPREGYUV420 					(0 << YUV422_OUT_EN)
#define VIPREGYUV422					(1 << YUV422_OUT_EN)

#define FIELD0_START 					(0 << CCIR_FILED)
#define FIELD1_START					(1 << CCIR_FILED)

#define CONTINUOUS						(0 << PING_PONG_EN)
#define PING_PONG						(1 << PING_PONG_EN)

#define POSITIVE_EDGE					(0 << PCLK_EDGE)
#define NEGATIVE_EDGE					(1 << PCLK_EDGE)

#define VIPREGNTSC						(0 << CCIR656_FMT)
#define VIPREGPAL						(1 << CCIR656_FMT)

#ifdef RK281X_VIP	/* for rk281x only */
#define LITTLE_END						(0 << END_SET)
#define BIG_END							(1 << END_SET)

#define VSY_LOW_ACTIVE					(0 << VSYNC_SENTIVE)
#define VSY_HIGH_ACTIVE 				(1 << VSYNC_SENTIVE)

#define RAW_INPUT_BYPASS				(0 << RAW_INPUT_EDGE)
#define RAW_INPUT_POSEDGE				(1 << RAW_INPUT_EDGE)
#define RAW_INPUT_NEGEDGE				(2 << RAW_INPUT_EDGE)
#define RAW_INPUT_DEFAULT				(3 << RAW_INPUT_EDGE)
#else
#define VSY_LOW_ACTIVE					(0 << VSY_POL_CTRL)
#define VSY_HIGH_ACTIVE 				(1 << VSY_POL_CTRL)
#endif

#define RK28_CAM_DRV_NAME "rk28-camera"

#define write_vip_reg(addr, val)        	__raw_writel(val, addr+VIP_BASE_ADDR_VA) 
#define read_vip_reg(addr)              	__raw_readl(addr+VIP_BASE_ADDR_VA)    
#define set_vip_vsp(val)    			__raw_writel(((val) | __raw_readl(RK28_CPU_API_REG)), RK28_CPU_API_REG)

struct camera_info {
	unsigned long flags; /* SOCAM_... */
	void (*enable_camera)(void);
	void (*disable_camera)(void);
};

static void camera_power_on(void){
    rockchip_mux_api_set(GPIOF6_VIPCLK_SEL_NAME, IOMUXB_VIP_CLKOUT);
    write_vip_reg(RK28_VIP_AHBR_CTRL, 0x05);//only 8 or 4 will be the actual length.
    write_vip_reg(RK28_VIP_INT_MASK, 0x01);//capture complete interrupt enable
    write_vip_reg(RK28_VIP_CRM,  0x00000000);//Y/CB/CR color modification
    write_vip_reg(RK28_VIP_FB_SR,  0x00000003);//frame1 has been ready to receive data,frame 2 is not used
}

static struct camera_info rk28_camera_info = {
	.flags = SOCAM_MASTER                       | \
                   SOCAM_HSYNC_ACTIVE_HIGH   | \
                   SOCAM_HSYNC_ACTIVE_LOW    | \
                   SOCAM_VSYNC_ACTIVE_HIGH   | \
                   SOCAM_VSYNC_ACTIVE_LOW    | \
                   SOCAM_PCLK_SAMPLE_RISING  | \
                   SOCAM_PCLK_SAMPLE_FALLING | \
                   SOCAM_SENSOR_UYVY              | \
                   SOCAM_SENSOR_YUYV              | \
                   SOCAM_DATAWIDTH_8,
	.enable_camera = camera_power_on,
};

static DEFINE_MUTEX(camera_lock);

/* per video frame buffer */
struct rk28_buffer {
	struct videobuf_buffer vb; /* v4l buffer must be first */
	const struct soc_camera_data_format        *fmt;
};

struct rk28_camera_dev {
	struct device		*dev;
	struct soc_camera_host ici;
	struct soc_camera_device *icd;
	struct resource		*res;

	unsigned int		irq;
	void __iomem		*base;
	unsigned long video_limit;

	/* lock used to protect videobuf */
	spinlock_t		lock;
	struct list_head capture;
	struct videobuf_buffer	*active;

	struct camera_info *pdata;
};

/*
 *  Videobuf operations
 */
static int rk28_videobuf_setup(struct videobuf_queue *vq,
					unsigned int *count,
					unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	int bytes_per_pixel = (icd->current_fmt->depth + 7) >> 3;

	*size = PAGE_ALIGN(icd->width * icd->height * bytes_per_pixel);

	if (0 == *count)
		*count = 4;

	if (pcdev->video_limit) {
		while (*size * *count > pcdev->video_limit)
			(*count)--;
	}

	dev_dbg(&icd->dev, "count=%d, size=%d\n", *count, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq,
			struct rk28_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		&buf->vb, buf->vb.baddr, buf->vb.bsize);

	if (in_interrupt())
		BUG();

	videobuf_dma_contig_free(vq, &buf->vb);
	dev_dbg(&icd->dev, "%s freed\n", __func__);
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}
		
static inline void rk28_videobuf_capture(struct videobuf_buffer *vb)
{
      unsigned int size;

    if (vb) {        
        size = vb->width * vb->height; /* Y pages UV pages, yuv422*/
		write_vip_reg(RK28_VIP_CAPTURE_F1SA_Y, vb->boff);
        write_vip_reg(RK28_VIP_CAPTURE_F1SA_UV, vb->boff + size);
        write_vip_reg(RK28_VIP_CAPTURE_F2SA_Y, vb->boff);
        write_vip_reg(RK28_VIP_CAPTURE_F2SA_UV, vb->boff + size);
        write_vip_reg(RK28_VIP_FB_SR,  0x00000002);//frame1 has been ready to receive data,frame 2 is not used        
    }
}

static int rk28_videobuf_prepare(struct videobuf_queue *vq,
					  struct videobuf_buffer *vb,
					  enum v4l2_field field)
{
    struct soc_camera_device *icd = vq->priv_data;
    struct rk28_buffer *buf;
    int ret;
    
    buf = container_of(vb, struct rk28_buffer, vb);
    
    dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
        vb, vb->baddr, vb->bsize);
    
    /* Added list head initialization on alloc */
    WARN_ON(!list_empty(&vb->queue));
    
#ifdef DEBUG
	/* This can be useful if you want to see if we actually fill
	 * the buffer with something */
	memset((void *)vb->baddr, 0xaa, vb->bsize);
#endif
    
    BUG_ON(NULL == icd->current_fmt);
    
    if (buf->fmt    != icd->current_fmt ||
        vb->width   != icd->width ||
        vb->height  != icd->height ||
        vb->field   != field) {
        buf->fmt    = icd->current_fmt;
        vb->width   = icd->width;
        vb->height  = icd->height;
        vb->field   = field;
        vb->state   = VIDEOBUF_NEEDS_INIT;
    }
    
    vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3) * 2;
    if (0 != vb->baddr && vb->bsize < vb->size) {
        ret = -EINVAL;
        goto out;
    }
    
    if (vb->state == VIDEOBUF_NEEDS_INIT) {
        ret = videobuf_iolock(vq, vb, NULL);
        if (ret)
            goto fail;
        vb->state = VIDEOBUF_PREPARED;
    }
    
    return 0;
fail:
    free_buffer(vq, buf);
out:
    return ret;
}

static void rk28_videobuf_queue(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	unsigned long flags;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		vb, vb->baddr, vb->bsize);

	vb->state = VIDEOBUF_ACTIVE;
	spin_lock_irqsave(&pcdev->lock, flags);
	list_add_tail(&vb->queue, &pcdev->capture);

	if (!pcdev->active) {
		pcdev->active = vb;
        rk28_videobuf_capture(vb);
	}

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void rk28_videobuf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	free_buffer(vq, container_of(vb, struct rk28_buffer, vb));
}

static struct videobuf_queue_ops rk28_videobuf_ops = {
	.buf_setup      = rk28_videobuf_setup,
	.buf_prepare    = rk28_videobuf_prepare,
	.buf_queue      = rk28_videobuf_queue,
	.buf_release    = rk28_videobuf_release,
};

static irqreturn_t rk28_camera_irq(int irq, void *data)
{
	struct rk28_camera_dev *pcdev = data;
	struct videobuf_buffer *vb;
	unsigned long flags;
	unsigned long int_sts = read_vip_reg(RK28_VIP_INT_STS);//clear vip interrupte single

	spin_lock_irqsave(&pcdev->lock, flags);

	vb = pcdev->active;
	list_del_init(&vb->queue);

	if (!list_empty(&pcdev->capture))
		pcdev->active = list_entry(pcdev->capture.next,
					   struct videobuf_buffer, queue);
	else
		pcdev->active = NULL;

    rk28_videobuf_capture(pcdev->active);

	vb->state = VIDEOBUF_DONE;
	do_gettimeofday(&vb->ts);
	vb->field_count++;
	wake_up(&vb->done);
	spin_unlock_irqrestore(&pcdev->lock, flags);

	return IRQ_HANDLED;
}

static int rk28_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	int ret = -EBUSY;

	mutex_lock(&camera_lock);

	if (pcdev->icd)
		goto err;

	dev_info(&icd->dev,
		"RK28 Camera driver attached to camera %d\n",
		 icd->devnum);

	if (pcdev->pdata->enable_camera)
		pcdev->pdata->enable_camera();

	ret = icd->ops->init(icd);
	if (ret)
		goto err;


	pcdev->icd = icd;
err:
	mutex_unlock(&camera_lock);

	return ret;
}

static void rk28_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	icd->ops->release(icd);
	if (pcdev->pdata->disable_camera)
		pcdev->pdata->disable_camera();

	dev_info(&icd->dev,
		"RK28 Camera driver detached from camera %d\n",
		 icd->devnum);

	pcdev->icd = NULL;
}

static int rk28_camera_set_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	int ret;
	unsigned long camera_flags, common_flags, value = 0;
	
	camera_flags = icd->ops->query_bus_param(icd);
	common_flags = soc_camera_bus_param_compatible(camera_flags,
						       pcdev->pdata->flags);
	if (!common_flags)
		return -EINVAL;

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	switch (pixfmt) {
	case V4L2_PIX_FMT_YUV422P:
		value |= VIPREGYUV422;
		break;
	case V4L2_PIX_FMT_YUV420:
		value |= VIPREGYUV420;
		break;
	default:
		return -EINVAL;
    }

#ifdef RK281X_VIP	/* for rk281x only */
	value |= (common_flags & SOCAM_VSYNC_ACTIVE_LOW) ? (VSY_LOW_ACTIVE) : (VSY_HIGH_ACTIVE);
#else
	if (common_flags & SOCAM_VSYNC_ACTIVE_HIGH)
		set_vip_vsp(VSY_HIGH_ACTIVE);
	else
		set_vip_vsp(VSY_LOW_ACTIVE);
#endif

	value |= (common_flags & SOCAM_PCLK_SAMPLE_FALLING) ? (NEGATIVE_EDGE) : (POSITIVE_EDGE);
	value |= (common_flags & SOCAM_HSYNC_ACTIVE_LOW) ? (HSY_LOW_ACTIVE) : (HSY_HIGH_ACTIVE);
	value |= (common_flags & SOCAM_SENSOR_YUYV) ? (SENSOR_YUYV) : (SENSOR_UYVY);
	
	value |= SENSOR | ONEFRAME | ENABLE_CAPTURE;	//need to be modified,nzy add
	
	write_vip_reg(RK28_VIP_CTRL, value);

	return 0;
}

static int rk28_camera_try_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	unsigned long camera_flags, common_flags;

	camera_flags = icd->ops->query_bus_param(icd);
	common_flags = soc_camera_bus_param_compatible(camera_flags,
						       pcdev->pdata->flags);
	if (!common_flags)
		return -EINVAL;

	return 0;
}

static int rk28_camera_set_fmt_cap(struct soc_camera_device *icd,
				  __u32 pixfmt, struct v4l2_rect *rect)
{
#if 1
    unsigned int sensorSize = ((rect->width + rect->left) << 16) + rect->height;
    unsigned int imageSize  = (rect->left << 16) + rect->top;
#else
    unsigned int sensorSize;
    unsigned int imageSize;
    unsigned int crop_x;
    unsigned int crop_y;

    if ((rect->width < 640) && (rect->height < 480)){
        crop_x = (640 - rect->width) >> 1;
        crop_y = (480 - rect->height) >> 1;
        sensorSize = ((rect->width + crop_x) << 16) + rect->height + crop_y;
        imageSize  = (crop_x << 16) + crop_y;
    } else {
        sensorSize = (rect->width << 16) + rect->height;
        imageSize  = (rect->left << 16) + rect->top;
    }
#endif    
    printk("\n%s..%s..%d    ******** nzy *********left = %d, top = %d, width = %d, height = %d\n",__FUNCTION__,__FILE__,__LINE__, rect->left, rect->top, rect->width, rect->height);

    write_vip_reg(RK28_VIP_CROP, imageSize);
    write_vip_reg(RK28_VIP_FS, sensorSize);
    
	return icd->ops->set_fmt_cap(icd, pixfmt, rect);
}

static int rk28_camera_try_fmt_cap(struct soc_camera_device *icd,
				  struct v4l2_format *f)
{
	/* limit to rk28 hardware capabilities */

	if (f->fmt.pix.height < 4)
		f->fmt.pix.height = 4;
#ifdef RK281X_VIP	/* 10M pixels */
	if (f->fmt.pix.height > 2764)
		f->fmt.pix.height = 2764;
#else				/* 3M pixels */
	if (f->fmt.pix.height > 1536)
		f->fmt.pix.height = 1536;
#endif
	if (f->fmt.pix.width < 2)
		f->fmt.pix.width = 2;
#ifdef RK281X_VIP	/* 10M pixels */
	if (f->fmt.pix.width > 3856)
		f->fmt.pix.width = 3856;
#else				/* 3M pixels */
	if (f->fmt.pix.width > 2048)
		f->fmt.pix.width = 2048;
#endif
	f->fmt.pix.width &= ~0x01;
	f->fmt.pix.height &= ~0x03;

	/* limit to sensor capabilities */
	return icd->ops->try_fmt_cap(icd, f);
}

static int rk28_camera_reqbufs(struct soc_camera_file *icf,
			      struct v4l2_requestbuffers *p)
{
	int i;

	/* This is for locking debugging only. I removed spinlocks and now I
	 * check whether .prepare is ever called on a linked buffer, or whether
	 * a dma IRQ can occur for an in-work or unlinked buffer. Until now
	 * it hadn't triggered */
	for (i = 0; i < p->count; i++) {
		struct rk28_buffer *buf;

		buf = container_of(icf->vb_vidq.bufs[i],
				   struct rk28_buffer, vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static unsigned int rk28_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct rk28_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next,
			 struct rk28_buffer, vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN|POLLRDNORM;

	return 0;
}

static int rk28_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	strlcpy(cap->card, RK28_CAM_DRV_NAME, sizeof(cap->card));
	cap->version = KERNEL_VERSION(0, 0, 5);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}
static void rk28_camera_init_videobuf(struct videobuf_queue *q,
			      struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;

	videobuf_queue_dma_contig_init(q,
				       &rk28_videobuf_ops,
				       &ici->dev, &pcdev->lock,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE,
				       V4L2_FIELD_NONE,
				       sizeof(struct rk28_buffer),
				       icd);
}

static struct soc_camera_host_ops rk28_soc_camera_host_ops = {
	.owner			= THIS_MODULE,
	.add 			= rk28_camera_add_device,
	.remove			= rk28_camera_remove_device,
	.set_fmt_cap		= rk28_camera_set_fmt_cap,
	.try_fmt_cap		= rk28_camera_try_fmt_cap,
	.reqbufs			= rk28_camera_reqbufs,
	.poll				= rk28_camera_poll,
	.querycap		= rk28_camera_querycap,
	.try_bus_param	= rk28_camera_try_bus_param,
	.set_bus_param	= rk28_camera_set_bus_param,
	.init_videobuf 		= rk28_camera_init_videobuf,
};

static int rk28_camera_probe(struct platform_device *pdev)
{
	struct rk28_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || !irq) {
		dev_err(&pdev->dev, "Not enough CEU platform resources.\n");
		err = -ENODEV;
		goto exit;
	}
	
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	platform_set_drvdata(pdev, pcdev);
	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);

	pdev->dev.platform_data = &rk28_camera_info;	/* should have set in rk28_devices.c */
	pcdev->pdata = pdev->dev.platform_data;
	if (!pcdev->pdata) {
		err = -EINVAL;
		dev_err(&pdev->dev, "CEU platform data not set.\n");
		goto exit_kfree;
	}
	
	base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (!base) {
		err = -ENXIO;
		dev_err(&pdev->dev, "Unable to ioremap VIP registers.\n");
		goto exit_kfree;
	}
	
	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->video_limit = 0; /* only enabled if third resource exists */
	pcdev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res) {
		err = dma_declare_coherent_memory(&pdev->dev, res->start,
						  res->start,
						  (res->end - res->start) + 1,
						  DMA_MEMORY_MAP |
						  DMA_MEMORY_EXCLUSIVE);
		if (!err) {
			dev_err(&pdev->dev, "Unable to declare CEU memory.\n");
			err = -ENXIO;
			goto exit_iounmap;
		}

		pcdev->video_limit = (res->end - res->start) + 1;
	}

	/* request irq */
	err = request_irq(pcdev->irq, rk28_camera_irq,  IRQF_DISABLED,
			  pdev->dev.bus_id, pcdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to register VIP interrupt.\n");
		goto exit_release_mem;
	}

	pcdev->ici.priv = pcdev;
	pcdev->ici.dev.parent = &pdev->dev;
	pcdev->ici.nr = pdev->id;
	pcdev->ici.drv_name = pdev->dev.bus_id,
	pcdev->ici.ops = &rk28_soc_camera_host_ops,

	err = soc_camera_host_register(&pcdev->ici);
	if (err)
		goto exit_free_irq;

	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_release_mem:
	if (platform_get_resource(pdev, IORESOURCE_MEM, 2))
		dma_release_declared_memory(&pdev->dev);
exit_iounmap:
	iounmap(base);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __devexit rk28_camera_remove(struct platform_device *pdev)
{
	struct rk28_camera_dev *pcdev = platform_get_drvdata(pdev);

	soc_camera_host_unregister(&pcdev->ici);
	free_irq(pcdev->irq, pcdev);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
	iounmap(pcdev->base);
	kfree(pcdev);
	return 0;
}

static struct platform_driver rk28_camera_driver = {
	.driver 	= {
		.name	= RK28_CAM_DRV_NAME,
	},
	.probe		= rk28_camera_probe,
	.remove		= rk28_camera_remove,
};

static int __devinit rk28_camera_init(void)
{
	return platform_driver_register(&rk28_camera_driver);
}

static void __exit rk28_camera_exit(void)
{
	platform_driver_unregister(&rk28_camera_driver);
}

module_init(rk28_camera_init);
module_exit(rk28_camera_exit);

MODULE_DESCRIPTION("RK28 Camera VIP driver");
MODULE_AUTHOR("nzy <nzy@rock-chips.com>");
MODULE_LICENSE("GPL");
