/*
 * for file xfer throught rkusb.
 * 20091214,HSL,redo.
 * 20100202,HSL@RK,use adb instead,so remark here
*/

#define RKUSB_XFILE                     0

#if RKUSB_XFILE
#define FI_WRITE( fi )                  (fi->rw & 0xff )
#define FI_INVALID( fi )                  ((fi->rw & 0xffffff00 ) != FILE_XFER_MAGIC)
#define FI_ERR_CLEAR                    1025

static int rkusb_xfer_error( FILE_INFO *fi , struct rkusb_dev *dev , int error )
{
        if( fi )  {
                        fi->error = error;
                        rkusb_wakeup_thread( dev );
                }
        return 0;
}
static FILE_INFO *rkusb_xfer_cmd_valid( struct rkusb_dev *dev )
{
        FILE_INFO               *fi = (FILE_INFO*)dev->private_tmp;
        if( !fi || FI_INVALID(fi) )
                return NULL;
        return fi;
}

static void rkusb_xfer_free_fi( struct rkusb_dev *dev )
{
        FILE_INFO               *fi = (FILE_INFO*)dev->private_tmp;
        if( !fi )
                return ;
        if( fi->file ) {
                if( FI_WRITE(fi) ) {
                        do_fsync( fi->file , 1 );
                }
                fput( fi->file );
                if( fi->error && fi->error != -FI_ERR_CLEAR  && FI_WRITE(fi) ) {
                        rk28printk("%s::write file %s error=%d , unlink it\n" , __func__ , fi->file_path , fi->error );
                        sys_unlink( fi->file_path );
                }
        }
        kfree( fi );
        dev->private_tmp = NULL;
}
static int rkusb_xfer_file_path_cb( struct rkusb_dev *dev )
{
        FILE_INFO       *fi = (FILE_INFO*)dev->req_out->buf;
        if( !fi || FI_INVALID(fi) || fi->size < sizeof(FILE_INFO) )
                return RKUSB_CB_FAILD_CSW;
        if( FI_WRITE(fi) && fi->file_size == 0 ) /* write */
                return RKUSB_CB_FAILD_CSW;
        fi = (FILE_INFO*)kmalloc( sizeof( FILE_INFO ), GFP_ATOMIC );
        memcpy( fi , dev->req_out->buf , dev->req_out->actual );
        fi->file = fi->private_data = NULL;
        fi->error = 0;
        if( !FI_WRITE(fi) ) { 
                fi->file_size = 0;
        }
        dev->private_tmp = fi;
        /*20100112,HSL@RK, for get cb at complete out.*/
        DEV_LENGTH(dev) = RKUSB_BULK_CB_WRAP_LEN;
        rkusb_wakeup_thread( dev );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_xfer_file_path( struct rkusb_dev *dev )
{
        if( DEV_LENGTH(dev) < sizeof(FILE_INFO) || DEV_LENGTH(dev) > RKUSB_BUFFER_SIZE ) {
                return RKUSB_CB_FAILD;
        }
        rkusb_normal_data_xfer( dev , rkusb_xfer_file_path_cb );
        return RKUSB_CB_OK_NONE;
}

static int rkusb_xfer_read_filesize( struct rkusb_dev *dev )
{
        FILE_INFO               *fi = rkusb_xfer_cmd_valid( dev );
        if( !fi  || DEV_LENGTH(dev)  < sizeof(unsigned long ) ) {
                rkusb_xfer_error( fi , dev , -FI_ERR_CLEAR);
                return RKUSB_CB_FAILD;
        }
        rk28printk("read file size =0x%lx,trans len=%d\n" , fi->file_size , DEV_LENGTH(dev))
        rkusb_normal_data_xfer_onetime(dev , &fi->file_size);
        return RKUSB_CB_OK_NONE;
}

static int rkusb_xfer_file( struct rkusb_dev *dev )
{
        FILE_INFO               *fi = rkusb_xfer_cmd_valid( dev );
        if( !fi || DEV_LENGTH(dev) > fi->file_size ||
                DEV_OFFSET(dev) > fi->file_size ||
                DEV_OFFSET(dev)+DEV_LENGTH(dev)>fi->file_size ) {
                rkusb_xfer_error( fi , dev , -EINVAL);
                return RKUSB_CB_FAILD;
        }
        #if 0
        if( FI_WRITE(fi) ) { /* write */
                dev->phase = RKUSB_PHASE_DATAOUT;
        } else {
                dev->phase = RKUSB_PHASE_DATAIN;
        }
        #endif
        rkusb_wakeup_thread( dev );
        return RKUSB_CB_OK_NONE;
}

/* call at irq , rkusb_command(dev ) 
*/
static int rkusb_do_xfer_file( struct rkusb_dev *dev )
{
        int     r = RKUSB_CB_FAILD;
        switch( DEV_FUNC(dev )) {
        case FUNC_XFER_FILE_PATH:
                r = rkusb_xfer_file_path( dev );
                break;
        case FUNC_RFILE_INFO:
                r = rkusb_xfer_read_filesize( dev );
                break;
        case FUNC_RFILE_DATA:
        case FUNC_WFILE_DATA:
                r = rkusb_xfer_file( dev );
                break;
        default:
                //rkusb_raise_exception( dev , RKUSB_PHASE_EXIT );
                break;
        }
        return r;
}

/*
 * call at main thread.
 */
static int rkusb_file_path( struct rkusb_dev *dev )
{
        FILE_INFO               *fi = (FILE_INFO*)dev->private_tmp;
        struct file                *fp;
        struct inode	        *inode = NULL;
        int                             flags ;
        if( FI_WRITE(fi) ) { /* write */
                flags = O_TRUNC|O_CREAT|O_WRONLY;
        } else {
                flags = O_RDONLY;
        }
        fp = filp_open(fi->file_path , flags , (fi->mod&0777) );
        rk28printk("%s::open file %s ,rw=0x%x,mod=0%o\n" , __func__ , fi->file_path , fi->rw , fi->mod );
        if (IS_ERR(fp)) {
                fi->error = (int)fp;
        	printk("%s::unable to open file: %s,ret=%d\n",__func__ , fi->file_path , fi->error );
        	goto failed;
        }
        if( !FI_WRITE(fi) )  {  /* file read ,get file size */
                inode = fp->f_path.dentry->d_inode;
                if( !inode || !S_ISREG( inode->i_mode ) ) {
                        fi->error = -EINVAL;
                        fput( fp );
                        goto failed;
                }
                fi->mod = inode->i_mode;
                fi->file_size = (unsigned long)i_size_read(inode);
                /* 20100115,HSL@RK,for sysfs,procfs ,the size will be zero or not right */
                if( !fi->file_size && (!strncmp(fi->file_path,"/proc/" , 6)||!strncmp(fi->file_path,"/sys/" , 5))) {
                        //fi->mod |= FILE_MOD_TREAT;
                        fi->file_size = PAGE_SIZE; 
                }
        }
        fi->file = fp;
        rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        return 0;
failed: 
        rkusb_send_csw( dev , RKUSB_STATUS_FAIL );
        return -1;
}

static int rkusb_rw_file_cb( struct rkusb_dev *dev )
{
        FILE_INFO               *fi = rkusb_xfer_cmd_valid( dev );
        int data_in = DEV_DATA_IN(dev);
        int actual = dev->req_out->actual;
        if( data_in )
                actual = dev->req_in->actual;
        DEV_LENGTH( dev ) -= actual;
        //rk28printk("%s::remain len=0x%x,actual=0x%x\n" ,__func__, DEV_LENGTH(dev) ,actual );
        #if 0
        if( DEV_LENGTH(dev) > 0 ) {
                if(!fi || (fi->error && fi->error != - FI_ERR_CLEAR) ) { /* error !*/
                        __rkusb_set_dest_or_source( dev , rwb->buf );
                        rkusb_normal_data_xfer_max_size( dev , rkusb_rw_file_cb , 0X40000000 );
                        return RKUSB_CB_FAILD_CSW;
                } 
        }
        #endif
        /* 20091222,HSL@RK,write last packed */
        if( FI_WRITE(fi) ) {
                dev->buf_filled = 1;
        } else {
                dev->buf_filled = 0;
        }
        rkusb_wakeup_thread( dev );
        return RKUSB_CB_OK_NONE;
}

static int rkusb_send_data_buffer( struct rkusb_dev *dev ,
        int len , data_cb cb )
{
        int ret;
        dev->phase = RKUSB_PHASE_DATAIN;
        dev->xfer_cb = cb;
        dev->req_in->length = len;
        ret = usb_ep_queue(dev->ep_in,  dev->req_in, GFP_ATOMIC);
       return ret;
}

static int rkusb_get_data_buffer( struct rkusb_dev *dev ,
        int len , data_cb cb)
{
        int ret;
        dev->phase = RKUSB_PHASE_DATAOUT;
        dev->xfer_cb = cb;
        dev->req_out->length = len;
        ret = usb_ep_queue(dev->ep_out,  dev->req_out, GFP_ATOMIC);
        return ret;
}



/* 
  * 20091221,add file offset for large file.
 */
static int rkusb_xfer_file_task( struct rkusb_dev *dev )
{
        int     nread;
        int     mount;
        int     total;
        int     real_tranfs;
        FILE_INFO               *fi = (FILE_INFO*)dev->private_tmp;
        char                            *buf_bak;
        loff_t                          pos = DEV_OFFSET(dev);
        real_tranfs = mount = __rkusb_rwbuffer_end(dev) -__rkusb_rwbuffer_start(dev);
        
        dev->buf_filled = 0;
        fi->error = 0;
        total = 0;
        if( FI_WRITE(fi) ) 
                buf_bak = rkusb_reset_req_buf(dev->req_out,dev->buf);
        else
                buf_bak = rkusb_reset_req_buf(dev->req_in,dev->buf);;
        for(; ;) {
                if (signal_pending(current))
                        break;
                if( FI_WRITE(fi) ) {
                        if( DEV_LENGTH( dev ) < mount ) {
                                //mount = DEV_LENGTH( dev ) ;
                                real_tranfs = DEV_LENGTH( dev ) ; 
                                //rk28printk("vfs r/w last pack , len =0x%x\n" , real_tranfs );
                        }
                        //rk28printk("total write=0x%x,remain=0x%x,file size=0x%lx,mount=0x%x\n" ,
                        //        total , DEV_LENGTH(dev) , fi->file_size , mount );
                        #if 1
                        rkusb_get_data_buffer( dev , mount , rkusb_rw_file_cb );
                        #else
                        //__rkusb_set_dest_or_source( dev , rwb->buf );
                        rkusb_normal_data_xfer_max_size( dev , rkusb_rw_file_cb , mount);
                        #endif
                        while( !dev->buf_filled ) {
                                rkusb_sleep_thread( dev );
                        }
                        if( fi->error )
                                break;
                        nread = vfs_write(fi->file, dev->buf , real_tranfs ,&pos);
                        if( nread < 0 ) {
                                rk28printk("vfs write failed , ret =%d\n" , nread );
                                fi->error = nread;
                        } else {
                                dev->buf_filled = 0;
                                total += nread;
                        }
                        if( DEV_LENGTH( dev ) == 0 )
                                break;
                } else {
                        //rk28printk("total read=0x%x,remain=0x%x,file size=0x%lx,mount=0x%x\n" ,
                        //        total , DEV_LENGTH(dev) , fi->file_size , mount );
                        while( dev->buf_filled ) {
                                rkusb_sleep_thread( dev );
                        }
                        if( fi->error )
                                break;
                        if( DEV_LENGTH( dev ) == 0 )
                                break;
                        nread = vfs_read(fi->file, dev->buf , real_tranfs ,&pos);
                        if( nread < 0 ) {
                                rk28printk("vfs read failed , ret =%d\n" , nread );
                                fi->error = nread;
                        } else {
                                dev->buf_filled = 1;
                                total += nread;
                                if( nread < real_tranfs && total < fi->file_size ) {
                                        /* 20100115,HSL@RK,set 0 at pc tools */
                                        //rwb->buf[nread] = 0;
                                        //if( real_tranfs - nread > 5 )
                                        //        strcpy(rwb->buf+nread+1,"@#$");
                                        nread = real_tranfs;
                                }
                        }
                        //rk28printk("read finish,nread=0x%x,start transfer,real transfer=0x%x\n" , 
                        //        nread , real_tranfs );
                        #if 1
                        rkusb_send_data_buffer( dev , nread , rkusb_rw_file_cb );
                        #else
                        __rkusb_set_dest_or_source( dev , rwb->buf );
                        rkusb_normal_data_xfer_max_size( dev , rkusb_rw_file_cb , mount);
                        #endif
                }
        }
        rk28printk("file transfer finish,error=%d\n" , fi->error );
        if( FI_WRITE(fi) ) {
                rkusb_reset_req_buf(dev->req_out , buf_bak);
                rkusb_get_cb( dev , RKUSB_BULK_CB_WRAP_LEN );
        } else
                rkusb_reset_req_buf(dev->req_in , buf_bak);
        
        /* close file , free FILE_INFO */
        if( !fi->error ) {
                fi->error = - FI_ERR_CLEAR;
                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        } else {
                rkusb_send_csw( dev , RKUSB_STATUS_FAIL );
        }
        return 0;
}
#else
#define rkusb_do_xfer_file( dev )               RKUSB_CB_FAILD
#endif /* RKUSB_XFILE */ 

/****************************************************************/
/*
 *  20100108,HSL@RK,new r/w sdram code. if not kernel space,read at thread one time.
 */
static int rkusb_do_read_sdram( struct rkusb_dev *dev )
{
        int r;
        char *dest = __rkusb_rwbuffer_start( dev );
        #if 0
        r= copy_from_user(dest , (void __user *) DEV_OFFSET( dev ),
                DEV_LENGTH(dev) );
        #else
        r = access_process_vm(dev->slave_task,  DEV_OFFSET( dev ), dest , DEV_LENGTH(dev) , 0);
        #endif
        rk28printk("%s::read task=%s,lba=0x%x,len=%d\n" , __func__ , dev->slave_task->comm,
                DEV_OFFSET( dev ) , DEV_LENGTH(dev));
        if( r  !=  DEV_LENGTH(dev)) {
                rk28printk("get user data error,task=%s,lba=0x%x,r=0x%x\n" , dev->slave_task->comm,DEV_OFFSET( dev ) , r);
        }
        rkusb_normal_data_xfer_onetime( dev , dest );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_read_sdram( struct rkusb_dev *dev )
{
        int pid;
        if( __rkusb_read_enable() ) {
                if( DEV_OFFSET(dev) >= 0xc0000000 ) {
                        rkusb_normal_data_xfer( dev , NULL ); 
                        return RKUSB_CB_OK_NONE;
                }
                pid = DEV_PID( dev );
                if( pid ) {
                        dev->slave_task = find_task_by_pid( pid );
                        if(!dev->slave_task )
                                return RKUSB_CB_FAILD;
                        //rk28printk("%s::pid=%d,task=%s\n" , __func__ , pid , dev->slave_task->comm );
                } else {
                        dev->slave_task = current ;
                }
                if( dev->slave_task == current ){
                        char *dest = __rkusb_rwbuffer_start( dev );
                        int n = copy_from_user((void*)dest, (const void __user*)DEV_OFFSET(dev), DEV_LENGTH(dev));
                        if( !n ) {
                                rkusb_normal_data_xfer_onetime( dev , dest );
                                return RKUSB_CB_OK_NONE;
                        } else 
                                return RKUSB_CB_FAILD;
                }
                rkusb_wakeup_thread( dev );
                return RKUSB_CB_OK_NONE;
        }
        return RKUSB_CB_FAILD;
}

static int rkusb_do_write_sdram( struct rkusb_dev *dev )
{
        int r;
        int     status = RKUSB_STATUS_PASS;
        char *src = __rkusb_rwbuffer_start( dev );
        rk28printk("%s::copy to user,task=%s,lba=0x%p,len=0x%x\n" , __func__ , dev->slave_task->comm,
             dev->private_tmp , dev->buf_nfill );
        if( dev->slave_task == current ) {
                r= copy_to_user((void __user *) dev->private_tmp,
                        src , dev->buf_nfill );
                if( r )
                        status = RKUSB_STATUS_FAIL;
        } else {
                r = access_process_vm(dev->slave_task,(unsigned long)dev->private_tmp,
                        src , dev->buf_nfill , 1 );
                if( r  != dev->buf_nfill ) {
                        status = RKUSB_STATUS_FAIL;
                        rk28printk("set user data error,task=%s,lba=0x%p,r=0x%x\n" , dev->slave_task->comm, dev->private_tmp, r);
                }
        }
        rkusb_send_csw( dev ,  status);
        return RKUSB_CB_OK_NONE;
}
static int rkusb_write_sdram_cb( struct rkusb_dev *dev )
{
        DEV_LENGTH(dev) = RKUSB_BULK_CB_WRAP_LEN;
        rkusb_wakeup_thread( dev );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_write_sdram( struct rkusb_dev *dev )
{
        int pid;
        if( __rkusb_write_enable() ) {
                int max_len;
                if( DEV_OFFSET(dev) >= 0xc0000000 ) {
                        rkusb_normal_data_xfer( dev , NULL ); 
                        return RKUSB_CB_OK_NONE;
                }
                max_len = DEV_RWBUF_LENGTH( dev );
                if( max_len < DEV_LENGTH(dev) ) {
                            return RKUSB_CB_FAILD;
                }
                pid = DEV_PID( dev );
                if( pid ) {
                        dev->slave_task = find_task_by_pid( pid );
                        if(!dev->slave_task )
                                return RKUSB_CB_FAILD;
                        //rk28printk("%s::pid=%d,task=%s\n" , __func__ , pid , dev->slave_task->comm );
                } else {
                        dev->slave_task = current ;
                }
                /* bakeup real offset and length */
                dev->private_tmp = (void*)DEV_OFFSET(dev);
                dev->buf_nfill = DEV_LENGTH(dev);
                __rkusb_set_dest_or_source( dev , __rkusb_rwbuffer_start( dev ) );
                rkusb_normal_data_xfer( dev , rkusb_write_sdram_cb);
                return RKUSB_CB_OK_NONE;
        }
        return RKUSB_CB_FAILD;
}

/********************************************************/
#if 0 /* use read sdram instead */
/* copy from fs/proc/base.c proc_pid_cmdline */
static int rkusb_get_task_cmdline(struct task_struct *task, 
        char * buffer , int size )
{
	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(task);
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;	/* Shh! No looking before we're done */

 	len = mm->arg_end - mm->arg_start;
 
	if (len > size)
		len = size;
 
	res = access_process_vm(task, mm->arg_start, buffer, len, 0);

	// If the nul at the end of args has been overwritten, then
	// assume application is using setproctitle(3).
	if (res > 0 && buffer[res-1] != '\0' && len < size) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > size - res)
				len = size - res;
			res += access_process_vm(task, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	return res;
}

static int rkusb_do_get_taskcmd_cb( struct rkusb_dev *dev )
{
        rkusb_send_csw_result( dev , dev->buf[0].filled );
        return RKUSB_CB_OK_NONE;
}
static int rkusb_do_get_task_cmd( struct rkusb_dev *dev )
{
        struct task_struct *p = dev->slave_task;
        int len = rkusb_get_task_cmdline( p , __rkusb_rwbuffer_start(dev) , DEV_RWBUF_LENGTH( dev ) ); 
        __rkusb_set_dest_or_source( dev , __rkusb_rwbuffer_start(dev) );
        rkusb_normal_data_xfer( dev , rkusb_do_get_taskcmd_cb );
        dev->buf[0].filled = len;
        //rkusb_normal_data_xfer_onetime( dev , __rkusb_rwbuffer_start(dev) );
        return RKUSB_CB_OK_NONE;
}

static int rkusb_get_task_cmd( struct rkusb_dev *dev )
{
        struct task_struct *p;
        int     pid = DEV_OFFSET(dev);
        if( DEV_LENGTH(dev) > DEV_RWBUF_LENGTH( dev ) )
                return RKUSB_CB_FAILD;
        if( pid == 0 )
                p = current ;
        else {
                p = find_task_by_pid( pid );
                if( !p )
                        return RKUSB_CB_FAILD;
        }
        dev->slave_task = p;
        return RKUSB_CB_OK_NONE;
}
#else
#define rkusb_do_get_task_cmd(dev)      RKUSB_CB_OK_NONE
#define rkusb_get_task_cmd(dev )        RKUSB_CB_FAILD
#endif
static int rkusb_do_func_call( struct rkusb_dev *dev )
{
        int ret = __rk28_scu_parse_cmd((char*) dev->req_out->buf);
        rkusb_send_csw_result( dev , ret );
        return RKUSB_CB_OK_NONE;
}

/*
 * for cmd must run at task env.not irq env.
 */
static int rkusb_task_cmd( struct rkusb_dev *dev )
{
        int r = RKUSB_CB_FAILD;
        switch( DEV_FUNC(dev )) {
        case FUNC_TASKF_PING:
                /*20100108,HSL@RK, for get cb at complete out.*/
                DEV_LENGTH(dev) = RKUSB_BULK_CB_WRAP_LEN;
                r= RKUSB_CB_OK_NONE;
                break;
        default:
                break;
        }
        if( r != RKUSB_CB_FAILD )
                rkusb_wakeup_thread( dev );
        return r;
}

static int rkusb_do_task_cmd( struct rkusb_dev *dev )
{
        switch( DEV_FUNC(dev )) {
        case FUNC_TASKF_PING:
                rkusb_send_csw( dev , RKUSB_STATUS_PASS );
                break;
        default:
                return RKUSB_CB_FAILD;
        }
        return RKUSB_CB_OK_NONE;
}

static int rkusb_do_thread_cmd( struct rkusb_dev *dev )
{
        rk28printk("rkusb thread cmd = 0x%x,func=0x%x,len=%d\n" , 
            DEV_CMD(dev),DEV_FUNC(dev) , DEV_LENGTH(dev));
        switch( DEV_CMD(dev)) {
        #if RKUSB_XFILE
        case K_FW_XFERFILE:
                {
                        FILE_INFO               *fi = rkusb_xfer_cmd_valid( dev );
                        if( !fi ) {
                                rk28printk("invalid FILE_INFO for K_FW_XFERFILE[0x%x]\n" , K_FW_XFERFILE );
                                break;
                        }
                        switch( DEV_FUNC(dev )) {
                        case FUNC_XFER_FILE_PATH:
                                rkusb_file_path( dev );
                                break;
                        case FUNC_RFILE_DATA:
                        case FUNC_WFILE_DATA:
                                rkusb_xfer_file_task( dev );
                                break;
                        default:
                                break;
                        }
                        if( fi->error )
                                rkusb_xfer_free_fi( dev );
                }
                break;
        #endif /* RKUSB_XFILE */
        case K_FW_TASKFUN:
                rkusb_do_task_cmd( dev );
                break;
        case K_FW_SDRAM_READ_10:
                rkusb_do_read_sdram( dev );
                break;
        case K_FW_SDRAM_WRITE_10:
                rkusb_do_write_sdram( dev );
                break;
        case K_FW_FUNCALL:
                rkusb_do_func_call( dev );
                break;
         case K_FW_USBCMD : 
                if(FUNC_UCMD_RKUSBVER== DEV_FUNC(dev ) && 
                    (DEV_OFFSET(dev) == ECC_SWITCH_MSC || DEV_OFFSET(dev) == ECC_RESET) ) {
                        char cmd[32];
                        char *p=(char *)DEV_INREQ_BUF(dev);
                        strncpy( cmd , p , sizeof(cmd) );
                        rkusb_send_csw( dev , RKUSB_STATUS_PASS );
        		  mdelay(10);
                        sys_sync();
        		  kernel_restart( cmd );
		  } else {
		         rkusb_send_csw( dev , RKUSB_STATUS_FAIL );
		  }
		  break;
        default:
                return -ENOTSUPP;
                break;
        }
        return 0;
}

/* 20100724,HSL@RK,copy from cpu freq.
*/
#include <linux/kernel_stat.h>

static cputime64_t      last_idle_ticks;
static cputime64_t      last_total_ticks;

static struct timer_list cal_load_timer;
static int cal_loading;
static inline cputime64_t rkusb_get_cpu_idle_time(unsigned int cpu)
{
	cputime64_t idle_time;
	cputime64_t cur_jiffies;
	cputime64_t busy_time;
	cur_jiffies = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);

	busy_time = cputime64_add(busy_time,	kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_jiffies, busy_time);
	S_INFO("%s:jiffies=%Ld,busy=%Ld,idle=%Ld" , __FILE__,
	        cur_jiffies,busy_time,idle_time);
	return idle_time;
}

/* Get Idle Time */
unsigned int rkusb_calc_laod( unsigned long data )
{
                unsigned int idle_ticks, total_ticks;
                unsigned int load = 0;
	//cputime64_t cur_jiffies;
	unsigned int j;
	int     next = (int)data;
	idle_ticks = UINT_MAX;
	{
		cputime64_t total_idle_ticks;
		cputime64_t total_total_ticks;
		unsigned int tmp_idle_ticks;
		j = 0;
		total_total_ticks = jiffies64_to_cputime64(get_jiffies_64());
	                total_ticks = (unsigned int) cputime64_sub(total_total_ticks,last_total_ticks);
	                last_total_ticks = get_jiffies_64();
	                
		total_idle_ticks = rkusb_get_cpu_idle_time(j);
		tmp_idle_ticks = (unsigned int) cputime64_sub(total_idle_ticks,last_idle_ticks);
		last_idle_ticks = total_idle_ticks;
		if (tmp_idle_ticks < idle_ticks)
			idle_ticks = tmp_idle_ticks;
	}
	if (likely(total_ticks > idle_ticks))
		load = (100 * (total_ticks - idle_ticks)) / total_ticks;
                mod_timer( &cal_load_timer , jiffies + (next*HZ));
                S_ERR("%s:load=%d,tick=%d\n" , __func__ , load , next); 
                return load;
}
int rk28_cal_load( int start , int sec )
{
        if( start  ) {
                if( sec == 0 )
                        sec = 1;
                if(cal_loading)
                        del_timer(&cal_load_timer);
                else
                        init_timer( &cal_load_timer );
                cal_load_timer.function = (void (*)(unsigned long))rkusb_calc_laod;
                cal_load_timer.data = (unsigned long)sec;
                mod_timer( &cal_load_timer ,jiffies + (HZ));
                last_total_ticks = get_jiffies_64();
                last_idle_ticks = rkusb_get_cpu_idle_time(0);
                cal_loading = 1;
        } else {
                del_timer(&cal_load_timer);
                cal_loading = 0;
        }
        return 0;
}
/* 20100311,HSL@RK,function for open/close profile demanded. 
  * call at irq when needed.
  * doing all profile function at pc host.
  * 20100401,HSL@RK,add funtion for profile user.
  * we may need to know whitch task call the kernel function.so add pid.
  * 20100626,HSL@RK,add profile irq ctrl.and use high byte of pid to record last irq.
  * 20100723,HSL@RK,move from profile.c to here.
 */
struct prof_entry{
        int     pid;
        unsigned long pc;
} ;
static struct prof_entry *prof_pc; 
static struct prof_entry *prof_user;
static int      prof_user_shift,prof_kernel_shift;
static unsigned long prof_count;
static unsigned int      prof_irq_mask32;
static unsigned int      prof_irq_mask48;


/* 20100723,HSL@RK,debug only , for cmp with pc tools. */
static int prof_num_irqs[NR_RK28_IRQS];
int profile_open( unsigned int irq_mask , int buf_len , unsigned int irq_mask48 )
{
        /*  
          * THREAD_SIZE.
          * MAX time = 4K/8/HZ = 2.5s.
        */
        if( prof_pc )
                return 0;
        printk("%s::irq mask=0x%x,buf_len=0x%x",__func__ , irq_mask,buf_len);
        if( !buf_len )
                buf_len = THREAD_SIZE;
        prof_pc = kzalloc(buf_len,GFP_ATOMIC);
        if( prof_pc == NULL ) {
                return -ENOMEM;
        }
        prof_kernel_shift = 0; /* as index */
        prof_user_shift = 0;
        // user/kernel 50%.
        prof_count = (buf_len>>1)/sizeof(struct prof_entry);
        prof_user = prof_pc+prof_count; /* one FOR kernel,last for user */
        prof_irq_mask32 = 0xffffffffU;
        prof_irq_mask48 = 0xffU;
        if( irq_mask )
                prof_irq_mask32 = irq_mask;
        if( irq_mask48 )
                prof_irq_mask48 = prof_irq_mask48;
        memset( prof_num_irqs,0,sizeof(prof_num_irqs) );               
        return 0;
}
int profile_clsoe( void )
{
        if( prof_pc ) {
                kfree(prof_pc);
                prof_pc = NULL;
        }
        {
        int i;
        for( i = 0 ; i < NR_RK28_IRQS ; i++ ){
                if( prof_num_irqs[i] > 0 ){
                        S_WARN("irq%02d:%d\n" , i , prof_num_irqs[i]);
                }
        }
        }
        return 0;
}
void profile_irq_check( int irq )
{
	struct pt_regs *regs ;
	unsigned long pc;
	if( !prof_pc )
	        return ;
	if ( (irq < 32 && (prof_irq_mask32& (1<<irq)))
	        || (irq >= 32 && prof_irq_mask48& (1<<(irq-32)))) {
	                regs = get_irq_regs();
	                pc = profile_pc(regs);
	                if( user_mode(regs) ) {
	                                prof_user[prof_user_shift].pc = pc;
	                                prof_user[prof_user_shift].pid = current->pid;
	                                prof_user[prof_user_shift].pid |= (irq<<24)&0xff000000;
                		prof_user_shift++;
                		if( prof_user_shift == prof_count )
                		        prof_user_shift --;
	                } else {
                		prof_pc[prof_kernel_shift].pc = pc;
                		prof_pc[prof_kernel_shift].pid = current->pid;
                		prof_pc[prof_kernel_shift].pid |= (irq<<24)&0xff000000;
                		prof_kernel_shift++;
                		if( prof_kernel_shift == prof_count ){
                		        S_WARN("profile kernel buf overflow\n");
                		        prof_kernel_shift --; // only cover last one.
                		}
                                }
                                prof_num_irqs[irq]++;
	} 
}
int profile_check( void *pc_buf , int len , int* kernel , int *user )
{
        if( !prof_pc )
                return 0;
        memcpy(pc_buf,prof_pc,len);
        *kernel = prof_kernel_shift;
        *user = prof_user_shift;
        prof_kernel_shift = 0;
        prof_user_shift = 0;
        return prof_count;
}

