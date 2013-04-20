/*
 * file: rk28_msc_ext.c.
 * 20100202,HSL@RK,for old version msc extersion.
*/

extern int fsg_lock( int lock );
extern void kernel_restart(char *cmd);

static int rkusb_omsc_ext_cmd( struct rkusb_dev *dev )
{
        int r = RKUSB_CB_FAILD;
        S_INFO("%s:get msc ext cmd 0x%x\n" , __func__ , DEV_OFFSET(dev) );
        switch( DEV_OFFSET(dev) ){
        case ECC_SWITCH_MSC:
                {
                /* reboot to loader rkusb*/
                //rk28_restart( 1 );
                char *p=(char *)DEV_INREQ_BUF(dev);
                strcpy( p , "loader" );
                rkusb_wakeup_thread( dev );
		  r = RKUSB_CB_OK_NONE;
                break;
                }
        case ECC_RESET:
                {
                /* normal reboot */
                //rk28_restart( 0 );
                char *p=(char *)DEV_INREQ_BUF(dev);
                strcpy( p , "normal" );
                rkusb_wakeup_thread( dev );
		  r = RKUSB_CB_OK_NONE;
                break;
                }
        case ECC_GET_VERSION:
                {
                /* get boot version and fireware version from cmdline
                  * bootver=2010-07-08#4.02 firmware_ver=1.0.0 // Firmware Ver:16.01.0000
                  * return format: 0x02 0x04 0x00 0x00 0x00 0x01 
                  */
                  #define ASC_BCD0( c )  (((c-'0'))&0xf)
                  #define ASC_BCD1( c )  (((c-'0')<<4)&0xf0)
                  
                        char ver[6];
                        char *p_l , *p_f;
                        p_l = strstr( saved_command_line , "bootver=" );
                        if( !p_l ) {
                                break;
                        } 
                        p_f = strstr( p_l , "firmware_ver=" );
                        if( !p_f ) {
                                break;
                        } 
                        if( !(p_l = strnchr( p_l, p_f - p_l , '#'))  )
                                break;
                        p_l++;
                        p_f+=strlen("firmware_ver=");
                        if( p_l[1] == '.' ) {
                                ver[1] = ASC_BCD0(p_l[0]);
                                p_l+=2;
                        } else {
                                ver[1] = ASC_BCD1(p_l[0])|ASC_BCD0(p_l[1]);
                                p_l+=3;
                        }
                        ver[0] = ASC_BCD1(p_l[0])|ASC_BCD0(p_l[1]);
                        if( p_f[1] == '.' ) {
                                ver[5] = ASC_BCD0(p_f[0]);
                                p_f+=2;
                        } else {
                                ver[5] = ASC_BCD1(p_f[0])|ASC_BCD0(p_f[1]);
                                p_f+=3;
                        } 
                        if( p_f[1] == '.' ) {
                                ver[4] = ASC_BCD0(p_f[0]);
                                p_f+=2;
                        } else {
                                ver[4] = ASC_BCD1(p_f[0])|ASC_BCD0(p_f[1]);
                                p_f+=3;
                        } 
                        ver[3] = ASC_BCD1(p_f[0]);
                        p_f++;
                        if( p_f[0] != ' ' ){
                                ver[3] |= ASC_BCD0(p_f[0]);
                                p_f++;
                        }
                        // only support 2 byte version.
                        ver[2] = 0;
                        rkusb_normal_data_xfer_onetime(dev , ver );
                        r = RKUSB_CB_OK_NONE;
                }
                break;
        case ECC_GET_CHIP:
                // return chip tpye, like rk2808,rk2818....
                { 
                unsigned int l ;
                unsigned int *s;
                int     chip=0x1e;
                char    p[16];
                memset( p , 0 , sizeof(p ) );
                p[0] = ' ';
                l = (unsigned int)kld_get_tag_data(0X524B0000,(void**)&s);
                if( l != 0 ){
                    chip = (*s)&0x1e;
                    if( chip == 0x00 ||chip == 0x02 || chip == 0x04 || chip == 0x0c)
                            strcpy( p ,  "RK28" );
                   else  if( chip == 0x14 || chip == 0x12 )
                            //p = "RK281x";
                            strcpy( p ,  "RK281X" );
                }
                S_INFO("get chip size=0x%x,buf=%s,chip=0x%x!\n" , l , p , chip );
                rkusb_normal_data_xfer_onetime(dev ,p );
                r = RKUSB_CB_OK_NONE;        
                }
                break;
        case ECC_SHOW_USER_DISK:
                /* not support !*/
                break;
        case ECC_SWITCH_MTP:
                /* not support !*/
                break;
        case ECC_LOCK_USER_DISK:
                fsg_lock( 1 );
                r = RKUSB_CB_OK_CSW; 
                break;
        case ECC_UNLOCK_USER_DISK:
                fsg_lock( 0 );
                r = RKUSB_CB_OK_CSW; 
                break;
        case ECC_GET_UID:
                {
                unsigned int l,i;
                char    *psn;
                char *p=(char *)DEV_INREQ_BUF(dev);
                char *pr = p;
                l = kld_get_tag_data(0X524B0009,(void**)&psn);
                if( !l || !*psn  )
                        break;
                i = 0;
                l = *psn++;
                while( i < l ) {
                        *p++ = rk28_hex2bcd(*psn>>4);
                        *p++ = rk28_hex2bcd(*psn);
                        i++;
                        psn++;
                }
                *p = 0;
                rkusb_normal_data_xfer_onetime(dev , pr );
                r = RKUSB_CB_OK_NONE;
                }
                
                break;
        case ECC_GET_SN:
                {
                unsigned int l;
                char    *psn;
                unsigned int *pr=(unsigned int *)DEV_INREQ_BUF(dev);
                l = kld_get_tag_data(0X524B0006,(void**)&psn);
                if( !l )
                        break;
                *pr++ = l ;
                memcpy( pr , psn , l );
                rkusb_normal_data_xfer_onetime(dev , pr );
                r = RKUSB_CB_OK_NONE;
                }
                break;
        default:
                r = RKUSB_CB_UNHANDLE;
                break;
        }
        return r;
}

