/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *                                        
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 
******************************************************************************/
#define _RTW_IOCTL_SET_C_


#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <rtw_ioctl_set.h>
#include <hal_init.h>

#ifdef CONFIG_USB_HCI
#include <usb_osintf.h>
#include <usb_ops.h>
#endif
#ifdef CONFIG_SDIO_HCI
#include <sdio_osintf.h>
#endif

extern void indicate_wx_scan_complete_event(_adapter *padapter);

#define IS_MAC_ADDRESS_BROADCAST(addr) \
( \
	( (addr[0] == 0xff) && (addr[1] == 0xff) && \
		(addr[2] == 0xff) && (addr[3] == 0xff) && \
		(addr[4] == 0xff) && (addr[5] == 0xff) )  ? _TRUE : _FALSE \
)

u8 rtw_validate_ssid(NDIS_802_11_SSID *ssid)
{
	u8	 i;
	u8	ret=_TRUE;

_func_enter_;	

	if (ssid->SsidLength > 32) {
		RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_, ("ssid length >32\n"));
		ret= _FALSE;
		goto exit;
	}

	for(i = 0; i < ssid->SsidLength; i++)
	{
		//wifi, printable ascii code must be supported
		if(!( (ssid->Ssid[i] >= 0x20) && (ssid->Ssid[i] <= 0x7e) )){
			RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_, ("ssid has nonprintabl ascii\n"));
			ret= _FALSE;
			break;
		}
	}

exit:	

_func_exit_;

	return ret;
}

u8 rtw_do_join(_adapter * padapter)
{
	_list	*plist, *phead;
	u8* pibss = NULL;
	struct	mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	_queue	*queue	= &(pmlmepriv->scanned_queue);
	u8 ret=_SUCCESS;
	
	phead = get_list_head(queue);
	plist = get_next(phead);

_func_enter_;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("\n rtw_do_join: phead = %p; plist = %p \n\n\n", phead, plist));

	pmlmepriv->cur_network.join_res = -2;
		
	set_fwstate(pmlmepriv, _FW_UNDER_LINKING);

	pmlmepriv->pscanned = plist;

	pmlmepriv->to_join = _TRUE;

	if(_rtw_queue_empty(queue)== _TRUE)
	{	
		_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
            		
		//when set_ssid/set_bssid for rtw_do_join(), but scanning queue is empty
		//we try to issue sitesurvey firstly	
            		
		if(pmlmepriv->LinkDetectInfo.bBusyTraffic==_FALSE
			#ifdef CONFIG_LAYER2_ROAMING
			|| pmlmepriv->to_roaming >0
			#endif
		)
		{
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_do_join(): site survey if scanned_queue is empty\n."));
			// submit site_survey_cmd
			if(_SUCCESS!=(ret=rtw_sitesurvey_cmd(padapter, &pmlmepriv->assoc_ssid)) ) {
				RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("rtw_do_join(): site survey return error\n."));
			}
		}
		
		goto exit;
	}	
	else 	
	{
		int select_ret;
		if((select_ret=rtw_select_and_join_from_scanned_queue(pmlmepriv))==_SUCCESS)
		{
			pmlmepriv->to_join = _FALSE;
			_set_timer(&pmlmepriv->assoc_timer, MAX_JOIN_TIMEOUT);
		}
#if 0
		else if(ret == 2)
		{
			DBG_8712("*****UNDER_LINKED WITH SAME NETWORK, RETURN*****\n");

			_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);
			
		}
#endif
		else	
		{
			if(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE)==_TRUE)
			{
				// submit createbss_cmd to change to a ADHOC_MASTER

 				//pmlmepriv->lock has been acquired by caller...
				WLAN_BSSID_EX    *pdev_network = &(padapter->registrypriv.dev_network);

				pmlmepriv->fw_state = WIFI_ADHOC_MASTER_STATE;
				
				pibss = padapter->registrypriv.dev_network.MacAddress;

				_rtw_memset(&pdev_network->Ssid, 0, sizeof(NDIS_802_11_SSID));
				_rtw_memcpy(&pdev_network->Ssid, &pmlmepriv->assoc_ssid, sizeof(NDIS_802_11_SSID));
	
				rtw_update_registrypriv_dev_network(padapter);

				rtw_generate_random_ibss(pibss);
					
				if(rtw_createbss_cmd(padapter)!=_SUCCESS)
				{
					RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("***Error=>do_goin: rtw_createbss_cmd status FAIL*** \n "));						
					return _FALSE;
				}

			     	pmlmepriv->to_join = _FALSE;

				RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("***Error=> rtw_select_and_join_from_scanned_queue FAIL under STA_Mode*** \n "));						

			}			
			else
			{ 
				// can't associate ; reset under-linking			
				_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

#if 0	
				if((check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _TRUE))
				{
					if(_rtw_memcmp(pmlmepriv->cur_network.network.Ssid.Ssid, pmlmepriv->assoc_ssid.Ssid, pmlmepriv->assoc_ssid.SsidLength))
					{ 
						// for funk to do roaming
						// funk will reconnect, but funk will not sitesurvey before reconnect
						RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("for funk to do roaming"));
						if(pmlmepriv->sitesurveyctrl.traffic_busy==_FALSE)
							rtw_sitesurvey_cmd(padapter, &pmlmepriv->assoc_ssid);
					}
				
				}				
#endif

				//when set_ssid/set_bssid for rtw_do_join(), but there are no desired bss in scanning queue
				//we try to issue sitesurvey firstly			
				if(pmlmepriv->LinkDetectInfo.bBusyTraffic==_FALSE
					#ifdef CONFIG_LAYER2_ROAMING
					|| pmlmepriv->to_roaming >0
					#endif
				)
				{
					//DBG_8192C("rtw_do_join() when   no desired bss in scanning queue \n");
					if( _SUCCESS!=(ret=rtw_sitesurvey_cmd(padapter, &pmlmepriv->assoc_ssid)) ){
						RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("do_join(): site survey return error\n."));
					}
				}				


			}

		}

	}
	
exit:
	
_func_exit_;	

	return ret;	
}

#ifdef PLATFORM_WINDOWS
u8 rtw_pnp_set_power_wakeup(_adapter* padapter)
{
	u8 res=_SUCCESS;

_func_enter_;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("==>rtw_pnp_set_power_wakeup!!!\n"));
	
	res = rtw_setstandby_cmd(padapter, 0);

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("<==rtw_pnp_set_power_wakeup!!!\n"));

_func_exit_;
	
	return res;
}

u8 rtw_pnp_set_power_sleep(_adapter* padapter)
{
	u8 res=_SUCCESS;	
	
_func_enter_;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("==>rtw_pnp_set_power_sleep!!!\n"));
	//DbgPrint("+rtw_pnp_set_power_sleep\n");

	res = rtw_setstandby_cmd(padapter, 1);

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("<==rtw_pnp_set_power_sleep!!!\n"));

_func_exit_;

	return res;
}

u8 rtw_set_802_11_reload_defaults(_adapter * padapter, NDIS_802_11_RELOAD_DEFAULTS reloadDefaults)
{
_func_enter_;

	switch( reloadDefaults)
	{
		case Ndis802_11ReloadWEPKeys:
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("SetInfo OID_802_11_RELOAD_DEFAULTS : Ndis802_11ReloadWEPKeys\n"));
			break;
	}

	// SecClearAllKeys(Adapter);
	// 8711 CAM was not for En/Decrypt only
	// so, we can't clear all keys.
	// should we disable WPAcfg (ox0088) bit 1-2, instead of clear all CAM
	
	//TO DO...

_func_exit_;
	
	return _TRUE;
}

u8 set_802_11_test(_adapter* padapter, NDIS_802_11_TEST *test)
{
	u8 ret=_TRUE;
	
_func_enter_;

	switch(test->Type)
	{
		case 1:
			NdisMIndicateStatus(padapter->hndis_adapter, NDIS_STATUS_MEDIA_SPECIFIC_INDICATION, (PVOID)&test->AuthenticationEvent, test->Length - 8);
			NdisMIndicateStatusComplete(padapter->hndis_adapter);
			break;

		case 2:
			NdisMIndicateStatus(padapter->hndis_adapter, NDIS_STATUS_MEDIA_SPECIFIC_INDICATION, (PVOID)&test->RssiTrigger, sizeof(NDIS_802_11_RSSI));
			NdisMIndicateStatusComplete(padapter->hndis_adapter);
			break;

		default:
			ret=_FALSE;
			break;
	}

_func_exit_;

	return ret;	
}

u8	rtw_set_802_11_pmkid(_adapter*	padapter, NDIS_802_11_PMKID *pmkid)
{
	u8	ret=_SUCCESS;

	return ret;
}

#endif

u8 rtw_set_802_11_bssid(_adapter* padapter, u8 *bssid)
{	
	_irqL irqL;	
	u8 status=_TRUE;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	_queue *queue = &pmlmepriv->scanned_queue;
	
_func_enter_;
	
	RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_notice_,
		 ("+rtw_set_802_11_bssid: bssid="MAC_FMT"\n", MAC_ARG(bssid) ));

	if ((bssid[0]==0x00 && bssid[1]==0x00 && bssid[2]==0x00 && bssid[3]==0x00 && bssid[4]==0x00 &&bssid[5]==0x00) ||
	    (bssid[0]==0xFF && bssid[1]==0xFF && bssid[2]==0xFF && bssid[3]==0xFF && bssid[4]==0xFF &&bssid[5]==0xFF))
	{
		status = _FALSE;
		return status;
	}
		
	_enter_critical_bh(&pmlmepriv->lock, &irqL);


 	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY|_FW_UNDER_LINKING) == _TRUE)
	{
	        RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_,
			 ("Set BSSID is not allowed under surveying || adhoc master || under linking, fw_state=0x%08x\n",
			  get_fwstate(pmlmepriv)));
		status = check_fwstate(pmlmepriv, _FW_UNDER_LINKING);
		goto _Abort_Set_BSSID;
	}

	if (check_fwstate(pmlmepriv, _FW_LINKED|WIFI_ADHOC_MASTER_STATE) == _TRUE)
	{
		RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_info_, ("set_bssid: _FW_LINKED||WIFI_ADHOC_MASTER_STATE\n"));

		if (_rtw_memcmp(&pmlmepriv->cur_network.network.MacAddress, bssid, ETH_ALEN) == _TRUE)
		{		
			if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _FALSE)
				goto _Abort_Set_BSSID;//it means driver is in WIFI_ADHOC_MASTER_STATE, we needn't create bss again.
		} else {
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("Set BSSID not the same ssid\n"));
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("set_bssid="MAC_FMT"\n", MAC_ARG(bssid) ));
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("cur_bssid="MAC_FMT"\n", MAC_ARG(pmlmepriv->cur_network.network.MacAddress) ));

			rtw_disassoc_cmd(padapter);

			if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
				rtw_indicate_disconnect(padapter);

			rtw_free_assoc_resources(padapter);

			if ((check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == _TRUE)) {
				_clr_fwstate_(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
			}		
		}
	}

	_rtw_memcpy(&pmlmepriv->assoc_bssid, bssid, ETH_ALEN);
	
	pmlmepriv->assoc_by_bssid=_TRUE;
	
	status = rtw_do_join(padapter);

	goto done;
	
_Abort_Set_BSSID:

	RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_, ("rtw_set_802_11_bssid: _Abort_Set_BSSID\n"));

done:
	
	_exit_critical_bh(&pmlmepriv->lock, &irqL);
	
_func_exit_;

	return status;
}

u8 rtw_set_802_11_ssid(_adapter* padapter, NDIS_802_11_SSID *ssid)
{	
	_irqL irqL;
	u8 status = _TRUE;
	int timeout = 50;

	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *pnetwork = &pmlmepriv->cur_network;
	_queue *queue = &pmlmepriv->scanned_queue;

#ifdef PLATFORM_WINDOWS
	LARGE_INTEGER	sys_time;
	u32  diff_time,cur_time ;
#endif

	
_func_enter_;
	
	RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_notice_,
		 ("+rtw_set_802_11_ssid: ssid=[%s] fw_state=0x%08x\n",
		  ssid->Ssid, get_fwstate(pmlmepriv)));

	if(padapter->hw_init_completed==_FALSE){
		RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_,
			 ("set_ssid: hw_init_completed==_FALSE=>exit!!!\n"));
		return _FALSE;
	}
		
	//_enter_critical_bh(&pmlmepriv->lock, &irqL);

	//if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY|_FW_UNDER_LINKING) == _TRUE) {
	if (check_fwstate(pmlmepriv, _FW_UNDER_LINKING) == _TRUE) {
		RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_,
			 ("Set SSID is not allowed under surveying || adhoc master || under linking\n"));
		status = check_fwstate(pmlmepriv, _FW_UNDER_LINKING);
		//goto _Abort_Set_SSID;
		return status;
	}
    while (timeout-- > 0)
	{
		if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) == _FALSE)
			break;

		msleep(100);
	}

	if (timeout <= 0)
	{
		printk("%s: Set SSID is not allowed under surveying\n", __func__);
		status = check_fwstate(pmlmepriv, _FW_UNDER_LINKING);
		goto _Abort_Set_SSID;
	}

	_enter_critical_bh(&pmlmepriv->lock, &irqL);

	if (check_fwstate(pmlmepriv, _FW_LINKED|WIFI_ADHOC_MASTER_STATE) == _TRUE)
	{
		RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_info_,
			 ("set_ssid: _FW_LINKED||WIFI_ADHOC_MASTER_STATE\n"));

		if ((pmlmepriv->assoc_ssid.SsidLength == ssid->SsidLength) &&
		    (_rtw_memcmp(&pmlmepriv->assoc_ssid.Ssid, ssid->Ssid, ssid->SsidLength) == _TRUE))
		{			
			if((check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _FALSE))
			{
				RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_,
					 ("Set SSID is the same ssid, fw_state=0x%08x\n",
					  get_fwstate(pmlmepriv)));

				if(rtw_is_same_ibss(padapter, pnetwork) == _FALSE)
				{				
					//if in WIFI_ADHOC_MASTER_STATE | WIFI_ADHOC_STATE, create bss or rejoin again
					rtw_disassoc_cmd(padapter);

					if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
						rtw_indicate_disconnect(padapter);
						
					rtw_free_assoc_resources(padapter);

					if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == _TRUE) {
						_clr_fwstate_(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
						set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
					}
				}
				else
				{
					goto _Abort_Set_SSID;//it means driver is in WIFI_ADHOC_MASTER_STATE, we needn't create bss again.
				}
			}
#ifdef CONFIG_LPS
			else {
				rtw_lps_ctrl_wk_cmd(padapter, LPS_CTRL_JOINBSS, 1);
			}
#endif
		}
		else
		{
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("Set SSID not the same ssid\n"));
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("set_ssid=[%s] len=0x%x\n", ssid->Ssid, (unsigned int)ssid->SsidLength));
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("assoc_ssid=[%s] len=0x%x\n", pmlmepriv->assoc_ssid.Ssid, (unsigned int)pmlmepriv->assoc_ssid.SsidLength));

			rtw_disassoc_cmd(padapter);

			if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
				rtw_indicate_disconnect(padapter);
			
			rtw_free_assoc_resources(padapter);

			if (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == _TRUE) {
				_clr_fwstate_(pmlmepriv, WIFI_ADHOC_MASTER_STATE);
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
			}
		}		
	}

#ifdef PLATFORM_WINDOWS
	if (padapter->securitypriv.btkip_countermeasure==_TRUE)
	{
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_ssid:padapter->securitypriv.btkip_countermeasure==_TRUE\n"));
		NdisGetCurrentSystemTime(&sys_time);	
		cur_time=(u32)(sys_time.QuadPart/10);  // In micro-second.
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_ssid:cur_time=0x%x\n",cur_time));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_ssid:psecuritypriv->last_mic_err_time=0x%x\n",padapter->securitypriv.btkip_countermeasure_time));
		diff_time = cur_time -padapter->securitypriv.btkip_countermeasure_time; // In micro-second.
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_ssid:diff_time=0x%x\n",diff_time));

		if (diff_time > 60000000) {
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_ssid(): countermeasure time >60s.\n"));
			padapter->securitypriv.btkip_countermeasure=_FALSE;
		// Update MIC error time.
			padapter->securitypriv.btkip_countermeasure_time=0;
		} else {
			// can't join  in 60 seconds.
			status = _FALSE;
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_ssid(): countermeasure time <60s.\n"));
			goto _Abort_Set_SSID;
		}
	}
#endif

#ifdef PLATFORM_LINUX
	if (padapter->securitypriv.btkip_countermeasure == _TRUE) {
		status = _FALSE;
            goto _Abort_Set_SSID;
        }
#endif

	if (rtw_validate_ssid(ssid) == _FALSE) {
		status = _FALSE;
		goto _Abort_Set_SSID;
	}

	_rtw_memcpy(&pmlmepriv->assoc_ssid, ssid, sizeof(NDIS_802_11_SSID));
	
	pmlmepriv->assoc_by_bssid=_FALSE;
	
	status = rtw_do_join(padapter);

	goto done;

_Abort_Set_SSID:
	
	RT_TRACE(_module_rtl871x_ioctl_set_c_, _drv_err_,
		 ("-rtw_set_802_11_ssid: _Abort_Set_SSID: status=%d\n", status));

done:
	
	_exit_critical_bh(&pmlmepriv->lock, &irqL);
	
_func_exit_;

	return status;
	
}

u8 rtw_set_802_11_infrastructure_mode(_adapter* padapter, 
	NDIS_802_11_NETWORK_INFRASTRUCTURE networktype)
{
	_irqL irqL;
	struct	mlme_priv	*pmlmepriv = &padapter->mlmepriv;
	struct	wlan_network	*cur_network = &pmlmepriv->cur_network;
	NDIS_802_11_NETWORK_INFRASTRUCTURE* pold_state = &(cur_network->network.InfrastructureMode);
	
_func_enter_;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_notice_,
		 ("+rtw_set_802_11_infrastructure_mode: old=%d new=%d fw_state=0x%08x\n",
		  *pold_state, networktype, get_fwstate(pmlmepriv)));
	
	if(*pold_state != networktype)
	{
		_enter_critical_bh(&pmlmepriv->lock, &irqL);
		
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,(" change mode!"));
		//DBG_871X("change mode, old_mode=%d, new_mode=%d, fw_state=0x%x\n", *pold_state, networktype, get_fwstate(pmlmepriv));

		if((check_fwstate(pmlmepriv, _FW_LINKED)== _TRUE) ||(*pold_state==Ndis802_11IBSS))
			rtw_disassoc_cmd(padapter);

		if((check_fwstate(pmlmepriv, _FW_LINKED)== _TRUE) ||
			(check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)== _TRUE) )
			rtw_free_assoc_resources(padapter);


		if((check_fwstate(pmlmepriv, _FW_LINKED)== _TRUE) || (*pold_state==Ndis802_11Infrastructure) ||(*pold_state==Ndis802_11IBSS))
		{		
			rtw_indicate_disconnect(padapter); //will clr Linked_state; before this function, we must have chked whether  issue dis-assoc_cmd or not
		}	
	
		if(*pold_state==Ndis802_11APMode)
		{		
			//change to other mode from Ndis802_11APMode			
			cur_network->join_res = -1;
			
#ifdef CONFIG_NATIVEAP_MLME
			stop_ap_mode(padapter);
#endif
		}	
		
		*pold_state = networktype;

				// clear WIFI_STATION_STATE; WIFI_AP_STATE; WIFI_ADHOC_STATE; WIFI_ADHOC_MASTER_STATE
		//pmlmepriv->fw_state &= 0xffffff87;		
		_clr_fwstate_(pmlmepriv, WIFI_STATION_STATE|WIFI_AP_STATE|WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE);
				
		switch(networktype)
		{
			case Ndis802_11IBSS:
				set_fwstate(pmlmepriv, WIFI_ADHOC_STATE);
				break;
				
			case Ndis802_11Infrastructure:
				set_fwstate(pmlmepriv, WIFI_STATION_STATE);
				break;
				
			case Ndis802_11APMode:
				set_fwstate(pmlmepriv, WIFI_AP_STATE);
#ifdef CONFIG_NATIVEAP_MLME
				start_ap_mode(padapter);
				//rtw_indicate_connect(padapter);
#endif				
				
				break;

			case Ndis802_11AutoUnknown:
			case Ndis802_11InfrastructureMax:
				break;                        				
		}

		//SecClearAllKeys(adapter);
		
		//RT_TRACE(COMP_OID_SET, DBG_LOUD, ("set_infrastructure: fw_state:%x after changing mode\n",
		//									get_fwstate(pmlmepriv) ));

		_exit_critical_bh(&pmlmepriv->lock, &irqL);
	}

_func_exit_;

	return _TRUE;
}


u8 rtw_set_802_11_disassociate(_adapter *padapter)
{
	_irqL irqL;
	struct mlme_priv * pmlmepriv = &padapter->mlmepriv;

_func_enter_;

	_enter_critical_bh(&pmlmepriv->lock, &irqL);

	if (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
	{
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("MgntActrtw_set_802_11_disassociate: rtw_indicate_disconnect\n"));

		rtw_disassoc_cmd(padapter);		
		rtw_indicate_disconnect(padapter);
		rtw_free_assoc_resources(padapter);			
	}

	_exit_critical_bh(&pmlmepriv->lock, &irqL);
	
_func_exit_;

	return _TRUE;	
}

u8 rtw_set_802_11_bssid_list_scan(_adapter* padapter)
{	
	_irqL	irqL;
	struct	mlme_priv		*pmlmepriv= &padapter->mlmepriv;
	u8	res=_TRUE;
	
_func_enter_;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("+rtw_set_802_11_bssid_list_scan(), fw_state=%x\n", get_fwstate(pmlmepriv)));

	if (padapter == NULL) {
		res=_FALSE;
		goto exit;
	}
	if (padapter->hw_init_completed==_FALSE){
		res = _FALSE;
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n===rtw_set_802_11_bssid_list_scan:hw_init_completed==_FALSE===\n"));
		goto exit;
	}
	
	if ((check_fwstate(pmlmepriv, _FW_UNDER_SURVEY|_FW_UNDER_LINKING) == _TRUE) ||
		(pmlmepriv->LinkDetectInfo.bBusyTraffic == _TRUE))
	{
		// Scan or linking is in progress, do nothing.
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("rtw_set_802_11_bssid_list_scan fail since fw_state = %x\n", get_fwstate(pmlmepriv)));
		res = _TRUE;

		if(check_fwstate(pmlmepriv, (_FW_UNDER_SURVEY|_FW_UNDER_LINKING))== _TRUE){
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n###_FW_UNDER_SURVEY|_FW_UNDER_LINKING\n\n"));
		} else {
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n###pmlmepriv->sitesurveyctrl.traffic_busy==_TRUE\n\n"));
		}
	} else {
		NDIS_802_11_SSID ssid;
		
		#ifdef CONFIG_SET_SCAN_DENY_TIMER
		if(ATOMIC_READ(&pmlmepriv->set_scan_deny)==1){
			DBG_871X("%s:%d CONFIG_SET_SCAN_DENY_TIMER deny scan\n", __FUNCTION__, __LINE__);
			indicate_wx_scan_complete_event(padapter);
			return _SUCCESS;
		}
		#endif
		
		_enter_critical_bh(&pmlmepriv->lock, &irqL);		
		
		_rtw_memset((unsigned char*)&ssid, 0, sizeof(NDIS_802_11_SSID));
		
		res = rtw_sitesurvey_cmd(padapter, &ssid);
		
		_exit_critical_bh(&pmlmepriv->lock, &irqL);
	}
exit:
	
_func_exit_;

	return res;	
}

u8 rtw_set_802_11_authentication_mode(_adapter* padapter, NDIS_802_11_AUTHENTICATION_MODE authmode) 
{
	struct security_priv *psecuritypriv = &padapter->securitypriv;
	int res;
	u8 ret;
	
_func_enter_;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("set_802_11_auth.mode(): mode=%x\n", authmode));

	psecuritypriv->ndisauthtype=authmode;
	
	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_authentication_mode:psecuritypriv->ndisauthtype=%d", psecuritypriv->ndisauthtype));
	
	if(psecuritypriv->ndisauthtype>3)
		psecuritypriv->dot11AuthAlgrthm=dot11AuthAlgrthm_8021X;
	
	res=rtw_set_auth(padapter,psecuritypriv);
	
	if(res==_SUCCESS)
		ret=_TRUE;
	else
		ret=_FALSE;
	
_func_exit_;

	return ret;
}

u8 rtw_set_802_11_add_wep(_adapter* padapter, NDIS_802_11_WEP *wep){

	u8		bdefaultkey;
	u8		btransmitkey;
	sint		keyid,res;
	struct security_priv* psecuritypriv=&(padapter->securitypriv);
	u8		ret=_SUCCESS;

_func_enter_;

	bdefaultkey=(wep->KeyIndex & 0x40000000) > 0 ? _FALSE : _TRUE;   //for ???
	btransmitkey= (wep->KeyIndex & 0x80000000) > 0 ? _TRUE  : _FALSE;	//for ???
	keyid=wep->KeyIndex & 0x3fffffff;

	if(keyid>4)
	{
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("MgntActrtw_set_802_11_add_wep:keyid>4=>fail\n"));
		ret=_FALSE;
		goto exit;
	}
	
	switch(wep->KeyLength)
	{
		case 5:
			psecuritypriv->dot11PrivacyAlgrthm=_WEP40_;
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("MgntActrtw_set_802_11_add_wep:wep->KeyLength=5\n"));
			break;
		case 13:
			psecuritypriv->dot11PrivacyAlgrthm=_WEP104_;
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("MgntActrtw_set_802_11_add_wep:wep->KeyLength=13\n"));
			break;
		default:
			psecuritypriv->dot11PrivacyAlgrthm=_NO_PRIVACY_;
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("MgntActrtw_set_802_11_add_wep:wep->KeyLength!=5 or 13\n"));
			break;
	}
	
	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_add_wep:befor memcpy, wep->KeyLength=0x%x wep->KeyIndex=0x%x  keyid =%x\n",wep->KeyLength,wep->KeyIndex,keyid));

	_rtw_memcpy(&(psecuritypriv->dot11DefKey[keyid].skey[0]),&(wep->KeyMaterial),wep->KeyLength);

	psecuritypriv->dot11DefKeylen[keyid]=wep->KeyLength;

	psecuritypriv->dot11PrivacyKeyIndex=keyid;

	RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_add_wep:security key material : %x %x %x %x %x %x %x %x %x %x %x %x %x \n",
		psecuritypriv->dot11DefKey[keyid].skey[0],psecuritypriv->dot11DefKey[keyid].skey[1],psecuritypriv->dot11DefKey[keyid].skey[2],
		psecuritypriv->dot11DefKey[keyid].skey[3],psecuritypriv->dot11DefKey[keyid].skey[4],psecuritypriv->dot11DefKey[keyid].skey[5],
		psecuritypriv->dot11DefKey[keyid].skey[6],psecuritypriv->dot11DefKey[keyid].skey[7],psecuritypriv->dot11DefKey[keyid].skey[8],
		psecuritypriv->dot11DefKey[keyid].skey[9],psecuritypriv->dot11DefKey[keyid].skey[10],psecuritypriv->dot11DefKey[keyid].skey[11],
		psecuritypriv->dot11DefKey[keyid].skey[12]));

	res=rtw_set_key(padapter,psecuritypriv, keyid, 1);
	
	if(res==_FAIL)
		ret= _FALSE;
exit:
	
_func_exit_;

	return ret;
	
}

u8 rtw_set_802_11_remove_wep(_adapter* padapter, u32 keyindex){
	
	u8 ret=_SUCCESS;
	
_func_enter_;

	if (keyindex >= 0x80000000 || padapter == NULL){
		
		ret=_FALSE;
		goto exit;

	}
	else 
	{
		int res;
		struct security_priv* psecuritypriv=&(padapter->securitypriv);
		if( keyindex < 4 ){
			
			_rtw_memset(&psecuritypriv->dot11DefKey[keyindex], 0, 16);
			
			res=rtw_set_key(padapter,psecuritypriv,keyindex, 0);
			
			psecuritypriv->dot11DefKeylen[keyindex]=0;
			
			if(res==_FAIL)
				ret=_FAIL;
			
		}
		else
		{			
			ret=_FAIL;
		}
		
	}
	
exit:	
	
_func_exit_;

	return ret;
	
}

u8 rtw_set_802_11_add_key(_adapter* padapter, NDIS_802_11_KEY *key){

	uint	encryptionalgo;
	u8 * pbssid;
	struct sta_info *stainfo;
	u8	bgroup = _FALSE;
	u8	bgrouptkey = _FALSE;//can be remove later
	u8	ret=_SUCCESS;
	
_func_enter_;

	if (((key->KeyIndex & 0x80000000) == 0) && ((key->KeyIndex & 0x40000000) > 0)){

		// It is invalid to clear bit 31 and set bit 30. If the miniport driver encounters this combination, 
		// it must fail the request and return NDIS_STATUS_INVALID_DATA.
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_add_key: ((key->KeyIndex & 0x80000000) == 0)[=%d] ",(int)(key->KeyIndex & 0x80000000) == 0));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_add_key:((key->KeyIndex & 0x40000000) > 0)[=%d]" , (int)(key->KeyIndex & 0x40000000) > 0));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_info_,("rtw_set_802_11_add_key: key->KeyIndex=%d \n" ,(int)key->KeyIndex));
		ret= _FAIL;
		goto exit;
	}

	if(key->KeyIndex & 0x40000000)
	{ 
		// Pairwise key

		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY: +++++ Pairwise key +++++\n"));
	
		pbssid=get_bssid(&padapter->mlmepriv);
		stainfo=rtw_get_stainfo(&padapter->stapriv, pbssid);

		if((stainfo!=NULL)&&(padapter->securitypriv.dot11AuthAlgrthm==dot11AuthAlgrthm_8021X)){
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY:( stainfo!=NULL)&&(Adapter->securitypriv.dot11AuthAlgrthm==dot11AuthAlgrthm_8021X)\n"));
			encryptionalgo=stainfo->dot118021XPrivacy;
		}
		else{
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY: stainfo==NULL)||(Adapter->securitypriv.dot11AuthAlgrthm!=dot11AuthAlgrthm_8021X)\n"));
			encryptionalgo=padapter->securitypriv.dot11PrivacyAlgrthm;
		}

		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("rtw_set_802_11_add_key: (encryptionalgo ==%d)!\n",encryptionalgo ));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("rtw_set_802_11_add_key: (Adapter->securitypriv.dot11PrivacyAlgrthm ==%d)!\n",padapter->securitypriv.dot11PrivacyAlgrthm));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("rtw_set_802_11_add_key: (Adapter->securitypriv.dot11AuthAlgrthm ==%d)!\n",padapter->securitypriv.dot11AuthAlgrthm));

		if((stainfo!=NULL)){
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("rtw_set_802_11_add_key: (stainfo->dot118021XPrivacy ==%d)!\n", stainfo->dot118021XPrivacy));
		}
		
		if(key->KeyIndex & 0x000000FF){
			// The key index is specified in the lower 8 bits by values of zero to 255.
			// The key index should be set to zero for a Pairwise key, and the driver should fail with
			// NDIS_STATUS_INVALID_DATA if the lower 8 bits is not zero
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,(" key->KeyIndex & 0x000000FF.\n"));
			ret= _FAIL;
			goto exit;
		}

		// check BSSID
		if (IS_MAC_ADDRESS_BROADCAST(key->BSSID) == _TRUE){

			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("MacAddr_isBcst(key->BSSID)\n"));
			ret= _FALSE;
			goto exit;
		}

		// Check key length for TKIP.
		//if(encryptionAlgorithm == RT_ENC_TKIP_ENCRYPTION && key->KeyLength != 32)
		if((encryptionalgo== _TKIP_)&& (key->KeyLength != 32)){
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("TKIP KeyLength:0x%x != 32\n", key->KeyLength));
			ret=_FAIL;
			goto exit;

		}

		// Check key length for AES.
		if((encryptionalgo== _AES_)&& (key->KeyLength != 16)) {
			// For our supplicant, EAPPkt9x.vxd, cannot differentiate TKIP and AES case.
			if(key->KeyLength == 32) {
				key->KeyLength = 16; 
			} else {
				ret= _FAIL;
				goto exit;
			}
		}

		// Check key length for WEP. For NDTEST, 2005.01.27, by rcnjko.
		if(	(encryptionalgo== _WEP40_|| encryptionalgo== _WEP104_) && (key->KeyLength != 5 || key->KeyLength != 13)) {
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("WEP KeyLength:0x%x != 5 or 13\n", key->KeyLength));
			ret=_FAIL;
			goto exit;
		}

		bgroup = _FALSE;

		// Check the pairwise key. Added by Annie, 2005-07-06.
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("------------------------------------------\n"));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("[Pairwise Key set]\n"));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("------------------------------------------\n"));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("key index: 0x%8x(0x%8x)\n", key->KeyIndex,(key->KeyIndex&0x3)));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("key Length: %d\n", key->KeyLength));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("------------------------------------------\n"));
	
	}
	else 
	{	
		// Group key - KeyIndex(BIT30==0)
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY: +++++ Group key +++++\n"));


		// when add wep key through add key and didn't assigned encryption type before
		if((padapter->securitypriv.ndisauthtype<=3)&&(padapter->securitypriv.dot118021XGrpPrivacy==0))
		{
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("keylen=%d( Adapter->securitypriv.dot11PrivacyAlgrthm=%x  )padapter->securitypriv.dot118021XGrpPrivacy(%x)\n", key->KeyLength,padapter->securitypriv.dot11PrivacyAlgrthm,padapter->securitypriv.dot118021XGrpPrivacy));

			switch(key->KeyLength)
			{
				case 5:
					padapter->securitypriv.dot11PrivacyAlgrthm=_WEP40_;
					RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("Adapter->securitypriv.dot11PrivacyAlgrthm= %x key->KeyLength=%u\n", padapter->securitypriv.dot11PrivacyAlgrthm,key->KeyLength));
					break;
				case 13:
					padapter->securitypriv.dot11PrivacyAlgrthm=_WEP104_;
					RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("Adapter->securitypriv.dot11PrivacyAlgrthm= %x key->KeyLength=%u\n", padapter->securitypriv.dot11PrivacyAlgrthm,key->KeyLength));
					break;
				default:
					padapter->securitypriv.dot11PrivacyAlgrthm=_NO_PRIVACY_;
					RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("Adapter->securitypriv.dot11PrivacyAlgrthm= %x key->KeyLength=%u \n", padapter->securitypriv.dot11PrivacyAlgrthm,key->KeyLength));
					break;
			}
			
			encryptionalgo=padapter->securitypriv.dot11PrivacyAlgrthm;
			
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,(" Adapter->securitypriv.dot11PrivacyAlgrthm=%x\n", padapter->securitypriv.dot11PrivacyAlgrthm));
			
		}
		else 
		{
			encryptionalgo=padapter->securitypriv.dot118021XGrpPrivacy;
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("( Adapter->securitypriv.dot11PrivacyAlgrthm=%x  )encryptionalgo(%x)=padapter->securitypriv.dot118021XGrpPrivacy(%x)keylen=%d\n", padapter->securitypriv.dot11PrivacyAlgrthm,encryptionalgo,padapter->securitypriv.dot118021XGrpPrivacy,key->KeyLength));

		}
		
		if((check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE)==_TRUE) && (IS_MAC_ADDRESS_BROADCAST(key->BSSID) == _FALSE)) {
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,(" IBSS but BSSID is not Broadcast Address.\n"));
			ret= _FAIL;
			goto exit;
		}

		// Check key length for TKIP
		if((encryptionalgo== _TKIP_) && (key->KeyLength != 32)) {

			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,(" TKIP GTK KeyLength:%u != 32\n", key->KeyLength));
			ret= _FAIL;
			goto exit;

		} else if(encryptionalgo== _AES_ && (key->KeyLength != 16 && key->KeyLength != 32) ) {
			
			// Check key length for AES
			// For NDTEST, we allow keylen=32 in this case. 2005.01.27, by rcnjko.
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("<=== SetInfo, OID_802_11_ADD_KEY: AES GTK KeyLength:%u != 16 or 32\n", key->KeyLength));
			ret= _FAIL;
			goto exit;
		}

		// Change the key length for EAPPkt9x.vxd. Added by Annie, 2005-11-03.
		if((encryptionalgo==  _AES_) && (key->KeyLength == 32) ) {
			key->KeyLength = 16; 
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("AES key length changed: %u\n", key->KeyLength) );
		}

		if(key->KeyIndex & 0x8000000) {//error ??? 0x8000_0000
			bgrouptkey = _TRUE;
		}

		if((check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE)==_TRUE)&&(check_fwstate(&padapter->mlmepriv, _FW_LINKED)==_TRUE))
		{
			bgrouptkey = _TRUE;
		}

		bgroup = _TRUE;

		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("------------------------------------------\n") );
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("[Group Key set]\n") );
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("------------------------------------------\n")) ;
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("key index: 0x%8x(0x%8x)\n", key->KeyIndex,(key->KeyIndex&0x3)));
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("key Length: %d\n", key->KeyLength)) ;
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("------------------------------------------\n"));
		
	}	

	// If WEP encryption algorithm, just call rtw_set_802_11_add_wep().
	if((padapter->securitypriv.dot11AuthAlgrthm !=dot11AuthAlgrthm_8021X)&&(encryptionalgo== _WEP40_  || encryptionalgo== _WEP104_))
	{
		u8 ret;		
		u32 keyindex;		
		u32 len = FIELD_OFFSET(NDIS_802_11_KEY, KeyMaterial) + key->KeyLength;
		NDIS_802_11_WEP *wep = &padapter->securitypriv.ndiswep;
				
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY: +++++ WEP key +++++\n"));

		wep->Length = len;
		keyindex = key->KeyIndex&0x7fffffff;
		wep->KeyIndex = keyindex ;
		wep->KeyLength = key->KeyLength;
		
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY:Before memcpy \n"));

		_rtw_memcpy(wep->KeyMaterial, key->KeyMaterial, key->KeyLength);	
		_rtw_memcpy(&(padapter->securitypriv.dot11DefKey[keyindex].skey[0]), key->KeyMaterial, key->KeyLength);

		padapter->securitypriv.dot11DefKeylen[keyindex]=key->KeyLength;		
		padapter->securitypriv.dot11PrivacyKeyIndex=keyindex;
		
		ret = rtw_set_802_11_add_wep(padapter, wep);
	
		goto exit;
		
	}

	if(key->KeyIndex & 0x20000000){
		// SetRSC
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("OID_802_11_ADD_KEY: +++++ SetRSC+++++\n"));
		if(bgroup == _TRUE)
		{
			NDIS_802_11_KEY_RSC keysrc=key->KeyRSC & 0x00FFFFFFFFFFFFULL;
			_rtw_memcpy(&padapter->securitypriv.dot11Grprxpn, &keysrc, 8);			
		} 
		else 
		{		
			NDIS_802_11_KEY_RSC keysrc=key->KeyRSC & 0x00FFFFFFFFFFFFULL;	
			_rtw_memcpy(&padapter->securitypriv.dot11Grptxpn, &keysrc, 8);			
		}
			
	}

	// Indicate this key idx is used for TX
	// Save the key in KeyMaterial
	if(bgroup == _TRUE) // Group transmit key
	{
		int res;
		
		if(bgrouptkey == _TRUE)
		{		
			padapter->securitypriv.dot118021XGrpKeyid=(u8)key->KeyIndex;
		}
		
		if((key->KeyIndex&0x3) == 0){
			ret = _FAIL;
			goto exit;
		}		
		
		_rtw_memset(&padapter->securitypriv.dot118021XGrpKey[(u8)((key->KeyIndex) & 0x03)], 0, 16);
		_rtw_memset(&padapter->securitypriv.dot118021XGrptxmickey[(u8)((key->KeyIndex) & 0x03)], 0, 16);
		_rtw_memset(&padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)], 0, 16);
		
		if((key->KeyIndex & 0x10000000))
		{
			_rtw_memcpy(&padapter->securitypriv.dot118021XGrptxmickey[(u8)((key->KeyIndex) & 0x03)], key->KeyMaterial + 16, 8);
			_rtw_memcpy(&padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)], key->KeyMaterial + 24, 8);
			
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n rtw_set_802_11_add_key:rx mic :0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[0],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[1],
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[2],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[3],
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[4],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[5],
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[6],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[7]));
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n rtw_set_802_11_add_key:set Group mic key!!!!!!!!\n"));

		}
		else
		{
			_rtw_memcpy(&padapter->securitypriv.dot118021XGrptxmickey[(u8)((key->KeyIndex) & 0x03)], key->KeyMaterial + 24, 8);
			_rtw_memcpy(&padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)], key->KeyMaterial + 16, 8);
			
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n rtw_set_802_11_add_key:rx mic :0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n",
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[0],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[1],
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[2],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[3],
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[4],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[5],
				padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex) & 0x03)].skey[6],padapter->securitypriv.dot118021XGrprxmickey[(u8)((key->KeyIndex-1) & 0x03)].skey[7]));
			RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n rtw_set_802_11_add_key:set Group mic key!!!!!!!!\n"));
		
		}

		//set group key by index
		_rtw_memcpy(&padapter->securitypriv.dot118021XGrpKey[(u8)((key->KeyIndex) & 0x03)], key->KeyMaterial, key->KeyLength);
		
		key->KeyIndex=key->KeyIndex & 0x03;
		
		padapter->securitypriv.binstallGrpkey=_TRUE;
		
		padapter->securitypriv.bcheck_grpkey=_FALSE;
		
		RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("reset group key"));
		
		res=rtw_set_key(padapter,&padapter->securitypriv, key->KeyIndex, 1);

		if(res==_FAIL)
			ret= _FAIL;

		goto exit;
			
	}
	else // Pairwise Key
	{
		u8 res;
		
		pbssid=get_bssid(&padapter->mlmepriv);
		stainfo=rtw_get_stainfo(&padapter->stapriv , pbssid );
		
		if(stainfo!=NULL)
		{			
			_rtw_memset( &stainfo->dot118021x_UncstKey, 0, 16);// clear keybuffer
			
			_rtw_memcpy(&stainfo->dot118021x_UncstKey, key->KeyMaterial, 16);
			
			if(encryptionalgo== _TKIP_)
			{
				padapter->securitypriv.busetkipkey=_FALSE;
				
				//_set_timer(&padapter->securitypriv.tkip_timer, 50);
				
				RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n ==========_set_timer\n"));
				
				// if TKIP, save the Receive/Transmit MIC key in KeyMaterial[128-255]
				if((key->KeyIndex & 0x10000000)){
					_rtw_memcpy(&stainfo->dot11tkiptxmickey, key->KeyMaterial + 16, 8);
					_rtw_memcpy(&stainfo->dot11tkiprxmickey, key->KeyMaterial + 24, 8);

				} else {
					_rtw_memcpy(&stainfo->dot11tkiptxmickey, key->KeyMaterial + 24, 8);
					_rtw_memcpy(&stainfo->dot11tkiprxmickey, key->KeyMaterial + 16, 8);

				}
		
			}
			else if(encryptionalgo == _AES_)
			{		
	
			}

		
			//Set key to CAM through H2C command
			if(bgrouptkey)//never go to here
			{
				res=rtw_setstakey_cmd(padapter, (unsigned char *)stainfo, _FALSE);
				RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n rtw_set_802_11_add_key:rtw_setstakey_cmd(group)\n"));
			}
			else{
				res=rtw_setstakey_cmd(padapter, (unsigned char *)stainfo, _TRUE);
				RT_TRACE(_module_rtl871x_ioctl_set_c_,_drv_err_,("\n rtw_set_802_11_add_key:rtw_setstakey_cmd(unicast)\n"));
			}
			
			if(res ==_FALSE)
				ret= _FAIL;
			
		}

	}

exit:
	
_func_exit_;

	return ret;	
}

u8 rtw_set_802_11_remove_key(_adapter*	padapter, NDIS_802_11_REMOVE_KEY *key){
	
	uint				encryptionalgo;
	u8 * pbssid;
	struct sta_info *stainfo;
	u8	bgroup = (key->KeyIndex & 0x4000000) > 0 ? _FALSE: _TRUE;
	u8	keyIndex = (u8)key->KeyIndex & 0x03;
	u8	ret=_SUCCESS;
	
_func_enter_;

	if ((key->KeyIndex & 0xbffffffc) > 0) {
		ret=_FAIL;
		goto exit;
	}

	if (bgroup == _TRUE) {
		encryptionalgo= padapter->securitypriv.dot118021XGrpPrivacy;
		// clear group key by index
		//NdisZeroMemory(Adapter->MgntInfo.SecurityInfo.KeyBuf[keyIndex], MAX_WEP_KEY_LEN);
		//Adapter->MgntInfo.SecurityInfo.KeyLen[keyIndex] = 0;
		
		_rtw_memset(&padapter->securitypriv.dot118021XGrpKey[keyIndex], 0, 16);
		
		//! \todo Send a H2C Command to Firmware for removing this Key in CAM Entry.
	
	} else {
	
		pbssid=get_bssid(&padapter->mlmepriv);
		stainfo=rtw_get_stainfo(&padapter->stapriv , pbssid );
		if(stainfo !=NULL){
			encryptionalgo=stainfo->dot118021XPrivacy;

		// clear key by BSSID
		_rtw_memset(&stainfo->dot118021x_UncstKey, 0, 16);
		
		//! \todo Send a H2C Command to Firmware for disable this Key in CAM Entry.

		}
		else{
			ret= _FAIL;
			goto exit;
		}
	}

exit:
	
_func_exit_;

	return _TRUE;
	
}

