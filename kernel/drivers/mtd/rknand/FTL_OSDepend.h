/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File    :   FTL_OSDepend.h
Desc    :   与系统相关的函数原型声明
Author  :   LXS
Date    :   2010-03-10
Notes   :   
********************************************************************/
extern void* ftl_malloc(int nSize);
extern void ftl_free(void *pBuffer);
extern void *ftl_memset(void *s, int c, int n);
extern void* ftl_memcpy(void* pvTo, const void* pvForm, int size);
extern int ftl_memcmp(void *str1, void *str2, unsigned int count);

extern int FlashCsIoMux(unsigned char chipSel);
extern int FlashDeCsIoMux(unsigned char chipSel);

