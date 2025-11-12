/*!
 @file:    ImpSeqs.h
 @author:  $Author: nxu2 $
 @brief:   4-wire BIA measurement header file.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef _BODYCOMPOSITION_H_
#define _BODYCOMPOSITION_H_
#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"


/* 
  Note: this example will use SEQID_0 as measurement sequence, and use SEQID_1 as init sequence. 
  SEQID_3 is used for calibration.
*/



#define BIACTRL_START          0
#define BIACTRL_STOPNOW        1
#define BIACTRL_STOPSYNC       2
#define BIACTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

AD5940Err AppBIAInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppBIAISR(void *pBuff, uint32_t *pCount);
AD5940Err AppBIACtrl(int32_t BcmCtrl, void *pPara);

#endif
