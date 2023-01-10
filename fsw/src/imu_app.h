/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as “core Flight System: Bootes”
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

/**
 * @file
 *
 * Main header file for the IMU application
 */

#ifndef IMU_APP_H
#define IMU_APP_H

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "imu_app_perfids.h"
#include "imu_app_msgids.h"
#include "imu_app_msg.h"


#include "sensor-mpu6050.h"
#define MPU6050

static const char bus_path[] = "/dev/i2c-2";
static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

/***********************************************************************/
#define IMU_APP_PIPE_DEPTH 32 /* Depth of the Command Pipe for Application */

#define PIN_HIGH		1
#define PIN_LOW			0



/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/
typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;
    uint8 ErrCounter;

    /*
    ** MPU6050 data...
    */
    float AccelRead[3];
    float GyroRead[3];
    float AngleRead[3];
    int RegisterPtr;
    uint8 DataVal;
    bool first_time;

    /*
    ** Housekeeping telemetry packet...
    */
    IMU_APP_HkTlm_t HkTlm;
    IMU_APP_OutData_t OutData;

    /*
    ** Run Status variable used in the main processing loop
    */
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[IMU_APP_EVENT_COUNTS];

} IMU_APP_Data_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (IMU_APP_Main), these
**       functions are not called from any other source module.
*/
void  IMU_APP_Main(void);
int32 IMU_APP_Init(void);
void  IMU_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  IMU_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr);
int32 IMU_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg);
int32 IMU_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg);
int32 IMU_APP_ResetCounters(const IMU_APP_ResetCountersCmd_t *Msg);
int32 IMU_APP_Noop(const IMU_APP_NoopCmd_t *Msg);
int32 IMU_APP_CalcOffsets(const IMU_APP_CalcOffsetsCmd_t *Msg);

int32 IMU_APP_Config_MPU6050(const IMU_APP_Config_MPU6050_t *Msg);

bool IMU_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

int32 mpu6050_init(void);
void mpu6050_read(void);


#endif /* IMU_APP_H */
