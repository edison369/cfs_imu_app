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
 * \file
 *   This file contains the source code for the IMU App.
 */

/*
** Include Files:
*/
#include "imu_app_events.h"
#include "imu_app_version.h"
#include "imu_app.h"

/*
** global data
*/
IMU_APP_Data_t IMU_APP_Data;
SENSOR_MPU6050_Priv_Data_t SENSOR_MPU6050_Priv_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/*                                                                            */
/* Application entry point and main process loop                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void IMU_APP_Main(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(IMU_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = IMU_APP_Init();
    if (status != CFE_SUCCESS)
    {
        IMU_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    status = mpu6050_init();
    if (status != CFE_SUCCESS)
    {
        IMU_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_ERROR,
                          "IMU APP: Error configurin MPU6050\n");
    }

    /*
    ** IMU Runloop
    */
    while (CFE_ES_RunLoop(&IMU_APP_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(IMU_APP_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, IMU_APP_Data.CommandPipe, CFE_SB_PEND_FOREVER);

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(IMU_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            IMU_APP_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(IMU_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "IMU APP: SB Pipe Read Error, App Will Exit");

            IMU_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(IMU_APP_PERF_ID);

    CFE_ES_ExitApp(IMU_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* Initialization                                                             */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 IMU_APP_Init(void)
{
    int32 status;

    IMU_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    IMU_APP_Data.CmdCounter = 0;
    IMU_APP_Data.ErrCounter = 0;

    /*
    ** Initialize app configuration data
    */
    IMU_APP_Data.PipeDepth = IMU_APP_PIPE_DEPTH;

    strncpy(IMU_APP_Data.PipeName, "IMU_APP_CMD_PIPE", sizeof(IMU_APP_Data.PipeName));
    IMU_APP_Data.PipeName[sizeof(IMU_APP_Data.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    IMU_APP_Data.EventFilters[0].EventID = IMU_APP_STARTUP_INF_EID;
    IMU_APP_Data.EventFilters[0].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[1].EventID = IMU_APP_COMMAND_ERR_EID;
    IMU_APP_Data.EventFilters[1].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[2].EventID = IMU_APP_COMMANDNOP_INF_EID;
    IMU_APP_Data.EventFilters[2].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[3].EventID = IMU_APP_COMMANDRST_INF_EID;
    IMU_APP_Data.EventFilters[3].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[4].EventID = IMU_APP_INVALID_MSGID_ERR_EID;
    IMU_APP_Data.EventFilters[4].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[5].EventID = IMU_APP_LEN_ERR_EID;
    IMU_APP_Data.EventFilters[5].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[6].EventID = IMU_APP_PIPE_ERR_EID;
    IMU_APP_Data.EventFilters[6].Mask    = 0x0000;
    IMU_APP_Data.EventFilters[7].EventID = IMU_APP_DEV_INF_EID;
    IMU_APP_Data.EventFilters[7].Mask    = 0x0000;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(IMU_APP_Data.EventFilters, IMU_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("IMU App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(CFE_MSG_PTR(IMU_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(IMU_APP_HK_TLM_MID),
                 sizeof(IMU_APP_Data.HkTlm));

    /*
    ** Initialize output RF packet.
    */
    CFE_MSG_Init(CFE_MSG_PTR(IMU_APP_Data.OutData.TelemetryHeader), CFE_SB_ValueToMsgId(IMU_APP_RF_DATA_MID),
                 sizeof(IMU_APP_Data.OutData));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&IMU_APP_Data.CommandPipe, IMU_APP_Data.PipeDepth, IMU_APP_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("IMU App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(IMU_APP_SEND_HK_MID), IMU_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("IMU App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to RF command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(IMU_APP_SEND_RF_MID), IMU_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("IMU App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(IMU_APP_CMD_MID), IMU_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("IMU App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }


    CFE_EVS_SendEvent(IMU_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU App Initialized.%s",
                      IMU_APP_VERSION_STRING);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the IMU    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void IMU_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case IMU_APP_CMD_MID:
            IMU_APP_ProcessGroundCommand(SBBufPtr);
            break;

        case IMU_APP_SEND_HK_MID:
            IMU_APP_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case IMU_APP_SEND_RF_MID:
            IMU_APP_ReportRFTelemetry((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        default:
            CFE_EVS_SendEvent(IMU_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "IMU: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* IMU ground commands                                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void IMU_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    /*
    ** Process "known" IMU app ground commands
    */
    switch (CommandCode)
    {
        case IMU_APP_NOOP_CC:
            if (IMU_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(IMU_APP_NoopCmd_t)))
            {
                IMU_APP_Noop((IMU_APP_NoopCmd_t *)SBBufPtr);
            }

            break;

        case IMU_APP_RESET_COUNTERS_CC:
            if (IMU_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(IMU_APP_ResetCountersCmd_t)))
            {
                IMU_APP_ResetCounters((IMU_APP_ResetCountersCmd_t *)SBBufPtr);
            }

            break;

        case IMU_APP_CONFIG_MPU6050_CC:
            if (IMU_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(IMU_APP_Config_MPU6050_t)))
            {
                IMU_APP_Config_MPU6050((IMU_APP_Config_MPU6050_t *)SBBufPtr);
            }

            break;

	// TODO: Add the commands for the imu control...

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(IMU_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }
}

int32 IMU_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg){

  /*
  ** Get command execution counters...
  */
  IMU_APP_Data.OutData.CommandErrorCounter = IMU_APP_Data.ErrCounter;
  IMU_APP_Data.OutData.CommandCounter      = IMU_APP_Data.CmdCounter;

  IMU_APP_Data.OutData.AppID_H = (uint8_t) ((IMU_APP_HK_TLM_MID >> 8) & 0xff);
  IMU_APP_Data.OutData.AppID_L = (uint8_t) (IMU_APP_HK_TLM_MID & 0xff);

  /* Copy the MPU6050 data */
  uint8_t *aux_array1;
  uint8_t *aux_array2;
  uint8_t *aux_array3;
  for(int i=0;i<3;i++){
      aux_array1 = NULL;
      aux_array1 = malloc(4 * sizeof(uint8_t));
      aux_array1 = (uint8_t*)(&IMU_APP_Data.AccelRead[i]);

      aux_array2 = NULL;
      aux_array2 = malloc(4 * sizeof(uint8_t));
      aux_array2 = (uint8_t*)(&IMU_APP_Data.GyroRead[i]);

      aux_array3 = NULL;
      aux_array3 = malloc(4 * sizeof(uint8_t));
      aux_array3 = (uint8_t*)(&IMU_APP_Data.AngleRead[i]);

      for(int j=0;j<4;j++){
          switch (i) {
            case 0:
              IMU_APP_Data.OutData.byte_group_1[j] = aux_array1[j];
              IMU_APP_Data.OutData.byte_group_4[j] = aux_array2[j];
              IMU_APP_Data.OutData.byte_group_7[j] = aux_array2[j];
              break;
            case 1:
              IMU_APP_Data.OutData.byte_group_2[j] = aux_array1[j];
              IMU_APP_Data.OutData.byte_group_5[j] = aux_array2[j];
              IMU_APP_Data.OutData.byte_group_8[j] = aux_array2[j];
              break;
            case 2:
              IMU_APP_Data.OutData.byte_group_3[j] = aux_array1[j];
              IMU_APP_Data.OutData.byte_group_6[j] = aux_array2[j];
              IMU_APP_Data.OutData.byte_group_9[j] = aux_array2[j];
              break;
          }
      }
  }

  /*
  ** Send housekeeping telemetry packet...
  */
  CFE_SB_TimeStampMsg(CFE_MSG_PTR(IMU_APP_Data.OutData.TelemetryHeader));
  CFE_SB_TransmitMsg(CFE_MSG_PTR(IMU_APP_Data.OutData.TelemetryHeader), true);

  return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 IMU_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{

    /*
    ** Get command execution counters...
    */
    IMU_APP_Data.HkTlm.Payload.CommandErrorCounter = IMU_APP_Data.ErrCounter;
    IMU_APP_Data.HkTlm.Payload.CommandCounter      = IMU_APP_Data.CmdCounter;

    /* Copy the MPU6050 data */
    mpu6050_read();
    for (int i = 0; i < 3; i++){
      IMU_APP_Data.HkTlm.Payload.AccelRead[i] = IMU_APP_Data.AccelRead[i];
      IMU_APP_Data.HkTlm.Payload.GyroRead[i] = IMU_APP_Data.GyroRead[i];
      IMU_APP_Data.HkTlm.Payload.AngleRead[i] = IMU_APP_Data.AngleRead[i];
    }

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(IMU_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(IMU_APP_Data.HkTlm.TelemetryHeader), true);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* IMU NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 IMU_APP_Noop(const IMU_APP_NoopCmd_t *Msg)
{
    IMU_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(IMU_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: NOOP command %s",
                      IMU_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 IMU_APP_ResetCounters(const IMU_APP_ResetCountersCmd_t *Msg)
{
    IMU_APP_Data.CmdCounter = 0;
    IMU_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(IMU_APP_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: RESET command");

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  IMU_APP_Config_MPU6050:                                              */
/*         This function allows to config the MPU6050 by sending data to      */
/*         sensor's registers.                                                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 IMU_APP_Config_MPU6050(const IMU_APP_Config_MPU6050_t *Msg){


  IMU_APP_Data.RegisterPtr = Msg->Register;
  IMU_APP_Data.DataVal = Msg->Data;

  int ptr = Msg->Register;

  if(ptr == 0x36 || (ptr >= 0x39 && ptr <= 0x61)){

    IMU_APP_Data.ErrCounter++;
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: The register %d is Read Only",IMU_APP_Data.RegisterPtr);

  }else{
    int fd;
    int rv;

    fd = open(&mpu6050_path[0], O_RDWR);

    rv = sensor_mpu6050_set_register(fd);
    if (rv == 0){
      CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Configure command %d to register %d", IMU_APP_Data.DataVal, IMU_APP_Data.RegisterPtr);
    }
    else{
      CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Failed to configure");
    }

    close(fd);
  }

  return CFE_SUCCESS;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Verify command packet length                                               */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool IMU_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(IMU_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        IMU_APP_Data.ErrCounter++;
    }

    return result;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Functions to interact with the MPU6050                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/

int32 mpu6050_init(void){
  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_mpu6050(
    &bus_path[0],
    &mpu6050_path[0]
  );
  if(rv == 0)
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Device registered correctly at %s",
                      mpu6050_path);

  fd = open(&mpu6050_path[0], O_RDWR);
  if(fd >= 0)
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Device opened correctly at %s",
                      mpu6050_path);

  // Device configuration
  rv = sensor_mpu6050_begin(fd);
  CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Device configured correctly %s",
                    mpu6050_path);

  close(fd);

  CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Calculating offsets, do not move MPU6050");
  sensor_mpu6050_calcOffsets();
  CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Done calculating offsets");

  return CFE_SUCCESS;

}

void mpu6050_read(void){
  SENSOR_MPU6050_Data_t mpu;

  //Data reading
  sensor_mpu6050_read_data();
  mpu = sensor_mpu6050_get_data();
  IMU_APP_Data.AccelRead[0] = mpu.accX;
  IMU_APP_Data.AccelRead[1] = mpu.accY;
  IMU_APP_Data.AccelRead[2] = mpu.accZ;
  IMU_APP_Data.GyroRead[0] = mpu.gyroX;
  IMU_APP_Data.GyroRead[1] = mpu.gyroY;
  IMU_APP_Data.GyroRead[2] = mpu.gyroZ;
  IMU_APP_Data.AngleRead[0] = mpu.angleX;
  IMU_APP_Data.AngleRead[1] = mpu.angleY;
  IMU_APP_Data.AngleRead[2] = mpu.angleZ;

}

/*
* PROTOTYPES
*/
static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val);
static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff);

static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis);
static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis);

static float wrap(float angle,float limit);
static void setGyroOffsets(float x, float y, float z);
static void setAccOffsets(float x, float y, float z);
static void setFilterGyroCoef(float gyro_coeff);

static int sensor_mpu6050_update_accel(void);

static int sensor_mpu6050_update_gyro(void);

static int sensor_mpu6050_update_temp(void);

/*
* STATIC FUNCTIONS
*/

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    printf("ioctl failed...\n");
  } else {

    free(*buff);
    *buff = malloc(nr_bytes * sizeof(uint8_t));

    for (i = 0; i < nr_bytes; ++i) {
      (*buff)[i] = value[i];
    }
  }

  return rv;
}

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val){
  uint8_t out[2] = { ptr, val };
  i2c_msg msgs[1] = {
    {
      .addr = dev->address,
      .flags = 0,
      .len = (uint16_t) sizeof(out),
      .buf = &out[0]
    }
  };

  return i2c_bus_transfer(dev->bus, &msgs[0], RTEMS_ARRAY_SIZE(msgs));
}

static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint16_t chip_address = (uint16_t) 0x68;
  uint8_t data_address = (uint8_t) register_add;

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  (*buff)[0] = *tmp;
  free(tmp);

  return rv;
}

static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;

  switch (command) {
    case SENSOR_MPU6050_BEGIN:
      setFilterGyroCoef(DEFAULT_GYRO_COEFF);
      setGyroOffsets(0,0,0);
      setAccOffsets(0,0,0);

      // Sanity check
      uint8_t *whoami;
      whoami = NULL;
      sensor_mpu6050_get_reg_8(MPU6050_WHOAMI, &whoami);
      if ((*whoami) != 0x68) {
        err = -ENOTTY;
        break;
      }

      // Sensor configuration
      err =       sensor_mpu6050_set_reg_8(dev, PWR_MGT_1, 0x00);               //Temp sensor enabled, internal 8MHz oscillator and cycle disabled
      err = err + sensor_mpu6050_set_reg_8(dev, SIGNAL_PATH_RESET, 0b00000111); //Accelerometer, Gyroscope and Thermometer path reset
      err = err + sensor_mpu6050_set_reg_8(dev, ACCEL_CONFIG, 0b00000001);      //5Hz filter +-2g
      err = err + sensor_mpu6050_set_reg_8(dev, GYRO_CONFIG, 0b00000000);       //+-250 deg/s
      err = err + sensor_mpu6050_set_reg_8(dev, INT_ENABLE, 1<<WOM_EN);         //Disables interrupts in MPU6050

      sensor_mpu6050_read_data();
      SENSOR_MPU6050_Priv_Data.angleX = SENSOR_MPU6050_Priv_Data.angleAccX;
      SENSOR_MPU6050_Priv_Data.angleY = SENSOR_MPU6050_Priv_Data.angleAccY;
      SENSOR_MPU6050_Priv_Data.preInterval = runtime_ms(); // may cause lack of angular accuracy if begin() is much before the first update()
      break;

    case SENSOR_MPU6050_SET_REG:
      err = sensor_mpu6050_set_reg_8(dev, IMU_APP_Data.RegisterPtr, IMU_APP_Data.DataVal);
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

static void setGyroOffsets(float x, float y, float z){
  SENSOR_MPU6050_Priv_Data.gyroXoffset = x;
  SENSOR_MPU6050_Priv_Data.gyroYoffset = y;
  SENSOR_MPU6050_Priv_Data.gyroZoffset = z;
}

static void setAccOffsets(float x, float y, float z){
  SENSOR_MPU6050_Priv_Data.accXoffset = x;
  SENSOR_MPU6050_Priv_Data.accYoffset = y;
  SENSOR_MPU6050_Priv_Data.accZoffset = z;
}

static void setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) || (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  SENSOR_MPU6050_Priv_Data.filterGyroCoef = gyro_coeff;
}

static int sensor_mpu6050_update_gyro(void){
  int16_t var;

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  tmp = NULL;
	err = sensor_mpu6050_get_gyro_axis(&tmp, X);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.gyroX = (float)(var * (250.0/32768.0)) - SENSOR_MPU6050_Priv_Data.gyroXoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Y);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.gyroY = (float)(var * (250.0/32768.0)) - SENSOR_MPU6050_Priv_Data.gyroYoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Z);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.gyroZ = (float)(var * (250.0/32768.0)) - SENSOR_MPU6050_Priv_Data.gyroZoffset;

  free(tmp);

  if(err != 0)
    printf("There was an error when reading gyroscope registers...\n");

  return err;
}

static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(2 * sizeof(uint8_t));

  switch (axis) {
    case X:
      err = sensor_mpu6050_get_reg_8(GYRO_XOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_XOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Y:
      err = sensor_mpu6050_get_reg_8(GYRO_YOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_YOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Z:
      err = sensor_mpu6050_get_reg_8(GYRO_ZOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_ZOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    default:
      err = -1;
  }

  return err;
}

static int sensor_mpu6050_update_accel(void){

  int16_t var;

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  tmp = NULL;
	err = sensor_mpu6050_get_accel_axis(&tmp, X);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.accX = (float)(var * (9.81/16384.0)) - SENSOR_MPU6050_Priv_Data.accXoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_accel_axis(&tmp, Y);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.accY = (float)(var * (9.81/16384.0)) - SENSOR_MPU6050_Priv_Data.accYoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_accel_axis(&tmp, Z);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.accZ = (float)(var * (9.81/16384.0)) - SENSOR_MPU6050_Priv_Data.accZoffset;

  free(tmp);

  if(err != 0)
    printf("There was an error when reading accelerometer registers...\n");

  return err;
}

static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(2 * sizeof(uint8_t));

  switch (axis) {
    case X:
      err = sensor_mpu6050_get_reg_8(ACCEL_XOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_XOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Y:
      err = sensor_mpu6050_get_reg_8(ACCEL_YOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_YOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Z:
      err = sensor_mpu6050_get_reg_8(ACCEL_ZOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_ZOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    default:
      err = -1;
  }

  return err;
}

static int sensor_mpu6050_update_temp(void){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  uint8_t raw_data[2];

  err = sensor_mpu6050_get_reg_8(TEMP_OUT_H, &tmp);
  raw_data[0] = (*tmp);
  tmp = NULL;
  err = err + sensor_mpu6050_get_reg_8(TEMP_OUT_L, &tmp);
  raw_data[1] = (*tmp);

  free(tmp);

  if(err != 0){
    printf("There was an error when reading temperature registers...\n");
    return -1;
  }

  int16_t raw_temp = raw_data[0] << 8 | raw_data[1];

  SENSOR_MPU6050_Priv_Data.temp = ((float)raw_temp / 340.0) + 36.53;

  return err;

}

/*
* PUBLIC FUNCTIONS
*/

int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPU6050_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_mpu6050_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_mpu6050_begin(int fd){
  return ioctl(fd, SENSOR_MPU6050_BEGIN, NULL);
}

int sensor_mpu6050_set_register(int fd){
  return ioctl(fd, SENSOR_MPU6050_SET_REG, NULL);
}

void sensor_mpu6050_calcOffsets(void){
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro

  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    sensor_mpu6050_update_accel();
    sensor_mpu6050_update_gyro();
    sensor_mpu6050_update_temp();
  	ag[0] += SENSOR_MPU6050_Priv_Data.accX;
  	ag[1] += SENSOR_MPU6050_Priv_Data.accY;
  	ag[2] += (SENSOR_MPU6050_Priv_Data.accZ-1.0);
  	ag[3] += SENSOR_MPU6050_Priv_Data.gyroX;
  	ag[4] += SENSOR_MPU6050_Priv_Data.gyroY;
  	ag[5] += SENSOR_MPU6050_Priv_Data.gyroZ;
  	delay_ms(1); // wait a little bit between 2 measurements
  }

  SENSOR_MPU6050_Priv_Data.accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.accZoffset = ag[2] / CALIB_OFFSET_NB_MES;

  SENSOR_MPU6050_Priv_Data.gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
}

void sensor_mpu6050_read_data(){
  // retrieve raw data
  sensor_mpu6050_update_accel();
  sensor_mpu6050_update_gyro();
  sensor_mpu6050_update_temp();

  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = SENSOR_MPU6050_Priv_Data.accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  SENSOR_MPU6050_Priv_Data.angleAccX =   atan2(SENSOR_MPU6050_Priv_Data.accY, sgZ*sqrt(SENSOR_MPU6050_Priv_Data.accZ*SENSOR_MPU6050_Priv_Data.accZ + SENSOR_MPU6050_Priv_Data.accX*SENSOR_MPU6050_Priv_Data.accX)) * RAD_2_DEG; // [-180,+180] deg
  SENSOR_MPU6050_Priv_Data.angleAccY = - atan2(SENSOR_MPU6050_Priv_Data.accX,     sqrt(SENSOR_MPU6050_Priv_Data.accZ*SENSOR_MPU6050_Priv_Data.accZ + SENSOR_MPU6050_Priv_Data.accY*SENSOR_MPU6050_Priv_Data.accY)) * RAD_2_DEG; // [- 90,+ 90] deg

  unsigned long Tnew = runtime_ms();
  float dt = (Tnew - SENSOR_MPU6050_Priv_Data.preInterval) * 1e-3;
  SENSOR_MPU6050_Priv_Data.preInterval = Tnew;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  SENSOR_MPU6050_Priv_Data.angleX = wrap(SENSOR_MPU6050_Priv_Data.filterGyroCoef*(SENSOR_MPU6050_Priv_Data.angleAccX + wrap(SENSOR_MPU6050_Priv_Data.angleX +     SENSOR_MPU6050_Priv_Data.gyroX*dt - SENSOR_MPU6050_Priv_Data.angleAccX,180)) + (1.0-SENSOR_MPU6050_Priv_Data.filterGyroCoef)*SENSOR_MPU6050_Priv_Data.angleAccX,180);
  SENSOR_MPU6050_Priv_Data.angleY = wrap(SENSOR_MPU6050_Priv_Data.filterGyroCoef*(SENSOR_MPU6050_Priv_Data.angleAccY + wrap(SENSOR_MPU6050_Priv_Data.angleY + sgZ*SENSOR_MPU6050_Priv_Data.gyroY*dt - SENSOR_MPU6050_Priv_Data.angleAccY, 90)) + (1.0-SENSOR_MPU6050_Priv_Data.filterGyroCoef)*SENSOR_MPU6050_Priv_Data.angleAccY, 90);
  SENSOR_MPU6050_Priv_Data.angleZ += SENSOR_MPU6050_Priv_Data.gyroZ*dt; // not wrapped

}

float sensor_mpu6050_get_temp(void){
  sensor_mpu6050_update_temp();
  return SENSOR_MPU6050_Priv_Data.temp;
}

SENSOR_MPU6050_Data_t sensor_mpu6050_get_data(void){
  SENSOR_MPU6050_Data_t Data;
  Data.temp = SENSOR_MPU6050_Priv_Data.temp;
  Data.accX = SENSOR_MPU6050_Priv_Data.accX;
  Data.accY = SENSOR_MPU6050_Priv_Data.accY;
  Data.accZ = SENSOR_MPU6050_Priv_Data.accZ;
  Data.gyroX = SENSOR_MPU6050_Priv_Data.gyroX;
  Data.gyroY = SENSOR_MPU6050_Priv_Data.gyroY;
  Data.gyroZ = SENSOR_MPU6050_Priv_Data.gyroZ;
  Data.angleX = SENSOR_MPU6050_Priv_Data.angleX;
  Data.angleY = SENSOR_MPU6050_Priv_Data.angleY;
  Data.angleZ = SENSOR_MPU6050_Priv_Data.angleZ;

  return Data;
}
