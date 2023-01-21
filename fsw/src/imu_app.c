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

#include <math.h>

/*
** global data
*/
IMU_APP_Data_t IMU_APP_Data;

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
    if (status == CFE_SUCCESS){

      status = mpu6050_init();
      if (status != CFE_SUCCESS)
      {
        IMU_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_ERROR,
          "IMU APP: Error initializing MPU6050\n");
        }

    }else{
      IMU_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
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
    IMU_APP_Data.first_time = true;

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
    ** Initialize temperature packet.
    */
    CFE_MSG_Init(CFE_MSG_PTR(IMU_APP_Data.TempData.TelemetryHeader), CFE_SB_ValueToMsgId(IMU_APP_TEMP_MID),
               sizeof(IMU_APP_Data.TempData));

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
    ** Subscribe to Read command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(IMU_APP_READ_MID), IMU_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("IMU App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }

    /*
    ** Subscribe to Send Temp command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(IMU_APP_SEND_TP_MID), IMU_APP_Data.CommandPipe);
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

        case IMU_APP_READ_MID:
            IMU_APP_ReadSensor((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case IMU_APP_SEND_TP_MID:
            IMU_APP_SendTemp((CFE_MSG_CommandHeader_t *)SBBufPtr);
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

        case IMU_APP_CALCULATE_OFFSET_CC:
            if (IMU_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(IMU_APP_CalcOffsetsCmd_t)))
            {
                IMU_APP_CalcOffsets((IMU_APP_CalcOffsetsCmd_t *)SBBufPtr);
            }

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(IMU_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }
}

int32 IMU_APP_ReadSensor(const CFE_MSG_CommandHeader_t *Msg){
  if(IMU_APP_Data.first_time){
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Calculating offsets, do not move MPU6050");
    mpu6050_calcOffsets();
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Done calculating offsets");

    IMU_APP_Data.first_time = false;
  }

  /* Get the MPU6050 data */
  mpu6050_get_data();

  /* Get the MPU6050 temperature */
  IMU_APP_Data.TempRead = sensor_mpu6050_get_temp();

  return CFE_SUCCESS;
}

int32 IMU_APP_SendTemp(const CFE_MSG_CommandHeader_t *Msg){
  /* Copy the MPU6050 temperature */
  IMU_APP_Data.TempData.temperature = IMU_APP_Data.TempRead;

  /*
  ** Send temperature packet...
  */
  CFE_SB_TimeStampMsg(CFE_MSG_PTR(IMU_APP_Data.TempData.TelemetryHeader));
  CFE_SB_TransmitMsg(CFE_MSG_PTR(IMU_APP_Data.TempData.TelemetryHeader), true);

  return CFE_SUCCESS;
}

int32 IMU_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg){

  /*
  ** Get command execution counters...
  */
  IMU_APP_Data.OutData.CommandErrorCounter = IMU_APP_Data.ErrCounter;
  IMU_APP_Data.OutData.CommandCounter      = IMU_APP_Data.CmdCounter;

  /* Copy the app ID */
  IMU_APP_Data.OutData.AppID_H = (uint8_t) ((IMU_APP_HK_TLM_MID >> 8) & 0xff);
  IMU_APP_Data.OutData.AppID_L = (uint8_t) (IMU_APP_HK_TLM_MID & 0xff);

  /* Copy the MPU6050 data */
  uint8_t *aux_array1;
  uint8_t *aux_array2;
  for(int i=0;i<3;i++){
      aux_array1 = NULL;
      aux_array1 = malloc(4 * sizeof(uint8_t));
      aux_array1 = (uint8_t*)(&IMU_APP_Data.AccelRead[i]);

      aux_array2 = NULL;
      aux_array2 = malloc(4 * sizeof(uint8_t));
      aux_array2 = (uint8_t*)(&IMU_APP_Data.GyroRead[i]);

      for(int j=0;j<4;j++){
          switch (i) {
            case 0: // X axis
              IMU_APP_Data.OutData.byte_group_1[j] = aux_array1[j]; // Accel
              IMU_APP_Data.OutData.byte_group_4[j] = aux_array2[j]; // Gyro
              break;
            case 1: // Y axis
              IMU_APP_Data.OutData.byte_group_2[j] = aux_array1[j]; // Accel
              IMU_APP_Data.OutData.byte_group_5[j] = aux_array2[j]; // Gyro
              break;
            case 2: // Z axis
              IMU_APP_Data.OutData.byte_group_3[j] = aux_array1[j]; // Accel
              IMU_APP_Data.OutData.byte_group_6[j] = aux_array2[j]; // Gyro
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
int32 IMU_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg){

    /*
    ** Get command execution counters...
    */
    IMU_APP_Data.HkTlm.Payload.CommandErrorCounter = IMU_APP_Data.ErrCounter;
    IMU_APP_Data.HkTlm.Payload.CommandCounter      = IMU_APP_Data.CmdCounter;

    /* Copy the MPU6050 data to the App data (used by RF Tlm and UDP Tlm) */
    IMU_APP_Data.AccelRead[0] = IMU_APP_Data.Sensor_Data.accX;
    IMU_APP_Data.AccelRead[1] = IMU_APP_Data.Sensor_Data.accY;
    IMU_APP_Data.AccelRead[2] = IMU_APP_Data.Sensor_Data.accZ;
    IMU_APP_Data.GyroRead[0]  = IMU_APP_Data.Sensor_Data.gyroX;
    IMU_APP_Data.GyroRead[1]  = IMU_APP_Data.Sensor_Data.gyroY;
    IMU_APP_Data.GyroRead[2]  = IMU_APP_Data.Sensor_Data.gyroZ;
    IMU_APP_Data.AngleRead[0] = IMU_APP_Data.Sensor_Data.angleAccX;
    IMU_APP_Data.AngleRead[1] = IMU_APP_Data.Sensor_Data.angleAccY;
    IMU_APP_Data.AngleRead[2] = -1;

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
/*  IMU_APP_CalcOffsets:                                                      */
/*         This function calculates the MPU6050 offset by reading data.       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 IMU_APP_CalcOffsets(const IMU_APP_CalcOffsetsCmd_t *Msg)
{
    IMU_APP_Data.CmdCounter++;
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Calculating offsets, do not move MPU6050");
    mpu6050_calcOffsets();
    CFE_EVS_SendEvent(IMU_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "IMU: Done calculating offsets");

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  IMU_APP_Config_MPU6050:                                                   */
/*         This function allows to config the MPU6050 by sending data to      */
/*         sensor's registers.                                                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 IMU_APP_Config_MPU6050(const IMU_APP_Config_MPU6050_t *Msg){

  static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

  IMU_APP_Data.CmdCounter++;

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

    rv = sensor_mpu6050_set_register(fd, IMU_APP_Data.RegisterPtr, IMU_APP_Data.DataVal);
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

  static const char bus_path[] = "/dev/i2c-2";
  static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

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

  return CFE_SUCCESS;

}

void mpu6050_get_accel_gyro(void){
  IMU_APP_Data.Sensor_Data.accX = sensor_mpu6050_get_accel(X, IMU_APP_Data.Sensor_Data.offset.accX);
  IMU_APP_Data.Sensor_Data.accY = sensor_mpu6050_get_accel(Y, IMU_APP_Data.Sensor_Data.offset.accY);
  IMU_APP_Data.Sensor_Data.accZ = sensor_mpu6050_get_accel(Z, IMU_APP_Data.Sensor_Data.offset.accZ);
  IMU_APP_Data.Sensor_Data.gyroX = sensor_mpu6050_get_gyro(X, IMU_APP_Data.Sensor_Data.offset.gyroX);
  IMU_APP_Data.Sensor_Data.gyroY = sensor_mpu6050_get_gyro(Y, IMU_APP_Data.Sensor_Data.offset.gyroY);
  IMU_APP_Data.Sensor_Data.gyroZ = sensor_mpu6050_get_gyro(Z, IMU_APP_Data.Sensor_Data.offset.gyroZ);
}

void mpu6050_get_data(void){
  //Data reading from the accelerometer and gyroscope
  mpu6050_get_accel_gyro();

  // Estimate angles
  float sgZ = IMU_APP_Data.Sensor_Data.accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  IMU_APP_Data.Sensor_Data.angleAccX =   atan2(IMU_APP_Data.Sensor_Data.accY,
                                              sgZ*sqrt(IMU_APP_Data.Sensor_Data.accZ*IMU_APP_Data.Sensor_Data.accZ
                                              + IMU_APP_Data.Sensor_Data.accX*IMU_APP_Data.Sensor_Data.accX)) * RAD_2_DEG; // [-180,+180] deg
  IMU_APP_Data.Sensor_Data.angleAccY = - atan2(IMU_APP_Data.Sensor_Data.accX,
                                              sqrt(IMU_APP_Data.Sensor_Data.accZ*IMU_APP_Data.Sensor_Data.accZ
                                              + IMU_APP_Data.Sensor_Data.accY*IMU_APP_Data.Sensor_Data.accY)) * RAD_2_DEG; // [- 90,+ 90] deg

}

void mpu6050_calcOffsets(void){

  IMU_APP_Data.Sensor_Data.offset.accX = 0;
  IMU_APP_Data.Sensor_Data.offset.accY = 0;
  IMU_APP_Data.Sensor_Data.offset.accZ = 0;

  IMU_APP_Data.Sensor_Data.offset.gyroX = 0;
  IMU_APP_Data.Sensor_Data.offset.gyroY = 0;
  IMU_APP_Data.Sensor_Data.offset.gyroZ = 0;
  float ag[6] = {0,0,0,0,0,0}; // 3 acc and 3 gyro

  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    mpu6050_get_data();
  	ag[0] += IMU_APP_Data.Sensor_Data.accX;
  	ag[1] += IMU_APP_Data.Sensor_Data.accY;
  	ag[2] += (IMU_APP_Data.Sensor_Data.accZ-1.0);
  	ag[3] += IMU_APP_Data.Sensor_Data.gyroX;
  	ag[4] += IMU_APP_Data.Sensor_Data.gyroY;
  	ag[5] += IMU_APP_Data.Sensor_Data.gyroZ;
  	OS_TaskDelay(1); // wait a little bit between measurements
  }

  IMU_APP_Data.Sensor_Data.offset.accX = ag[0] / CALIB_OFFSET_NB_MES;
  IMU_APP_Data.Sensor_Data.offset.accY = ag[1] / CALIB_OFFSET_NB_MES;
  IMU_APP_Data.Sensor_Data.offset.accZ = ag[2] / CALIB_OFFSET_NB_MES;

  IMU_APP_Data.Sensor_Data.offset.gyroX = ag[3] / CALIB_OFFSET_NB_MES;
  IMU_APP_Data.Sensor_Data.offset.gyroY = ag[4] / CALIB_OFFSET_NB_MES;
  IMU_APP_Data.Sensor_Data.offset.gyroZ = ag[5] / CALIB_OFFSET_NB_MES;
}
