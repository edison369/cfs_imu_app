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
 *   This file contains the source code for the Altitude App.
 */

/*
** Include Files:
*/
#include "altitude_app_events.h"
#include "altitude_app_version.h"
#include "altitude_app.h"

/*
** global data
*/
ALTITUDE_APP_Data_t ALTITUDE_APP_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/*                                                                            */
/* Application entry point and main process loop                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void ALTITUDE_APP_Main(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(ALTITUDE_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = ALTITUDE_APP_Init();
    if (status != CFE_SUCCESS)
    {
        ALTITUDE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    status = mpu6050_conf();
    if (status != CFE_SUCCESS)
    {
        ALTITUDE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_ERROR,
                          "ALTITUDE APP: Error configurin MPU6050\n");
    }

    /*
    ** ALTITUDE Runloop
    */
    while (CFE_ES_RunLoop(&ALTITUDE_APP_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(ALTITUDE_APP_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, ALTITUDE_APP_Data.CommandPipe, CFE_SB_PEND_FOREVER);

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(ALTITUDE_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            ALTITUDE_APP_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(ALTITUDE_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ALTITUDE APP: SB Pipe Read Error, App Will Exit");

            ALTITUDE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(ALTITUDE_APP_PERF_ID);

    CFE_ES_ExitApp(ALTITUDE_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* Initialization                                                             */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 ALTITUDE_APP_Init(void)
{
    int32 status;

    ALTITUDE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    ALTITUDE_APP_Data.CmdCounter = 0;
    ALTITUDE_APP_Data.ErrCounter = 0;

    /*
    ** Initialize app configuration data
    */
    ALTITUDE_APP_Data.PipeDepth = ALTITUDE_APP_PIPE_DEPTH;

    strncpy(ALTITUDE_APP_Data.PipeName, "ALTITUDE_APP_CMD_PIPE", sizeof(ALTITUDE_APP_Data.PipeName));
    ALTITUDE_APP_Data.PipeName[sizeof(ALTITUDE_APP_Data.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    ALTITUDE_APP_Data.EventFilters[0].EventID = ALTITUDE_APP_STARTUP_INF_EID;
    ALTITUDE_APP_Data.EventFilters[0].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[1].EventID = ALTITUDE_APP_COMMAND_ERR_EID;
    ALTITUDE_APP_Data.EventFilters[1].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[2].EventID = ALTITUDE_APP_COMMANDNOP_INF_EID;
    ALTITUDE_APP_Data.EventFilters[2].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[3].EventID = ALTITUDE_APP_COMMANDRST_INF_EID;
    ALTITUDE_APP_Data.EventFilters[3].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[4].EventID = ALTITUDE_APP_INVALID_MSGID_ERR_EID;
    ALTITUDE_APP_Data.EventFilters[4].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[5].EventID = ALTITUDE_APP_LEN_ERR_EID;
    ALTITUDE_APP_Data.EventFilters[5].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[6].EventID = ALTITUDE_APP_PIPE_ERR_EID;
    ALTITUDE_APP_Data.EventFilters[6].Mask    = 0x0000;
    ALTITUDE_APP_Data.EventFilters[7].EventID = ALTITUDE_APP_DEV_INF_EID;
    ALTITUDE_APP_Data.EventFilters[7].Mask    = 0x0000;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(ALTITUDE_APP_Data.EventFilters, ALTITUDE_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Altitude App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(CFE_MSG_PTR(ALTITUDE_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(ALTITUDE_APP_HK_TLM_MID),
                 sizeof(ALTITUDE_APP_Data.HkTlm));

    /*
    ** Initialize output RF packet.
    */
    CFE_MSG_Init(CFE_MSG_PTR(ALTITUDE_APP_Data.OutData.TelemetryHeader), CFE_SB_ValueToMsgId(ALTITUDE_APP_RF_DATA_MID),
                 sizeof(ALTITUDE_APP_Data.OutData));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&ALTITUDE_APP_Data.CommandPipe, ALTITUDE_APP_Data.PipeDepth, ALTITUDE_APP_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Altitude App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ALTITUDE_APP_SEND_HK_MID), ALTITUDE_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Altitude App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to RF command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ALTITUDE_APP_SEND_RF_MID), ALTITUDE_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Altitude App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ALTITUDE_APP_CMD_MID), ALTITUDE_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Altitude App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }


    CFE_EVS_SendEvent(ALTITUDE_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE App Initialized.%s",
                      ALTITUDE_APP_VERSION_STRING);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ALTITUDE    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void ALTITUDE_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case ALTITUDE_APP_CMD_MID:
            ALTITUDE_APP_ProcessGroundCommand(SBBufPtr);
            break;

        case ALTITUDE_APP_SEND_HK_MID:
            ALTITUDE_APP_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case ALTITUDE_APP_SEND_RF_MID:
            ALTITUDE_APP_ReportRFTelemetry((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        default:
            CFE_EVS_SendEvent(ALTITUDE_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "ALTITUDE: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ALTITUDE ground commands                                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void ALTITUDE_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    /*
    ** Process "known" ALTITUDE app ground commands
    */
    switch (CommandCode)
    {
        case ALTITUDE_APP_NOOP_CC:
            if (ALTITUDE_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ALTITUDE_APP_NoopCmd_t)))
            {
                ALTITUDE_APP_Noop((ALTITUDE_APP_NoopCmd_t *)SBBufPtr);
            }

            break;

        case ALTITUDE_APP_RESET_COUNTERS_CC:
            if (ALTITUDE_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ALTITUDE_APP_ResetCountersCmd_t)))
            {
                ALTITUDE_APP_ResetCounters((ALTITUDE_APP_ResetCountersCmd_t *)SBBufPtr);
            }

            break;

        case ALTITUDE_APP_CONFIG_MPU6050_CC:
            if (ALTITUDE_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(ALTITUDE_APP_Config_MPU6050_t)))
            {
                ALTITUDE_APP_Config_MPU6050((ALTITUDE_APP_Config_MPU6050_t *)SBBufPtr);
            }

            break;

	// TODO: Add the commands for the altitude control...

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(ALTITUDE_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }
}

int32 ALTITUDE_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg){

  /*
  ** Get command execution counters...
  */
  ALTITUDE_APP_Data.OutData.CommandErrorCounter = ALTITUDE_APP_Data.ErrCounter;
  ALTITUDE_APP_Data.OutData.CommandCounter      = ALTITUDE_APP_Data.CmdCounter;

  ALTITUDE_APP_Data.OutData.AppID_H = (uint8_t) ((ALTITUDE_APP_HK_TLM_MID >> 8) & 0xff);
  ALTITUDE_APP_Data.OutData.AppID_L = (uint8_t) (ALTITUDE_APP_HK_TLM_MID & 0xff);

  /* Copy the MPU6050 data */
  uint8_t *aux_array1;
  uint8_t *aux_array2;
  for(int i=0;i<3;i++){
      aux_array1 = NULL;
      aux_array1 = malloc(4 * sizeof(uint8_t));
      aux_array1 = (uint8_t*)(&ALTITUDE_APP_Data.AccelRead[i]);

      aux_array2 = NULL;
      aux_array2 = malloc(4 * sizeof(uint8_t));
      aux_array2 = (uint8_t*)(&ALTITUDE_APP_Data.GyroRead[i]);

      for(int j=0;j<4;j++){
          switch (i) {
            case 0:
              ALTITUDE_APP_Data.OutData.byte_group_1[j] = aux_array1[j];
              ALTITUDE_APP_Data.OutData.byte_group_4[j] = aux_array2[j];
              break;
            case 1:
              ALTITUDE_APP_Data.OutData.byte_group_2[j] = aux_array1[j];
              ALTITUDE_APP_Data.OutData.byte_group_5[j] = aux_array2[j];
              break;
            case 2:
              ALTITUDE_APP_Data.OutData.byte_group_3[j] = aux_array1[j];
              ALTITUDE_APP_Data.OutData.byte_group_6[j] = aux_array2[j];
              break;
          }
      }
  }

  /*
  ** Send housekeeping telemetry packet...
  */
  CFE_SB_TimeStampMsg(CFE_MSG_PTR(ALTITUDE_APP_Data.OutData.TelemetryHeader));
  CFE_SB_TransmitMsg(CFE_MSG_PTR(ALTITUDE_APP_Data.OutData.TelemetryHeader), true);

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
int32 ALTITUDE_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{

    /*
    ** Get command execution counters...
    */
    ALTITUDE_APP_Data.HkTlm.Payload.CommandErrorCounter = ALTITUDE_APP_Data.ErrCounter;
    ALTITUDE_APP_Data.HkTlm.Payload.CommandCounter      = ALTITUDE_APP_Data.CmdCounter;

    /* Copy the MPU6050 data */
    mpu6050_read_proc();
    for (int i = 0; i < 3; i++){
      ALTITUDE_APP_Data.HkTlm.Payload.AccelRead[i] = ALTITUDE_APP_Data.AccelRead[i];
      ALTITUDE_APP_Data.HkTlm.Payload.GyroRead[i] = ALTITUDE_APP_Data.GyroRead[i];
    }

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(ALTITUDE_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(ALTITUDE_APP_Data.HkTlm.TelemetryHeader), true);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* ALTITUDE NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 ALTITUDE_APP_Noop(const ALTITUDE_APP_NoopCmd_t *Msg)
{
    ALTITUDE_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(ALTITUDE_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: NOOP command %s",
                      ALTITUDE_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 ALTITUDE_APP_ResetCounters(const ALTITUDE_APP_ResetCountersCmd_t *Msg)
{
    ALTITUDE_APP_Data.CmdCounter = 0;
    ALTITUDE_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(ALTITUDE_APP_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: RESET command");

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  ALTITUDE_APP_Config_MPU6050:                                              */
/*         This function allows to config the MPU6050 by sending data to      */
/*         sensor's registers.                                                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 ALTITUDE_APP_Config_MPU6050(const ALTITUDE_APP_Config_MPU6050_t *Msg){


  ALTITUDE_APP_Data.RegisterPtr = Msg->Register;
  ALTITUDE_APP_Data.DataVal = Msg->Data;

  int ptr = Msg->Register;

  if(ptr == 0x36 || (ptr >= 0x39 && ptr <= 0x61)){

    ALTITUDE_APP_Data.ErrCounter++;
    CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: The register %d is Read Only",ALTITUDE_APP_Data.RegisterPtr);

  }else{
    int fd;
    int rv;

    static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

    fd = open(&mpu6050_path[0], O_RDWR);

    rv = sensor_mpu6050_set_register(fd);
    if (rv == 0){
      CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: Configure command %d to register %d", ALTITUDE_APP_Data.DataVal, ALTITUDE_APP_Data.RegisterPtr);
    }
    else{
      CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: Failed to configure");
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
bool ALTITUDE_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
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

        CFE_EVS_SendEvent(ALTITUDE_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        ALTITUDE_APP_Data.ErrCounter++;
    }

    return result;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Functions to interact with the MPU6050                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/

#ifdef MPU6050

  static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
  static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

  static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val);
  static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff);

  int32 mpu6050_conf(void){
    int rv;
    int fd;

    // Device registration
    rv = i2c_dev_register_sensor_mpu6050(
      &bus_path[0],
      &mpu6050_path[0]
    );
    if(rv == 0)
      CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: Device registered correctly at %s",
                        mpu6050_path);

    fd = open(&mpu6050_path[0], O_RDWR);
    if(fd >= 0)
      CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: Device opened correctly at %s",
                        mpu6050_path);

    // Device configuration
    rv = sensor_mpu6050_set_conf(fd);
    CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: Device configured correctly %s",
                      mpu6050_path);

    close(fd);

    fd = open(&bus_path[0], O_RDWR);
    if(fd >= 0)
      CFE_EVS_SendEvent(ALTITUDE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "ALTITUDE: Bus opened correctly at %s",
                        bus_path);
    close(fd);

    return CFE_SUCCESS;

  }

  void mpu6050_read_proc(void){

    //Data reading
    int16_t *accel_buff;
    int16_t *gyro_buff;

    accel_buff = NULL;
    sensor_mpu6050_get_accel(&accel_buff);

    ALTITUDE_APP_Data.AccelRead[X] = accel_buff[0] * (9.81/16384.0);
    ALTITUDE_APP_Data.AccelRead[Y] = accel_buff[1] * (9.81/16384.0);
    ALTITUDE_APP_Data.AccelRead[Z] = accel_buff[2] * (9.81/16384.0);

    free(accel_buff);

    gyro_buff = NULL;
    sensor_mpu6050_get_gyro(&gyro_buff);

    ALTITUDE_APP_Data.GyroRead[X] = gyro_buff[0] * (250.0/32768.0);
    ALTITUDE_APP_Data.GyroRead[Y] = gyro_buff[1] * (250.0/32768.0);
    ALTITUDE_APP_Data.GyroRead[Z] = gyro_buff[2] * (250.0/32768.0);

    free(gyro_buff);

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
      case SENSOR_MPU6050_SET_CONF:
        err = sensor_mpu6050_set_reg_8(dev, PWR_MGT_1, 0x08);                     //Temp sensor disabled, internal 8MHz oscillator and cycle disabled
        err = err + sensor_mpu6050_set_reg_8(dev, SIGNAL_PATH_RESET, 0x07); 	//Accelerometer, Gyroscope and Thermometer path reset
        err = err + sensor_mpu6050_set_reg_8(dev, ACCEL_CONFIG, 0x01);      	//5Hz filter +-2g
        err = err + sensor_mpu6050_set_reg_8(dev, INT_ENABLE, 1<<WOM_EN);         //Disables interrupts in MPU6050
        break;

      case SENSOR_MPU6050_SET_REG:
        err = sensor_mpu6050_set_reg_8(dev, ALTITUDE_APP_Data.RegisterPtr, ALTITUDE_APP_Data.DataVal);
        break;

      default:
        err = -ENOTTY;
        break;
    }

    return err;
  }

  int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path){
    i2c_dev *dev;

    dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPU6050_ADDRESS);
    if (dev == NULL) {
      return -1;
    }

    dev->ioctl = sensor_mpu6050_ioctl;

    return i2c_dev_register(dev, dev_path);
  }

  int sensor_mpu6050_set_conf(int fd){
    return ioctl(fd, SENSOR_MPU6050_SET_CONF, NULL);
  }

  int sensor_mpu6050_set_register(int fd){
    return ioctl(fd, SENSOR_MPU6050_SET_REG, NULL);
  }

  #ifdef gyroscope_read

  static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis);

  int sensor_mpu6050_get_gyro(int16_t **buff){

  	uint8_t *tmp;
    tmp = NULL;

    int err = 0;

    free(*buff);
    *buff = malloc(3 * sizeof(uint8_t));

    tmp = NULL;
  	err = sensor_mpu6050_get_gyro_axis(&tmp, X);
  	(*buff)[0] = (tmp[0]<<8)|(tmp[1]);

    tmp = NULL;
  	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Y);
  	(*buff)[1] = (tmp[0]<<8)|(tmp[1]);

    tmp = NULL;
  	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Z);
  	(*buff)[2] = (tmp[0]<<8)|(tmp[1]);

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
  #endif

  #ifdef accelerometer_read

  static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis);

  int sensor_mpu6050_get_accel(int16_t **buff){

  	uint8_t *tmp;
    tmp = NULL;

    int err = 0;

    free(*buff);
    *buff = malloc(3 * sizeof(uint8_t));

    tmp = NULL;
  	err = sensor_mpu6050_get_accel_axis(&tmp, X);
  	(*buff)[0] = (tmp[0]<<8)|(tmp[1]);

    tmp = NULL;
  	err = err + sensor_mpu6050_get_accel_axis(&tmp, Y);
  	(*buff)[1] = (tmp[0]<<8)|(tmp[1]);

    tmp = NULL;
  	err = err + sensor_mpu6050_get_accel_axis(&tmp, Z);
  	(*buff)[2] = (tmp[0]<<8)|(tmp[1]);

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
  #endif

#endif
