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
 * Define IMU App  Messages and info
 */

#ifndef IMU_APP_MSG_H
#define IMU_APP_MSG_H

/*
** IMU App command codes
*/
#define IMU_APP_NOOP_CC           0
#define IMU_APP_RESET_COUNTERS_CC 1
#define IMU_APP_CONFIG_MPU6050_CC 2
#define IMU_APP_CALCULATE_OFFSET_CC 4

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
} IMU_APP_NoArgsCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef IMU_APP_NoArgsCmd_t IMU_APP_NoopCmd_t;
typedef IMU_APP_NoArgsCmd_t IMU_APP_ResetCountersCmd_t;
typedef IMU_APP_NoArgsCmd_t IMU_APP_CalcOffsetsCmd_t;

/*************************************************************************/

/*
** Type definition (config registers of the MPU6050)
*/
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
    uint8                   Register;  /**< \brief Register Pointer     */
    uint8                   Data;  /**< \brief Data Value     */
} IMU_APP_Config_MPU6050_t;

/*************************************************************************/
/*
** Type definition (IMU App housekeeping)
*/

typedef struct
{
    uint8 CommandCounter;
    uint8 CommandErrorCounter;
    uint8 spare[2];
    float AccelRead[3];
    float GyroRead[3];
    float AngleRead[3];
} IMU_APP_HkTlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader; /**< \brief Telemetry header */
    uint8_t AppID_H;
    uint8_t AppID_L;
    uint8 CommandCounter;
    uint8 CommandErrorCounter;
    uint8 spare[2];
    uint8 byte_group_1[4];  // Accelx
    uint8 byte_group_2[4];  // Accely
    uint8 byte_group_3[4];  // Accelz
    uint8 byte_group_4[4];  // Gyrox
    uint8 byte_group_5[4];  // Gyroy
    uint8 byte_group_6[4];  // Gyroz
    uint8 byte_group_7[4];  // Anglex
    uint8 byte_group_8[4];  // Angley
    uint8 byte_group_9[4];  // Anglez
} IMU_APP_OutData_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader; /**< \brief Telemetry header */
    IMU_APP_HkTlm_Payload_t Payload;         /**< \brief Telemetry payload */
} IMU_APP_HkTlm_t;

#endif /* IMU_APP_MSG_H */
