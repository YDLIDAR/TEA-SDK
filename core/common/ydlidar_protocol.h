/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#pragma once
#include <core/base/v8stdint.h>

#define Node_Sync 1     /// Starting Node
#define Node_NotSync 2  /// Normal Node

#if defined(_WIN32)
#pragma pack(1)
#endif
/**
 * @brief UDP Data format
 */
#define DATABLOCK_COUNT (12)
#define DATA_COUNT (16)
struct TIADataBlock {
    uint16_t frameHead;
    uint16_t startAngle;
    uint32_t data[DATA_COUNT];
}__attribute__((packed));

struct TIADataFrame {
    TIADataBlock dataBlock[DATABLOCK_COUNT];
    uint32_t timeStamp;
    uint32_t factory;
}__attribute__((packed));

#if defined(_WIN32)
#pragma pack()
#endif

///JSON命令
typedef struct _TIALidarConfig {
    int samplerate;
    int motorSpeed;
    int angleCompensation;
    int isMultiPoint;
    int APD;
    int LD;
    int distanceCompensation;
    int measureMode;
    int calMode;
    int heartbeat;
    int scanType;
    int restart;
} TIALidarConfig;

///获取在线雷达
struct TIALidarListInfo {
    /*! Address of the serial port (this can be passed to the constructor of Serial). */
    std::string ip;
    /*! Human readable description of serial device if available. */
    std::string model;
    /*! Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available. */
    std::string hardware;
    /*! Hardware Device ID or "" if not available. */
    std::string software;
};