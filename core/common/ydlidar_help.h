/*********************************************************************
* Software License Agreement (MIT License)
*
* Copyright © 2020 EAIBOT, Inc.
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation
* files (the “Software”), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*  @file     ydlidar_help.h                                                  *
*  @brief    LiDAR Help function                                             *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuirurong618@eaibot.com                                     *
*  @version  1.0.0                                                           *
*  @date     2020/02/14                                                      *
*  @license  MIT                               *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2020/02/14 | 1.0.0     | Tony           | Lidar Help File                 *
*----------------------------------------------------------------------------*
*                                                                            *
*********************************************************************/
#pragma once
#include "DriverInterface.h"
#include "ydlidar_protocol.h"
#include <sstream>
#include <vector>

/**
 * @brief ydlidar
 */
namespace ydlidar {
/**
 * @brief ydlidar core
 */
namespace core {
using namespace base;
/**
 * @brief ydlidar common
 */
namespace common {


/*!
 * @brief convert lidar model to string
 * @param model lidar model
 * @return lidar model name
 */
inline std::string lidarModelToString(int model) {
    std::string name = "unkown";
    switch (model) {
        case DriverInterface::YDLIDAR_TIA:
        name = "TIA";
        break;

        default:
        name = "unkown(YD-" + std::to_string(model) + ")";
        break;
  }
  return name;
}

/*!
 * @brief Get LiDAR default sampling rate.
 * @param model lidar model.
 * @return lidar sampling rate.
 */
inline std::vector<int> getDefaultSampleRate(int model) {
    std::vector<int> srs;
    switch (model) {
        case DriverInterface::YDLIDAR_TIA:
        srs.push_back(20);
        break;

        default:
        srs.push_back(4);
        break;
    }
    return srs;
}

/*!
 * @brief Supports multiple sampling rate
 * @param model   lidar model
 * @return true if THere are multiple sampling rate, otherwise false.
 */
inline bool hasSampleRate(int model) {
    bool ret = false;
    if (model == DriverInterface::YDLIDAR_TIA) {
        ret = true;
    }
    return ret;
}

/*!
 * @brief Is there a zero offset angle
 * @param model   lidar model
 * @return true if there are zero offset angle, otherwise false.
 */

inline bool hasZeroAngle(int model) {
    bool ret = false;
    if (model == DriverInterface::YDLIDAR_TIA) {
        ret = true;
    }
    return ret;
}

/*!
 * @brief Whether to support adjusting the scanning frequency .
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool hasScanFrequencyCtrl(int model) {
    bool ret = true;
    if (model == DriverInterface::YDLIDAR_TIA) {
        ret = false;
    }
    return ret;
}

/*!
 * @brief Does SDK support the LiDAR model.
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool isSupportLidar(int model) {
    if (model == DriverInterface::YDLIDAR_TIA) {
        return false;
    }
    return true;
}

/*!
 * @brief Whether to support intensity.
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool hasIntensity(int model) {
    bool ret = false;
    if (model == DriverInterface::YDLIDAR_TIA) {
        ret = true;
    }
    return ret;
}

/*!
 * @brief Whether to support serial DTR enable motor.
 * @param model   lidar model
 * @return true if support serial DTR enable motor, otherwise false.
 */
inline bool isSupportMotorCtrl(int model) {
    bool ret = false;
    if (model == DriverInterface::YDLIDAR_TIA) {
        ret = true;
    }
    return true;
}

/*!
 * @brief Whether the scanning frequency is supported
 * @param model     lidar model
 * @param frequency scanning frequency
 * @return true if supported, otherwise false.
 */
inline bool isSupportScanFrequency(int model, double frequency) {
    bool ret = false;
    if (model = DriverInterface::YDLIDAR_TIA) {
        if (1 <= frequency && frequency <= 64) {
            ret = true;
        }
    }
    return ret;
}


/**
 * @brief Whether it is a TOF type LiDAR
 * @param type  LiDAR type
 * @return true if it is a TOF type, otherwise false.
 */
inline bool isTOFLidar(int type) {
    bool ret = false;
    if (type == TYPE_TOF) {
        ret = true;
    }
    return ret;
}

/**
 * @brief Whether it is a network hardware interface TOF type LiDAR
 * @param type  LiDAR type
 * @return true if it is a network hardware interface TOF type, otherwise false.
 */
inline bool isNetTOFLidar(int type) {
    bool ret = false;
    if (type == TYPE_TOF_NET) {
        ret = true;
    }
    return ret;
}

/**
 * @brief Whether it is a Triangle type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isTriangleLidar(int type) {
    bool ret = false;
    if (type == TYPE_TRIANGLE) {
        ret = true;
    }
    return ret;
}

/**
 * @brief Whether it is a GS type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isGSLidar(int type) {
    return (type == TYPE_GS1 ||
        type == TYPE_GS);
}

/**
 * @brief Whether it is a GS1 type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isGS1Lidar(int type) {
    return (type == TYPE_GS1);
}

/**
 * @brief Whether it is a GS2 type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isGS2Lidar(int type) {
    return (type == TYPE_GS);
}

/**
 * @brief Whether it is a GS2 type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isTIALidar(int type) {
    return (type == TYPE_TIA);
}


/*!
 * @brief Whether to support Heartbeat.
 * @param model   lidar model
 * @return true if support heartbeat, otherwise false.
 */
inline bool isSupportHeartBeat(int model) {
    bool ret = false;
    if (model == DriverInterface::YDLIDAR_TIA) {
        ret = true;
    }
    return true;
}

/**
 * @brief Whether the sampling rate is valid
 * @param smap  sampling rate map
 * @return true if it is valid, otherwise false.
 */
inline bool isValidSampleRate(std::map<int, int>  smap) {
    if (smap.size() < 1) {
        return false;
    }
    if (smap.size() == 1) {
        if (smap.begin()->second > 2) {
        return true;
        }
        return false;
    }
    return false;
}


/**
 * @brief print LiDAR version information
 * @param info      LiDAR Device information
 * @param port      LiDAR serial port or IP Address
 * @param baudrate  LiDAR serial baudrate or network port
 * @return true if Device information is valid, otherwise false
 */
inline bool printfVersionInfo(const device_info &info,
                              const std::string &port,
                              int baudrate) {
    if (info.firmware_version == 0 &&
        info.hardware_version == 0) {
        return false;
    }

    uint8_t Major = (uint8_t)(info.firmware_version >> 8);
    uint8_t Minjor = (uint8_t)(info.firmware_version & 0xff);
    printf("[YDLIDAR] Connection established in [%s][%d]:\n"
            "Firmware version: %u.%u\n"
            "Hardware version: %u\n"
            "Model: %s\n"
            "Serial: ",
            port.c_str(),
            baudrate,
            Major,
            Minjor,
            (unsigned int)info.hardware_version,
            lidarModelToString(info.model).c_str());

    for (int i = 0; i < 16; i++) {
        printf("%01X", info.serialnum[i] & 0xff);
    }

    printf("\n");
    fflush(stdout);
    return true;
}

/**
 * @brief split string to vector by delim format
 * @param s       string
 * @param delim   split format
 * @return split vector
 */
inline std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;

    while (std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }

    return elems;
}

///以16进制打印数据
inline void printHex(const uint8_t *data, int size)
{
    if (!data)
        return;
    for (int i=0; i<size; ++i)
        printf("%02X", data[i]);
    printf("\n");
}


/// Count the number of elements in a statically allocated array.
#if !defined(_countof)
    #define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

///日志打印
#define COLOR
#ifdef COLOR
    #define COLOFF             "\033[0m"      ///关闭所有属性
    #define RED                "\033[0;31m"   ///"\033[显示方式;字体颜色;背景颜色m"
    #define GREEN              "\033[0;32m"
    #define YELLOW             "\033[0;33m"
    #define BLUE               "\033[0;34m"
    #define PURPLE             "\033[0;35m"
#else
    #define COLOFF              
    #define RED               
    #define GREEN              
    #define YELLOW             
    #define BLUE               
    #define PURPLE             
#endif

#define LOG(Color, Severity, format, ...)  do{ printf(Color "[LIDAR SDK] [%s]> " COLOFF format "\n", #Severity, ##__VA_ARGS__); fflush(stdout); } while(0)
#define LOGD(...) LOG(GREEN,  DEBUG,   __VA_ARGS__)
#define LOGI(...) LOG(BLUE,   INFOR,   __VA_ARGS__)
#define LOGW(...) LOG(YELLOW, WARRING, __VA_ARGS__)
#define LOGE(...) LOG(RED,    ERROR,   __VA_ARGS__)
#define LOGF(...) LOG(PURPLE, FALT,    __VA_ARGS__) 


/// 短整型大小端互换
#define BigLittleSwap16(A) ((((uint16_t)(A) & 0xff00) >> 8) | \
                             (((uint16_t)(A) & 0x00ff) << 8))

/// 长整型大小端互换
#define BigLittleSwap32(A) ((((uint32_t)(A) & 0xff000000) >> 24) | \
                            (((uint32_t)(A) & 0x00ff0000) >>  8) | \
                            (((uint32_t)(A) & 0x0000ff00) <<  8) | \
                            (((uint32_t)(A) & 0x000000ff) << 24))

}//common
}//core
}//ydlidar
