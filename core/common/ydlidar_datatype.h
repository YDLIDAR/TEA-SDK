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
#include <core/base/datatype.h>
#include <vector>
#include "ydlidar_def.h"


/**
 * @brief The Laser Debug struct
 */
typedef struct  {
    uint8_t     W3F4CusMajor_W4F0CusMinor;
    uint8_t     W4F3Model_W3F0DebugInfTranVer;
    uint8_t     W3F4HardwareVer_W4F0FirewareMajor;
    uint8_t     W7F0FirewareMinor;
    uint8_t     W3F4BoradHardVer_W4F0Moth;
    uint8_t     W2F5Output2K4K5K_W5F0Date;
    uint8_t     W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear;
    uint8_t     W7F0SnNumH;
    uint8_t     W7F0SnNumL;
    uint8_t     W7F0Health;
    uint8_t     W3F4CusHardVer_W4F0CusSoftVer;
    uint8_t     W7F0LaserCurrent;
    uint8_t     MaxDebugIndex;
} LaserDebug;


/**
 * @brief The Laser Scan Data struct
 * @par usage
 * @code
 * LaserScan data;
 * for(int i = 0; i < data.points.size(); i++) {
 *  //current LiDAR angle
 *  float angle = data.points[i].angle;
 *  //current LiDAR range
 *  float range = data.points[i].range;
 *  //current LiDAR intensity
 *  float intensity = data.points[i].intensity;
 *  //current LiDAR point stamp
 *  uint64_t timestamp = data.stamp + i * data.config.time_increment * 1e9;
 * }
 * LaserScanDestroy(&data);
 * @endcode
 * @par convert to ROS sensor_msgs::LaserScan
 * @code
 * LaserScan scan;
 * sensor_msgs::LaserScan scan_msg;
 * std::string frame_id = "laser_frame";
 * ros::Time start_scan_time;
 * start_scan_time.sec = scan.stamp/1000000000ul;
 * start_scan_time.nsec = scan.stamp%1000000000ul;
 * scan_msg.header.stamp = start_scan_time;
 * scan_msg.header.frame_id = frame_id;
 * scan_msg.angle_min =(scan.config.min_angle);
 * scan_msg.angle_max = (scan.config.max_angle);
 * scan_msg.angle_increment = (scan.config.angle_increment);
 * scan_msg.scan_time = scan.config.scan_time;
 * scan_msg.time_increment = scan.config.time_increment;
 * scan_msg.range_min = (scan.config.min_range);
 * scan_msg.range_max = (scan.config.max_range);
 * int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
 * scan_msg.ranges.resize(size);
 * scan_msg.intensities.resize(size);
 * for(int i=0; i < scan.points.size(); i++) {
 *  int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
 *  if(index >=0 && index < size) {
 *      scan_msg.ranges[index] = scan.points[i].range;
 *      scan_msg.intensities[index] = scan.points[i].intensity;
 *  }
 * }
 * @endcode
 */
typedef struct {
    uint64_t stamp = 0;/// System time when first range was measured in nanoseconds
    std::vector<LaserPoint> points;/// Array of lidar points
    LaserConfig config;/// Configuration of scan
    int moduleNum = 0;
    uint16_t envFlag = 0; //环境标记（目前只针对GS2）
} LaserScan;


//雷达节点信息
struct node_info {
    uint8_t sync_flag; //首包标记
    uint8_t is; //抗干扰标志
    uint16_t sync_quality; //信号强度
    uint16_t angle_q6_checkbit; //角度值（°）
    uint16_t distance_q2; //距离值
    uint64_t stamp; //时间戳
    uint32_t delay_time; ///< delay time
    uint8_t scan_frequence; //扫描频率
    uint8_t debugInfo; ///< debug information
    uint8_t index; //包序号
    uint8_t error_package; ///< error package state
} __attribute__((packed));


/// LiDAR Device Information
struct device_info {
    uint8_t   model; ///< LiDAR model
    uint16_t  firmware_version; ///< firmware version
    uint8_t   hardware_version; ///< hardare version
    uint8_t   serialnum[16];    ///< serial number
} __attribute__((packed)) ;


/// LiDAR Health Information
struct device_health {
    uint8_t   status; ///< health state
    uint16_t  error_code; ///< error code
} __attribute__((packed))  ;


/// LiDAR sampling Rate struct
struct sampling_rate {
    uint8_t rate;	///< sample rate
} __attribute__((packed))  ;


/// LiDAR scan frequency struct
struct scan_frequency {
    uint32_t frequency;	///< scan frequency
} __attribute__((packed))  ;


struct scan_rotation {
    uint8_t rotation;
} __attribute__((packed))  ;


/// LiDAR Exposure struct
struct scan_exposure {
    uint8_t exposure;	///< low exposure
} __attribute__((packed))  ;


/// LiDAR Heart beat struct
struct scan_heart_beat {
    uint8_t enable;	///< heart beat
} __attribute__((packed));


struct scan_points {
    uint8_t flag;
} __attribute__((packed))  ;


struct function_state {
    uint8_t state;
} __attribute__((packed))  ;


/// LiDAR Zero Offset Angle
struct offset_angle {
    int32_t angle;
} __attribute__((packed))  ;