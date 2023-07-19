
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "ydlidar_sdk.h"
#if defined(_MSC_VER)
#pragma comment(lib, "TEA_SDK.lib")
#endif


int main(int argc, char *argv[]) {
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);
    char port[20];
    os_init();

    printf("Please enter the lidar IP:");
    scanf("%s", port);

    if (!os_isOk()) {
        return 0;
    }

    float frequency = 20.0;
    while (os_isOk()) {
        printf("Please enter the lidar scan frequency[10-30]:");
        scanf("%f", &frequency);
        if (frequency <= 30 && frequency >= 10.0) {
            break;
        }
        fprintf(stderr,
                "Invalid scan frequency,The scanning frequency range is 10 to 30 HZ, Please re-enter.\n");
    }

    if (!os_isOk()) {
        return 0;
    }

    YDLidar *lidar = lidarCreate();

    //////////////////////string property/////////////////  

    setlidaropt(lidar, LidarPropSerialPort, port, sizeof(port));///雷达ip

    //////////////////////int property/////////////////
    int optval = 8090;
    setlidaropt(lidar, LidarPropSerialBaudrate, &optval, sizeof(int));///tcp端口，用于配置

    optval = TYPE_TIA;
    setlidaropt(lidar, LidarPropLidarType, &optval, sizeof(int));///雷达型号

    //////////////////////bool property/////////////////
    bool b_optvalue = true;
    setlidaropt(lidar, LidarPropAutoReconnect, &b_optvalue, sizeof(bool));///是否重连/热插拔

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 330.f;
    setlidaropt(lidar, LidarPropMaxAngle, &f_optvalue, sizeof(float));///扫描最大角度

    f_optvalue = 30.f;
    setlidaropt(lidar, LidarPropMinAngle, &f_optvalue, sizeof(float));///扫描最小角度

    /// unit: m
    f_optvalue = 64.f;
    setlidaropt(lidar, LidarPropMaxRange, &f_optvalue, sizeof(float));///最大有效扫描半径

    f_optvalue = 0.01;
    setlidaropt(lidar, LidarPropMinRange, &f_optvalue, sizeof(float));///最小有效扫描半径

    /// unit: Hz
    f_optvalue = frequency;
    setlidaropt(lidar, LidarPropScanFrequency, &f_optvalue, sizeof(float));///扫描频率
    

    bool ret = initialize(lidar);///雷达初始化

    if (ret) {
        ret = turnOn(lidar);///启动
    }  else {
        fprintf(stderr, "%s\n", DescribeError(lidar));
        fflush(stderr);
    }

    LaserFan scan;
    LaserFanInit(&scan);
    while (ret && os_isOk()) {
        if (doProcessSimple(lidar, &scan)) {//获取一圈点云数据
            fprintf(stdout, "Scan received[%lu]: %u pionts, scanning frequency is [%f]Hz.\n",
                    scan.stamp,
                    (unsigned int)scan.npoints, 
                    1.0 / scan.config.scan_time);
            fflush(stdout);
        } else {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }

    if (ret) {
        turnOff(lidar);///关闭
    }
    disconnecting(lidar);///断开连接
    return 0;
}
