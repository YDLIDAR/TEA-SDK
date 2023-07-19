#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "TEA_SDK.lib")
#endif

int main(int argc, char *argv[]) 
{
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);
    std::string port;
    
    ydlidar::os_init();
    if (!ydlidar::os_isOk())
        return 0;

    //让用户选择IP或者手动输入IP
    {
        //命令TCP 192.168.0.11 8090 
        //点云UDP 8000 
        //广播UDP 7777
        std::map<std::string, std::string> ports;
        std::map<std::string, std::string>::iterator it;
        ports["IP1"] = "192.168.0.11";
        ports["IP2"] = "Manual input IP";
        int id = 0;
        for (it = ports.begin(); it != ports.end(); ++it)
        {
            printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
            id ++;
        }

        while (ydlidar::os_isOk())
        {
            printf("Please select the lidar port:");
            std::string number;
            std::cin >> number;

            if ((size_t)atoi(number.c_str()) >= ports.size())
                continue;

            it = ports.begin();
            id = atoi(number.c_str());
            while (id)
            {
                id --;
                it ++;
            }
            port = it->second;
            break;
        }

        if (port == ports["IP2"])
        {
            printf("Please enter the lidar IP:"); 
            std::cin >> port;
        }
    }

    std::string input_frequency;
    float frequency = 20.0;
    while (ydlidar::os_isOk()) 
    {
        printf("Please enter the lidar scan frequency[10-30]:");
        std::cin >> input_frequency;
        frequency = atof(input_frequency.c_str());
        if (frequency <= 30 && frequency >= 10.0) {
            break;
        }
        fprintf(stderr,
                "Invalid scan frequency,The scanning frequency range is 10 to 30 HZ, Please re-enter.\n");
    }

    if (!ydlidar::os_isOk()) {
        return 0;
    }

    CYdLidar lidar;

    //////////////////////string property/////////////////  
    lidar.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());///雷达ip
    //////////////////////int property/////////////////
    int optval = 8090;
    lidar.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));///tcp端口，用于配置
    optval = TYPE_TEA;
    lidar.setlidaropt(LidarPropLidarType, &optval, sizeof(int));///雷达型号
    //////////////////////bool property/////////////////
    bool b_optvalue = true;
    lidar.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));///是否重连/热插拔
    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 330.f;
    lidar.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));///扫描最大角度
    f_optvalue = 30.f;
    lidar.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));///扫描最小角度
    /// unit: m
    f_optvalue = 64.f;
    lidar.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));///最大有效扫描半径
    f_optvalue = 0.01;
    lidar.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));///最小有效扫描半径
    /// unit: Hz
    f_optvalue = frequency;
    lidar.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));///扫描频率
    
    bool ret = lidar.initialize();///雷达初始化

    if (ret) {
        ret = lidar.turnOn();///启动
    }  else {
        fprintf(stderr, "%s\n", lidar.DescribeError());
        fflush(stderr);
    }

    LaserScan scan;

    while (ret && ydlidar::os_isOk())
    {
        if (lidar.doProcessSimple(scan))
        { 
            // 获取一圈点云数据
            fprintf(stdout, "Scan received [%llu] points, Rpm is [%f]Hz\n",
                    scan.points.size(),
                    1.0 / scan.config.scan_time);
            fflush(stdout);
        }
        else
        {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }

    if (ret)
        lidar.turnOff(); //关闭
    lidar.disconnecting(); //断开连接

    return 0;
}
