#ifndef CYDLIDAR_H
#define CYDLIDAR_H
#include <core/base/utils.h>
#include <core/common/ydlidar_def.h>
#include <core/common/DriverInterface.h>
#include <string>
#include <map>

using namespace std;
using namespace ydlidar;
using namespace ydlidar::core;
using namespace ydlidar::core::common;

class YDLIDAR_API CYdLidar {
    private:
        DriverInterface *m_lidarPtr;      ///< LiDAR Driver Interface pointer
        string m_SerialPort;              ///< LiDAR serial port or network ip
        int m_SerialBaudrate;             ///< LiDAR serial baudrate or network port
        int m_LidarType;                  ///< LiDAR type
        int m_lidar_model;                ///< LiDAR Model
        bool m_AutoReconnect;             ///< LiDAR hot plug 
        float m_MaxAngle;                 ///< LiDAR maximum angle
        float m_MinAngle;                 ///< LiDAR minimum angle
        float m_MaxRange;                 ///< LiDAR maximum range
        float m_MinRange;                 ///< LiDAR minimum range
        float m_field_of_view;            ///< LiDAR Field of View Angle.
        float m_ScanFrequency;            ///< LiDAR scanning frequency
        node_info *m_global_nodes;  

    public:
        /**
         * @brief create object
         */
        CYdLidar();

        /**
         * @brief destroy object
         */
        virtual ~CYdLidar();

        /**
         * @brief set lidar properties
         * @param optname        option name
         * @param optval         option value
         * @param optlen         option length
         * @return true if the Property is set successfully, otherwise false.
         */
        bool setlidaropt(int optname, const void *optval, int optlen);

        /**
         * @brief get lidar property
         * @param optname         option name
         * @param optval          option value
         * @param optlen          option length
         * @return true if the Property is get successfully, otherwise false.
         */
        bool getlidaropt(int optname, void *optval, int optlen);

        /**
         * @brief Initialize the SDK and LiDAR.
         * @return true if successfully initialized, otherwise false.
         */
        bool initialize();

        /**
         * @brief check LiDAR instance and connect to LiDAR,
         *  try to create a comms channel.
         * @return true if communication has been established with the device.
         *  If it's not false on error.
         */
        bool checkCOMMs();

        /**
         * @brief Start the device scanning routine which runs on a separate thread and enable motor.
         * @return true if successfully started, otherwise false.
         */
        bool turnOn();

        /**
         * @brief Stop the device scanning thread and disable motor.
         * @return true if successfully Stoped, otherwise false.
         */
        bool turnOff();

        /**
         * @brief Get the LiDAR Scan Data. turnOn is successful before doProcessSimple scan data.
         * @param[out] outscan             LiDAR Scan Data
         * @return true if successfully started, otherwise false.
         */
        bool doProcessSimple(LaserScan &outscan);

        /**
         * @brief Uninitialize the SDK and Disconnect the LiDAR.
         */
        void disconnecting();

        /**
         * @brief Get the last error information of a (socket or serial)
         * @return a human-readable description of the given error information
         * or the last error information of a (socket or serial)
         */
        const char *DescribeError() const;

        /**
         * @brief Get the last error information of lidar device
         * @return error information of lidar device
         */
        DriverError getDriverError() const;

        /**
         * @brief Get lidar lists
         * @return online lidars
         */
        map<string, string> lidarPortList();
};	// End of class
#endif // CYDLIDAR_H

//os
namespace ydlidar {
    /**
     * @brief system signal initialize
     */
    YDLIDAR_API void os_init();
    /**
     * @brief Whether system signal is initialized.
     * @return
     */
    YDLIDAR_API bool os_isOk();
    /**
     * @brief shutdown system signal
     */
    YDLIDAR_API void os_shutdown();
}


