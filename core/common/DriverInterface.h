#pragma once
#include <core/base/v8stdint.h>
#include <core/base/thread.h>
#include <core/base/locker.h>
#include <map>
#include "ydlidar_def.h"
#include "ydlidar_datatype.h"
#include <ydlidar_config.h>

namespace ydlidar {
namespace core {
using namespace base;
namespace common {

class DriverInterface {
public:
    enum YDLIDAR_MODLES {
        YDLIDAR_TIA     = 300,/**< TIA LiDAR Model. */
        YDLIDAR_Tail,
    };

    enum YDLIDAR_RATE {
        YDLIDAR_RATE_4K = 0,/**< 4K sample rate code */
        YDLIDAR_RATE_8K = 1,/**< 8K sample rate code */
        YDLIDAR_RATE_9K = 2,/**< 9K sample rate code */
        YDLIDAR_RATE_10K = 3,/**< 10K sample rate code */
    };

    enum {
        DEFAULT_TIMEOUT = 2000,    /**< Default timeout. */
        DEFAULT_HEART_BEAT = 1000, /**< Default heartbeat timeout. */
        MAX_SCAN_NODES = 7200,	   /**< Default Max Scan Count. */
        DEFAULT_TIMEOUT_COUNT = 1, /**< Default Timeout Count. */
    };

protected:
    node_info *m_ScanNodeBuf;
    size_t m_ScanNodeCount;
    DriverError m_DriverErrno;
    Thread m_Thread;
    Event m_DataEvent;
    Locker m_Lock;
    Locker m_CmdLock;
    Locker m_DataLock;
    Locker m_ErrorLock;
    PropertyBuilderByName(bool, IsScanning, protected);
    PropertyBuilderByName(bool, IsConnected, protected);
    PropertyBuilderByName(bool, IsAutoReconnect, protected);
    PropertyBuilderByName(bool, IsAutoconnting, protected);

public:
    /**
     * @par Constructor
     *
     */
    DriverInterface(){
        node_info *m_ScanNodeBuf = NULL; 
        size_t m_ScanNodeCount = 0;
        DriverError m_DriverErrno = NoError;
        setIsScanning(false);
        setIsConnected(false);
        setIsAutoReconnect(true);
        setIsAutoconnting(false);
    }

    /**
     * @par Destructor
     *
     */
    virtual ~DriverInterface() {}

    /**
     * @brief Connecting Lidar \n
     * After the connection if successful, you must use ::disconnect to close
     * @param[in] port_path    serial port or network ip
     * @param[in] baudrate     serial baudrate or network port
     * @return connection status
     * @retval RESULT_OK     success
     * @retval RESULT_FAILE   failed
     * @note After the connection if successful, you must use ::disconnect to close
     * @see function ::YDlidarDriver::disconnect ()
     */
    virtual result_t connect(const char *path, uint32_t baudrate) = 0;

    /**
     * @brief Disconnect the LiDAR.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Get a circle of laser data \n
     * @param[in] nodebuffer Laser data
     * @param[in] count      one circle of laser points
     * @param[in] timeout    timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     * @note Before starting, you must start the start the scan successfully with the ::startScan function
     */
    virtual result_t grabScanData(node_info *nodebuffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT) = 0 ;

    /**
     * @brief Turn on scanning \n
     * @param[in] timeout  timeout
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t startScan(uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Turn off scanning \n
     * @param[in] timeout  timeout
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Get lidar scan frequency \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t getScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Set lidar scan frequency \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Increase the scanning frequency by 1.0 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyAdd(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Reduce the scanning frequency by 1.0 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyDis(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Increase the scanning frequency by 0.1 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyAddMic(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Reduce the scanning frequency by 0.1 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyDisMic(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Get lidar sampling frequency \n
     * @param[in] frequency    sampling frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t getSamplingRate(sampling_rate &rate, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Set the lidar sampling frequency \n
     * @param[in] rate    　　　sampling frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setSamplingRate(sampling_rate &rate, uint32_t timeout = DEFAULT_TIMEOUT) = 0;

    /**
     * @brief Returns a human-readable description of the given error code
     *  or the last error code of a socket or serial port
     * @param isTCP   TCP or UDP
     * @return error information
     */
    virtual const char *DescribeError(bool isTCP = true) = 0;

    /**
     * @brief Get lidar lists
     * @return online lidars
     */
    virtual std::map<std::string, std::string> lidarPortList() = 0;
    
    /**
     * @brief Get SDK Version \n
     * static function
     * @return Version
     */
    virtual std::string getSDKVersion(){ 
        return YDLIDAR_SDK_VERSION_STR;
    }

    /**
     * @brief Set driver error code
     * @param er
     */
    virtual void setDriverError(const DriverError &er) {
        ScopedLocker l(m_ErrorLock);
        if(m_DriverErrno == NoError){
            m_DriverErrno = er;
        }
    }

    /**
     * @brief Get driver error code
     * @return
     */
    virtual DriverError getDriverError() {
        ScopedLocker l(m_ErrorLock);
        return m_DriverErrno;
    }

    /**
     * @brief Returns a human-readable description of the given error code
     *  or the last error code of lidar driver.
     * @param err  error code
     * @return error information
     */
    static const char *DescribeDriverError(DriverError err) {
        char const *errorString = "Unknown error";
        switch (err) {
            case NoError:
            errorString = ("No error");
            break;

            case DeviceNotFoundError:
            errorString = ("Device is not found");
            break;

            case PermissionError:
            errorString = ("Device is not permission");
            break;

            case UnsupportedOperationError:
            errorString = ("unsupported operation");
            break;

            case NotOpenError:
            errorString = ("Device is not open");
            break;

            case TimeoutError:
            errorString = ("Operation timed out");
            break;

            case BlockError:
            errorString = ("Device Block");
            break;

            case NotBufferError:
            errorString = ("Device Failed");
            break;

            case TrembleError:
            errorString = ("Device Tremble");
            break;

            case LaserFailureError:
            errorString = ("Laser Failure");
            break;

            default:
            break;
        }
        return errorString;
    }
};

}//common
}//core
}//ydlidar
