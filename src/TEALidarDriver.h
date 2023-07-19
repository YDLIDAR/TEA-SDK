#ifndef TEALIDAR_DRIVER_H
#define TEALIDAR_DRIVER_H
#include <stdlib.h>
#include <core/common/DriverInterface.h>
#include <core/network/PassiveSocket.h>

namespace ydlidar {

using namespace std;   
using namespace core::base;
using namespace core::common;
using namespace core::network;


class TEALidarDriver : public DriverInterface {

private:
    string m_ip;
    uint32_t m_cmd_port;
    uint32_t m_data_port;
    uint32_t m_list_port;
    CActiveSocket *m_socket_cmd;
    CPassiveSocket *m_socket_data;
    CPassiveSocket *m_socket_list;
    Locker m_ListLock;
    Thread m_ListThread;
    vector<NetLidarListInfo> m_lidarList;
    NetLidarConfig m_lidarConfig;

public:
    /**
     * @par Constructor
     *
     */
    TEALidarDriver();

    /**
     * @par Destructor
     *
     */
    ~TEALidarDriver();

/*--------------------------------------------------------------------------------------------------------------
                                                     本类的私有函数
---------------------------------------------------------------------------------------------------------------*/
private:

    /**
     * @brief TCP connect(8090) \n
     * @param[in] lidarIP    Ip Address
     * @param[in] tcpPort     network port
     * @param[in] timeout     timeout
     * @return connection status
     * @retval true  success
     * @retval fase  failed
     */
    bool configPortConnect(const char *lidarIP, int tcpPort = 8090, uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief TCP disconnect(8090).
     */
    bool configPortDisconnect();   

    /**
     * @brief Transfer command by tcp \n
     * @param[in] transBuf      command buffer
     * @param[in] transLen      command length
     * @param[out] recvBuf      recv buffer
     * @param[out] recvMaxSize  recv max size
     * @retval true  success
     * @retval fase  failed
     */
    bool configPortTransfer(char *transBuf, int transLen, char *recvBuf, int recvMaxSize);

    /**
     * @brief Transfer command by tcp \n
     * @param[in] transBuf      The command buffer
     * @param[in] transLen      The command length
     * @param[out] recvBuf      The recv buffer
     * @param[out] recvMaxSize  recv max size
     * @retval true  success
     * @retval fase  failed
     */
    result_t configMessage(char opn, const char *descriptor, int &value, uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief send start scan commamd \n
     * @param[in] timeout  timeout
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    result_t startMeasure(uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief send stop scan commamd \n
     * @param[in] timeout  timeout
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    result_t stopMeasure(uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief UDP connect(8000) \n
     * @param[in] lidarIP    Ip Address
     * @param[in] tcpPort     network port
     * @param[in] timeout     timeout
     * @return connection status
     * @retval true  success
     * @retval fase  failed
     */
    bool dataPortConnect(const char *lidarIP, int localPort = 8000);

    /**
     * @brief UDP disconnect(8000).
     */
    bool dataPortDisconnect();

    /**
     * @brief Disable data grabbing.
     */
    void disableDataGrabbing();

    /**
     * @brief UDP connect(8001) \n
     * @param[in] lidarIP    Ip Address
     * @param[in] tcpPort     network port
     * @param[in] timeout     timeout
     * @return connection status
     * @retval true  success
     * @retval fase  failed
     */
    bool listPortConnect(const char *lidarIP, int localPort = 8001);

    /**
     * @brief UDP disconnect(8001).
     */
    bool listPortDisconnect();

    /**
     * @brief Reconnect to the network \n
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    result_t checkAutoConnecting();

    /**
     * @brief Receiving the scan data \n
     */ 
    int32_t receiveData(uint8_t *buf, uint32_t len);

    /**
     * @brief explaining the scan data \n
     */ 
    result_t waitScanData(node_info *nodebuffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT);  

    /**
     * @brief cache the scan data \n
     */ 
    int cacheScanData();

    /**
     * @brief Creating a Process to receiving scan data \n
     */   
    result_t createThread();  

    /**
     * @brief Receiving broadcast data \n
     */
    int GetListInfo();

    /**
     * @brief Creating a Process to receiving broadcast data \n
     */
    result_t createGetListThread(); 



/*--------------------------------------------------------------------------------------------------------------
                                           从DriverInterface继承的纯虚函数
---------------------------------------------------------------------------------------------------------------*/
public:
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
    virtual result_t connect(const char *path, uint32_t baudrate);

    /**
     * @brief Disconnect the LiDAR.
     */
    virtual void disconnect();

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
    virtual result_t grabScanData(node_info *nodebuffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief Turn on scanning \n
     * @param[in] timeout  timeout
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t startScan(uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief Turn off scanning \n
     * @param[in] timeout  timeout
     * @return result status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

    
    /**
     * @brief Get lidar scan frequency \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t getScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Set lidar scan frequency \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Increase the scanning frequency by 1.0 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyAdd(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Reduce the scanning frequency by 1.0 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyDis(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Increase the scanning frequency by 0.1 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyAddMic(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Reduce the scanning frequency by 0.1 HZ \n
     * @param[in] frequency    scanning frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setScanFrequencyDisMic(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Get lidar sampling frequency \n
     * @param[in] frequency    sampling frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t getSamplingRate(sampling_rate &rate, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Set the lidar sampling frequency \n
     * @param[in] rate    　　　sampling frequency
     * @param[in] timeout      timeout
     * @return return status
     * @retval RESULT_OK       success
     * @retval RESULT_FAILE    failed
     */
    virtual result_t setSamplingRate(sampling_rate &rate, uint32_t timeout = DEFAULT_TIMEOUT);
    
    /**
     * @brief Returns a human-readable description of the given error code
     *  or the last error code of a socket or serial port
     * @param isTCP   TCP or UDP
     * @return error information
     */
    virtual const char *DescribeError(bool isTCP = true);
    
    /**
     * @brief Get lidar lists
     * @return online lidars
     */
    virtual map<string, string> lidarPortList(); 
};

} // namespace ydlidar

#endif //TEALIDAR_DRIVER_H