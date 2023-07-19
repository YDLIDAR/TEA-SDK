#include "TEALidarDriver.h"
#include <core/serial/common.h>
#include <math.h>
#include <core/tools/cJSON.h>
#include <core/base/thread.h>
#include <core/common/ydlidar_help.h>


namespace ydlidar {

TEALidarDriver::TEALidarDriver() 
{
    m_ip = "192.168.0.11";
    m_cmd_port = 8090;
    m_data_port = 8000;
    m_list_port = 8001;
    m_socket_cmd = new CActiveSocket(CSimpleSocket::SocketTypeTcp);
    m_socket_cmd->SetConnectTimeout(DEFAULT_CONNECTION_TIMEOUT_SEC, DEFAULT_CONNECTION_TIMEOUT_USEC);
    m_socket_data = new CPassiveSocket(CSimpleSocket::SocketTypeUdp);
    m_socket_data->SetSocketType(CSimpleSocket::SocketTypeUdp);
    m_socket_list = new CPassiveSocket(CSimpleSocket::SocketTypeUdp);
    m_socket_list->SetSocketType(CSimpleSocket::SocketTypeUdp);

    //父类成员变量
    m_ScanNodeBuf = new node_info[MAX_SCAN_NODES];
}

TEALidarDriver::~TEALidarDriver() {
    disconnect();
    ScopedLocker data_lock(m_DataLock);
    if (m_socket_data) {
        delete m_socket_data;
        m_socket_data = NULL;
    }
    ScopedLocker cmd_lock(m_CmdLock);
    if (m_socket_cmd) {
        delete m_socket_cmd;
        m_socket_cmd = NULL;
    }
    ScopedLocker list_lock(m_ListLock);
    if (m_socket_list) {
        delete m_socket_list;
        m_socket_list = NULL;
    }
    if (m_ScanNodeBuf) {
        delete[]  m_ScanNodeBuf;
        m_ScanNodeBuf = nullptr;
    }
}

/*--------------------------------------------------------------------------------------------------------------
                                                     本类的私有函数
---------------------------------------------------------------------------------------------------------------*/

bool TEALidarDriver::configPortConnect(const char *lidarIP, int tcpPort, uint32_t timeout) {
    ScopedLocker lock(m_CmdLock);
    if (!m_socket_cmd) {
        return false;
    }

    if (!m_socket_cmd->IsSocketValid()) {
        if (!m_socket_cmd->Initialize()) {
            return false;
        }
    }else {
        return m_socket_cmd->IsSocketValid();
    }
    m_socket_cmd->SetNonblocking();
    if (!m_socket_cmd->Open(lidarIP, tcpPort)) { 
        m_socket_cmd->Close();
        return false;
    }

    m_socket_cmd->SetSendTimeout(timeout / 1000, (timeout % 1000) * 1000);
    m_socket_cmd->SetReceiveTimeout(timeout / 1000, (timeout % 1000) * 1000);
    m_socket_cmd->SetBlocking();
    return m_socket_cmd->IsSocketValid();
}


bool TEALidarDriver::configPortDisconnect() {
    ScopedLocker lock(m_CmdLock);
    if (!m_socket_cmd) {
            return false;
    }
    return m_socket_cmd->Close();
}


bool TEALidarDriver::configPortTransfer(char *transBuf, int transLen, char *recvBuf, int recvMaxSize) {
    ScopedLocker lock(m_CmdLock);
    int len = 0;
    if (!m_socket_cmd){
        return false;
    }

    do{
        len += m_socket_cmd->Send(reinterpret_cast<uint8_t *>(transBuf + len), transLen - len);
    }while(len < transLen);
    
    LOGD("TCP SNED(%d):\n%s", len, transBuf);
    if (m_socket_cmd->Select(0, 800000)) {
        if(m_socket_cmd->Receive(recvMaxSize, reinterpret_cast<uint8_t *>(recvBuf)) > 0) {
            return true;
        }
    }
    return false;
}


result_t TEALidarDriver::configMessage(char op, const char *descriptor, int &value, uint32_t timeout) {
    char name[64] = {0};
    char transbuf[256] = {0};
    char recvbuf[256] = {0};
    cJSON *root = NULL;
    cJSON *item = NULL;

    strncpy(name, descriptor, sizeof(name));
    valLastName(name);
    if(op == 'w' || op == 'W') {
        sprintf(transbuf, "{\"%s\":%d}", name, value);
    }else if(op == 'r' || op == 'R') {
        sprintf(transbuf, "{\"Read\":\"%s\"}", name);
    }else {
        LOGW("op error!");
        return RESULT_FAIL;
    }

    if(!configPortConnect(m_ip.c_str(), m_cmd_port, timeout)) {
        setDriverError(NotOpenError);
        return RESULT_FAIL;
    }    
    if(!configPortTransfer(transbuf, strlen(transbuf), recvbuf, sizeof(recvbuf))) {
        configPortDisconnect();
        return RESULT_FAIL;
    }
    configPortDisconnect();

    root = cJSON_Parse(recvbuf);
    if (!root){
        return RESULT_FAIL;
    }
    item = cJSON_GetObjectItem(root, name);
    if(!cJSON_IsNumber(item)) {
        cJSON_Delete(root);
        return RESULT_FAIL;
    }
    value = item->valueint;
    LOGD("TCP RECV(%d):\n%s,valid value = %d", strlen(recvbuf), recvbuf, item->valueint);
    cJSON_Delete(root);
    return RESULT_OK;
}


result_t TEALidarDriver::startMeasure(uint32_t timeout) {
    m_lidarConfig.scanType = 0;
    if(!IS_OK(configMessage('w', valName(m_lidarConfig.scanType), m_lidarConfig.scanType))) {
        return RESULT_FAIL;
    }
    return RESULT_OK;
}


result_t TEALidarDriver::stopMeasure(uint32_t timeout) {
    m_lidarConfig.scanType = -1;
    if(!IS_OK(configMessage('w', valName(m_lidarConfig.scanType), m_lidarConfig.scanType))) {
        return RESULT_FAIL;
    }
    return RESULT_OK;
}

bool TEALidarDriver::dataPortConnect(const char *lidarIP, int localPort) {
    ScopedLocker lock(m_DataLock);

    if (!m_socket_data) {
        return false;
    }
    if (!m_socket_data->IsSocketValid()) {
        if (m_socket_data->Initialize()) {
            if (!m_socket_data->Listen(lidarIP, localPort)) {
                m_socket_data->Close();
                return false;
            }
            m_socket_data->SetReceiveTimeout(DEFAULT_TIMEOUT / 1000, (DEFAULT_TIMEOUT % 1000) * 1000);
        }
    }
    return m_socket_data->IsSocketValid();
}

bool TEALidarDriver::dataPortDisconnect() {
    ScopedLocker lock(m_DataLock);

    if (!m_socket_data) {
        return false;
    }
    return m_socket_data->Close();

}


void TEALidarDriver::disableDataGrabbing() {
    ScopedLocker l(m_Lock);
    m_DataEvent.set();
    m_Thread.join();
}


bool TEALidarDriver::listPortConnect(const char *lidarIP, int localPort) {
    ScopedLocker lock(m_ListLock);

    if (!m_socket_list) {
        return false;
    }
    if (!m_socket_list->IsSocketValid()) {
        if (m_socket_list->Initialize()) {
            if (!m_socket_list->Listen(NULL, localPort)) {
                m_socket_list->Close();
                return false;
            }
            //m_socket_list->SetReceiveTimeout(DEFAULT_TIMEOUT / 1000, (DEFAULT_TIMEOUT % 1000) * 1000);
        }
    }

    if (!IS_OK(createGetListThread())){
        return false;
    }

    return m_socket_list->IsSocketValid();
}


bool TEALidarDriver::listPortDisconnect() {
    ScopedLocker lock(m_ListLock);
    if (!m_socket_list) {
        return false;
    }
    m_ListThread.join();
    return m_socket_list->Close();
}


result_t TEALidarDriver::checkAutoConnecting() {
    int retryConnect = 0;
    setIsAutoconnting(true);
    while(getIsAutoReconnect()) {
        disconnect();
        retryConnect = retryConnect > 25 ? 25 : retryConnect + 1;
        for(int i = 0; i < retryConnect; i++){
            delay(200);    
        }
        LOGD("Reconnecting...");
        if(!IS_OK(connect(m_ip.c_str(), m_cmd_port))) {
            setDriverError(NotOpenError);
        }else {
            setDriverError(NoError);
            break;
        }
    }
    setIsAutoconnting(false);
    return RESULT_OK;
}

int32_t TEALidarDriver::receiveData(uint8_t *buf, uint32_t len) 
{
    /* wait data from socket. */
    ScopedLocker lock(m_DataLock);
    if (!m_socket_data) {
            return -1;
    }
    int32_t l = m_socket_data->Receive(len, buf);
    // LOGD("UDP RECV(%d): ", l);
    // for (int32_t i=0; i<l; ++i)
    //     printf("%02X", buf[i]);
    // printf("\n");
    return l;
}

//解析大包数据（没有校验和，没有定义数据大小）
result_t TEALidarDriver::waitScanData(
    node_info *nodebuffer, 
    size_t &count, 
    uint32_t timeout) 
{
    result_t ret = RESULT_FAIL;
    NetDataFrame frame; //大包数据（包含12 * 小包数据16个点）
    uint8_t* p = reinterpret_cast<uint8_t*>(&frame);
    node_info *n = NULL;
    static uint16_t lastPointAngle = 0;
    static uint8_t lastNum = 0xff;
    count = 0;

    // uint8_t buff[DATA_ONESIZE] = {0};
    // receiveData(buff, DATA_ONESIZE);
    // return RESULT_OK;

    static uint8_t s_data[DATA_ONESIZE] = {0};
    static int s_dataSize = 0;
    int nl = 0;
    int rl = 0; //实际接收数据长度
    int pos = 0;
    uint32_t st = getms(); //开始时间
    uint32_t rt = 0; //当前时间差
    while ((rt = getms() - st) < timeout)
    {
        //每次读取固定大小的数据
        uint8_t data[DATA_ONESIZE] = {0};
        if (s_dataSize) // 将上次剩余的数据加进来
        {
            memcpy(data, s_data, s_dataSize);
            rl = s_dataSize;
            s_dataSize = 0;
        }
        else //新读取数据
        {
            rl = receiveData(data, DATA_ONESIZE);
            if (rl < 0)
            {
                LOGE("Recv data timeout");
                return RESULT_TIMEOUT;
            }
        }
        //遍历数据查找包结束标识0x214365
        for (size_t i = 0; i < rl; ++i)
        {
            uint8_t c = data[i];
            switch (pos)
            {
            case 0:
                if (c != 0x65)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 1:
                if (c != 0x43)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != 0x21)
                {
                    pos = 0;
                    continue;
                }
                break;
            }
            pos ++;

            if (pos == TEA_TAILSIZE)
            {
                pos = 0;
                i ++;
                //找到上一包结束标记以后获取整大包数据
                // LOGD("Start pos %llu", i);
                int rs = rl - i; // 数据实际大小
                int ss = 0;
                memcpy(p, data + i, rs); //截取当前UDP包中的剩余数据
                //继续获取UDP数据以补全一整大包数据
                while (rs < NETDATAFRAMESIXE2)
                {
                    rl = receiveData(data, DATA_ONESIZE);
                    if (rl < 0 || (rt = getms() - st) > timeout)
                    {
                        LOGE("Recv data timeout");
                        return RESULT_TIMEOUT;
                    }
                    ss = NETDATAFRAMESIXE2 - rs > DATA_ONESIZE ? DATA_ONESIZE :
                        NETDATAFRAMESIXE2 - rs;
                    memcpy(p + rs, data, ss);
                    rs += rl;
                    // if (rs >= NETDATAFRAMESIXE2)
                    //     LOGD("End pos %d %d", ss, DATA_ONESIZE * 2 - ss);
                    ss = DATA_ONESIZE - ss;
                }
                //将多余部分暂存在静态缓存中，等待下一次使用
                if (ss)
                {
                    memcpy(s_data, data + (DATA_ONESIZE - ss), ss);
                    s_dataSize += ss;
                }
                ret = RESULT_OK;
                break;
            }
        }

        if (IS_OK(ret))
        {
            break;
        }
    }

    //判断每小包数据的头部是否有效
    // for (int i = 0; i < DATABLOCK_COUNT; i++) 
    // {
    //     if (BigLittleSwap16(frame.dataBlock[i].frameHead) != 0xFFEE) {
    //         LOGE("data error, frameHead[%d] != 0xFFEE", i);
    //         return RESULT_FAIL;
    //     }
    // }

    //uint8_t curNum = (BigLittleSwap32(frame.factory) & 0x000F0000) >> 16;
    uint8_t curNum = (BigLittleSwap32(frame.factory) & 0x0F000000) >> 24;
    if ((curNum - lastNum != 1) && 
        (curNum - lastNum != -15) && 
        lastNum != 0xff) 
    {
        LOGE("data packet dropout, curNum = %d, lastNum = %d", curNum, lastNum);
        lastNum = curNum;
        return RESULT_FAIL;
    }
    lastNum = curNum;
    
    for (int i = 0; i < DATABLOCK_COUNT; i++) 
    {
        if (BigLittleSwap16(frame.dataBlock[i].frameHead) != 0xFFEE)
            continue;

        uint16_t startAngle = BigLittleSwap16(frame.dataBlock[i].startAngle);
        uint16_t addAngle = 0;
        for (int j = 0; j < DATA_COUNT; j++) 
        {
            uint32_t data = BigLittleSwap32(frame.dataBlock[i].data[j]);
            if (data != 0) {
                n = nodebuffer + count;
                addAngle += ((data & 0x3f000000) >> 24);
                n->angle_q6_checkbit = startAngle + addAngle;
                n->sync_flag = (n->angle_q6_checkbit < lastPointAngle) ? Node_Sync : Node_NotSync; //当前点的角度小于上一个点的角度，则认为当前点为零位点
                n->sync_quality = (data & 0xff0000) >> 16;
                n->distance_q2 = (data & 0xffff) >> 0;
                //n->stamp = timeStamp;
                lastPointAngle = n->angle_q6_checkbit;
                count ++;
            } else {
                break;
            }
        }
    }

    //处理时间戳
    static uint64_t TimeStampCount = 0;
    static uint64_t TimeStamp = 0;
    static uint64_t lastTimeStamp = 0;
    static uint32_t TimeStampTmp = 0;
    static uint32_t lastTimeStampTmp = 0;

    TimeStampTmp = BigLittleSwap32(frame.timeStamp);
    TimeStampCount = TimeStampTmp > lastTimeStampTmp ? 
        TimeStampCount : TimeStampCount + 1; //当前时间戳比上一轮时间戳小，说明时间戳溢出重新计数
    TimeStamp = 0xffffffff * TimeStampCount + TimeStampTmp;
    lastTimeStampTmp = TimeStampTmp;

    for (int i = 0; i < count; i++) {
        n = nodebuffer + i;
        n->stamp = TimeStamp - (TimeStamp - lastTimeStamp) * (count - i - 1) / count;
    }
    lastTimeStamp = TimeStamp;

    return RESULT_OK;
}

result_t TEALidarDriver::cacheScanData() 
{
    LOGD("Thread Start: [%s]", __func__);
    node_info      local_buf[DATABLOCK_COUNT * DATA_COUNT];
    node_info      local_scan[MAX_SCAN_NODES];
    size_t         timeout_count = 0;
    size_t         scan_count = 0;
    size_t         count = 0;
    result_t       ans = RESULT_FAIL;

    memset(&local_buf, 0, sizeof(local_buf));
    memset(&local_scan, 0, sizeof(local_scan));

    // while (!IS_OK(waitScanData(local_buf, count)));//丢弃一包

    while (getIsScanning()) 
    {
        count = 0;
        memset(local_buf, 0, sizeof(local_buf));
        ans = waitScanData(local_buf, count);
        if (IS_FAIL(ans)) {
            LOGE("bad data block!!!");
            // waitScanData(local_buf, count);//丢弃一包
            local_scan[0].sync_flag = Node_Sync;    
            continue;
        } else if (IS_TIMEOUT(ans)) {
            timeout_count++;
            LOGE("get data timeout(%d)!!!", timeout_count);
            if(timeout_count > DEFAULT_TIMEOUT_COUNT){
                setDriverError(TimeoutError);
                if (IS_OK(checkAutoConnecting())) {
                    // waitScanData(local_buf, count);//丢弃一包
                    local_scan[0].sync_flag = Node_Sync;  
                    timeout_count = 0;
                } else {
                    LOGE("exit scanning thread!!!");
                    return RESULT_FAIL;
                }
            }
            continue;
        } else {
            timeout_count = 0;
        }

        for (size_t pos = 0; pos < count; pos++) 
        {
            if (local_buf[pos].sync_flag & Node_Sync) {
                if ((local_scan[0].sync_flag & Node_Sync)) {
                    m_Lock.lock();//timeout lock, wait resource copy
                    memcpy(m_ScanNodeBuf, local_scan, scan_count * sizeof(node_info));
                    m_ScanNodeCount = scan_count;
                    m_DataEvent.set();
                    m_Lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) {
                scan_count -= 1;
            }
        }       
    }
    return RESULT_OK;
}

result_t TEALidarDriver::createThread() 
{
    m_Thread = CLASS_THREAD(TEALidarDriver, cacheScanData);
    if (m_Thread.getHandle() == 0) {
        return RESULT_FAIL;
    }
    return RESULT_OK;
}

result_t TEALidarDriver::GetListInfo() {
    LOGD("Thread Start:  [%s]", __func__);
    char name[64] = {0};
    char buf[256] = {0};
    int i;
    
    while (1) {
        if (!m_socket_list) {
            return -1;
        }
        memset(buf, 0, sizeof(buf));

        if(m_socket_list->Receive(sizeof(buf), (uint8_t*)buf) > 0) {
            NetLidarListInfo lst;
            cJSON *item = NULL;
            cJSON *root = cJSON_Parse(buf);
            if (!root){
                return RESULT_FAIL;
            }

            strncpy(name, (const char*)valName(lst.ip), sizeof(name));
            valLastName(name); 
            item = cJSON_GetObjectItem(root, name);
            lst.ip = item->valuestring;

            strncpy(name, (const char*)valName(lst.model), sizeof(name));
            valLastName(name); 
            item = cJSON_GetObjectItem(root, name);
            lst.model = item->valuestring;

            strncpy(name, (const char*)valName(lst.hardware), sizeof(name));
            valLastName(name); 
            item = cJSON_GetObjectItem(root, name);
            lst.hardware = item->valuestring;

            strncpy(name, (const char*)valName(lst.software), sizeof(name));
            valLastName(name); 
            item = cJSON_GetObjectItem(root, name);
            lst.software = item->valuestring;
            for(i = 0; i < m_lidarList.size(); i++) {
                if(m_lidarList[i].ip == lst.ip) {
                    break;
                }
            }
            if(i >= m_lidarList.size()) {
                m_lidarList.push_back(lst);
                LOGD("Find a new device, ip: %s, model: %s, hardware: %s, software: %s ", 
                      lst.ip.c_str(), lst.model.c_str(), lst.hardware.c_str(), lst.software.c_str());
            }
            cJSON_Delete(root);
        }
    }
    return RESULT_OK;
}


result_t TEALidarDriver::createGetListThread() {
    m_ListThread = CLASS_THREAD(TEALidarDriver, GetListInfo);
    if (m_ListThread.getHandle() == 0) {
        return RESULT_FAIL;
    }
    return RESULT_OK;
}





/*--------------------------------------------------------------------------------------------------------------
                                        从DriverInterface虚基类继承的纯虚函数
---------------------------------------------------------------------------------------------------------------*/

result_t TEALidarDriver::connect(const char *port_path, uint32_t baudrate) {
    m_ip = port_path;
    m_cmd_port = baudrate;

    if (!configPortConnect(port_path, baudrate)) {
        setDriverError(NotOpenError);
        return RESULT_FAIL;
    }
    configPortDisconnect();

    if (!dataPortConnect(NULL, m_data_port)) {
        setDriverError(NotOpenError);
        return RESULT_FAIL;
    }

    if (!listPortConnect(NULL, m_list_port)) {
        setDriverError(NotOpenError);
        return RESULT_FAIL;
    }

    setIsConnected(true);

    LOGD("Network connect success!");
    return RESULT_OK;
}


void TEALidarDriver::disconnect() {
    configPortDisconnect();
    dataPortDisconnect();
    listPortDisconnect();
    setIsConnected(false);
    LOGD("Network disconnection!");
}


result_t TEALidarDriver::grabScanData(node_info *nodebuffer, size_t &count, uint32_t timeout) {
    switch (m_DataEvent.wait(timeout)) {

        case Event::EVENT_TIMEOUT: {
            count = 0;
            return RESULT_TIMEOUT;
        }

        case Event::EVENT_OK: {
            if (m_ScanNodeCount == 0) {
                return RESULT_FAIL;
            }

            ScopedLocker l(m_Lock);
            size_t size_to_copy = std::min(count, m_ScanNodeCount);
            memcpy(nodebuffer, m_ScanNodeBuf, size_to_copy * sizeof(node_info));
            count = size_to_copy;
            m_ScanNodeCount = 0;
            return RESULT_OK;
        }
        
        default:
            count = 0;
            return RESULT_FAIL;
    }
}


result_t TEALidarDriver::startScan(uint32_t timeout) {
    if(getIsScanning()){
        LOGD("The lidar is scanning");
        return RESULT_OK;
    }
    if (!IS_OK(startMeasure())){
        stopMeasure();
        return RESULT_FAIL;
    }
    setIsScanning(true);  
    if (!IS_OK(createThread())){
        setIsScanning(false);  
        stopMeasure();
        return RESULT_FAIL;
    }
    LOGD("The radar starts scanning");
    return RESULT_OK;
}


result_t TEALidarDriver::stopScan(uint32_t timeout) {
    if(!getIsScanning()){
        LOGD("The lidar is not scanning");
        return RESULT_OK;
    }
    if(getIsAutoconnting()){
        setIsAutoReconnect(false);
        while(getIsAutoconnting());
        setIsAutoReconnect(true);
    }

    if (!IS_OK(stopMeasure())){
        return RESULT_FAIL;
    }
    setIsScanning(false);  
    disableDataGrabbing();
    LOGD("Radar stop scanning");
    return RESULT_OK;
}


result_t TEALidarDriver::getScanFrequency(scan_frequency &frequency, uint32_t timeout) {
    if(!IS_OK(configMessage('r', valName(m_lidarConfig.motorSpeed), m_lidarConfig.motorSpeed))) {
        return RESULT_FAIL;
    }
    frequency.frequency = m_lidarConfig.motorSpeed;
    return RESULT_OK;
}


result_t TEALidarDriver::setScanFrequency(scan_frequency &frequency, uint32_t timeout) {
    m_lidarConfig.motorSpeed = frequency.frequency;
    if(!IS_OK(configMessage('w', valName(m_lidarConfig.motorSpeed), m_lidarConfig.motorSpeed))) {
        return RESULT_FAIL;
    }
    return RESULT_OK;
}


result_t TEALidarDriver::setScanFrequencyAdd(scan_frequency &frequency, uint32_t timeout) {
    if(!IS_OK(getScanFrequency(frequency))) {
        return RESULT_FAIL;
    }
    frequency.frequency += 100;
    return setScanFrequency(frequency);
}


result_t TEALidarDriver::setScanFrequencyDis(scan_frequency &frequency, uint32_t timeout) {
    if(!IS_OK(getScanFrequency(frequency))) {
        return RESULT_FAIL;
    }
    frequency.frequency -= 100;
    return setScanFrequency(frequency);
}


result_t TEALidarDriver::setScanFrequencyAddMic(scan_frequency &frequency, uint32_t timeout) {
    if(!IS_OK(getScanFrequency(frequency))) {
        return RESULT_FAIL;
    }
    frequency.frequency += 10;
    return setScanFrequency(frequency);
}


result_t TEALidarDriver::setScanFrequencyDisMic(scan_frequency &frequency, uint32_t timeout) {
    if(!IS_OK(getScanFrequency(frequency))) {
        return RESULT_FAIL;
    }
    frequency.frequency -= 10;
    return setScanFrequency(frequency);
}


result_t TEALidarDriver::getSamplingRate(sampling_rate &rate, uint32_t timeout) {
    if(!IS_OK(configMessage('r', valName(m_lidarConfig.samplerate), m_lidarConfig.samplerate))) {
        return RESULT_FAIL;
    }
    rate.rate = m_lidarConfig.samplerate;
    return RESULT_OK;
}


result_t TEALidarDriver::setSamplingRate(sampling_rate &rate, uint32_t timeout) {
    m_lidarConfig.samplerate = rate.rate;
    if(!IS_OK(configMessage('w', valName(m_lidarConfig.samplerate), m_lidarConfig.samplerate))) {
        return RESULT_FAIL;
    }
    return RESULT_OK;
}


const char* TEALidarDriver::DescribeError(bool isTCP) {
    if (isTCP) {
        ScopedLocker lock(m_CmdLock);
        return m_socket_cmd != NULL ? m_socket_cmd->DescribeError() : "NO Socket";
    } else {
        ScopedLocker lock(m_DataLock);
        return m_socket_data != NULL ? m_socket_data->DescribeError() : "NO Socket";
    }
}


map<string, string> TEALidarDriver::lidarPortList() {
    vector<NetLidarListInfo> lst = m_lidarList;
    map<string, string> ports;

    for (vector<NetLidarListInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
        string port = "ydlidar" + (*it).ip;
        ports[port] = (*it).model;
    }
    return ports;
}

}//namespace ydlidar
