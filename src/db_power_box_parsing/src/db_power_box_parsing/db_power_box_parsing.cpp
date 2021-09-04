#include <db_power_box_parsing/db_power_box_parsing.h>


DodobotPowerBoxParsing::DodobotPowerBoxParsing(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ros::param::param<string>("~serial_port", _serialPort, "");
    ros::param::param<int>("~serial_baud", _serialBaud, 115200);
    ros::param::param<int>("~write_thread_rate", write_thread_rate, 60);
    ros::param::param<bool>("~use_sensor_msg_time", use_sensor_msg_time, true);

    ROS_INFO_STREAM("serial_port: " << _serialPort);
    ROS_INFO_STREAM("serial_baud: " << _serialBaud);

    large_packet_len = 0x1000;

    was_reporting = false;

    battery_msg.header.frame_id = "battery";
    battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    _serialBufferIndex = 0;
    _currentSegmentNum = -1;
    _readPacketNum = -1;
    _writePacketNum = 0;
    _currentBufferSegment = new char[SERIAL_BUFFER_SIZE];
    _recvCharIndex = 0;
    _readPacketLen = 0;
    _recvCharBuffer = new char[SERIAL_BUFFER_SIZE];
    _writeCharIndex = 0;
    _writeCharBuffer = new char[SERIAL_BUFFER_SIZE];

    readyState = new StructReadyState;
    readyState->robot_name = "";
    readyState->is_ready = false;
    readyState->time_ms = 0;

    deviceStartTime = ros::Time::now();
    offsetTimeMs = 0;

    battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 50);

    // gripper_sub = nh.subscribe<db_parsing::DodobotGripper>("gripper_cmd", 50, &DodobotPowerBoxParsing::gripperCallback, this);

    write_stop_flag = false;
    write_thread = new boost::thread(boost::bind(&DodobotPowerBoxParsing::write_thread_task, this));

    write_timer = ros::Time::now();

    packet_ok_timeout = ros::Duration(1.0);

    ROS_INFO("Dodobot serial bridge init done");
}


void DodobotPowerBoxParsing::configure()
{
    ROS_INFO("Configuring serial device.");
    // attempt to open the serial port
    try
    {
        ROS_DEBUG_STREAM("Selected port: " << _serialPort);
        _serialRef.setPort(_serialPort);
        ROS_DEBUG_STREAM("Selected baud: " << _serialBaud);
        _serialRef.setBaudrate(_serialBaud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        _serialRef.setTimeout(timeout);
        _serialRef.open();
        ROS_INFO("Serial device configured.");
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << _serialPort);
        ROS_ERROR_STREAM("Serial exception: " << e.what());
        cerr << "Serial exception: " << e.what() << endl;
        throw;
    }
}

void DodobotPowerBoxParsing::setStartTime(uint32_t time_ms) {
    deviceStartTime = ros::Time::now();
    offsetTimeMs = time_ms;
}

ros::Time DodobotPowerBoxParsing::getDeviceTime(uint32_t time_ms) {
    return deviceStartTime + ros::Duration((double)(time_ms - offsetTimeMs) / 1000.0);
}

void DodobotPowerBoxParsing::checkReady()
{
    ROS_INFO("Checking if the serial device is ready.");

    ros::Time begin_time = ros::Time::now();
    ros::Time write_time = ros::Time::now();
    ros::Duration general_timeout = ros::Duration(5.0);
    ros::Duration write_timeout = ros::Duration(1.0);

    writeSerial("?", "s", "dodobot");

    while (!readyState->is_ready)
    {
        if (!ros::ok()) {
            break;
        }
        if ((ros::Time::now() - begin_time) > general_timeout) {
            throw ReadyTimeoutException;
        }
        if ((ros::Time::now() - write_time) > write_timeout) {
            ROS_INFO("Writing signal again");
            writeSerial("?", "s", "dodobot");
            write_time = ros::Time::now();
        }
        if (_serialRef.available() > 2) {
            readSerial();
        }
    }

    if (readyState->is_ready) {
        setStartTime(readyState->time_ms);
        ROS_INFO_STREAM("Serial device is ready. Robot name is " << readyState->robot_name);
    }
    else {
        ROS_ERROR("Failed to receive ready signal!");
    }
}

bool DodobotPowerBoxParsing::waitForPacketStart()
{
    stringstream msg_buffer;
    char c1, c2;
    ros::Time wait_time = ros::Time::now();
    ros::Duration wait_timeout = ros::Duration(0.05);
    while (true) {
        if (ros::Time::now() - wait_time > wait_timeout) {
            return false;
        }
        if (_serialRef.available() < 2) {
            continue;
        }
        c1 = _serialRef.read(1).at(0);
        if (c1 == PACKET_START_0) {
            c2 = _serialRef.read(1).at(0);
            if (c2 == PACKET_START_1) {
                return true;
            }
        }

        else if (c1 == PACKET_STOP || c2 == PACKET_STOP) {
            ROS_INFO_STREAM("Device message: " << msg_buffer.str());
            msg_buffer.str(std::string());
            return false;
        }
        else {
            msg_buffer << c1;
        }
    }
}

uint16_t DodobotPowerBoxParsing::segment_as_uint16() {
    uint16_union u16_union;
    u16_union.byte[1] = _currentBufferSegment[0];
    u16_union.byte[0] = _currentBufferSegment[1];
    return u16_union.integer;
}

uint32_t DodobotPowerBoxParsing::segment_as_uint32() {
    uint32_union u32_union;
    for (unsigned short i = 0; i < 4; i++) {
        u32_union.byte[3 - i] = _currentBufferSegment[i];
    }
    return u32_union.integer;
}

int32_t DodobotPowerBoxParsing::segment_as_int32() {
    int32_union i32_union;
    for (unsigned short i = 0; i < 4; i++) {
        i32_union.byte[3 - i] = _currentBufferSegment[i];
    }
    return i32_union.integer;
}

float DodobotPowerBoxParsing::segment_as_float() {
    float_union f_union;
    for (unsigned short i = 0; i < 4; i++) {
        f_union.byte[i] = _currentBufferSegment[i];
    }
    return f_union.floating_point;
}

string DodobotPowerBoxParsing::segment_as_string() {
    return _currentBufferSegment;
}

bool DodobotPowerBoxParsing::readline()
{
    char c;
    _recvCharIndex = 0;
    char len_bytes[2];
    short len_bytes_found = 0;
    uint16_t packet_len = 0;
    size_t counter = 0;
    while (true) {
        if (!_serialRef.available()) {
            continue;
        }
        c = _serialRef.read(1).at(0);
        if (len_bytes_found < 2) {
            len_bytes[len_bytes_found++] = c;
            if (len_bytes_found == 2) {
                uint16_union u16_union;
                u16_union.byte[1] = len_bytes[0];
                u16_union.byte[0] = len_bytes[1];
                packet_len = u16_union.integer;
            }
            continue;
        }

        counter++;
        if (counter > packet_len) {
            if (c != PACKET_STOP) {
                _recvCharBuffer[_recvCharIndex] = '\0';
                ROS_ERROR("Packet didn't end with stop character: %c, %s", c, _recvCharBuffer);
                return false;
            }
            break;
        }

        _recvCharBuffer[_recvCharIndex] = c;
        _recvCharIndex++;
    }
    _recvCharBuffer[_recvCharIndex] = '\0';
    _readPacketLen = _recvCharIndex;
    // ROS_DEBUG_STREAM("_recvCharBuffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));
    return true;
}

bool DodobotPowerBoxParsing::readSerial()
{
    if (!waitForPacketStart()) {
        return false;
    }

    if (!readline()) {
        ROS_ERROR("Failed to read packet");
    }

    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    if (_readPacketLen < 5) {
        ROS_ERROR_STREAM("Received packet has an invalid number of characters! " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));
        _readPacketNum++;
        return false;
    }

    _serialBufferIndex = 0;
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (size_t index = 0; index < _readPacketLen - 2; index++) {
        calc_checksum += (uint8_t)_recvCharBuffer[index];
    }

    char recv_checksum_array[3];  // extra character for null
    recv_checksum_array[0] = _recvCharBuffer[_readPacketLen - 2];
    recv_checksum_array[1] = _recvCharBuffer[_readPacketLen - 1];
    // memcpy(recv_checksum_array, _recvCharBuffer + _readPacketLen - 2, 2);
    recv_checksum_array[2] = '\0';
    uint8_t recv_checksum = strtol(recv_checksum_array, NULL, 16);

    if (calc_checksum != recv_checksum) {
        // checksum failed
        ROS_ERROR("Checksum failed! recv %02x != calc %02x", recv_checksum, calc_checksum);
        ROS_ERROR_STREAM("Buffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));
        _readPacketNum++;
        return false;
    }

    // get packet num segment
    if (!getNextSegment(4)) {
        ROS_ERROR_STREAM("Failed to find packet number segment! " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));
        _readPacketNum++;
        return false;
    }
    uint32_t recv_packet_num = segment_as_uint32();
    if (_readPacketNum == -1) {
        _readPacketNum = recv_packet_num;
    }
    else if (recv_packet_num != _readPacketNum) {
        ROS_ERROR("Received packet num doesn't match local count. recv %d != local %d", recv_packet_num, _readPacketNum);
        ROS_ERROR_STREAM("Buffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));
        _readPacketNum = recv_packet_num;
    }

    // find category segment
    if (!getNextSegment()) {
        ROS_ERROR_STREAM("Failed to find category segment! Buffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));
        _readPacketNum++;
        return false;
    }

    string category = string(_currentBufferSegment);
    // ROS_INFO_STREAM("category: " << category << ", Buffer: " << formatPacketToPrint(_recvCharBuffer, _readPacketLen));

    try {
        processSerialPacket(category);
    }
    catch (exception& e) {
        ROS_ERROR_STREAM("Exception in processSerialPacket: " << e.what());
        return false;
    }

    _readPacketNum++;
    return true;
}

bool DodobotPowerBoxParsing::getNextSegment(int length)
{
    if (_serialBufferIndex >= _readPacketLen) {
        _currentSegmentNum = -1;
        return false;
    }
    uint16_union u16_union;
    if (length < 0) {
        // assume first 2 bytes contain the length
        u16_union.byte[1] = _recvCharBuffer[_serialBufferIndex++];
        u16_union.byte[0] = _recvCharBuffer[_serialBufferIndex++];
        length = (int)u16_union.integer;
    }
    memcpy(_currentBufferSegment, _recvCharBuffer + _serialBufferIndex, length);
    _currentBufferSegment[length] = '\0';
    _serialBufferIndex += length;
    return true;
}

bool DodobotPowerBoxParsing::getNextSegment()
{
    if (_serialBufferIndex >= _readPacketLen) {
        _currentSegmentNum = -1;
        return false;
    }
    int separator = -1;
    for (size_t i = _serialBufferIndex; i < _readPacketLen; i++) {
        if (_recvCharBuffer[i] == '\t') {
            separator = i;
            break;
        }
    }
    _currentSegmentNum++;
    if (separator < 0) {
        int length = _readPacketLen - _serialBufferIndex;
        memcpy(_currentBufferSegment, _recvCharBuffer + _serialBufferIndex, length);
        _currentBufferSegment[length] = '\0';
        _serialBufferIndex = length;
        return true;
    }
    else {
        int length = separator - _serialBufferIndex;
        memcpy(_currentBufferSegment, _recvCharBuffer + _serialBufferIndex, length);
        _currentBufferSegment[length] = '\0';
        _serialBufferIndex = separator + 1;
        return true;
    }
}

int DodobotPowerBoxParsing::getSegmentNum() {
    return _currentSegmentNum;
}

void DodobotPowerBoxParsing::processSerialPacket(string category)
{
    if (category.compare("txrx") == 0) {
        CHECK_SEGMENT(4); uint32_t packet_num = segment_as_uint32();
        CHECK_SEGMENT(4); int error_code = segment_as_uint32();

        if (wait_for_ok_reqs.find(packet_num) != wait_for_ok_reqs.end()) {
            wait_for_ok_reqs[packet_num] = error_code;
            ROS_INFO("txrx ok_req %u: %d", packet_num, error_code);
        }

        if (error_code != 0) {
            if (getNextSegment(4)) {
                logPacketErrorCode(error_code, packet_num, _currentBufferSegment);
            }
            else {
                logPacketErrorCode(error_code, packet_num);
            }
        }
    }
}

void DodobotPowerBoxParsing::writeSerialLarge(string name, vector<unsigned char>* data)
{
    vector<unsigned char> segment_buf;
    size_t data_len = data->size();

    uint16_union u16_union;
    size_t num_segments = 0;
    for (size_t segment_index = 0; segment_index < data_len; segment_index += large_packet_len) {
        num_segments++;
    }

    size_t count = 0;
    for (size_t segment_index = 0; segment_index < data_len; segment_index += large_packet_len)
    {
        uint16_t offset = std::min(large_packet_len, data_len - segment_index);
        ROS_INFO_STREAM("offset: " << offset);
        u16_union.integer = offset;
        segment_buf.assign(data->begin() + segment_index, data->begin() + segment_index + offset);
        segment_buf.insert(segment_buf.begin(), u16_union.byte[0]);
        segment_buf.insert(segment_buf.begin(), u16_union.byte[1]);
        auto *bytes = reinterpret_cast<char*>(segment_buf.data());

        writeSerial(name, "ddx", count, num_segments, bytes);
        count++;
        if (!waitForOK()) {
            ROS_WARN("Failed to receive ok signal on segment %lu of %lu. %s", count, num_segments, name.c_str());
            return;
        }
        ros::Duration(0.05).sleep();
    }
}


void DodobotPowerBoxParsing::writeSerial(string name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    // stringstream sstream;
    // sstream << PACKET_START_0 << PACKET_START_1 << _writePacketNum << "\t" << name;

    _writeCharBuffer[0] = PACKET_START_0;
    _writeCharBuffer[1] = PACKET_START_1;
    _writeCharIndex = 4;  // bytes 2 and 3 are for packet length

    int32_union i32_union;
    uint32_union u32_union;
    uint16_union u16_union;
    float_union f_union;

    u32_union.integer = _writePacketNum;
    for (unsigned short i = 0; i < 4; i++) {
        _writeCharBuffer[_writeCharIndex++] = u32_union.byte[3 - i];
    }
    sprintf(_writeCharBuffer + _writeCharIndex, "%s", name.c_str());
    _writeCharIndex += name.length();
    _writeCharBuffer[_writeCharIndex++] = '\t';

    while (*formats != '\0') {
        if (*formats == 'd') {
            i32_union.integer = va_arg(args, int32_t);
            for (unsigned short i = 0; i < 4; i++) {
                _writeCharBuffer[_writeCharIndex++] = i32_union.byte[3 - i];
            }
        }
        else if (*formats == 'u') {
            u32_union.integer = va_arg(args, uint32_t);
            for (unsigned short i = 0; i < 4; i++) {
                _writeCharBuffer[_writeCharIndex++] = u32_union.byte[3 - i];
            }
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            u16_union.integer = (uint16_t)strlen(s);
            _writeCharBuffer[_writeCharIndex++] = u16_union.byte[1];
            _writeCharBuffer[_writeCharIndex++] = u16_union.byte[0];
            sprintf(_writeCharBuffer + _writeCharIndex, "%s", s);
            _writeCharIndex += strlen(s);
        }
        else if (*formats == 'x') {
            char *s = va_arg(args, char*);
            u16_union.byte[1] = s[0];
            u16_union.byte[0] = s[1];
            for (size_t i = 0; i < 2 + u16_union.integer; i++) {
                _writeCharBuffer[_writeCharIndex++] = s[i];
            }
        }
        else if (*formats == 'f') {
            f_union.floating_point = (float)va_arg(args, double);
            for (unsigned short i = 0; i < 4; i++) {
                _writeCharBuffer[_writeCharIndex++] = f_union.byte[i];
            }
        }
        else {
            ROS_ERROR("Invalid format %c", *formats);
        }
        ++formats;
    }
    va_end(args);

    uint8_t calc_checksum = 0;
    for (size_t index = 4; index < _writeCharIndex; index++) {
        calc_checksum += (uint8_t)_writeCharBuffer[index];
    }

    sprintf(_writeCharBuffer + _writeCharIndex, "%02x", calc_checksum);
    _writeCharIndex += 2;
    _writeCharBuffer[_writeCharIndex++] = '\n';
    _writeCharBuffer[_writeCharIndex] = '\0';

    u16_union.integer = _writeCharIndex - 5;  // subtract start, length, and stop bytes

    _writeCharBuffer[2] = u16_union.byte[1];
    _writeCharBuffer[3] = u16_union.byte[0];

    _writePacketNum++;

    string packet = "";
    for (size_t i = 0; i < _writeCharIndex; i++) {
        packet += _writeCharBuffer[i];
    }

    ROS_DEBUG_STREAM("Queuing: " << formatPacketToPrint(_writeCharBuffer, _writeCharIndex) << "\tlength: " << packet.length());
    // _serialRef.write(packet);
    // ros::Duration(0.0005).sleep();
    write_queue.push(packet);

    if (_writePacketNum % 100 == 0) {
        ros::Duration dt = ros::Time::now() - write_timer;
        ROS_INFO("Write rate: %f", 100.0 / dt.toSec());
        write_timer = ros::Time::now();
    }
}

bool DodobotPowerBoxParsing::waitForOK(uint32_t packet_num, ros::Duration ok_timeout)
{
    if (ok_timeout == ros::Duration(0.0)) {
        ok_timeout = packet_ok_timeout;
    }
    wait_for_ok_reqs[packet_num] = -1;
    ros::Time start_timer = ros::Time::now();
    while (true)
    {
        if (!ros::ok()) {
            return false;
        }
        loop();
        if (ros::Time::now() - start_timer > ok_timeout) {
            ROS_WARN_STREAM("Timed out while waiting for response from packet #" << packet_num);
            return false;
        }
        if (wait_for_ok_reqs[packet_num] != -1) {
            int error_code = wait_for_ok_reqs[packet_num];
            ROS_INFO("Received response for packet #%u: %d", packet_num, error_code);
            wait_for_ok_reqs.erase(packet_num);
            return error_code == 0 || error_code == 6;
        }
    }
}

bool DodobotPowerBoxParsing::waitForOK(ros::Duration ok_timeout) {
    return waitForOK(_writePacketNum - 1, ok_timeout);
}


void DodobotPowerBoxParsing::setup()
{
    configure();

    // wait for startup messages from the microcontroller
    checkReady();
}

void DodobotPowerBoxParsing::write_packet_from_queue()
{
    if (write_queue.empty()) {
        return;
    }

    string packet = write_queue.front();
    write_queue.pop();
    ROS_DEBUG_STREAM("Writing: " << formatPacketToPrint(packet) << "\tlength: " << packet.length());
    _serialRef.write(packet);
    if (packet.size() >= 50) {
        ros::Duration(0.005).sleep();  // give microcontroller a chance to catch up to a large packet
    }
}

void DodobotPowerBoxParsing::write_thread_task()
{
    ros::Rate clock_rate(write_thread_rate);  // Hz
    while (!write_stop_flag)
    {
        write_packet_from_queue();
        clock_rate.sleep();
    }
    ROS_INFO("Dodobot write thread task finished");
}


void DodobotPowerBoxParsing::loop()
{
    // if the serial buffer has data, parse it
    if (_serialRef.available() > 2) {
        while (_serialRef.available()) {
            readSerial();
        }
    }

    ROS_INFO_THROTTLE(15, "Read packet num: %d", _readPacketNum);
}

void DodobotPowerBoxParsing::stop()
{
    setReadyFlag(false);
    if (active_on_start) {
        setActive(false);
    }

    write_stop_flag = true;
    while (!write_queue.empty()) {
        write_packet_from_queue();
    }

    // leave reporting for other modules
    // setReporting(false);
    _serialRef.close();
}


int DodobotPowerBoxParsing::run()
{
    setup();

    ros::Rate clock_rate(300);  // run loop at 300 Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    stop();

    write_thread->join();

    return exit_code;
}

void DodobotPowerBoxParsing::logPacketErrorCode(int error_code, uint32_t packet_num, string message) {
    logPacketErrorCode(error_code, packet_num);
    ROS_WARN_STREAM("txrx message: " << formatPacketToPrint(message));
}

void DodobotPowerBoxParsing::logPacketErrorCode(int error_code, uint32_t packet_num)
{
    ROS_WARN("Packet %d returned an error!", packet_num);
    switch (error_code) {
        case 1: ROS_WARN("c1 != \\x12: #%d", packet_num); break;
        case 2: ROS_WARN("c2 != \\x34: #%d", packet_num); break;
        case 3: ROS_WARN("packet is too short: #%d", packet_num); break;
        case 4: ROS_WARN("checksums don't match: #%d", packet_num); break;
        case 5: ROS_WARN("packet count segment not found: #%d", packet_num); break;
        case 6: ROS_WARN("packet counts not synchronized: #%d", packet_num); break;
        case 7: ROS_WARN("failed to find category segment: #%d", packet_num); break;
        case 8: ROS_WARN("invalid format: #%d", packet_num); break;
        case 9: ROS_WARN("packet didn't end with stop character: #%d", packet_num); break;
    }
}
string DodobotPowerBoxParsing::getPrintChar(unsigned char c)
{
    switch (c) {
        case 0: return "\\x00";
        case 1: return "\\x01";
        case 2: return "\\x02";
        case 3: return "\\x03";
        case 4: return "\\x04";
        case 5: return "\\x05";
        case 6: return "\\x06";
        case 7: return "\\x07";
        case 8: return "\\x08";
        case 9: return "\\t";
        case 10: return "\\n";
        case 11: return "\\x0b";
        case 12: return "\\x0c";
        case 13: return "\\r";
        case 14: return "\\x0e";
        case 15: return "\\x0f";
        case 16: return "\\x10";
        case 17: return "\\x11";
        case 18: return "\\x12";
        case 19: return "\\x13";
        case 20: return "\\x14";
        case 21: return "\\x15";
        case 22: return "\\x16";
        case 23: return "\\x17";
        case 24: return "\\x18";
        case 25: return "\\x19";
        case 26: return "\\x1a";
        case 27: return "\\x1b";
        case 28: return "\\x1c";
        case 29: return "\\x1d";
        case 30: return "\\x1e";
        case 31: return "\\x1f";
        case 92: return "\\\\";
        case 127: return "\\x7f";
        case 128: return "\\x80";
        case 129: return "\\x81";
        case 130: return "\\x82";
        case 131: return "\\x83";
        case 132: return "\\x84";
        case 133: return "\\x85";
        case 134: return "\\x86";
        case 135: return "\\x87";
        case 136: return "\\x88";
        case 137: return "\\x89";
        case 138: return "\\x8a";
        case 139: return "\\x8b";
        case 140: return "\\x8c";
        case 141: return "\\x8d";
        case 142: return "\\x8e";
        case 143: return "\\x8f";
        case 144: return "\\x90";
        case 145: return "\\x91";
        case 146: return "\\x92";
        case 147: return "\\x93";
        case 148: return "\\x94";
        case 149: return "\\x95";
        case 150: return "\\x96";
        case 151: return "\\x97";
        case 152: return "\\x98";
        case 153: return "\\x99";
        case 154: return "\\x9a";
        case 155: return "\\x9b";
        case 156: return "\\x9c";
        case 157: return "\\x9d";
        case 158: return "\\x9e";
        case 159: return "\\x9f";
        case 160: return "\\xa0";
        case 161: return "\\xa1";
        case 162: return "\\xa2";
        case 163: return "\\xa3";
        case 164: return "\\xa4";
        case 165: return "\\xa5";
        case 166: return "\\xa6";
        case 167: return "\\xa7";
        case 168: return "\\xa8";
        case 169: return "\\xa9";
        case 170: return "\\xaa";
        case 171: return "\\xab";
        case 172: return "\\xac";
        case 173: return "\\xad";
        case 174: return "\\xae";
        case 175: return "\\xaf";
        case 176: return "\\xb0";
        case 177: return "\\xb1";
        case 178: return "\\xb2";
        case 179: return "\\xb3";
        case 180: return "\\xb4";
        case 181: return "\\xb5";
        case 182: return "\\xb6";
        case 183: return "\\xb7";
        case 184: return "\\xb8";
        case 185: return "\\xb9";
        case 186: return "\\xba";
        case 187: return "\\xbb";
        case 188: return "\\xbc";
        case 189: return "\\xbd";
        case 190: return "\\xbe";
        case 191: return "\\xbf";
        case 192: return "\\xc0";
        case 193: return "\\xc1";
        case 194: return "\\xc2";
        case 195: return "\\xc3";
        case 196: return "\\xc4";
        case 197: return "\\xc5";
        case 198: return "\\xc6";
        case 199: return "\\xc7";
        case 200: return "\\xc8";
        case 201: return "\\xc9";
        case 202: return "\\xca";
        case 203: return "\\xcb";
        case 204: return "\\xcc";
        case 205: return "\\xcd";
        case 206: return "\\xce";
        case 207: return "\\xcf";
        case 208: return "\\xd0";
        case 209: return "\\xd1";
        case 210: return "\\xd2";
        case 211: return "\\xd3";
        case 212: return "\\xd4";
        case 213: return "\\xd5";
        case 214: return "\\xd6";
        case 215: return "\\xd7";
        case 216: return "\\xd8";
        case 217: return "\\xd9";
        case 218: return "\\xda";
        case 219: return "\\xdb";
        case 220: return "\\xdc";
        case 221: return "\\xdd";
        case 222: return "\\xde";
        case 223: return "\\xdf";
        case 224: return "\\xe0";
        case 225: return "\\xe1";
        case 226: return "\\xe2";
        case 227: return "\\xe3";
        case 228: return "\\xe4";
        case 229: return "\\xe5";
        case 230: return "\\xe6";
        case 231: return "\\xe7";
        case 232: return "\\xe8";
        case 233: return "\\xe9";
        case 234: return "\\xea";
        case 235: return "\\xeb";
        case 236: return "\\xec";
        case 237: return "\\xed";
        case 238: return "\\xee";
        case 239: return "\\xef";
        case 240: return "\\xf0";
        case 241: return "\\xf1";
        case 242: return "\\xf2";
        case 243: return "\\xf3";
        case 244: return "\\xf4";
        case 245: return "\\xf5";
        case 246: return "\\xf6";
        case 247: return "\\xf7";
        case 248: return "\\xf8";
        case 249: return "\\xf9";
        case 250: return "\\xfa";
        case 251: return "\\xfb";
        case 252: return "\\xfc";
        case 253: return "\\xfd";
        case 254: return "\\xfe";
        case 255: return "\\xff";
        default: return string(1, (char)c);
    }
}

string DodobotPowerBoxParsing::formatPacketToPrint(string packet)
{
    string str = "";
    for (size_t i = 0; i < packet.length(); i++) {
        str += getPrintChar((unsigned char)packet[i]);
    }
    return str;
}

string DodobotPowerBoxParsing::formatPacketToPrint(char* packet, uint32_t length)
{
    string str = "";
    for (size_t i = 0; i < length; i++) {
        str += getPrintChar(packet[i]);
    }
    return str;
}

void DodobotPowerBoxParsing::parseReady()
{
    CHECK_SEGMENT(4); readyState->time_ms = segment_as_uint32();
    CHECK_SEGMENT(-1); readyState->robot_name = segment_as_string();
    readyState->is_ready = true;
    ROS_INFO_STREAM("Received ready signal! Rover name: " << readyState->robot_name);

    state_msg.header.stamp = getDeviceTime(readyState->time_ms);
    state_msg.is_ready = readyState->is_ready;
    state_msg.robot_name = readyState->robot_name;

    state_pub.publish(state_msg);

    setReadyFlag(true);  // signal that ROS is ready
}
