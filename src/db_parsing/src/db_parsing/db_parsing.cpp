#include <db_parsing/db_parsing.h>


DodobotParsing::DodobotParsing(ros::NodeHandle* nodehandle):nh(*nodehandle),image_transport(nh)
{
    string drive_cmd_topic_name = "";
    int stepper_max_speed_param, stepper_max_accel_param;

    ros::param::param<string>("~serial_port", _serialPort, "");
    ros::param::param<int>("~serial_baud", _serialBaud, 115200);
    ros::param::param<string>("~drive_cmd_topic", drive_cmd_topic_name, "drive_cmd");
    ros::param::param<int>("~write_thread_rate", write_thread_rate, 60);
    ros::param::param<bool>("~use_sensor_msg_time", use_sensor_msg_time, true);
    ros::param::param<bool>("~active_on_start", active_on_start, true);
    ros::param::param<bool>("~reporting_on_start", reporting_on_start, true);
    ros::param::param<string>("~display_img_topic", display_img_topic, "image");
    ros::param::param<int>("~jpeg_image_quality", jpeg_image_quality, 50);
    ros::param::param<int>("~image_resize_width", image_resize_width, 160);
    ros::param::param<int>("~image_resize_height", image_resize_height, 128 - 20);
    ros::param::param<int>("~stepper_max_speed", stepper_max_speed_param, 420000000);
    ros::param::param<int>("~stepper_max_accel", stepper_max_accel_param, 20000000);
    stepper_max_speed = (uint32_t)stepper_max_speed_param;
    stepper_max_accel = (uint32_t)stepper_max_accel_param;
    stepper_low_speed = stepper_max_speed / 1000;
    stepper_low_accel = stepper_max_accel / 1000;

    ROS_INFO_STREAM("serial_port: " << _serialPort);
    ROS_INFO_STREAM("serial_baud: " << _serialBaud);
    ROS_INFO_STREAM("drive_cmd_topic: " << drive_cmd_topic_name);

    large_packet_len = 0x1000;
    ready_for_images = false;

    was_reporting = false;

    int num_servos = 0;

    drive_msg.header.frame_id = "drive";
    gripper_msg.header.frame_id = "gripper";
    tilter_msg.header.frame_id = "tilter";
    bumper_msg.header.frame_id = "bumper";
    linear_msg.header.frame_id = "linear";
    fsr_msg.header.frame_id = "fsrs";

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

    robotState = new StructRobotState;
    robotState->battery_ok = false;
    robotState->motors_active = false;
    robotState->loop_rate = 0.0;

    state_msg.battery_ok = true;
    state_msg.motors_active = false;
    state_msg.loop_rate = 0.0;
    state_msg.is_ready = false;
    state_msg.robot_name = "";

    deviceStartTime = ros::Time::now();
    offsetTimeMs = 0;

    pidConstants = new PidKs;
    pidConstants->kp_A = 0.0;
    pidConstants->ki_A = 0.0;
    pidConstants->kd_A = 0.0;
    pidConstants->kp_B = 0.0;
    pidConstants->ki_B = 0.0;
    pidConstants->kd_B = 0.0;
    pidConstants->speed_kA = 0.0;
    pidConstants->speed_kB = 0.0;

    prev_left_setpoint = 0.0;
    prev_right_setpoint = 0.0;

    display_img_buf = new vector<unsigned char>();

    prev_charge_state = false;

    gripper_pub = nh.advertise<db_parsing::DodobotGripper>("gripper", 50);
    tilter_pub = nh.advertise<db_parsing::DodobotTilter>("tilter", 50);
    linear_pub = nh.advertise<db_parsing::DodobotLinear>("linear", 50);
    linear_event_pub = nh.advertise<db_parsing::DodobotLinearEvent>("linear_events", 50);
    battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 50);
    drive_pub = nh.advertise<db_parsing::DodobotDrive>("drive", 50);
    bumper_pub = nh.advertise<db_parsing::DodobotBumper>("bumper", 50);
    fsr_pub = nh.advertise<db_parsing::DodobotFSRs>("fsrs", 50);
    state_pub = nh.advertise<db_parsing::DodobotState>("state", 50);
    robot_functions_pub = nh.advertise<db_parsing::DodobotFunctionsListing>("selected_fn", 10);

    gripper_sub = nh.subscribe<db_parsing::DodobotGripper>("gripper_cmd", 50, &DodobotParsing::gripperCallback, this);
    tilter_sub = nh.subscribe<db_parsing::DodobotTilter>("tilter_cmd", 50, &DodobotParsing::tilterCallback, this);
    linear_sub = nh.subscribe<db_parsing::DodobotLinear>("linear_cmd", 50, &DodobotParsing::linearCallback, this);
    drive_sub = nh.subscribe<db_parsing::DodobotDrive>(drive_cmd_topic_name, 50, &DodobotParsing::driveCallback, this);
    image_sub = image_transport.subscribe(display_img_topic, 1, &DodobotParsing::imgCallback, this);

    keyboard_sub = nh.subscribe<keyboard_listener::KeyEvent>("keys", 50, &DodobotParsing::keyboardCallback, this);
    robot_functions_sub = nh.subscribe<db_parsing::DodobotFunctionsListing>("functions", 50, &DodobotParsing::robotFunctionsCallback, this);
    notification_sub = nh.subscribe<db_parsing::DodobotNotify>("notify", 50, &DodobotParsing::notifyCallback, this);

    is_charging_sub = nh.subscribe<std_msgs::Bool>("is_charging", 10, &DodobotParsing::isChargingCallback, this);

    pid_service = nh.advertiseService("dodobot_pid", &DodobotParsing::set_pid, this);
    file_service = nh.advertiseService("dodobot_file", &DodobotParsing::upload_file, this);
    listdir_service = nh.advertiseService("dodobot_listdir", &DodobotParsing::db_listdir, this);
    set_state_service = nh.advertiseService("set_state", &DodobotParsing::set_state, this);
    get_state_service = nh.advertiseService("get_state", &DodobotParsing::get_state, this);

    write_stop_flag = false;
    write_thread = new boost::thread(boost::bind(&DodobotParsing::write_thread_task, this));

    write_timer = ros::Time::now();

    packet_ok_timeout = ros::Duration(1.0);
    linear_ok_packet_timeout = ros::Duration(7.0);

    jpeg_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    jpeg_params.push_back(jpeg_image_quality);

    ROS_INFO("Dodobot serial bridge init done");
}


void DodobotParsing::configure()
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

void DodobotParsing::setStartTime(uint32_t time_ms) {
    deviceStartTime = ros::Time::now();
    offsetTimeMs = time_ms;
}

ros::Time DodobotParsing::getDeviceTime(uint32_t time_ms) {
    return deviceStartTime + ros::Duration((double)(time_ms - offsetTimeMs) / 1000.0);
}

void DodobotParsing::checkReady()
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

bool DodobotParsing::waitForPacketStart()
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

uint16_t DodobotParsing::segment_as_uint16() {
    uint16_union u16_union;
    u16_union.byte[1] = _currentBufferSegment[0];
    u16_union.byte[0] = _currentBufferSegment[1];
    return u16_union.integer;
}

uint32_t DodobotParsing::segment_as_uint32() {
    uint32_union u32_union;
    for (unsigned short i = 0; i < 4; i++) {
        u32_union.byte[3 - i] = _currentBufferSegment[i];
    }
    return u32_union.integer;
}

int32_t DodobotParsing::segment_as_int32() {
    int32_union i32_union;
    for (unsigned short i = 0; i < 4; i++) {
        i32_union.byte[3 - i] = _currentBufferSegment[i];
    }
    return i32_union.integer;
}

float DodobotParsing::segment_as_float() {
    float_union f_union;
    for (unsigned short i = 0; i < 4; i++) {
        f_union.byte[i] = _currentBufferSegment[i];
    }
    return f_union.floating_point;
}

string DodobotParsing::segment_as_string() {
    return _currentBufferSegment;
}

bool DodobotParsing::readline()
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

bool DodobotParsing::readSerial()
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

bool DodobotParsing::getNextSegment(int length)
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

bool DodobotParsing::getNextSegment()
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

int DodobotParsing::getSegmentNum() {
    return _currentSegmentNum;
}

void DodobotParsing::processSerialPacket(string category)
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
    else if (category.compare("state") == 0) {
        parseState();
    }
    else if (category.compare("enc") == 0) {
        parseDrive();
    }
    else if (category.compare("bump") == 0) {
        parseBumper();
    }
    else if (category.compare("fsr") == 0) {
        parseFSR();
    }
    else if (category.compare("grip") == 0) {
        parseGripper();
    }
    // else if (category.compare("ir") == 0) {
    //     parseIR();
    // }
    else if (category.compare("linear") == 0) {
        parseLinear();
    }
    else if (category.compare("le") == 0) {
        parseLinearEvent();
    }
    else if (category.compare("batt") == 0) {
        parseBattery();
    }
    else if (category.compare("tilt") == 0) {
        parseTilter();
    }
    else if (category.compare("pidks") == 0) {
        CHECK_SEGMENT(4); bool success = (bool)segment_as_uint32();
        if (!success) {
            ROS_WARN("Failed to set PID constants. Waiting 1.0s and writing again");
            resendPidKsTimed();
        }
        else {
            ROS_INFO("PID constants set successfully!");
        }
    }
    else if (category.compare("ready") == 0) {
        parseReady();
    }
    else if (category.compare("listdir") == 0) {
        CHECK_SEGMENT(-1);  string filename = segment_as_string();
        CHECK_SEGMENT(4);  int32_t size = segment_as_int32();
        ROS_INFO_STREAM("filename: " << filename << ", size: " << size);
    }
    else if (category.compare("recvimage") == 0) {
        CHECK_SEGMENT(4); ready_for_images = (bool)segment_as_int32();
        ROS_INFO_STREAM("Receive images: " << ready_for_images);
    }
    else if (category.compare("robotfn") == 0) {
        parseSelectedRobotFn();
    }
}

void DodobotParsing::writeSerialLarge(string name, vector<unsigned char>* data)
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


void DodobotParsing::writeSerial(string name, const char *formats, ...)
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

bool DodobotParsing::waitForOK(uint32_t packet_num, ros::Duration ok_timeout)
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

bool DodobotParsing::waitForOK(ros::Duration ok_timeout) {
    return waitForOK(_writePacketNum - 1, ok_timeout);
}


void DodobotParsing::setup()
{
    configure();

    // wait for startup messages from the microcontroller
    checkReady();

    // tell the microcontroller to start
    if (reporting_on_start) {
        setReporting(true);
    }
    if (active_on_start) {
        setActive(true);
    }
    setReadyFlag(true);  // signal that ROS is ready

    // Send starter image
    // ros::Duration(0.25).sleep();
    // cv::Mat starter_image;
    // starter_image = cv::imread(starter_image_path, cv::IMREAD_COLOR);
    // writeImage(starter_image);
}

void DodobotParsing::write_packet_from_queue()
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

void DodobotParsing::write_thread_task()
{
    ros::Rate clock_rate(write_thread_rate);  // Hz
    while (!write_stop_flag)
    {
        write_packet_from_queue();
        clock_rate.sleep();
    }
    ROS_INFO("Dodobot write thread task finished");
}


void DodobotParsing::loop()
{
    // if the serial buffer has data, parse it
    if (_serialRef.available() > 2) {
        while (_serialRef.available()) {
            readSerial();
        }
    }

    ROS_INFO_THROTTLE(15, "Read packet num: %d", _readPacketNum);
}

void DodobotParsing::stop()
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


int DodobotParsing::run()
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

bool DodobotParsing::motorsReady() {
    return readyState->is_ready && robotState->motors_active;
}

bool DodobotParsing::robotReady() {
    return readyState->is_ready;
}

void DodobotParsing::driveCallback(const db_parsing::DodobotDrive::ConstPtr& msg)
{
    // motor commands in ticks per second
    ROS_DEBUG("left motor: %f, right motor: %f", msg->left_setpoint, msg->right_setpoint);
    // if (prev_left_setpoint != msg->left_setpoint || prev_right_setpoint != msg->right_setpoint) {
    writeDriveChassis(msg->left_setpoint, msg->right_setpoint);
    //     prev_left_setpoint = msg->left_setpoint;
    //     prev_right_setpoint = msg->right_setpoint;
    // }
}

void DodobotParsing::linearCallback(const db_parsing::DodobotLinear::ConstPtr& msg) {
    if (!motorsReady()) {
        ROS_WARN("Motors aren't ready! Skipping write linear");
        return;
    }

    if (msg->max_speed != -1)
    {
        if (0 < msg->max_speed && msg->max_speed <= stepper_max_speed) {
            if (msg->max_speed < stepper_low_speed) {
                ROS_WARN("Requested linear speed %d is very low (threshold: %d)", msg->max_speed, stepper_low_speed);
            }
            ROS_INFO("Setting linear max speed: %d", msg->max_speed);
            writeSerial("lincfg", "dd", 0, msg->max_speed);
            ros::Duration(0.005).sleep();
        }
        else {
            ROS_ERROR("Requested linear speed %d is out of bounds: %d", msg->max_speed, stepper_max_speed);
        }
    }
    if (msg->acceleration != -1) {
        if (0 < msg->acceleration && msg->acceleration <= stepper_max_accel) {
            if (msg->acceleration < stepper_low_accel) {
                ROS_WARN("Requested linear speed %d is very low (threshold: %d)", msg->acceleration, stepper_low_accel);
            }
            ROS_INFO("Setting linear max acceleration: %d", msg->acceleration);
            writeSerial("lincfg", "dd", 1, msg->acceleration);
            ros::Duration(0.005).sleep();
        }
        else {
            ROS_ERROR("Requested linear accel %d is out of bounds: %d", msg->acceleration, stepper_max_accel);
        }
    }

    switch (msg->command_type) {
        case 0:  // position: 0
        case 1:  // velocity: 1
            do {
                writeSerial("linear", "dd", msg->command_type, msg->command_value);
            }
            while (!waitForOK(ros::Duration(linear_ok_packet_timeout)));
            ROS_INFO("Linear command send successfully");
            break;
        case 2:  // stop linear:  2
        case 3:  // reset linear: 3
        case 4:  // home linear:  4
            do {
                writeSerial("linear", "d", msg->command_type);
            }
            while (!waitForOK(ros::Duration(linear_ok_packet_timeout)));
            ROS_INFO("Linear command send successfully");
            break;
        default:
            // no operation: -1 (for only writing lincfg)
            break;

    }
}

void DodobotParsing::tilterCallback(const db_parsing::DodobotTilter::ConstPtr& msg) {
    writeTilter(msg->command, msg->position);
}

void DodobotParsing::writeTilter(uint8_t command, int position) {
    if (!motorsReady()) {
        ROS_WARN("Motors aren't ready! Skipping writeTilter");
        return;
    }
    if (command <= 2) {
        // Up, Down, Toggle
        writeSerial("tilt", "d", command);
    }
    else {
        // Set with position
        writeSerial("tilt", "dd", command, position);
    }
}

void DodobotParsing::gripperCallback(const db_parsing::DodobotGripper::ConstPtr& msg) {
    writeGripper(msg->position, msg->force_threshold);
}

void DodobotParsing::writeGripper(int position, int force_threshold) {
    if (!motorsReady()) {
        ROS_WARN("Motors aren't ready! Skipping writeGripper");
        return;
    }

    if (position < gripper_position) {
        // Open gripper
        writeSerial("grip", "dd", 0, position);
    }
    else {
        // Close gripper
        writeSerial("grip", "ddd", 1, force_threshold, position);
    }
}

bool DodobotParsing::set_state(db_parsing::DodobotSetState::Request &req, db_parsing::DodobotSetState::Response &res)
{
    if (!robotReady()) {
        ROS_WARN("Robot isn't ready! Skipping set_state");
        res.resp = false;
        return false;
    }

    setReporting(req.reporting);
    setActive(req.active);

    res.resp = true;
    return true;
}

bool DodobotParsing::get_state(db_parsing::DodobotGetState::Request &req, db_parsing::DodobotGetState::Response &res)
{
    if (!robotReady())
    {
        res.ready = false;
        res.active = false;
        res.reporting = false;
    }
    else
    {
        res.ready = true;
        res.active = state_msg.motors_active;
        res.reporting = was_reporting;
    }
    return true;
}

bool DodobotParsing::set_pid(db_parsing::DodobotPidSrv::Request  &req,
         db_parsing::DodobotPidSrv::Response &res)
{
    if (!robotReady()) {
        ROS_WARN("Robot isn't ready! Skipping set_pid");
        return false;
    }
    pidConstants->kp_A = req.kp_A;
    pidConstants->ki_A = req.ki_A;
    pidConstants->kd_A = req.kd_A;
    pidConstants->kp_B = req.kp_B;
    pidConstants->ki_B = req.ki_B;
    pidConstants->kd_B = req.kd_B;
    pidConstants->speed_kA = req.speed_kA;
    pidConstants->speed_kB = req.speed_kB;
    writeK(pidConstants);

    res.resp = true;
    return true;
}

bool DodobotParsing::upload_file(db_parsing::DodobotUploadFile::Request &req, db_parsing::DodobotUploadFile::Response &res)
{
    if (!robotReady()) {
        ROS_WARN("Robot isn't ready! Skipping upload_file");
        return false;
    }

    ifstream local_file(req.path, ios::in | ios::binary | ios::ate);
    if (!local_file) {
        ROS_WARN("Local file '%s' failed to open", req.path.c_str());
        return false;
    }

    ifstream::pos_type size = local_file.tellg();
	ROS_INFO_STREAM("Size of file: " << size);
	local_file.seekg(0, ios::beg);

    char* temp = new char[1];

    std::vector<unsigned char>* file_buf = new std::vector<unsigned char>();

    for (size_t index = 0; index < size; index++) {
        local_file.read(temp, 1);
        file_buf->push_back(temp[0]);
    }

    ROS_INFO("Uploading file '%s' to '%s'", req.path.c_str(), req.dest.c_str());
    writeSerial("setpath", "s", req.dest.c_str());
    writeSerialLarge("file", file_buf);

    res.resp = true;
    return true;
}

bool DodobotParsing::db_listdir(db_parsing::DodobotListDir::Request &req, db_parsing::DodobotListDir::Response &res)
{
    if (!robotReady()) {
        ROS_WARN("Robot isn't ready! Skipping upload_file");
        return false;
    }
    writeSerial("listdir", "s", req.dirname.c_str());
    res.resp = true;
    return true;
}


void DodobotParsing::setActive(bool state)
{
    if (state) {
        writeSerial("<>", "d", 1);
    }
    else {
        writeSerial("<>", "d", 0);
    }
}

void DodobotParsing::softRestart() {
    writeSerial("<>", "d", 2);
}

void DodobotParsing::setReporting(bool state)
{
    was_reporting = state;
    if (state) {
        writeSerial("[]", "d", 1);
    }
    else {
        writeSerial("[]", "d", 0);
    }
}

void DodobotParsing::setReadyFlag(bool state)
{
    // signal that ROS is or is not ready
    if (state) {
        writeSerial("ros", "d", 1);
    }
    else {
        writeSerial("ros", "d", 0);
    }
}

void DodobotParsing::writeDriveChassis(float speedA, float speedB) {
    if (!motorsReady()) {
        ROS_WARN("Motors aren't ready! Skipping writeDriveChassis");
        return;
    }
    writeSerial("drive", "ff", speedA, speedB);
}

void DodobotParsing::writeK(PidKs* constants) {
    if (!robotReady()) {
        ROS_WARN("Robot isn't ready! Skipping writeK");
        return;
    }
    ROS_INFO("Setting pid: kp_A=%f, ki_A=%f, kd_A=%f, kp_B=%f, ki_B=%f, kd_B=%f, speed_kA=%f, speed_kB=%f",
        constants->kp_A,
        constants->ki_A,
        constants->kd_A,
        constants->kp_B,
        constants->ki_B,
        constants->kd_B,
        constants->speed_kA,
        constants->speed_kB
    );
    for (size_t attempts = 0; attempts < 5; attempts++) {
        writeSerial("ks", "ffffffff",
            constants->kp_A,
            constants->ki_A,
            constants->kd_A,
            constants->kp_B,
            constants->ki_B,
            constants->kd_B,
            constants->speed_kA,
            constants->speed_kB
        );
        if (waitForOK()) {
            break;
        }
        ROS_WARN("Failed to receive ok signal for PID. Trying again.");
    }
}

void DodobotParsing::robotFunctionsCallback(const db_parsing::DodobotFunctionsListing::ConstPtr& msg)
{
    size_t total_length = 0;
    for (size_t menu_index = 0; menu_index < msg->menu.size(); menu_index++) {
        for (size_t fn_index = 0; fn_index < msg->menu[menu_index].functions.size(); fn_index++) {
            total_length++;
        }
    }

    size_t index = 0;
    for (size_t menu_index = 0; menu_index < msg->menu.size(); menu_index++) {
        size_t fn_length = msg->menu[menu_index].functions.size();
        for (size_t fn_index = 0; fn_index < fn_length; fn_index++) {
            string name = msg->menu[menu_index].functions[fn_index];
            int blank_space = 0;
            if (fn_index == fn_length - 1) {
                blank_space = 6;
            }
            writeSerial("robotfn", "ddsd", index, total_length, name.c_str(), blank_space);
            index++;
        }
    }
}

void DodobotParsing::notifyCallback(const db_parsing::DodobotNotify::ConstPtr& msg) {
    sendNotification(msg->level, msg->message, msg->timeout);
}

void DodobotParsing::sendNotification(int level, string message, int timeout_ms) {
    writeSerial("notify", "dsu", level, message.c_str(), timeout_ms);
}


void DodobotParsing::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!ready_for_images) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    writeImage(cv_ptr->image);
}

void DodobotParsing::isChargingCallback(const std_msgs::BoolConstPtr& msg)
{
    writeIsCharging(msg->data);
}

void DodobotParsing::writeIsCharging(bool state)
{
    writeSerial("charge", "d", state);
    if (state && state != prev_charge_state) {
        sendNotification(0, "Charging", 3000);
    }
    prev_charge_state = state;
}


void DodobotParsing::writeImage(const cv::Mat& image)
{
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(image_resize_width, image_resize_height));

    cv::imencode(".jpg", resized, *display_img_buf, jpeg_params);

    size_t img_size = display_img_buf->size();
    if (img_size == 0) {
        ROS_WARN("Image is too small, len: %lu. Skipping image", img_size);
        return;
    }
    // if (img_size > SERIAL_BUFFER_SIZE) {
    //     ROS_WARN("Image is too large, len: %lu. Skipping image", img_size);
    //     return;
    // }

    ROS_INFO("sending image: %d", (int)img_size);
    writeSerial("setpath", "s", "CAMERA.JPG");
    writeSerialLarge("file", display_img_buf);
}

void DodobotParsing::resendPidKs() {
    writeK(pidConstants);
}

void DodobotParsing::resendPidKsTimed() {
    if (pid_resend_timer == NULL) {
        pid_resend_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&DodobotParsing::resendPidKs, this), true);  // oneshot timer
    }
    else {
        pid_resend_timer.stop();
        pid_resend_timer.setPeriod(ros::Duration(1.0));
        pid_resend_timer.start();
    }
}

void DodobotParsing::keyboardCallback(const keyboard_listener::KeyEvent::ConstPtr& msg)
{
    writeSerial("key", "sd", msg->data.c_str(), msg->event_type);
}

void DodobotParsing::logPacketErrorCode(int error_code, uint32_t packet_num, string message) {
    logPacketErrorCode(error_code, packet_num);
    ROS_WARN_STREAM("txrx message: " << formatPacketToPrint(message));
}

void DodobotParsing::logPacketErrorCode(int error_code, uint32_t packet_num)
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
string DodobotParsing::getPrintChar(unsigned char c)
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

string DodobotParsing::formatPacketToPrint(string packet)
{
    string str = "";
    for (size_t i = 0; i < packet.length(); i++) {
        str += getPrintChar((unsigned char)packet[i]);
    }
    return str;
}

string DodobotParsing::formatPacketToPrint(char* packet, uint32_t length)
{
    string str = "";
    for (size_t i = 0; i < length; i++) {
        str += getPrintChar(packet[i]);
    }
    return str;
}
void DodobotParsing::parseState()
{
    CHECK_SEGMENT(4); robotState->time_ms = (uint32_t)segment_as_uint32();
    CHECK_SEGMENT(4); robotState->battery_ok = (bool)segment_as_uint32();
    CHECK_SEGMENT(4); robotState->motors_active = (bool)segment_as_uint32();
    CHECK_SEGMENT(4); robotState->loop_rate = (double)segment_as_float();

    state_msg.header.stamp = getDeviceTime(robotState->time_ms);
    state_msg.battery_ok = robotState->battery_ok;
    state_msg.motors_active = robotState->motors_active;
    state_msg.loop_rate = robotState->loop_rate;

    state_pub.publish(state_msg);
}
void DodobotParsing::parseReady()
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

void DodobotParsing::parseDrive()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        drive_msg.header.stamp = getDeviceTime((uint32_t)segment_as_uint32());
    }
    else {
        drive_msg.header.stamp = ros::Time::now();
    }

    CHECK_SEGMENT(4); drive_msg.left_enc_pos = segment_as_int32();
    CHECK_SEGMENT(4); drive_msg.right_enc_pos = segment_as_int32();
    CHECK_SEGMENT(4); drive_msg.left_enc_speed = segment_as_float();
    CHECK_SEGMENT(4); drive_msg.right_enc_speed = segment_as_float();

    // double now = ros::Time::now().toSec();
    // double then = drive_msg.header.stamp.toSec();
    // ROS_INFO("current time: %f, sensor time: %f. diff: %f", now, then, now - then);

    drive_pub.publish(drive_msg);
}

void DodobotParsing::parseBumper()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        bumper_msg.header.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        bumper_msg.header.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); bumper_msg.left = segment_as_uint32();
    CHECK_SEGMENT(4); bumper_msg.right = segment_as_uint32();

    bumper_pub.publish(bumper_msg);
}

void DodobotParsing::parseFSR()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        fsr_msg.header.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        fsr_msg.header.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); fsr_msg.right = (uint16_t)segment_as_uint32();
    CHECK_SEGMENT(4); fsr_msg.left = (uint16_t)segment_as_uint32();

    fsr_pub.publish(fsr_msg);
}

void DodobotParsing::parseGripper()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        gripper_msg.header.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        gripper_msg.header.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); gripper_position = (int)segment_as_int32();

    gripper_msg.position = gripper_position;

    gripper_pub.publish(gripper_msg);
}

void DodobotParsing::parseLinear()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        linear_msg.header.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        linear_msg.header.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); linear_msg.position = segment_as_int32();
    CHECK_SEGMENT(4); linear_msg.has_error = segment_as_uint32();
    CHECK_SEGMENT(4); linear_msg.is_homed = segment_as_uint32();
    CHECK_SEGMENT(4); linear_msg.is_active = segment_as_uint32();

    linear_pub.publish(linear_msg);
}

void DodobotParsing::parseLinearEvent()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        linear_event_msg.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        linear_event_msg.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); linear_event_msg.event_num = segment_as_uint32();

    switch (linear_event_msg.event_num) {
        case 1:  ROS_INFO("Linear event: ACTIVE_TRUE"); break;
        case 2:  ROS_INFO("Linear event: ACTIVE_FALSE"); break;
        case 3:  ROS_INFO("Linear event: HOMING_STARTED"); break;
        case 4:  ROS_INFO("Linear event: HOMING_FINISHED"); break;
        case 5:  ROS_INFO("Linear event: MOVE_STARTED"); break;
        case 6:  ROS_INFO("Linear event: MOVE_FINISHED"); break;
        case 7:  ROS_WARN("Linear event: POSITION_ERROR"); break;
        case 8:  ROS_WARN("Linear event: NOT_HOMED"); break;
        case 9:  ROS_WARN("Linear event: NOT_ACTIVE"); break;
        default: break;
    }

    linear_event_pub.publish(linear_event_msg);
}

void DodobotParsing::parseBattery()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        battery_msg.header.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        battery_msg.header.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); battery_msg.current = segment_as_float();
    CHECK_SEGMENT(4); // battery_msg doesn't have a slot for power
    CHECK_SEGMENT(4); battery_msg.voltage = segment_as_float();
    ROS_INFO_THROTTLE(3, "Voltage (V): %f, Current (mA): %f", battery_msg.voltage, battery_msg.current);

    battery_pub.publish(battery_msg);
}

void DodobotParsing::parseIR()
{
    // CHECK_SEGMENT(4);  // time ms
    // CHECK_SEGMENT(4);  // remote type
    // CHECK_SEGMENT(4);  // received value
}

void DodobotParsing::parseTilter()
{
    CHECK_SEGMENT(4);
    if (use_sensor_msg_time) {
        tilter_msg.header.stamp = getDeviceTime(segment_as_uint32());
    }
    else {
        tilter_msg.header.stamp = ros::Time::now();
    }
    CHECK_SEGMENT(4); tilter_msg.position = segment_as_int32();

    tilter_pub.publish(tilter_msg);
}

void DodobotParsing::parseSelectedRobotFn()
{
    CHECK_SEGMENT(-1); selected_fn_msg.selected = segment_as_string();
    robot_functions_pub.publish(selected_fn_msg);
}
