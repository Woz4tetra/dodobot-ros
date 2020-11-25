#include <db_parsing/db_parsing.h>


DodobotParsing::DodobotParsing(ros::NodeHandle* nodehandle):nh(*nodehandle),image_transport(nh)
{
    string drive_cmd_topic_name = "";

    nh.param<string>("serial_port", _serialPort, "");
    nh.param<int>("serial_baud", _serialBaud, 115200);
    nh.param<string>("drive_cmd_topic", drive_cmd_topic_name, "drive_cmd");
    nh.param<int>("write_thread_rate", write_thread_rate, 60);
    nh.param<bool>("use_sensor_msg_time", use_sensor_msg_time, true);
    nh.param<string>("display_img_topic", display_img_topic, "image");
    nh.param<int>("jpeg_image_quality", jpeg_image_quality, 50);
    nh.param<int>("image_resize_width", image_resize_width, 160);
    nh.param<int>("image_resize_height", image_resize_height, 128 - 20);
    nh.param<string>("starter_image_path", starter_image_path, "./dodobot.jpeg");

    ROS_INFO_STREAM("serial_port: " << _serialPort);
    ROS_INFO_STREAM("serial_baud: " << _serialBaud);
    ROS_INFO_STREAM("drive_cmd_topic: " << drive_cmd_topic_name);

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

    gripper_pub = nh.advertise<db_parsing::DodobotGripper>("gripper", 50);
    tilter_pub = nh.advertise<db_parsing::DodobotTilter>("tilter", 50);
    linear_pub = nh.advertise<db_parsing::DodobotLinear>("linear", 50);
    linear_event_pub = nh.advertise<db_parsing::DodobotLinearEvent>("linear_events", 50);
    battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 50);
    drive_pub = nh.advertise<db_parsing::DodobotDrive>("drive", 50);
    bumper_pub = nh.advertise<db_parsing::DodobotBumper>("bumper", 50);
    fsr_pub = nh.advertise<db_parsing::DodobotFSRs>("fsrs", 50);

    gripper_sub = nh.subscribe<db_parsing::DodobotGripper>("gripper_cmd", 50, &DodobotParsing::gripperCallback, this);
    tilter_sub = nh.subscribe<db_parsing::DodobotTilter>("tilter_cmd", 50, &DodobotParsing::tilterCallback, this);
    linear_sub = nh.subscribe<db_parsing::DodobotLinear>("linear_cmd", 50, &DodobotParsing::linearCallback, this);
    drive_sub = nh.subscribe<db_parsing::DodobotDrive>(drive_cmd_topic_name, 50, &DodobotParsing::driveCallback, this);
    image_sub = image_transport.subscribe(display_img_topic, 1, &DodobotParsing::imgCallback, this);

    pid_service = nh.advertiseService("dodobot_pid", &DodobotParsing::set_pid, this);

    write_stop_flag = false;
    write_thread = new boost::thread(boost::bind(&DodobotParsing::write_thread_task, this));

    write_timer = ros::Time::now();

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
    // ROS_DEBUG_STREAM("category: " << category);

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

        if (error_code != 0) {
            if (getNextSegment(4)) {
                logPacketErrorCode(error_code, packet_num, _currentBufferSegment);
            }
            else {
                logPacketErrorCode(error_code, packet_num);
            }
        }
    }
    else if (category.compare("state") == 0)
    {
        CHECK_SEGMENT(4); robotState->time_ms = (uint32_t)segment_as_uint32();
        CHECK_SEGMENT(4); robotState->battery_ok = (bool)segment_as_uint32();
        CHECK_SEGMENT(4); robotState->motors_active = (bool)segment_as_uint32();
        CHECK_SEGMENT(4); robotState->loop_rate = (double)segment_as_float();

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
        CHECK_SEGMENT(4); readyState->time_ms = segment_as_uint32();
        CHECK_SEGMENT(-1); readyState->robot_name = string(_currentBufferSegment);
        readyState->is_ready = true;
        ROS_INFO_STREAM("Received ready signal! Rover name: " << readyState->robot_name);
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

void DodobotParsing::setup()
{
    configure();

    // wait for startup messages from the microcontroller
    checkReady();

    // tell the microcontroller to start
    setActive(true);
    setReporting(true);

    // Send starter image
    ros::Duration(0.25).sleep();
    cv::Mat starter_image;
    starter_image = cv::imread(starter_image_path, cv::IMREAD_COLOR);
    writeImage(starter_image);
}

void DodobotParsing::write_packet_from_queue()
{
    if (write_queue.empty()) {
        return;
    }

    string packet = write_queue.front();
    write_queue.pop();
    _serialRef.write(packet);
    // ros::Duration(0.0005).sleep();
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
    write_stop_flag = true;
    setActive(false);

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

    if (msg->max_speed != -1) {
        writeSerial("lincfg", "dd", 0, msg->max_speed);
        ros::Duration(0.005).sleep();
    }
    if (msg->acceleration != -1) {
        writeSerial("lincfg", "dd", 1, msg->acceleration);
        ros::Duration(0.005).sleep();
    }

    if (msg->command_value == 2 || msg->command_value == 3 || msg->command_value == 4) {
        // stop linear:  2
        // reset linear: 3
        // home linear:  4
        writeSerial("linear", "d", msg->command_type);
    }
    else {
        // position: 0
        // velocity: 1
        writeSerial("linear", "dd", msg->command_type, msg->command_value);
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
        writeSerial("tilter", "d", command);
    }
    else {
        // Set with position
        writeSerial("tilter", "dd", command, position);
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
    if (state) {
        writeSerial("[]", "d", 1);
    }
    else {
        writeSerial("[]", "d", 0);
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
    _serialRef.write("\n");
}

void DodobotParsing::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
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
void DodobotParsing::writeImage(const cv::Mat& image)
{
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(image_resize_width, image_resize_height));

    cv::imencode(".jpg", resized, display_img_buf, jpeg_params);

    size_t img_size = display_img_buf.size();
    if (img_size == 0) {
        ROS_WARN("Image is too small, len: %lu. Skipping image", img_size);
        return;
    }
    if (img_size > SERIAL_BUFFER_SIZE) {
        ROS_WARN("Image is too large, len: %lu. Skipping image", img_size);
        return;
    }
    uint16_union u16_union;
    u16_union.integer = img_size;
    display_img_buf.insert(display_img_buf.begin(), u16_union.byte[0]);
    display_img_buf.insert(display_img_buf.begin(), u16_union.byte[1]);
    auto *enc_msg = reinterpret_cast<char*>(display_img_buf.data());

    ROS_INFO("sending image: %d", (int)img_size);
    writeSerial("img", "x", enc_msg);
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

void DodobotParsing::logPacketErrorCode(int error_code, uint32_t packet_num, string message) {
    logPacketErrorCode(error_code, packet_num);
    ROS_WARN_STREAM("txrx message:" << message);
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

string DodobotParsing::formatPacketToPrint(char* packet, uint32_t length)
{
    string str = "";
    for (size_t i = 0; i < length; i++)
    {
        switch (packet[i]) {
            case 0: str += "\\x00"; break;
            case 1: str += "\\x01"; break;
            case 2: str += "\\x02"; break;
            case 3: str += "\\x03"; break;
            case 4: str += "\\x04"; break;
            case 5: str += "\\x05"; break;
            case 6: str += "\\x06"; break;
            case 7: str += "\\x07"; break;
            case 8: str += "\\x08"; break;
            case 9: str += "\\t"; break;
            case 10: str += "\\n"; break;
            case 11: str += "\\x0b"; break;
            case 12: str += "\\x0c"; break;
            case 13: str += "\\r"; break;
            case 14: str += "\\x0e"; break;
            case 15: str += "\\x0f"; break;
            case 16: str += "\\x10"; break;
            case 17: str += "\\x11"; break;
            case 18: str += "\\x12"; break;
            case 19: str += "\\x13"; break;
            case 20: str += "\\x14"; break;
            case 21: str += "\\x15"; break;
            case 22: str += "\\x16"; break;
            case 23: str += "\\x17"; break;
            case 24: str += "\\x18"; break;
            case 25: str += "\\x19"; break;
            case 26: str += "\\x1a"; break;
            case 27: str += "\\x1b"; break;
            case 28: str += "\\x1c"; break;
            case 29: str += "\\x1d"; break;
            case 30: str += "\\x1e"; break;
            case 31: str += "\\x1f"; break;
            case 92: str += "\\\\"; break;
            case 127: str += "\\x7f"; break;
            case 128: str += "\\x80"; break;
            case 129: str += "\\x81"; break;
            case 130: str += "\\x82"; break;
            case 131: str += "\\x83"; break;
            case 132: str += "\\x84"; break;
            case 133: str += "\\x85"; break;
            case 134: str += "\\x86"; break;
            case 135: str += "\\x87"; break;
            case 136: str += "\\x88"; break;
            case 137: str += "\\x89"; break;
            case 138: str += "\\x8a"; break;
            case 139: str += "\\x8b"; break;
            case 140: str += "\\x8c"; break;
            case 141: str += "\\x8d"; break;
            case 142: str += "\\x8e"; break;
            case 143: str += "\\x8f"; break;
            case 144: str += "\\x90"; break;
            case 145: str += "\\x91"; break;
            case 146: str += "\\x92"; break;
            case 147: str += "\\x93"; break;
            case 148: str += "\\x94"; break;
            case 149: str += "\\x95"; break;
            case 150: str += "\\x96"; break;
            case 151: str += "\\x97"; break;
            case 152: str += "\\x98"; break;
            case 153: str += "\\x99"; break;
            case 154: str += "\\x9a"; break;
            case 155: str += "\\x9b"; break;
            case 156: str += "\\x9c"; break;
            case 157: str += "\\x9d"; break;
            case 158: str += "\\x9e"; break;
            case 159: str += "\\x9f"; break;
            case 160: str += "\\xa0"; break;
            case 161: str += "\\xa1"; break;
            case 162: str += "\\xa2"; break;
            case 163: str += "\\xa3"; break;
            case 164: str += "\\xa4"; break;
            case 165: str += "\\xa5"; break;
            case 166: str += "\\xa6"; break;
            case 167: str += "\\xa7"; break;
            case 168: str += "\\xa8"; break;
            case 169: str += "\\xa9"; break;
            case 170: str += "\\xaa"; break;
            case 171: str += "\\xab"; break;
            case 172: str += "\\xac"; break;
            case 173: str += "\\xad"; break;
            case 174: str += "\\xae"; break;
            case 175: str += "\\xaf"; break;
            case 176: str += "\\xb0"; break;
            case 177: str += "\\xb1"; break;
            case 178: str += "\\xb2"; break;
            case 179: str += "\\xb3"; break;
            case 180: str += "\\xb4"; break;
            case 181: str += "\\xb5"; break;
            case 182: str += "\\xb6"; break;
            case 183: str += "\\xb7"; break;
            case 184: str += "\\xb8"; break;
            case 185: str += "\\xb9"; break;
            case 186: str += "\\xba"; break;
            case 187: str += "\\xbb"; break;
            case 188: str += "\\xbc"; break;
            case 189: str += "\\xbd"; break;
            case 190: str += "\\xbe"; break;
            case 191: str += "\\xbf"; break;
            case 192: str += "\\xc0"; break;
            case 193: str += "\\xc1"; break;
            case 194: str += "\\xc2"; break;
            case 195: str += "\\xc3"; break;
            case 196: str += "\\xc4"; break;
            case 197: str += "\\xc5"; break;
            case 198: str += "\\xc6"; break;
            case 199: str += "\\xc7"; break;
            case 200: str += "\\xc8"; break;
            case 201: str += "\\xc9"; break;
            case 202: str += "\\xca"; break;
            case 203: str += "\\xcb"; break;
            case 204: str += "\\xcc"; break;
            case 205: str += "\\xcd"; break;
            case 206: str += "\\xce"; break;
            case 207: str += "\\xcf"; break;
            case 208: str += "\\xd0"; break;
            case 209: str += "\\xd1"; break;
            case 210: str += "\\xd2"; break;
            case 211: str += "\\xd3"; break;
            case 212: str += "\\xd4"; break;
            case 213: str += "\\xd5"; break;
            case 214: str += "\\xd6"; break;
            case 215: str += "\\xd7"; break;
            case 216: str += "\\xd8"; break;
            case 217: str += "\\xd9"; break;
            case 218: str += "\\xda"; break;
            case 219: str += "\\xdb"; break;
            case 220: str += "\\xdc"; break;
            case 221: str += "\\xdd"; break;
            case 222: str += "\\xde"; break;
            case 223: str += "\\xdf"; break;
            case 224: str += "\\xe0"; break;
            case 225: str += "\\xe1"; break;
            case 226: str += "\\xe2"; break;
            case 227: str += "\\xe3"; break;
            case 228: str += "\\xe4"; break;
            case 229: str += "\\xe5"; break;
            case 230: str += "\\xe6"; break;
            case 231: str += "\\xe7"; break;
            case 232: str += "\\xe8"; break;
            case 233: str += "\\xe9"; break;
            case 234: str += "\\xea"; break;
            case 235: str += "\\xeb"; break;
            case 236: str += "\\xec"; break;
            case 237: str += "\\xed"; break;
            case 238: str += "\\xee"; break;
            case 239: str += "\\xef"; break;
            case 240: str += "\\xf0"; break;
            case 241: str += "\\xf1"; break;
            case 242: str += "\\xf2"; break;
            case 243: str += "\\xf3"; break;
            case 244: str += "\\xf4"; break;
            case 245: str += "\\xf5"; break;
            case 246: str += "\\xf6"; break;
            case 247: str += "\\xf7"; break;
            case 248: str += "\\xf8"; break;
            case 249: str += "\\xf9"; break;
            case 250: str += "\\xfa"; break;
            case 251: str += "\\xfb"; break;
            case 252: str += "\\xfc"; break;
            case 253: str += "\\xfd"; break;
            case 254: str += "\\xfe"; break;
            case 255: str += "\\xff"; break;
            default: str += packet[i];
        }
    }
    return str;
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
    CHECK_SEGMENT(4); fsr_msg.left = (uint16_t)segment_as_uint32();
    CHECK_SEGMENT(4); fsr_msg.right = (uint16_t)segment_as_uint32();

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
