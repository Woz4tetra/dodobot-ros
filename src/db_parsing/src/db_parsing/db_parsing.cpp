#include <db_parsing/db_parsing.h>


DodobotParsing::DodobotParsing(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    string drive_cmd_topic_name = "";

    nh.param<string>("serial_port", _serialPort, "");
    nh.param<int>("serial_baud", _serialBaud, 115200);
    nh.param<string>("drive_cmd_topic", drive_cmd_topic_name, "drive_cmd");

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

    _serialBuffer = "";
    _serialBufferIndex = 0;
    _currentBufferSegment = "";
    _currentSegmentNum = -1;
    _readPacketNum = -1;
    _writePacketNum = 0;
    _recvCharIndex = 0;
    _recvCharBuffer = new char[0xfff];

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

    pid_service = nh.advertiseService("dodobot_pid", &DodobotParsing::set_pid, this);

    write_timer = ros::Time::now();

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
        ROS_INFO_STREAM("Serial device is ready. Rover name is " << readyState->robot_name);
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

bool DodobotParsing::readSerial()
{
    if (!waitForPacketStart()) {
        return false;
    }
    char c;
    _recvCharIndex = 0;
    while (true) {
        if (_serialRef.available()) {
            c = _serialRef.read(1).at(0);
            if (c == PACKET_STOP) {
                break;
            }
            _recvCharBuffer[_recvCharIndex] = c;
            _recvCharIndex++;
        }
    }
    _recvCharBuffer[_recvCharIndex] = '\0';

    // _serialBuffer = _serialRef.readline();
    _serialBuffer = string(_recvCharBuffer);
    // _serialBuffer = _serialBuffer.substr(0, _serialBuffer.length() - 1);  // remove newline character
    // ROS_DEBUG_STREAM("Buffer: " << _serialBuffer);
    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    if (_serialBuffer.length() < 5) {
        ROS_ERROR_STREAM("Received packet has an invalid number of characters! " << _serialBuffer);
        _readPacketNum++;
        return false;
    }

    _serialBufferIndex = 0;
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (size_t index = 0; index < _serialBuffer.length() - 2; index++) {
        calc_checksum += (uint8_t)_serialBuffer.at(index);
    }

    uint16_t recv_checksum = 0;
    try {
        recv_checksum = std::stoul(_serialBuffer.substr(_serialBuffer.length() - 2), nullptr, 16);
    }
    catch (exception& e) {
        ROS_ERROR_STREAM("Failed to parse checksum. Buffer: " << _serialBuffer << ". Exception: " << e.what());
        return false;
    }

    if (calc_checksum != recv_checksum) {
        // checksum failed
        ROS_ERROR("Checksum failed! recv %d != calc %d", calc_checksum, recv_checksum);
        ROS_ERROR_STREAM("Buffer: " << _serialBuffer);
        _readPacketNum++;
        return false;
    }

    // get packet num segment
    if (!getNextSegment()) {
        ROS_ERROR_STREAM("Failed to find packet number segment! " << _serialBuffer);
        _readPacketNum++;
        return false;
    }
    unsigned long long  recv_packet_num = (unsigned long long )stoi(_currentBufferSegment);
    if (_readPacketNum == -1) {
        _readPacketNum = recv_packet_num;
    }
    else if (recv_packet_num != _readPacketNum) {
        ROS_ERROR("Received packet num doesn't match local count. recv %llu != local %llu", recv_packet_num, _readPacketNum);
        ROS_ERROR_STREAM("Buffer: " << _serialBuffer);
        _readPacketNum = recv_packet_num;
    }

    // find category segment
    if (!getNextSegment()) {
        ROS_ERROR_STREAM("Failed to find category segment! Buffer: " << _serialBuffer);
        _readPacketNum++;
        return false;
    }

    string category = _currentBufferSegment;
    // ROS_INFO_STREAM("category: " << category);

    // remove checksum
    _serialBuffer = _serialBuffer.substr(0, _serialBuffer.length() - 2);

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

bool DodobotParsing::getNextSegment()
{
    if (_serialBufferIndex >= _serialBuffer.length()) {
        _currentSegmentNum = -1;
        return false;
    }
    size_t separator = _serialBuffer.find('\t', _serialBufferIndex);
    _currentSegmentNum++;
    if (separator == std::string::npos) {
        _currentBufferSegment = _serialBuffer.substr(_serialBufferIndex);
        _serialBufferIndex = _serialBuffer.length();
        return true;
    }
    else {
        _currentBufferSegment = _serialBuffer.substr(_serialBufferIndex, separator - _serialBufferIndex);
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
        CHECK_SEGMENT; unsigned long long packet_num = (unsigned long long)stol(_currentBufferSegment);
        CHECK_SEGMENT; int error_code = stoi(_currentBufferSegment);
        // CHECK_SEGMENT; string message = _currentBufferSegment;

        if (error_code != 0) {
            logPacketErrorCode(error_code, packet_num);
        }
    }
    else if (category.compare("state") == 0)
    {
        CHECK_SEGMENT; robotState->time_ms = (uint32_t)stoi(_currentBufferSegment);
        CHECK_SEGMENT; robotState->battery_ok = (bool)stoi(_currentBufferSegment);
        CHECK_SEGMENT; robotState->motors_active = (bool)stoi(_currentBufferSegment);
        CHECK_SEGMENT; robotState->loop_rate = (double)stof(_currentBufferSegment);

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
        CHECK_SEGMENT; bool success = (bool)stoi(_currentBufferSegment);
        if (!success) {
            ROS_WARN("Failed to set PID constants. Waiting 1.0s and writing again");
            resendPidKsTimed();
        }
        else {
            ROS_INFO("PID constants set successfully!");
        }
    }
    else if (category.compare("ready") == 0) {
        CHECK_SEGMENT; readyState->time_ms = (uint32_t)stoi(_currentBufferSegment);
        CHECK_SEGMENT; readyState->robot_name = _currentBufferSegment;
        readyState->is_ready = true;
        ROS_INFO_STREAM("Received ready signal! Rover name: " << _currentBufferSegment);
    }
}


void DodobotParsing::writeSerial(string name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    string packet;
    stringstream sstream;
    sstream << PACKET_START_0 << PACKET_START_1 << _writePacketNum << "\t" << name;

    while (*formats != '\0') {
        sstream << "\t";
        if (*formats == 'd') {
            int i = va_arg(args, int32_t);
            sstream << i;
        }
        else if (*formats == 'u') {
            uint32_t u = va_arg(args, uint32_t);
            sstream << u;
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            sstream << s;
        }
        else if (*formats == 'f') {
            double f = va_arg(args, double);
            sstream << fixed << setprecision(4) << f;
        }
        else {
            ROS_ERROR("Invalid format %c", *formats);
        }
        ++formats;
    }
    va_end(args);

    packet = sstream.str();

    uint8_t calc_checksum = 0;
    for (size_t index = 2; index < packet.length(); index++) {
        calc_checksum += (uint8_t)packet.at(index);
    }
    ROS_DEBUG("calc_checksum: %d", calc_checksum);

    if (calc_checksum < 0x10) {
        sstream << "0";
    }
    // can't pass uint8_t to std::hex, has to be int
    sstream << std::hex << (int)calc_checksum;

    sstream << PACKET_STOP;

    packet = sstream.str();

    // checksum might be inserting null characters. Force the buffer to extend
    // to include packet stop and checksum

    _writePacketNum++;

    ROS_DEBUG_STREAM("Writing: " << packet);
    _serialRef.write(packet);
    ros::Duration(0.0005).sleep();

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
}


void DodobotParsing::loop()
{
    // if the serial buffer has data, parse it
    if (_serialRef.available() > 2) {
        while (_serialRef.available()) {
            readSerial();
        }
    }

    ROS_INFO_THROTTLE(15, "Read packet num: %llu", _readPacketNum);
}

void DodobotParsing::stop()
{
    setActive(false);

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

void DodobotParsing::logPacketErrorCode(int error_code, unsigned long long packet_num)
{
    ROS_WARN("Packet %llu returned an error!", packet_num);
    switch (error_code) {
        case 1: ROS_WARN("c1 != \\x12: #%llu", packet_num); break;
        case 2: ROS_WARN("c2 != \\x34: #%llu", packet_num); break;
        case 3: ROS_WARN("packet is too short: #%llu", packet_num); break;
        case 4: ROS_WARN("checksums don't match: #%llu", packet_num); break;
        case 5: ROS_WARN("packet count segment not found: #%llu", packet_num); break;
        case 6: ROS_WARN("packet counts not synchronized: #%llu", packet_num); break;
        case 7: ROS_WARN("failed to find category segment: #%llu", packet_num); break;
        case 8: ROS_WARN("invalid format: #%llu", packet_num); break;
    }
}


void DodobotParsing::parseDrive()
{
    CHECK_SEGMENT; drive_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; drive_msg.left_enc_pos = stol(_currentBufferSegment);
    CHECK_SEGMENT; drive_msg.right_enc_pos = stol(_currentBufferSegment);
    CHECK_SEGMENT; drive_msg.left_enc_speed = stof(_currentBufferSegment);
    CHECK_SEGMENT; drive_msg.right_enc_speed = stof(_currentBufferSegment);

    // double now = ros::Time::now().toSec();
    // double then = drive_msg.header.stamp.toSec();
    // ROS_INFO("current time: %f, sensor time: %f. diff: %f", now, then, now - then);

    drive_pub.publish(drive_msg);
}

void DodobotParsing::parseBumper()
{
    CHECK_SEGMENT; bumper_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; bumper_msg.left = stol(_currentBufferSegment);
    CHECK_SEGMENT; bumper_msg.right = stol(_currentBufferSegment);

    bumper_pub.publish(bumper_msg);
}

void DodobotParsing::parseFSR()
{
    CHECK_SEGMENT; fsr_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; fsr_msg.left = (uint16_t)stoi(_currentBufferSegment);
    CHECK_SEGMENT; fsr_msg.right = (uint16_t)stoi(_currentBufferSegment);

    fsr_pub.publish(fsr_msg);
}

void DodobotParsing::parseGripper()
{
    CHECK_SEGMENT; gripper_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; gripper_position = (int)stoi(_currentBufferSegment);

    gripper_msg.position = gripper_position;

    gripper_pub.publish(gripper_msg);
}

void DodobotParsing::parseLinear()
{
    CHECK_SEGMENT; linear_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; linear_msg.position = (int32_t)stoi(_currentBufferSegment);
    CHECK_SEGMENT; linear_msg.has_error = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT; linear_msg.is_homed = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT; linear_msg.is_active = (bool)stoi(_currentBufferSegment);

    linear_pub.publish(linear_msg);
}

void DodobotParsing::parseLinearEvent()
{
    CHECK_SEGMENT; linear_event_msg.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; linear_event_msg.event_num = (int)stoi(_currentBufferSegment);

    linear_event_pub.publish(linear_event_msg);
}

void DodobotParsing::parseBattery()
{
    CHECK_SEGMENT; battery_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; battery_msg.current = stof(_currentBufferSegment);
    CHECK_SEGMENT; // battery_msg doesn't have a slot for power
    CHECK_SEGMENT; battery_msg.voltage = stof(_currentBufferSegment);
    ROS_INFO_THROTTLE(3, "Voltage (V): %f, Current (mA): %f", battery_msg.voltage, battery_msg.current);

    battery_pub.publish(battery_msg);
}

void DodobotParsing::parseIR()
{
    // CHECK_SEGMENT;  // time ms
    // CHECK_SEGMENT;  // remote type
    // CHECK_SEGMENT;  // received value
}

void DodobotParsing::parseTilter()
{
    CHECK_SEGMENT; tilter_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT; tilter_msg.position = (int)stoi(_currentBufferSegment);

    tilter_pub.publish(tilter_msg);
}
