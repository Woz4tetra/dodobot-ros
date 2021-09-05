#include <dodobot_serial.h>

typedef union uint16_union
{
    uint16_t integer;
    unsigned char byte[2];
} uint16_union_t;

typedef union uint32_union
{
    uint32_t integer;
    unsigned char byte[4];
} uint32_union_t;

typedef union int32_union
{
    int32_t integer;
    unsigned char byte[4];
} int32_union_t;

typedef union float_union
{
    float floating_point;
    unsigned char byte[8];
} float_union_t;


DodobotSerial::DodobotSerial()
{
    init_variables();
}

void DodobotSerial::begin(string address, int baud)
{
    DODOBOT_SERIAL.setPort(address);
    DODOBOT_SERIAL.setBaudrate(baud);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    DODOBOT_SERIAL.setTimeout(timeout);
    DODOBOT_SERIAL.open();
}

void DodobotSerial::close() {
    DODOBOT_SERIAL.close();
}

void DodobotSerial::write(string name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    if (ready() && make_packet(name, formats, args))
    {
        DODOBOT_SERIAL.write((uint8_t*)write_char_buffer, write_char_index);
        ROS_DEBUG("Writing packet: %s", format_packet(write_char_buffer, write_char_index).c_str());
        write_packet_num++;
    }
    va_end(args);
}

void DodobotSerial::check_device_ready()
{
    ROS_INFO("Checking if the serial device is ready.");

    ros::Time begin_time = ros::Time::now();
    ros::Time write_time = ros::Time::now();
    ros::Duration general_timeout = ros::Duration(5.0);
    ros::Duration write_timeout = ros::Duration(1.0);

    write("?", "s", "dodobot");

    while (!ready_state->is_ready)
    {
        if (!ros::ok()) {
            break;
        }
        if ((ros::Time::now() - begin_time) > general_timeout) {
            throw ReadyTimeoutException;
        }
        if ((ros::Time::now() - write_time) > write_timeout) {
            ROS_INFO("Writing signal again");
            write("?", "s", "dodobot");
            write_time = ros::Time::now();
        }
        if (DODOBOT_SERIAL.available() > 2) {
            read();
        }
    }

    if (ready_state->is_ready) {
        set_start_time(ready_state->time_ms);
        ROS_INFO_STREAM("Serial device is ready. Robot name is " << ready_state->robot_name);
    }
    else {
        ROS_ERROR("Failed to receive ready signal!");
    }
}

void DodobotSerial::set_start_time(uint32_t device_time)
{
    start_time_local = ros::Time::now();
    start_time_device = device_time;
}

ros::Time DodobotSerial::get_device_time_as_local(uint32_t device_time) {
    return start_time_local + ros::Duration((double)(device_time - start_time_device) / 1000.0);
}

bool DodobotSerial::write_large(string name, string destination, vector<char>* data)
{
    vector<char> segment_buf;
    unsigned int data_len = data->size();

    uint16_union_t u16_union;
    unsigned int num_segments = (unsigned int)(data_len / MAX_SEGMENT_LEN) + 1;

    unsigned int count = 0;
    for (unsigned int segment_index = 0; segment_index < data_len; segment_index += MAX_SEGMENT_LEN)
    {
        uint16_t offset = std::min(MAX_SEGMENT_LEN, data_len - segment_index);
        ROS_DEBUG_STREAM("Large packet offset: " << offset);
        u16_union.integer = offset;
        segment_buf.assign(data->begin() + segment_index, data->begin() + segment_index + offset);
        segment_buf.insert(segment_buf.begin(), u16_union.byte[0]);
        segment_buf.insert(segment_buf.begin(), u16_union.byte[1]);
        auto *bytes = reinterpret_cast<char*>(segment_buf.data());

        write(name, "sddx", destination, count, num_segments, bytes);
        count++;
        if (!wait_for_ok()) {
            ROS_WARN("Failed to receive ok signal on segment %u of %u. %s", count, num_segments, name.c_str());
            return false;
        }
        ros::Duration(large_packet_segment_delay).sleep();
    }
    return true;
}

void DodobotSerial::write_txrx(uint32_t code)  {
    write("txrx", "du", read_packet_num, code);
}

bool DodobotSerial::wait_for_ok(uint32_t expected_packet_num)
{
    start_wait_time = ros::Time::now();
    recv_txrx_packet_num = 0;
    wait_for_ok_reqs[expected_packet_num] = -1;
    while (true)
    {
        read();
        if (wait_for_ok_reqs[expected_packet_num] != -1)
        {
            int error_code = wait_for_ok_reqs[expected_packet_num];
            ROS_INFO("Received response for packet #%u: %d", expected_packet_num, error_code);
            wait_for_ok_reqs.erase(expected_packet_num);
            break;
        }
        if (ros::Time::now() - start_wait_time > stop_timeout)
        {
            ROS_INFO("Timed out while waiting for ok from packet %d", expected_packet_num);
            return false;
        }
    }
    if (recv_txrx_code == 0 || recv_txrx_code == 6) {
        return true;        
    }
    else {
        ROS_INFO("Received error code %d for packet %d", recv_txrx_code, expected_packet_num);
        return false;
    }
}

bool DodobotSerial::wait_for_ok() {
    return wait_for_ok(write_packet_num - 1);
}


bool DodobotSerial::segment_large(string& destination, int32_t& segment_index, int32_t& num_segments, uint16_t& length, int32_t prev_segment_index, char** bytes)
{
    if (!segment_as_string(destination)) {
        ROS_INFO("Failed to parse segment destination!");
        return false;
    }
    if (!segment_as_int32(segment_index)) {
        ROS_INFO("Failed to parse segment_index!");
        return false;
    }
    if (!segment_as_int32(num_segments)) {
        ROS_INFO("Failed to parse num_segments!");
        return false;
    }
    if (!segment_as_uint16(length)) {
        ROS_INFO("Failed to parse segment length!");
        return false;
    }
    
    if (!segment_as_char_array(bytes, length)) {
        ROS_INFO("Failed to parse segment!");
        return false;
    }

    if (segment_index != 0 && segment_index - prev_segment_index != 1)
    {
        ROS_INFO("Received segments out of order!");
        return false;
    }
    return true;
}


bool DodobotSerial::read()
{
    if (!ready()) {
        return false;
    }
    // 2 start chars
    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    // 1 new line
    if (DODOBOT_SERIAL.available() <= 2) {
        return false;
    }
    char c;
    bool start_found = false;
    c = DODOBOT_SERIAL.read()[0];
    if (c == PACKET_START_0) {
        if (DODOBOT_SERIAL.available()) {
            c = DODOBOT_SERIAL.read()[0];
            if (c == PACKET_START_1) {
                start_found = true;
            }
        }
    }
    else if (c == PACKET_STOP) {
        ROS_INFO("Device message: '%s'", format_packet(recv_char_buffer).c_str());
        return false;
    }
    else {
        recv_char_buffer[recv_char_index] = c;
        recv_char_index++;
        if (recv_char_index >= MAX_PACKET_LEN) {
            recv_char_index = 0;
        }
        return false;
    }
    if (!start_found) {
        return false;
    }
    start_wait_time = ros::Time::now();
    recv_char_index = 0;

    bool len1_set = false;
    bool len2_set = false;
    uint8_t len1;
    uint8_t len2;
    uint16_t packet_len;
    size_t counter = 0;
    while (true)
    {
        if (ros::Time::now() - start_wait_time > stop_timeout) {
            write_txrx(11);  // error 11: packet receive timed out
            break;
        }
        if (!DODOBOT_SERIAL.available()) {
            continue;
        }

        c = DODOBOT_SERIAL.read()[0];
        if (!len1_set) {
            len1 = c;
            len1_set = true;
            continue;
        }
        else if (!len2_set) {
            len2 = c;
            len2_set = true;
            packet_len = (len1 << 8) + len2;
            continue;
        }

        counter++;
        if (counter > packet_len)
        {
            if (c != PACKET_STOP)
            {
                recv_char_buffer[recv_char_index] = '\0';
                write_txrx(9);  // error 9: packet didn't end properly
                return false;
            }
            break;
        }
        recv_char_buffer[recv_char_index] = c;
        recv_char_index++;
    }
    recv_char_buffer[recv_char_index] = '\0';
    read_packet_len = recv_char_index;

    return parse_packet();
}


void DodobotSerial::flush_read() {
    uint32_t available = DODOBOT_SERIAL.available();
    for (uint32_t i = 0; i < available; i++) {
        DODOBOT_SERIAL.read()[0];
    }
}

bool DodobotSerial::next_segment(int length)
{
    if (read_packet_index >= read_packet_len)
    {
        ROS_INFO("Ran out of characters in buffer!");
        current_segment_num = -1;
        return false;
    }
    if (length < 0) {
        // assume first 2 bytes contain the length
        uint16_union_t union_segment_len;
        union_segment_len.byte[1] = recv_char_buffer[read_packet_index++];
        union_segment_len.byte[0] = recv_char_buffer[read_packet_index++];
        length = (int)union_segment_len.integer;
    }
    if (length > MAX_SEGMENT_LEN)
    {
        ROS_INFO("Parsed segment length %d is longer than max segment length %d", length, MAX_SEGMENT_LEN);
        return false;
    }
    memcpy(segment, recv_char_buffer + read_packet_index, length);
    segment[length] = '\0';
    read_packet_index += length;
    
    return true;
}

bool DodobotSerial::next_segment()
{
    if (read_packet_index >= read_packet_len) {
        current_segment_num = -1;
        return false;
    }

    int separator = -1;
    for (size_t i = read_packet_index; i < read_packet_len; i++) {
        if (recv_char_buffer[i] == PACKET_SEGMENT_SEPARATOR) {
            separator = i;
            break;
        }
    }
    current_segment_num++;
    if (separator < 0) {
        int length = read_packet_len - read_packet_index;
        memcpy(segment, recv_char_buffer + read_packet_index, length);
        segment[length] = '\0';
        read_packet_index = read_packet_len;
        return true;
    }
    else {
        int length = separator - read_packet_index;
        memcpy(segment, recv_char_buffer + read_packet_index, length);
        segment[length] = '\0';
        read_packet_index = separator + 1;
        return true;
    }
}


bool DodobotSerial::segment_as_string(string& result)
{
    if (!next_segment(-1))  { return false; }
    result = string(segment);
    return true;
}
bool DodobotSerial::segment_as_char_array(char** result)
{
    if (!next_segment(-1))  { return false; }
    *result = segment;
    return true;
}
bool DodobotSerial::segment_as_char_array(char** result, int length)
{
    if (!next_segment(length))  { return false; }
    *result = segment;
    return true;
}

bool DodobotSerial::segment_as_uint16(uint16_t& result)
{
    if (!next_segment(2))  { return false; }
    uint16_union_t union_data;
    union_data.byte[1] = segment[0];
    union_data.byte[0] = segment[1];
    result = union_data.integer;
    return true;
}

bool DodobotSerial::segment_as_uint32(uint32_t& result)
{
    if (!next_segment(4))  { return false; }
    uint32_union_t union_data;
    for (unsigned short i = 0; i < 4; i++) {
        union_data.byte[3 - i] = segment[i];
    }
    result = union_data.integer;
    return true;
}

bool DodobotSerial::segment_as_int32(int32_t& result)
{
    if (!next_segment(4))  { return false; }
    int32_union_t union_data;
    for (unsigned short i = 0; i < 4; i++) {
        union_data.byte[3 - i] = segment[i];
    }
    result = union_data.integer;
    return true;
}

bool DodobotSerial::segment_as_float(float& result)
{
    if (!next_segment(4))  { return false; }
    float_union_t union_data;
    for (unsigned short i = 0; i < 4; i++) {
        union_data.byte[i] = segment[i];
    }
    result = union_data.floating_point;
    return true;
}

void DodobotSerial::init_variables()
{
    segment = new char[MAX_SEGMENT_LEN];

    read_packet_num = 0;
    write_packet_num = 0;
    buffer_index = 0;
    read_packet_index = 0;
    current_segment_num = -1;

    recv_char_buffer = new char[MAX_PACKET_LEN];
    recv_char_index = 0;

    write_char_buffer = new char[MAX_PACKET_LEN];
    write_char_index = 0;

    start_wait_time = ros::Time::now();

    current_category = "";

    ready_state = new StructReadyState_t;
    ready_state->time_ms = 0;
    ready_state->robot_name = "";
    ready_state->is_ready = false;
}

bool DodobotSerial::make_packet(string name, const char *formats, va_list args)
{
    write_char_buffer[0] = PACKET_START_0;
    write_char_buffer[1] = PACKET_START_1;
    write_char_index = 4;  // bytes 2 and 3 are for packet length

    uint32_union_t union_packet_num;
    union_packet_num.integer = write_packet_num;
    for (unsigned short i = 0; i < 4; i++) {
        write_char_buffer[write_char_index++] = union_packet_num.byte[3 - i];
    }
    sprintf(write_char_buffer + write_char_index, "%s", name.c_str());
    write_char_index += name.length();
    write_char_buffer[write_char_index++] = '\t';

    while (*formats != '\0')
    {
        if (*formats == 'd') {
            int32_union_t union_data;
            union_data.integer = va_arg(args, int32_t);
            for (unsigned short i = 0; i < 4; i++) {
                write_char_buffer[write_char_index++] = union_data.byte[3 - i];
            }
        }
        else if (*formats == 'u') {
            uint32_union_t union_data;
            union_data.integer = va_arg(args, uint32_t);
            for (unsigned short i = 0; i < 4; i++) {
                write_char_buffer[write_char_index++] = union_data.byte[3 - i];
            }
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            uint16_union_t union_str_len;
            union_str_len.integer = (uint16_t)strlen(s);
            write_char_buffer[write_char_index++] = union_str_len.byte[1];
            write_char_buffer[write_char_index++] = union_str_len.byte[0];
            sprintf(write_char_buffer + write_char_index, "%s", s);
            write_char_index += strlen(s);
        }
        else if (*formats == 'x') {
            char *s = va_arg(args, char*);
            uint16_union_t union_str_len;
            union_str_len.byte[1] = s[0];
            union_str_len.byte[0] = s[1];
            if (union_str_len.integer > MAX_SEGMENT_LEN) {
                write_txrx(10);  // error 10: segment is too long
                return false;
            }
            write_char_buffer[write_char_index++] = s[0];
            write_char_buffer[write_char_index++] = s[1];
            for (size_t i = 0; i < union_str_len.integer; i++) {
                write_char_buffer[write_char_index++] = s[2 + i];
            }
        }
        else if (*formats == 'f') {
            float_union_t union_data;
            union_data.floating_point = (float)va_arg(args, double);
            for (unsigned short i = 0; i < 4; i++) {
                write_char_buffer[write_char_index++] = union_data.byte[i];
            }
        }
        else {
            write_txrx(8);  // error 8: invalid format
            return false;
        }
        ++formats;
    }
    uint8_t calc_checksum = 0;
    for (size_t index = 4; index < write_char_index; index++) {
        calc_checksum += (uint8_t)write_char_buffer[index];
    }

    sprintf(write_char_buffer + write_char_index, "%02x", calc_checksum);
    write_char_index += 2;
    write_char_buffer[write_char_index++] = '\n';
    write_char_buffer[write_char_index] = '\0';

    uint16_t packet_len = write_char_index - 5;  // subtract start, length, and stop bytes

    // insert packet length
    uint16_union_t union_packet_len;
    union_packet_len.integer = packet_len;
    write_char_buffer[2] = union_packet_len.byte[1];
    write_char_buffer[3] = union_packet_len.byte[0];

    return true;
}

bool DodobotSerial::parse_packet()
{
    // read_packet assumes PACKET_START_0, PACKET_START_1, and PACKET_STOP have been removed

    read_packet_index = 0;
    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    if (read_packet_len < 5) {
        write_txrx(3);  // error 3: packet is too short
        read_packet_num++;
        return false;
    }

    // Calculate checksum
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (size_t index = 0; index < read_packet_len - 2; index++) {
        calc_checksum += (uint8_t)recv_char_buffer[index];
    }

    // extract checksum from packet
    char recv_checksum_array[2];
    memcpy(recv_checksum_array, recv_char_buffer + read_packet_len - 2, 2);
    uint8_t recv_checksum = strtol(recv_checksum_array, NULL, 16);

    if (calc_checksum != recv_checksum) {
        // checksum failed
        ROS_WARN("Found a packet with a mismatching checksum. 0x%02x != 0x%02x", calc_checksum, recv_checksum);
        write_txrx(4);  // error 4: checksums don't match
        read_packet_num++;
        return false;
    }

    uint32_t recv_packet_num;
    if (!segment_as_uint32(recv_packet_num)) {
        // failed to find packet num segment
        write_txrx(5);  // error 5: packet count segment not found
        read_packet_num++;
        return false;
    }

    if (recv_packet_num != read_packet_num) {
        // this is considered a warning since it isn't critical for packet
        // numbers to be in sync
        write_txrx(6);  // error 6: packet counts not synchronized
        read_packet_num = recv_packet_num;
    }

    // find category segment
    if (!next_segment()) {
        write_txrx(7);  // error 7: failed to find category segment
        read_packet_num++;
        return false;
    }
    string category = string(segment);

    bool did_receive_user_packet = false;
    if (category.compare("txrx") == 0)
    {
        if (segment_as_uint32(recv_txrx_packet_num) && segment_as_uint32(recv_txrx_code))
        {
            if (wait_for_ok_reqs.find(recv_txrx_packet_num) != wait_for_ok_reqs.end())
            {
                wait_for_ok_reqs[recv_txrx_packet_num] = recv_txrx_code;
                ROS_INFO("txrx ok_req %u: %d", recv_txrx_packet_num, recv_txrx_code);
            }
            log_packet_error_code(recv_txrx_code, recv_txrx_packet_num);
        }
    }
    else if (category.compare("ready") == 0)
    {
        segment_as_uint32(ready_state->time_ms);
        segment_as_string(ready_state->robot_name);
        ready_state->is_ready = true;
        ROS_INFO_STREAM("Received ready signal! Device name: " << ready_state->robot_name);
    }
    else
    {
        did_receive_user_packet = true;
        current_category = category;
        write_txrx(0);  // 0: no error
    }
    read_packet_num++;
    return did_receive_user_packet;
}

void DodobotSerial::log_packet_error_code(uint32_t code, uint32_t packet_num)
{
    if (code == 0) {
        return;
    }
    string error_message;
    switch (code) {
        case 1: error_message = "c1 != \\x12"; break;
        case 2: error_message = "c2 != \\x13"; break;
        case 3: error_message = "packet is too short"; break;
        case 4: error_message = "checksums don't match"; break;
        case 5: error_message = "packet count segment not found"; break;
        case 6: error_message = "packet counts not synchronized"; break;
        case 7: error_message = "failed to find category segment"; break;
        case 8: error_message = "invalid format"; break;
        case 9: error_message = "packet didn't end with stop character"; break;
        case 10: error_message = "packet segment is too long"; break;
        case 11: error_message = "packet didn't end with stop character"; break;
        default: error_message = "unknown error code!"; break;
    }

    ROS_WARN("Packet %d returned error code %u: %s", packet_num, code, error_message.c_str());
}

string format_char(unsigned char c)
{
    if (c == 92) return "\\\\";
    else if (c == 9) return "\\t";
    else if (c == 10) return "\\n";
    else if (c == 13) return "\\r";
    else if (c == 11 || c == 12 || c <= 9 || (14 <= c && c <= 31) || 127 <= c)
    {
        char* temp_buf = new char[4];
        sprintf(temp_buf, "\\x%02x", c);
        return string(temp_buf);
    }
    else {
        return string(1, (char)c);
    }
}


string format_packet(string packet)
{
    string str = "";
    for (size_t i = 0; i < packet.length(); i++) {
        str += format_char((unsigned char)packet[i]);
    }
    return str;
}

string format_packet(char *packet, uint32_t length)
{
    string str = "";
    for (size_t i = 0; i < length; i++) {
        str += format_char(packet[i]);
    }
    return str;
}
