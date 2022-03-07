#include <dodobot_serial/dodobot_serial.h>

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


DodobotSerial::DodobotSerial(void (*read_callback)(DodobotSerial*, String))
{
    this->read_callback = read_callback;
    init_variables();
}

void DodobotSerial::write(String name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    if (ready() && make_packet(name, formats, args))
    {
        DODOBOT_SERIAL.write(write_char_buffer, write_char_index);
        write_packet_num++;
    }
    va_end(args);
}

bool DodobotSerial::write_large(String name, String destination, uint32_t segment_num, uint32_t num_segments, char* buffer)
{
    // assumed that the first two bytes of buffer contain the length of buffer
    write(name, "sddx", destination.c_str(), segment_num, num_segments, buffer);
    return wait_for_ok();
}

void DodobotSerial::write_txrx(uint32_t code)  {
    write("txrx", "du", read_packet_num, code);
}

bool DodobotSerial::wait_for_ok()
{
    uint32_t start_wait_time = millis();
    recv_txrx_packet_num = 0;
    uint32_t expected_packet_num = write_packet_num - 1;
    while (true)
    {
        read();
        if (recv_txrx_packet_num == expected_packet_num) {
            break;
        }
        if (millis() - start_wait_time > PACKET_STOP_TIMEOUT)
        {
            DODOBOT_SERIAL.print("Timed out while waiting for ok from packet ");
            DODOBOT_SERIAL.println(expected_packet_num);
            return false;
        }
    }
    if (recv_txrx_code != 0)
    {
        DODOBOT_SERIAL.print("Received error code ");
        DODOBOT_SERIAL.println(recv_txrx_code);
        DODOBOT_SERIAL.print(" for packet ");
        DODOBOT_SERIAL.println(expected_packet_num);
        return false;
    }
    return true;
}


bool DodobotSerial::segment_large(String& destination, int32_t& segment_index, int32_t& num_segments, uint16_t& length, int32_t prev_segment_index, char** bytes)
{
    if (!segment_as_string(destination)) {
        DODOBOT_SERIAL.println("Failed to parse segment destination!");
        return false;
    }
    if (!segment_as_int32(segment_index)) {
        DODOBOT_SERIAL.println("Failed to parse segment_index!");
        return false;
    }
    if (!segment_as_int32(num_segments)) {
        DODOBOT_SERIAL.println("Failed to parse num_segments!");
        return false;
    }
    if (!segment_as_uint16(length)) {
        DODOBOT_SERIAL.println("Failed to parse segment length!");
        return false;
    }
    
    if (!segment_as_char_array(bytes, length)) {
        DODOBOT_SERIAL.println("Failed to parse segment!");
        return false;
    }

    if (segment_index != 0 && segment_index - prev_segment_index != 1)
    {
        DODOBOT_SERIAL.println("Received segments out of order!");
        return false;
    }
    return true;
}


void DodobotSerial::read()
{
    if (!ready()) {
        return;
    }
    // 2 start chars
    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    // 1 new line
    if (DODOBOT_SERIAL.available() <= 2) {
        return;
    }
    char c;
    bool start_found = false;
    c = DODOBOT_SERIAL.read();
    if (c == PACKET_START_0) {
        if (DODOBOT_SERIAL.available()) {
            c = DODOBOT_SERIAL.read();
            if (c == PACKET_START_1) {
                start_found = true;
            }
        }
    }
    if (!start_found) {
        return;
    }
    start_wait_time = millis();
    recv_char_index = 0;

    bool len1_set = false;
    bool len2_set = false;
    uint8_t len1;
    uint8_t len2;
    uint16_t packet_len;
    size_t counter = 0;
    while (true)
    {
        if (millis() - start_wait_time > PACKET_STOP_TIMEOUT) {
            write_txrx(11);  // error 11: packet receive timed out
            break;
        }
        if (!DODOBOT_SERIAL.available()) {
            continue;
        }

        c = DODOBOT_SERIAL.read();
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
                return;
            }
            break;
        }
        recv_char_buffer[recv_char_index] = c;
        recv_char_index++;
    }
    recv_char_buffer[recv_char_index] = '\0';
    read_packet_len = recv_char_index;

    parse_packet();
}


void DodobotSerial::flush_read() {
    uint32_t available = DODOBOT_SERIAL.available();
    for (uint32_t i = 0; i < available; i++) {
        DODOBOT_SERIAL.read();
    }
}

bool DodobotSerial::next_segment(int length)
{
    if (read_packet_index >= read_packet_len)
    {
        DODOBOT_SERIAL.println("Ran out of characters in buffer!");
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
        DODOBOT_SERIAL.print("Parsed segment length ");
        DODOBOT_SERIAL.print(length);
        DODOBOT_SERIAL.print(" is longer than max segment length ");
        DODOBOT_SERIAL.println(MAX_SEGMENT_LEN);
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


bool DodobotSerial::segment_as_string(String& result)
{
    if (!next_segment(-1))  { return false; }
    result = String(segment);
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
int DodobotSerial::get_segment_num() {
    return current_segment_num;
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

    start_wait_time = 0;
}

bool DodobotSerial::make_packet(String name, const char *formats, va_list args)
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

void DodobotSerial::parse_packet()
{
    // read_packet assumes PACKET_START_0, PACKET_START_1, and PACKET_STOP have been removed

    read_packet_index = 0;
    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    if (read_packet_len < 5) {
        write_txrx(3);  // error 3: packet is too short
        read_packet_num++;
        return;
    }

    // Calculate checksum
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (size_t index = 0; index < read_packet_len - 2; index++) {
        calc_checksum += (uint8_t)recv_char_buffer[index];
    }

    // extract checksum from packet
    char recv_checksum_array[3];
    memcpy(recv_checksum_array, recv_char_buffer + read_packet_len - 2, 2);
    recv_checksum_array[2] = '\0';
    uint8_t recv_checksum = strtol(recv_checksum_array, NULL, 16);

    if (calc_checksum != recv_checksum) {
        // checksum failed
        DODOBOT_SERIAL.print("Checksums don't match. ");
        DODOBOT_SERIAL.print(calc_checksum, HEX);
        DODOBOT_SERIAL.print(" != ");
        DODOBOT_SERIAL.println(recv_checksum, HEX);
        write_txrx(4);  // error 4: checksums don't match
        read_packet_num++;
        return;
    }

    uint32_t recv_packet_num;
    if (!segment_as_uint32(recv_packet_num)) {
        // failed to find packet num segment
        write_txrx(5);  // error 5: packet count segment not found
        read_packet_num++;
        return;
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
        return;
    }
    String category = String(segment);

    if (category == "txrx") {
        // process txrx segments and do nothing
        segment_as_uint32(recv_txrx_packet_num) && segment_as_uint32(recv_txrx_code);
    }
    else {
        (*read_callback)(this, category);
        write_txrx(0);  // 0: no error
    }
    read_packet_num++;
}