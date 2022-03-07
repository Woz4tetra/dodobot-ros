#ifndef __DODOBOT_SERIAL_H__
#define __DODOBOT_SERIAL_H__

#include <Arduino.h>

#define DATA_SERIAL  Serial5
#define INFO_SERIAL  Serial
#define SERIAL_MSG_BUFFER_SIZE 0x7f
char SERIAL_MSG_BUFFER[SERIAL_MSG_BUFFER_SIZE];
char SERIAL_PACKET_HEADER_BUFFER[4];
#define PACKET_START_0 '\x12'
#define PACKET_START_1 '\x34'
#define PACKET_STOP '\n'
#define PACKET_STOP_TIMEOUT 500
#define MAX_PACKET_LEN  0x1020  // Large packet segment size: 0x1000 + extras

#define CHECK_SEGMENT(__SERIAL_OBJ__, PARAM)  if (!__SERIAL_OBJ__->next_segment(PARAM)) {  println_error("Not enough segments supplied for #%d", __SERIAL_OBJ__->get_segment_num());  return;  }
#define CHECK_SEGMENT_BREAK(__SERIAL_OBJ__, PARAM)  if (!__SERIAL_OBJ__->next_segment(PARAM)) {  println_error("Not enough segments supplied for #%d", __SERIAL_OBJ__->get_segment_num());  break;  }
#define CHECK_SEGMENT_FLAG(__SERIAL_OBJ__, PARAM)  if (!__SERIAL_OBJ__->next_segment(PARAM)) {  println_error("Not enough segments supplied for #%d", __SERIAL_OBJ__->get_segment_num());  return false;  }

#define DODOBOT_SERIAL_WRITE_BOTH(...)  dodobot_serial::data->write(__VA_ARGS__);  dodobot_serial::info->write(__VA_ARGS__);

union
{
    uint16_t integer;
    unsigned char byte[2];
} uint16_union;

union
{
    uint32_t integer;
    unsigned char byte[4];
} uint32_union;

union
{
    int32_t integer;
    unsigned char byte[4];
} int32_union;

union
{
    float floating_point;
    unsigned char byte[8];
} float_union;


namespace dodobot_serial
{
    enum PRINT_BUFFER_TYPES {
        PRINT_INFO,
        PRINT_ERROR
    };
    class DodobotSerial {
    public:

        DodobotSerial(void (*read_callback)(String)) {
            this->read_callback = read_callback;
            init_variables();
        }
        virtual bool ready() {  // override
            return false;
        }
        virtual Stream* device() {  // override
            return 0;
        }

        uint32_t get_write_packet_num() {
            return write_packet_num;
        }

        uint32_t get_read_packet_num() {
            return read_packet_num;
        }

        void write(String name, const char *formats, ...) {
            va_list args;
            va_start(args, formats);
            if (ready()) {
                make_packet(name, formats, args);
                device()->write(write_char_buffer, write_char_index);
                write_packet_num++;
            }
            va_end(args);
        }
        void write(String packet) {
            if (!ready()) {
                return;
            }
            device()->print(packet);
        }

        void flush_read() {
            uint32_t available = device()->available();
            for (uint32_t i = 0; i < available; i++) {
                device()->read();
            }
        }

        void read() {
            if (!ready()) {
                return;
            }
            // 2 start chars
            // at least 1 char for packet num
            // \t + at least 1 category char
            // 2 chars for checksum
            // 1 new line
            if (device()->available() > 2) {
                char c;
                bool start_found = false;
                c = device()->read();
                if (c == PACKET_START_0) {
                    if (device()->available()) {
                        c = device()->read();
                        if (c == PACKET_START_1) {
                            start_found = true;
                        }
                    }
                }
                if (start_found) {
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
                            break;
                        }
                        if (!device()->available()) {
                            continue;
                        }

                        c = device()->read();
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
                                write("txrx", "dd", read_packet_num, 9);  // error 9: packet didn't end properly
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
            }
        }

        bool next_segment(int length)
        {
            if (read_packet_index >= read_packet_len) {
                current_segment_num = -1;
                return false;
            }
            if (length < 0) {
                // assume first 2 bytes contain the length
                uint16_union.byte[1] = recv_char_buffer[read_packet_index++];
                uint16_union.byte[0] = recv_char_buffer[read_packet_index++];
                length = (int)uint16_union.integer;
            }
            memcpy(segment, recv_char_buffer + read_packet_index, length);
            segment[length] = '\0';
            read_packet_index += length;

            return true;
        }

        bool next_segment()
        {
            if (read_packet_index >= read_packet_len) {
                current_segment_num = -1;
                return false;
            }

            int separator = -1;
            for (size_t i = read_packet_index; i < read_packet_len; i++) {
                if (recv_char_buffer[i] == '\t') {
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

        String get_segment() {
            return String(segment);
        }
        char* get_segment_raw() {
            return segment;
        }
        int get_segment_num() {
            return current_segment_num;
        }

        void print_buffer(PRINT_BUFFER_TYPES type, bool newline, const char* message) {
            if (!ready()) {
                return;
            }
            switch (type) {
                case PRINT_INFO:  device()->print("INFO\t"); break;
                case PRINT_ERROR:  device()->print("ERROR\t"); break;
                default: return;
            }
            device()->print(message);
            if (newline) {
                device()->print('\n');
            }
        }

        String get_written_packet() {
            return String(write_char_buffer);
        }

        uint16_t segment_as_uint16() {
            uint16_union.byte[1] = segment[0];
            uint16_union.byte[0] = segment[1];
            return uint16_union.integer;
        }

        uint32_t segment_as_uint32() {
            for (unsigned short i = 0; i < 4; i++) {
                uint32_union.byte[3 - i] = segment[i];
            }
            return uint32_union.integer;
        }

        int32_t segment_as_int32() {
            for (unsigned short i = 0; i < 4; i++) {
                int32_union.byte[3 - i] = segment[i];
            }
            return int32_union.integer;
        }

        float segment_as_float() {
            for (unsigned short i = 0; i < 4; i++) {
                float_union.byte[i] = segment[i];
            }
            return float_union.floating_point;
        }

    protected:
        char *segment;
        uint32_t read_packet_num;
        uint32_t write_packet_num;
        uint32_t buffer_index;
        uint32_t read_packet_index;
        int current_segment_num;
        bool prev_ready_state;
        char *recv_char_buffer;
        size_t recv_char_index;
        size_t read_packet_len;
        char *write_char_buffer;
        size_t write_char_index;
        uint32_t start_wait_time;

        void init_variables() {
            segment = new char[MAX_PACKET_LEN];

            read_packet_num = 0;
            write_packet_num = 0;
            buffer_index = 0;
            read_packet_index = 0;
            current_segment_num = -1;
            prev_ready_state = false;

            recv_char_buffer = new char[MAX_PACKET_LEN];
            recv_char_index = 0;

            write_char_buffer = new char[MAX_PACKET_LEN];
            write_char_index = 0;

            start_wait_time = 0;
        }

        void (*read_callback)(String);
        void make_packet(String name, const char *formats, va_list args)
        {
            write_char_buffer[0] = PACKET_START_0;
            write_char_buffer[1] = PACKET_START_1;
            write_char_index = 4;  // bytes 2 and 3 are for packet length

            uint32_union.integer = write_packet_num;
            for (unsigned short i = 0; i < 4; i++) {
                write_char_buffer[write_char_index++] = uint32_union.byte[3 - i];
            }
            sprintf(write_char_buffer + write_char_index, "%s", name.c_str());
            write_char_index += name.length();
            write_char_buffer[write_char_index++] = '\t';

            while (*formats != '\0') {
                // write_char_buffer[write_char_index++] = '\t';
                if (*formats == 'd') {
                    int32_union.integer = va_arg(args, int32_t);
                    for (unsigned short i = 0; i < 4; i++) {
                        write_char_buffer[write_char_index++] = int32_union.byte[3 - i];
                    }
                }
                else if (*formats == 'u') {
                    uint32_union.integer = va_arg(args, uint32_t);
                    for (unsigned short i = 0; i < 4; i++) {
                        write_char_buffer[write_char_index++] = uint32_union.byte[3 - i];
                    }
                }
                else if (*formats == 's') {
                    char *s = va_arg(args, char*);
                    uint16_union.integer = (uint16_t)strlen(s);
                    write_char_buffer[write_char_index++] = uint16_union.byte[1];
                    write_char_buffer[write_char_index++] = uint16_union.byte[0];
                    sprintf(write_char_buffer + write_char_index, "%s", s);
                    write_char_index += strlen(s);
                }
                else if (*formats == 'x') {
                    char *s = va_arg(args, char*);
                    uint16_union.byte[1] = s[0];
                    uint16_union.byte[0] = s[1];
                    for (size_t i = 0; i < uint16_union.integer; i++) {
                        write_char_buffer[write_char_index++] = s[2 + i];
                    }
                    write_char_index += 2 + uint16_union.integer;
                }
                else if (*formats == 'f') {
                    float_union.floating_point = (float)va_arg(args, double);
                    for (unsigned short i = 0; i < 4; i++) {
                        write_char_buffer[write_char_index++] = float_union.byte[i];
                    }
                }
                else {
                    write("txrx", "dd", read_packet_num, 8);  // error 8: invalid format
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

            uint16_union.integer = write_char_index - 5;  // subtract start, length, and stop bytes

            write_char_buffer[2] = uint16_union.byte[1];
            write_char_buffer[3] = uint16_union.byte[0];
        }

        void parse_packet()
        {
            // read_packet assumes PACKET_START_0, PACKET_START_1, and PACKET_STOP have been removed

            /*char c1 = get_char();
            if (c1 != PACKET_START_0) {
                write("txrx", "dd", read_packet_num, 1);  // error 1: c1 != \x12
                return;
            }
            char c2 = get_char();
            if (c2 != PACKET_START_1) {
                write("txrx", "dd", read_packet_num, 2);  // error 2: c2 != \x34
                return;
            }

            read_packet.remove(0, 2);*/  // remove start characters

            read_packet_index = 0;
            // at least 1 char for packet num
            // \t + at least 1 category char
            // 2 chars for checksum
            if (read_packet_len < 5) {
                write("txrx", "dd", read_packet_num, 3);  // error 3: packet is too short
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
            char recv_checksum_array[2];
            memcpy(recv_checksum_array, recv_char_buffer + read_packet_len - 2, 2);
            uint8_t recv_checksum = strtol(recv_checksum_array, NULL, 16);

            if (calc_checksum != recv_checksum) {
                // checksum failed
                write("txrx", "dd", read_packet_num, 4);  // error 4: checksums don't match
                read_packet_num++;
                return;
            }

            if (!next_segment(4)) {
                // failed to find packet num segment
                write("txrx", "dd", read_packet_num, 5);  // error 5: packet count segment not found
                read_packet_num++;
                return;
            }

            uint32_t recv_packet_num = segment_as_uint32();
            if (recv_packet_num != read_packet_num) {
                // this is considered a warning since it isn't critical for packet
                // numbers to be in sync
                write("txrx", "dd", read_packet_num, 6);  // error 6: packet counts not synchronized
                read_packet_num = recv_packet_num;
            }

            // find category segment
            if (!next_segment()) {
                write("txrx", "dd", read_packet_num, 7);  // error 7: failed to find category segment
                read_packet_num++;
                return;
            }
            String category = String(segment);

            // remove checksum
            // read_packet.remove(read_packet.length() - 2, 2);
            (*read_callback)(category);
            write("txrx", "dd", read_packet_num, 0);  // 0: no error
            read_packet_num++;
        }
    };

    class DodobotHWSerial : public DodobotSerial {
    private:
        HardwareSerial* __device;

    public:
        DodobotHWSerial (HardwareSerial* device, void (*read_callback)(String)) : DodobotSerial(read_callback) {
            __device = device;
        }
        Stream* device() {
            return __device;
        }
        bool ready() {
            return true;  // UART is always ready
        }
    };

    class DodobotUSBSerial : public DodobotSerial {
    private:
        usb_serial_class* __device;

    public:
        DodobotUSBSerial (usb_serial_class* device, void (*read_callback)(String)) : DodobotSerial(read_callback) {
            __device = device;
        }
        Stream* device() {
            return __device;
        }
        bool ready() {
            bool is_ready = (bool)__device->dtr();
            if (is_ready != prev_ready_state) {
                prev_ready_state = is_ready;
            }

            return is_ready;
        }
    };

    DodobotHWSerial* data;
    DodobotUSBSerial* info;

    void packet_callback(DodobotSerial* serial_obj, String category);

    void info_packet_callback(String category) {
        packet_callback(info, category);
    }

    void data_packet_callback(String category) {
        packet_callback(data, category);
    }

    // void println_info(const char* message)
    // {
    //     data->print_buffer(PRINT_INFO, true, message);
    //     info->print_buffer(PRINT_INFO, true, message);
    // }

    void println_info(const char* message, ...)
    {
        va_list args;
        va_start(args, message);
        vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
        va_end(args);
        data->print_buffer(PRINT_INFO, true, SERIAL_MSG_BUFFER);
        info->print_buffer(PRINT_INFO, true, SERIAL_MSG_BUFFER);
    }

    // void println_error(const char* message)
    // {
    //     data->print_buffer(PRINT_ERROR, true, message);
    //     info->print_buffer(PRINT_ERROR, true, message);
    // }

    void println_error(const char* message, ...)
    {
        va_list args;
        va_start(args, message);
        vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
        va_end(args);
        data->print_buffer(PRINT_ERROR, true, SERIAL_MSG_BUFFER);
        info->print_buffer(PRINT_ERROR, true, SERIAL_MSG_BUFFER);
    }

    // void println_info(const char* message)
    // {
    //     data->print_buffer(PRINT_INFO, false, message);
    //     info->print_buffer(PRINT_INFO, false, message);
    // }

    void print_info(const char* message, ...)
    {
        va_list args;
        va_start(args, message);
        vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
        va_end(args);
        data->print_buffer(PRINT_INFO, false, SERIAL_MSG_BUFFER);
        info->print_buffer(PRINT_INFO, false, SERIAL_MSG_BUFFER);
    }

    // void print_error(const char* message)
    // {
    //     data->print_buffer(PRINT_ERROR, false, message);
    //     info->print_buffer(PRINT_ERROR, false, message);
    // }

    void print_error(const char* message, ...)
    {
        va_list args;
        va_start(args, message);
        vsnprintf(SERIAL_MSG_BUFFER, SERIAL_MSG_BUFFER_SIZE, message, args);
        va_end(args);
        data->print_buffer(PRINT_ERROR, false, SERIAL_MSG_BUFFER);
        info->print_buffer(PRINT_ERROR, false, SERIAL_MSG_BUFFER);
    }

    void flush_read()
    {
        data->flush_read();
        info->flush_read();
    }

    bool parse_large(DodobotSerial* serial_obj, int32_t& segment_index, int32_t& num_segments, uint16_t& length, int32_t prev_segment_index, char** bytes)
    {
        CHECK_SEGMENT_FLAG(serial_obj, 4); segment_index = serial_obj->segment_as_int32();
        CHECK_SEGMENT_FLAG(serial_obj, 4); num_segments = serial_obj->segment_as_int32();
        CHECK_SEGMENT_FLAG(serial_obj, 2); length = serial_obj->segment_as_uint16();
        CHECK_SEGMENT_FLAG(serial_obj, length); *bytes = serial_obj->get_segment_raw();

        if (segment_index != 0 && segment_index - prev_segment_index != 1) {
            dodobot_serial::println_error("Received segments out of order! %d - %d != 1", segment_index, prev_segment_index);
            return false;
        }
        return true;
    }

    void setup_serial()
    {
        DATA_SERIAL.begin(1000000);
        INFO_SERIAL.begin(1000000);

        // DATA_SERIAL.setTimeout(1000);
        // INFO_SERIAL.setTimeout(1000);
        // DATA_SERIAL.begin(500000);  // see https://www.pjrc.com/teensy/td_uart.html for UART info
        // while (!INFO_SERIAL) {
        //     delay(1);
        // }

        data = new DodobotHWSerial(&DATA_SERIAL, data_packet_callback);
        info = new DodobotUSBSerial(&INFO_SERIAL, info_packet_callback);

        println_info("Dobobot!");
        println_info("Serial buses initialized.");
    }
};  // namespace dodobot_serial

#endif // __DODOBOT_SERIAL_H__
