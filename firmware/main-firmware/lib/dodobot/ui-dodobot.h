// Trust me, I tried to split this into multiple files.
// I couldn't get the compiler to cooperate
#pragma once

#include <JPEGDecoder.h>  // JPEG decoder library

#include "serial-dodobot.h"
#include "sd-dodobot.h"
#include "display-dodobot.h"
#include "latch-circuit-dodobot.h"
#include "breakout-dodobot.h"
#include "chassis-dodobot.h"
#include "linear-dodobot.h"
#include "tilter-dodobot.h"
#include "speed-pid-dodobot.h"
#include "gripper-dodobot.h"


using namespace dodobot_display;


#define RETURN_IF_NOT_LOADED  if (!is_loaded())  { return; }

namespace dodobot_ui
{
    const uint32_t UI_DELAY_MS_DEFAULT = 200;
    uint32_t UI_DELAY_MS = UI_DELAY_MS_DEFAULT;
    uint32_t ui_timer = 0;

    uint32_t UI_BRIGHTNESS_TIMEOUT = 300000;
    uint32_t ui_sleep_brightness_timer = 0;

    enum EventType {
        UP_EVENT,
        DOWN_EVENT,
        LEFT_EVENT,
        RIGHT_EVENT,
        BACK_EVENT,
        ENTER_EVENT,
        NONE_EVENT,
        EVENT_0,
        EVENT_1,
        EVENT_2,
        EVENT_3,
        EVENT_4,
        EVENT_5,
        EVENT_6,
        EVENT_7,
        EVENT_8,
        EVENT_9,
        KEYBOARD_UP,
        KEYBOARD_DOWN,
        KEYBOARD_HOLD,
        FILE_EVENT
    };


    void load(String name);

    class ViewController
    {
    public:
        ViewController()  {}
        virtual void draw()  {}
        virtual void on_load()  {}
        virtual void on_unload()  {}
        virtual void on_up()  {}
        virtual void on_down()  {}
        virtual void on_left()  {}
        virtual void on_right()  {}
        virtual void on_back()  {}
        virtual void on_enter()  {}
        virtual void on_repeat(EventType e)  {}
        virtual void on_numpad(int number)  {}
        virtual void on_keyboard(EventType e, char c)  {}
        virtual void on_file(String filename)  {}

        bool is_loaded() {
            return _is_loaded;
        }

        void set_loaded(bool state) {
            _is_loaded = state;
        }
    protected:
        bool _is_loaded = false;

    };

    class ViewWithOverlayController : public ViewController
    {
    public:
        ViewWithOverlayController(ViewController* overlay) : ViewController() {
            this->overlay = overlay;
        }
        void draw() {
            RETURN_IF_NOT_LOADED;
            overlay->draw();
            draw_with_overlay();
        }
        virtual void draw_with_overlay() {}

        void on_load() {
            overlay->on_load();
            this->set_loaded(true);
            overlay->set_loaded(true);
            on_load_with_overlay();
        }
        virtual void on_load_with_overlay() {}

        void on_unload() {
            overlay->on_unload();
            this->set_loaded(false);
            overlay->set_loaded(false);
            on_unload_with_overlay();
        }
        virtual void on_unload_with_overlay() {}

    protected:
        ViewController* overlay;
    };

    const int max_num_entries = 20;
    struct MenuEntry {
        String name;
        void (*callback)(String, int);
    };
    struct MenuBlank {
        int index;
        uint16_t h;
    };
    class ScrollingMenu
    {
    public:
        int16_t x0, y0;
        uint16_t menu_w, entry_h;

        bool show_numbers;
        int16_t text_size;

        ScrollingMenu(int16_t x = 0, int16_t y = 0, uint16_t w = 0, uint16_t h = 0, bool show_numbers = false, int16_t text_size = 1)
        {
            x0 = x;
            y0 = y;
            menu_w = w;
            entry_h = h;

            menu_h = 0;

            this->show_numbers = show_numbers;
            this->text_size = text_size;
            entries = new MenuEntry[max_num_entries];
            blanks = new MenuBlank[max_num_entries];
            num_entries = 0;
            num_blanks = 0;
        }

        int get_selected() {
            return selected_index;
        }

        int add_entry(String name, void (*callback)(String, int))
        {
            MenuEntry entry;
            entry.name = name;
            entry.callback = callback;
            return add_entry(entry);
        }

        int add_entry(MenuEntry entry)
        {
            entries[num_entries++] = entry;
            return num_entries - 1;
        }

        bool set_entry(int index, const char* name)
        {
            if (index < 0 || index >= num_entries) {
                return false;
            }
            entries[index].name = String(name);
            return true;
        }

        bool set_entry(int index, MenuEntry entry)
        {
            if (index < 0 || index >= num_entries) {
                return false;
            }
            entries[index] = entry;
            return true;
        }

        void add_blank(uint16_t h, int index = -1)
        {
            MenuBlank blank;
            if (index == -1) {
                index = num_entries - 1;
            }
            else {
                if (index < 0 || index >= num_entries) {
                    return;
                }
            }
            blank.index = index;
            blank.h = h;
            add_blank(blank);
        }

        void add_blank(MenuBlank blank) {
            blanks[num_blanks++] = blank;
        }

        void remove_blank(int index) {
            for (int shift_index = index; shift_index < num_blanks - 1; shift_index++) {
                blanks[index] = blanks[index + 1];
            }
            num_blanks--;
        }

        void remove_all_blanks() {
            num_blanks = 0;
        }

        void on_up()
        {
            if (selected_index == -2) {
                return;
            }
            select(selected_index - 1);
        }

        void on_down()
        {
            if (selected_index == -2) {
                return;
            }
            select(selected_index + 1);
        }

        void on_enter() {
            if (selected_index < 0) {
                return;
            }
            if (selected_index >= num_entries) {
                return;
            }
            entries[selected_index].callback(entries[selected_index].name, selected_index);
        }

        void select(int index)
        {
            selected_index = index;
            if (selected_index == -2) {
                return;
            }
            if (selected_index < 0) {
                selected_index = num_entries - 1;
            }
            if (selected_index >= num_entries) {
                selected_index = 0;
            }

            if (selected_index < in_view_start) {
                in_view_stop -= in_view_start - selected_index;
                in_view_start = selected_index;
            }
            if (selected_index >= in_view_stop) {
                in_view_start += selected_index - (in_view_stop - 1);
                in_view_stop = selected_index + 1;
            }
            draw();
        }

        void deselect() {
            selected_index = -2;
        }

        void on_load()
        {
            if (menu_w == 0) {
                menu_w = tft.width() - x0 - 1;
            }
            if (entry_h == 0) {
                entry_h = 10;
            }

            compute_stop_index();
            draw();
        }

        int index_has_blank(int index) {
            for (int blank_index = 0; blank_index < num_blanks; blank_index++) {
                if (blanks[blank_index].index == index) {
                    return blank_index;
                }
            }
            return -1;
        }

        void on_unload() {

        }
        void draw() {
            tft.setTextSize(text_size);
            uint16_t text_w, text_h;
            int16_t draw_x;
            uint16_t draw_w;

            dodobot_display::textBounds("  ", text_w, text_h);
            if (show_numbers) {
                text_w += 2;
                draw_x = x0 + text_w;
                draw_w = menu_w - text_w;
            }
            else {
                draw_x = x0 - 1;
                draw_w = menu_w;
            }
            int16_t draw_y = y0;

            if (in_view_stop < in_view_start) {
                in_view_stop = in_view_start;
                dodobot_serial::println_error("ScrollingMenu: Stop index is less than start index!");
            }

            int16_t rect_x = draw_x - 2;
            tft.fillRect(rect_x, y0, menu_w, menu_h, ST77XX_BLACK);

            for (int index = in_view_start; index < in_view_stop; index++)
            {
                if (show_numbers) {
                    tft.setCursor(x0, draw_y);
                    tft.print(String(index + 1));
                }

                tft.setCursor(draw_x, draw_y);
                tft.print(entries[index].name);

                int16_t rect_y = draw_y + (text_h - entry_h) / 2 - 1;
                uint16_t rect_h = entry_h + 2;

                if (selected_index != prev_selected_index && index == prev_selected_index) {
                    tft.drawRect(rect_x, rect_y, draw_w, rect_h, ST77XX_BLACK);
                }
                if (index == selected_index) {
                    tft.drawRect(rect_x, rect_y, draw_w, rect_h, ST77XX_WHITE);
                }

                draw_y += entry_h;
                int blank_index = index_has_blank(index);
                if (blank_index != -1) {
                    draw_y += blanks[blank_index].h;
                }

            }
            prev_selected_index = selected_index;
        }

    private:
        MenuEntry* entries;
        int num_entries;

        MenuBlank* blanks;
        int num_blanks;

        int selected_index = 0;
        int prev_selected_index = -2;
        int in_view_start = 0;  // inclusive index
        int in_view_stop = 0;  // exclusive index

        uint16_t menu_h;

        void compute_stop_index()
        {
            int draw_y = 0;
            for (int index = 0; index < num_entries; index++) {
                if (index == in_view_start) {
                    draw_y = y0;
                }
                draw_y += entry_h;
                in_view_stop = index;
                int blank_index = index_has_blank(index);
                if (blank_index != -1) {
                    draw_y += blanks[blank_index].h;
                }
                if (draw_y >= tft.height() - 1) {
                    break;
                }
            }
            menu_h = draw_y;
            in_view_stop++;
        }
    };

    const size_t max_num_views = 100;
    size_t num_views = 0;
    ViewController* views[max_num_views];
    String names[max_num_views];

    ViewController* current_view;

    class SplashScreenController : public ViewController
    {
    public:
        SplashScreenController() {

        }
        void draw()  {}
        void on_load()
        {
            set_loaded(true);
            dodobot_serial::println_info("Splash view loaded");
            dodobot_sd::loadGIF("BW_BOT~1.GIF");
            while (dodobot_sd::drawGIFframe())  {}
            dodobot_sd::closeGIF();

            for (int i = 255; i >= 0; i--) {
                dodobot_display::set_display_brightness(i);
                delay(1);
            }

            load("main-menu");

            for (int i = 0; i <= 255; i++) {
                dodobot_display::set_display_brightness(i);
                delay(1);
            }
        }

        void on_unload() {
            set_loaded(false);
        }
        void on_up()  {}
        void on_down()  {}
        void on_left()  {}
        void on_right()  {}
        void on_back()  {}
        void on_enter()  {}

    };


    enum NotificationLevel {
        INFO = 0,
        WARN = 1,
        ERROR = 2
    };

    struct Notification {
        NotificationLevel level;
        String text;
        uint32_t timeout;
        bool spent;
    };

    Notification make_notif(NotificationLevel level, String text, uint32_t timeout)
    {
        Notification notif;
        notif.level = level;
        notif.text = text;
        notif.timeout = timeout;
        notif.spent = false;
        return notif;
    }
    void notify(NotificationLevel level, String text, uint32_t timeout);

    template <class T>
    class Queue
    {
    public:
        Queue(int max_len) {
            this->max_len = max_len;
            buffer = new T[max_len];
        }

        bool empty() {
            return len == 0;
        }

        bool full() {
            return len == max_len;
        }

        int size() {
            return len;
        }

        bool queue(T item)
        {
            if (full()) {
                return false;
            }
            if (is_present(item)) {
                return false;
            }

            if (back == max_len - 1) {
                back = -1;
            }
            buffer[++back] = item;
            len++;

            return true;
        }

        int deque()
        {
            if (empty()) {
                return -1;
            }
            int index = front;
            front++;
            if (front == max_len) {
                front = 0;
            }
            len--;
            return index;
        }
        int peek() {
            if (empty()) {
                return -1;
            }
            return front;
        }

        virtual bool is_present(T item) {
            return false;
        }

        T get_index(int index) {
            if (index < 0) {
                index = 0;
            }
            if (index >= max_len) {
                index = max_len - 1;
            }
            return buffer[index];
        }
    protected:
        T* buffer;
        int front = 0;
        int back = -1;
        int len = 0;
        int max_len;
    };

    const int max_notifs = 5;
    class NotificationQueue : public Queue<Notification>
    {
    public:
        NotificationQueue() : Queue(max_notifs)  {}

        bool is_present(Notification notif)
        {
            for (int index = 0; index < max_len; index++) {
                if (buffer[index].spent) {
                    continue;
                }
                if (notif.text.equals(buffer[index].text) && notif.level == buffer[index].level) {
                    return true;
                }
            }
            return false;
        }

        void set_spent(int index) {
            buffer[index].spent = true;
        }
    };

    class NumberSequence : public Queue<int>
    {
    public:
        NumberSequence(int max_len) : Queue<int>(max_len)  {}

        bool check_sequence(int* sequence, int seq_len)
        {
            if (empty()) {
                return false;
            }
            if (seq_len > len) {
                return false;
            }
            int index = front;
            int seq_index = 0;
            while (index != back)
            {
                if (buffer[index] == sequence[seq_index]) {
                    seq_index++;
                }
                else {
                    seq_index = 0;

                    // edge case for if the current mismatch is actually the start of a sequence
                    if (buffer[index] == sequence[seq_index]) {
                        seq_index++;
                    }
                }

                index++;
                if (index >= max_len) {
                    index = 0;
                }
                if (seq_index >= seq_len) {
                    break;
                }
            }
            if (seq_index == seq_len - 1) {
                return buffer[index] == sequence[seq_index];
            }
            return false;
        }
    };

    enum ConnectState {
        NO_USB_POWER,
        USB_POWER_DETECTED,
        USB_STABLE,
        SHUTTING_DOWN,
        USB_UNKNOWN
    };
    String date_string = "00:00:00AM";
    uint32_t prev_date_str_update = 0;

    void update_date(String date) {
        date_string = date;
        prev_date_str_update = CURRENT_TIME;
    }

    bool is_charging = false;

    class TopbarController : public ViewController
    {
    public:
        TopbarController()
        {
            text = starting_text;
            usb_state = NO_USB_POWER;
            prev_text_w = 0;
            prev_text_h = 0;
            battery_update_timer = CURRENT_TIME;
        }

        bool notify(NotificationLevel level, String text, uint32_t timeout) {
            return notify(make_notif(level, text, timeout));
        }

        bool notify(Notification notif) {
            if (notif.text.length() == 0) {
                dodobot_serial::println_error("Notification has no text. Skipping");
                return false;
            }
            if (notif.timeout == 0) {
                dodobot_serial::println_error("Timeout is 0. Skipping notification: %s", notif.text.c_str());
                return false;
            }

            return queue.queue(notif);
        }

        void fillBottomScreen() {
            tft.fillRect(0, get_height(), tft.width(), tft.height(), ST77XX_BLACK);
        }
        void draw()
        {
            update_usb_state();
            update_text();
            draw_text();
            draw_status_icons();
            draw_battery();
        }
        void on_load()
        {
            width = tft.width();

            top_icon_r = height / 2;
            motor_icon_r = top_icon_r / 2;
            status_arc_w = top_icon_r - motor_icon_r - 1;

            battery_V_cy = height / 3;
            battery_mA_cy = 2 * height / 3;

            report_icon_cx = top_icon_r + 2;
            report_icon_cy = height / 2;
            connection_icon_cx = report_icon_cx + 5;
            connection_icon_cy = height / 2;
            battery_x = width - 5;

            prev_reporting = !dodobot::robot_state.is_reporting_enabled;
            prev_motors = !dodobot::robot_state.motors_active;
            prev_usb_state = USB_UNKNOWN;
        }
        void on_unload()  {}
        void on_up()  {}
        void on_down()  {}
        void on_left()  {}
        void on_right()  {}
        void on_back()  {}
        void on_enter()  {}

        int get_height() {
            return height + 1;
        }

        int get_width() {
            return width;
        }

    private:
        int height = 21;
        int width;

        int report_icon_cx = 0;
        int report_icon_cy = 0;

        int connection_icon_cx = 0;
        int connection_icon_cy = 0;

        int top_icon_r;
        int motor_icon_r;
        int status_arc_w;

        int battery_x = 0;
        int battery_V_cy;
        int battery_mA_cy;
        int batt_nub_h = 7;
        int batt_nub_y;
        uint16_t single_char_w = 6;
        uint16_t w_V, h_V;
        uint16_t w_mA, h_mA;

        int notif_box_x = 0;
        int notif_box_y = 0;
        int notif_box_width = 70;
        int notif_box_height = 5;
        int notif_height_offset = 3;


        String starting_text = "Connecting...";
        String text;

        uint16_t notif_color = 0;
        uint32_t notif_timer = 0;

        uint16_t prev_text_w, prev_text_h;

        NotificationQueue queue;
        int current_notif_index = -1;

        bool prev_reporting = false;
        bool prev_motors = false;
        ConnectState usb_state = USB_UNKNOWN;
        ConnectState prev_usb_state = USB_UNKNOWN;

        uint32_t battery_update_timer;

        void check_notif_queue()
        {
            if (current_notif_index == -1) {
                current_notif_index = queue.deque();
                if (current_notif_index == -1) {
                    return;
                }
                notif_timer = CURRENT_TIME;
            }
        }

        void update_text()
        {
            if (queue.empty() && current_notif_index == -1)
            {
                if (usb_state == USB_STABLE) {
                    text = date_string;
                }
                else {
                    text = starting_text;
                }
                notif_color = ST77XX_WHITE;
            }
            else {
                check_notif_queue();

                switch (queue.get_index(current_notif_index).level) {
                    case INFO:  notif_color = ST77XX_WHITE; break;
                    case WARN:  notif_color = ST77XX_YELLOW; break;
                    case ERROR:  notif_color = ST77XX_RED; break;
                }

                text = queue.get_index(current_notif_index).text;

                if (CURRENT_TIME - notif_timer > queue.get_index(current_notif_index).timeout) {
                    clear_notif_bar();
                    queue.set_spent(current_notif_index);
                    current_notif_index = -1;
                }
            }
        }

        void draw_text()
        {
            tft.setTextSize(1);
            uint16_t w, h;
            dodobot_display::textBounds(text, w, h);
            int16_t x = (width - w) / 2;
            int16_t y = (height - h) / 2;

            int16_t prev_x = (width - prev_text_w) / 2;
            int16_t prev_y = (height - prev_text_h) / 2;

            dodobot_display::fillPrevText(prev_x, prev_y, prev_text_w, prev_text_h, x, y, w, h, ST77XX_BLACK);
            prev_text_w = w;
            prev_text_h = h;

            tft.setCursor(x, y);
            tft.setTextColor(notif_color, ST77XX_BLACK);
            // tft.fillRect(0, y, width, y + 8, ST77XX_BLACK);
            tft.print(text);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

            draw_notif_bar(h);
        }

        void draw_notif_bar(uint16_t text_h)
        {
            if (current_notif_index == -1) {
                return;
            }
            notif_box_x = (width - notif_box_width) / 2;
            notif_box_y = (height + text_h) / 2 + notif_height_offset;
            tft.fillRect(notif_box_x, notif_box_y, notif_box_width, notif_box_height, ST77XX_GRAY);

            uint32_t notif_time = queue.get_index(current_notif_index).timeout;
            uint32_t dt = CURRENT_TIME - notif_timer;
            uint16_t w = (uint16_t)(notif_box_width * (1.0 - (double)dt / notif_time));
            tft.fillRect(notif_box_x, notif_box_y, w, notif_box_height, notif_color);
        }

        void clear_notif_bar() {
            tft.fillRect(notif_box_x, notif_box_y, notif_box_width, notif_box_height, ST77XX_BLACK);
        }

        void draw_status_icons()
        {
            if (prev_reporting != dodobot::robot_state.is_reporting_enabled)
            {
                prev_reporting = dodobot::robot_state.is_reporting_enabled;
                uint16_t status_color = ST77XX_RED;
                if (dodobot::robot_state.is_reporting_enabled) {
                    status_color = ST77XX_GREEN;
                }
                dodobot_display::drawArc(report_icon_cx, report_icon_cy, 90, 270, top_icon_r, top_icon_r, status_arc_w, status_color);
            }

            if (prev_usb_state != usb_state)
            {
                prev_usb_state = usb_state;
                uint16_t conn_color = ST77XX_WHITE;
                if (usb_state == NO_USB_POWER) {
                    conn_color = ST77XX_WHITE;
                }
                else if (usb_state == USB_POWER_DETECTED) {
                    conn_color = ST77XX_YELLOW;
                }
                else if (usb_state == USB_STABLE) {
                    conn_color = ST77XX_GREEN;
                }
                else if (usb_state == SHUTTING_DOWN) {
                    conn_color = ST77XX_WHITE;
                    notify(INFO, "Shutting down", dodobot_latch_circuit::POWER_OFF_THRESHOLD_MS);
                }
                dodobot_display::drawArc(connection_icon_cx, connection_icon_cy, 90, -90, top_icon_r, top_icon_r, status_arc_w, conn_color);
            }

            if (prev_motors != dodobot::robot_state.motors_active)
            {
                prev_motors = dodobot::robot_state.motors_active;
                uint16_t motor_color = ST77XX_BLACK;
                if (dodobot::robot_state.motors_active) {
                    // motor_color = ST77XX_DARK_GREEN;
                    motor_color = ST77XX_GREEN;
                }
                tft.fillCircle(report_icon_cx, report_icon_cy, motor_icon_r, motor_color);
                tft.fillCircle(connection_icon_cx, connection_icon_cy, motor_icon_r, motor_color);

                int w = connection_icon_cx - report_icon_cx;
                int h = 2 * motor_icon_r;
                tft.fillRect(report_icon_cx, report_icon_cy - motor_icon_r, w, h, motor_color);
            }
        }
        void draw_battery()
        {
            if (CURRENT_TIME - battery_update_timer < INA_SAMPLERATE_DELAY_MS) {
                return;
            }
            battery_update_timer = CURRENT_TIME;

            String voltage_text = String(dodobot_power_monitor::ina219_loadvoltage) + "V";
            String current_text = String((int)dodobot_power_monitor::ina219_current_mA) + "mA";

            tft.setTextSize(1);
            dodobot_display::textBounds(voltage_text, w_V, h_V);
            int16_t x_V = battery_x - w_V;
            int16_t y_V = battery_V_cy - h_V / 2;

            dodobot_display::textBounds(current_text, w_mA, h_mA);
            int16_t x_mA = battery_x - w_mA;
            int16_t y_mA = battery_mA_cy - h_mA / 2 + 2;

            int gauge_count = dodobot_power_monitor::power_level();
            uint16_t battery_color;
            if (is_charging) {
                battery_color = ST77XX_DARKER_GREEN;
            }
            else {
                battery_color = ST77XX_WHITE;
            }
            if (gauge_count <= 1) {
                battery_color = ST77XX_RED;
            }
            uint16_t battery_nub_color = ST77XX_GRAY;
            if (gauge_count == dodobot_power_monitor::max_power_level) {
                battery_nub_color = battery_color;
            }

            int gauge_len = dodobot_power_monitor::max_power_level;
            int h_gauge = h_V + h_mA + 2;

            batt_nub_y = (height - batt_nub_h) / 2 + 1;
            tft.fillRect(battery_x, batt_nub_y, 4, batt_nub_h, battery_nub_color);

            // int stop_index = max(voltage_text.length(), current_text.length());
            int stop_index = max(5, gauge_len + 1);  // text can extend past gauge by 1
            for (int index = 0; index < stop_index; index++)
            {
                int gauge_x = battery_x - single_char_w * (index + 1);
                int inv_index = gauge_len - index;
                uint16_t capacity_color = ST77XX_GRAY;

                if (index < gauge_len)
                {
                    if (inv_index <= gauge_count) {
                        capacity_color = battery_color;
                    }
                }
                else {
                    capacity_color = ST77XX_BLACK;
                }
                tft.fillRect(gauge_x, y_V - 1, single_char_w, h_gauge, capacity_color);
            }

            uint16_t shadow_color, text_color;
            if (is_charging) {
                shadow_color = ST77XX_BLACK;
                text_color = ST77XX_WHITE;
            }
            else {
                shadow_color = ST77XX_WHITE;
                text_color = ST77XX_BLACK;
            }
            for (int16_t x_offset = -1; x_offset <= 1; x_offset++) {
                for (int16_t y_offset = -1; y_offset <= 1; y_offset++) {
                    if (x_offset == 0 && y_offset == 0) {
                        continue;
                    }
                    draw_power_text(x_V + x_offset, y_V + y_offset, x_mA + x_offset, y_mA + y_offset, voltage_text, current_text, shadow_color);
                }
            }
            draw_power_text(x_V, y_V, x_mA, y_mA, voltage_text, current_text, text_color);
            
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        }

        void draw_power_text(int16_t x_V, int16_t y_V, int16_t x_mA, int16_t y_mA, String voltage_text, String current_text, uint16_t color)
        {
            tft.setTextColor(color, color);  // transparent text background
            tft.setCursor(x_V, y_V);
            tft.print(voltage_text);

            tft.setCursor(x_mA, y_mA);
            tft.print(current_text);
        }

        void update_usb_state()
        {
            if (!dodobot_latch_circuit::state.usb_voltage_state) {
                if (dodobot_latch_circuit::state.usb_connected_once) {
                    usb_state = SHUTTING_DOWN;
                }
                else {
                    usb_state = NO_USB_POWER;
                }
            }
            else {
                if (prev_date_str_update == 0 || CURRENT_TIME - prev_date_str_update > 2000) {
                    usb_state = USB_POWER_DETECTED;
                }
                else {
                    usb_state = USB_STABLE;
                }
            }
        }
    };

    const int num_main_menu_entries = 4;

    enum MainMenuEntry {
        NETWORK = 0,
        ROBOT = 1,
        SYSTEM = 2,
        SHUTDOWN = 3
    };

    const int breakout_seq_len = 5;
    int breakout_sequence[breakout_seq_len] = {8, 1, 5, 3, 8};

    class MainMenuController : public ViewWithOverlayController
    {
    public:
        MainMenuController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            draw_slots = new int[num_draw_slots];
            number_sequence = new NumberSequence(num_seq_len);
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()  {}
        void on_load_with_overlay()
        {
            // tft.fillScreen(ST77XX_BLUE);
            draw_slots[0] = tft.width() / 4 - 30;
            draw_slots[1] = tft.width() / 2;
            draw_slots[2] = 3 * tft.width() / 4 + 30;
            draw_height = tft.height() / 2;

            icon_text_height = draw_height + selected_h / 2 + 10;

            overview_x = tft.width() / 2;
            overview_y = draw_height + selected_h / 2 + 25;

            draw_all();

            // topbar()->notify(INFO, "1", 2000);
            // topbar()->notify(INFO, "2", 2000);
            // topbar()->notify(INFO, "3", 2000);
            // topbar()->notify(INFO, "4", 2000);
            // topbar()->notify(INFO, "5", 2000);
            // topbar()->notify(INFO, "6", 2000);
        }

        void move_select_left()
        {
            selected_index--;
            if (selected_index < 0) {
                selected_index = num_main_menu_entries - 1;
            }
        }
        void move_select_right()
        {
            selected_index++;
            if (selected_index >= num_main_menu_entries) {
                selected_index = 0;
            }
        }

        void on_unload_with_overlay()  {}
        void on_up()  {}
        void on_down()  {}
        void on_left()  {
            RETURN_IF_NOT_LOADED;
            move_select_left();
            draw_all();
        }
        void on_right()  {
            RETURN_IF_NOT_LOADED;
            move_select_right();
            draw_all();
        }
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("splash");
        }
        void on_enter()  {
            RETURN_IF_NOT_LOADED;
            if (selected_index < 0 || selected_index >= num_main_menu_entries) {
                return;
            }
            load(view_name((MainMenuEntry)selected_index));
        }

        void on_numpad(int number)
        {
            RETURN_IF_NOT_LOADED;
            dodobot_serial::println_info("number: %d", number);
            if (number_sequence->full()) {
                number_sequence->deque();
            }
            number_sequence->queue(number);
            if (number_sequence->check_sequence(breakout_sequence, breakout_seq_len)) {
                for (int i = 0; i < num_seq_len; i++) {
                    number_sequence->deque();
                }
                load("breakout");
            }
        }

        void draw_all()
        {
            RETURN_IF_NOT_LOADED;
            topbar()->fillBottomScreen();

            for (int index = 0; index < num_draw_slots; index++) {
                int cx = draw_slots[index];
                int draw_index = (selected_index + index - 1) % num_main_menu_entries;
                if (draw_index < 0) {
                    draw_index = num_main_menu_entries - abs(draw_index);
                }
                draw_icon(cx, (MainMenuEntry)draw_index);
            }

            int cx = draw_slots[1];
            int cy = draw_height;
            int x = cx - selected_w / 2;
            int y = cy - selected_h / 2;
            tft.drawRoundRect(x, y, selected_w, selected_h, selected_r, ST77XX_WHITE);

            draw_selection_overview();
        }

        void draw_selection_overview()
        {
            int total_width = (overview_r + 2) * 2 * num_main_menu_entries;
            int offset = overview_x - total_width / 2;
            int increment = total_width / (num_main_menu_entries - 1);
            uint16_t color;
            for (int index = 0; index < num_main_menu_entries; index++)
            {
                if (index == selected_index) {
                    color = ST77XX_WHITE;
                }
                else {
                    color = ST77XX_GRAY;
                }
                tft.fillCircle(offset, overview_y, overview_r, color);
                offset += increment;
            }
        }

        void draw_icon(int cx, MainMenuEntry index)
        {
            int cy = draw_height;
            int x = cx - icon_w / 2;
            int y = cy - icon_h / 2;

            switch (index) {
                case NETWORK: draw_network_icon(x, y, cx, cy);  break;
                case ROBOT:  draw_robot_status_icon(x, y, cx, cy);  break;
                case SYSTEM:  draw_system_icon(x, y, cx, cy);  break;
                case SHUTDOWN:  draw_shutdown_icon(x, y, cx, cy);  break;
                default: tft.fillRoundRect(x, y, icon_w, icon_h, icon_r, ST77XX_WHITE); break;
            }

            tft.setTextSize(1);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            String text = entry_name(index);
            uint16_t w, h;
            dodobot_display::textBounds(text, w, h);
            x = cx - w / 2;
            y = icon_text_height - h / 2;
            tft.setCursor(x, y);
            tft.print(text);
        }

        void draw_network_icon(int x, int y, int cx, int cy)
        {
            tft.fillRoundRect(x, y, icon_w, icon_h, icon_r, ST77XX_WHITE);
            int fan_angle = 35;
            int center_angle = 270;
            int max_fan_w = icon_h - 10;
            int mid_fan_w = 2 * max_fan_w / 3 + 3;
            int low_fan_w = max_fan_w / 3 + 5;
            int thickness = 6;
            x = cx;
            y = cy + icon_h / 2 - 5;
            dodobot_display::drawArc(x, y, center_angle - fan_angle, center_angle + fan_angle, max_fan_w, max_fan_w, thickness, ST77XX_GRAY);
            dodobot_display::drawArc(x, y, center_angle - fan_angle, center_angle + fan_angle, mid_fan_w, mid_fan_w, thickness, ST77XX_GRAY);
            dodobot_display::drawArc(x, y, center_angle - fan_angle, center_angle + fan_angle, low_fan_w, low_fan_w, thickness, ST77XX_GRAY);
            tft.fillCircle(x, y - 3, 5, ST77XX_GRAY);
        }

        void draw_robot_status_icon(int x, int y, int cx, int cy)
        {
            tft.fillRoundRect(x, y, icon_w, icon_h, icon_r, ST77XX_LIGHT_BLUE);
            int triangle_cx = cx + 2;
            int triangle_cy = cy + 5;
            int x0 = triangle_cx;
            int y0 = triangle_cy;
            int x1 = cx - icon_w / 2 + 7;
            int y1 = cy + icon_h / 2 - 15;
            int x2 = cx + icon_w / 2 - 15;
            int y2 = cy - icon_h / 2 + 7;
            int x3 = cx + icon_w / 2 - 15;
            int y3 = cy + icon_h / 2 - 7;
            uint16_t icon_color = ST77XX_GRAY;
            if (dodobot::robot_state.is_ros_ready) {
                icon_color = ST77XX_DARKER_BLUE;
            }
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, icon_color);
            tft.fillTriangle(x0, y0, x2, y2, x3, y3, icon_color);
        }

        void draw_system_icon(int x, int y, int cx, int cy)
        {
            tft.fillRoundRect(x, y, icon_w, icon_h, icon_r, ST77XX_GRAY);

            int outer_r = icon_w / 2 - 7;
            int inner_r = icon_w / 7;
            dodobot_display::drawCircle(cx, cy, outer_r, 5, ST77XX_BLACK);
            tft.fillCircle(cx, cy, inner_r, ST77XX_BLACK);

            int center_angle = 0;
            for (int index = 0; index < 8; index++)
            {
                int tooth_cx = (float)outer_r * cos(TO_RADIANS(center_angle)) + cx;
                int tooth_cy = (float)outer_r * sin(TO_RADIANS(center_angle)) + cy;

                tft.fillCircle(tooth_cx, tooth_cy, 5, ST77XX_BLACK);

                center_angle += 45;
            }
        }

        void draw_shutdown_icon(int x, int y, int cx, int cy)
        {
            tft.fillRoundRect(x, y, icon_w, icon_h, icon_r, ST77XX_LIGHT_PINK);

            int outer_r = icon_w / 2 - 7;
            int rect_w = 5;
            int rect_h = 20;
            int rect_border = 6;

            dodobot_display::drawCircle(cx, cy, outer_r, rect_w, ST77XX_BLACK);

            x = cx - rect_w / 2;
            y = cy - outer_r - 3;
            tft.fillRect(x - rect_border / 2, y - rect_border / 2, rect_w + rect_border, rect_h + rect_border, ST77XX_LIGHT_PINK);
            tft.fillRoundRect(x, y, rect_w, rect_h, 2, ST77XX_BLACK);
        }

        String view_name(MainMenuEntry entry) {
            switch (entry) {
                case NETWORK: return "network";
                case ROBOT:  return "robot";
                case SYSTEM:  return "system";
                case SHUTDOWN:  return "shutdown";
                default: return "";
            }
        }
    private:
        int num_draw_slots = 3;
        int* draw_slots;

        int num_seq_len = 10;
        NumberSequence* number_sequence;

        int draw_height = 0;
        int icon_text_height = 0;
        int icon_w = 50;
        int icon_h = 50;
        int icon_r = 10;

        int selected_w = 60;
        int selected_h = 60;
        int selected_r = 15;

        int overview_x = 0;
        int overview_y = 0;
        int overview_r = 5;

        int selected_index = 1;

        String entry_name(MainMenuEntry entry) {
            switch (entry) {
                case NETWORK: return "Network";
                case ROBOT:  return "Robot";
                case SYSTEM:  return "Systems";
                case SHUTDOWN:  return "Shutdown";
                default: return "";
            }
        }

    };

    class BreakoutController : public ViewController
    {
    public:
        int current_level_index = -1;

        void draw() {
            RETURN_IF_NOT_LOADED;

            dodobot_breakout::draw();
            if (dodobot_chassis::is_bump1_active()) {
                on_right();
            }
            if (dodobot_chassis::is_bump2_active()) {
                on_left();
            }
        }
        void on_load()
        {
            set_loaded(true);
            UI_DELAY_MS = dodobot_breakout::UPDATE_DELAY_MS;
            load_level(-1);  // load random level
            dodobot_breakout::on_load();
        }

        void load_level(int level_index)
        {
            dodobot_breakout::level_config = dodobot_sd::load_breakout_level(&dodobot_breakout::level_name, level_index);
            current_level_index = level_index;
            dodobot_serial::println_info("level: %s, #%d", dodobot_breakout::level_config.c_str(), current_level_index);
        }

        void on_unload()  {
            set_loaded(false);
            UI_DELAY_MS = UI_DELAY_MS_DEFAULT;
            dodobot_breakout::on_unload();
            tft.fillScreen(ST77XX_BLACK);
        }

        void on_up()  // load previous level
        {
            RETURN_IF_NOT_LOADED;
            int index = current_level_index - 1;
            if (index < 0) {
                index = 0;
            }
            load_level(index);
            dodobot_breakout::enter_event();
        }
        void on_down()  // load next level
        {
            RETURN_IF_NOT_LOADED;
            int index = current_level_index + 1;
            if (index >= (int)dodobot_sd::num_breakout_files) {
                index = dodobot_sd::num_breakout_files - 1;
            }
            load_level(index);
            dodobot_breakout::enter_event();
        }
        void on_left() {
            RETURN_IF_NOT_LOADED;
            dodobot_breakout::left_event();
        }
        void on_right() {
            RETURN_IF_NOT_LOADED;
            dodobot_breakout::right_event();
        }
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("main-menu");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            load_level(-1);  // load random level
            dodobot_breakout::enter_event();
        }
        void on_repeat(EventType e) {
            RETURN_IF_NOT_LOADED;
            dodobot_breakout::repeat_key_event();
        }
        void on_numpad(int number)
        {
            RETURN_IF_NOT_LOADED;
            load_level(number);
            dodobot_breakout::on_load();
        }
    };

    bool wifi_is_on = true;
    void toggle_wifi_callback(String name, int index)
    {
        String text = "Wifi ";
        if (wifi_is_on) {
            text += "off";
        }
        else {
            text += "on";
        }
        notify(INFO, text, 1000);
        dodobot_serial::info->write("network", "dd", 0, !wifi_is_on);
    }

    bool hotspot_is_on = true;
    void toggle_hotspot_callback(String name, int index)
    {
        String text = "Hotspot ";
        if (hotspot_is_on) {
            text += "off";
        }
        else {
            text += "on";
        }
        notify(INFO, text, 1000);
        dodobot_serial::info->write("network", "dd", 3, !hotspot_is_on);
    }

    void load_connect_screen(String name, int index)
    {
        if (wifi_is_on) {
            load("connect-network");
        }
        else {
            notify(WARN, "Wifi is off", 1000);
        }
    }

    void refresh_network_info()
    {
        dodobot_serial::println_info("Requesting network info refresh");
        dodobot_serial::info->write("network", "d", 1);
    }
    void refresh_network_info_callback(String name, int index) {
        refresh_network_info();
    }

    class NetworkScreenController : public ViewWithOverlayController
    {
    public:
        NetworkScreenController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            menu = new ScrollingMenu();
            menu->add_entry("Refresh", refresh_network_info_callback);
            menu->add_entry("Connect", load_connect_screen);
            wifi_entry_index = menu->add_entry("Wifi off", toggle_wifi_callback);
            hotspot_entry_index = menu->add_entry("Host off", toggle_hotspot_callback);
        }

        void set_wifi_state(bool wifi_state)
        {
            wifi_is_on = wifi_state;
            if (wifi_is_on) {
                menu->set_entry(wifi_entry_index, "Wifi off");
            }
            else {
                menu->set_entry(wifi_entry_index, "Wifi on");
            }
            RETURN_IF_NOT_LOADED;
            menu->draw();
            draw_network_info();
        }

        void set_hotspot_state(bool hotspot_state)
        {
            hotspot_is_on = hotspot_state;
            if (hotspot_is_on) {
                menu->set_entry(hotspot_entry_index, "Host off");
            }
            else {
                menu->set_entry(hotspot_entry_index, "Host on");
            }
            RETURN_IF_NOT_LOADED;
            menu->draw();
            draw_network_info();
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()  {}

        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
            refresh_network_info();
            int border = 3;
            menu->x0 = border;
            menu->y0 = topbar()->get_height() + border + 1;
            menu->menu_w = 52;
            menu->entry_h = 15;
            menu->on_load();
            draw_network_info();
        }

        void draw_network_info()
        {
            RETURN_IF_NOT_LOADED;
            int16_t text_x = menu->menu_w + menu->x0;
            int16_t text_y = menu->y0;
            int prev_index = -1;
            int str_index = 0;
            int str_len = dodobot::network_info.length();

            tft.fillRect(text_x, text_y, tft.width() - text_x, tft.height() - text_y, ST77XX_BLACK);
            while (str_index < str_len)
            {
                int next_index = dodobot::network_info.indexOf('\n', str_index);
                if (next_index == -1) {
                    next_index = str_len;
                }
                String line = dodobot::network_info.substring(str_index, next_index);
                str_index = next_index + 1;
                if (str_index == prev_index) {
                    return;
                }
                prev_index = str_index;

                tft.setCursor(text_x, text_y);
                tft.print(line);
                text_y += info_row_size;
            }
        }

        void on_unload_with_overlay() {
            menu->on_unload();
        }
        void on_up() {
            RETURN_IF_NOT_LOADED;
            menu->on_up();
            draw_network_info();
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            menu->on_down();
            draw_network_info();
        }
        void on_left()  {}
        void on_right()  {}
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("main-menu");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            menu->on_enter();
        }
        void on_numpad(int number) {
            RETURN_IF_NOT_LOADED;
            if (number == 0) {
                number = 10;
            }
            menu->select(number - 1);
            draw_network_info();
        }
    private:
        ScrollingMenu* menu;
        int wifi_entry_index = 0;
        int hotspot_entry_index = 0;
        int16_t info_row_size = 10;
    };

    void request_robot_function(String name, int index)
    {
        if (name.length() == 0) {
            return;
        }
        dodobot_serial::println_info("Requesting robot function: %s", name.c_str());
        dodobot_serial::data->write("robotfn", "s", name.c_str());
    }

    class RobotScreenController : public ViewWithOverlayController
    {
    public:
        RobotScreenController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            menu = new ScrollingMenu();
            for (int index = 0; index < dodobot::max_robot_fns_len; index++) {
                menu->add_entry("", request_robot_function);
            }
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()  {}
        void on_load_with_overlay()
        {
            menu->x0 = border;
            menu->y0 = topbar()->get_height() + border;
            menu->menu_w = menu_w_selected;
            menu->entry_h = 11;

            menu->on_load();
            draw_list();
        }
        void on_unload_with_overlay() {
            menu->on_unload();
        }
        void on_up() {
            RETURN_IF_NOT_LOADED;
            menu->on_up();
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            menu->on_down();
        }
        void on_left()
        {
            RETURN_IF_NOT_LOADED;

            if (menu->get_selected() == -2) {
                menu->select(prev_selected);
            }
            menu->menu_w = menu_w_selected;
            draw_list();
        }
        void on_right()
        {
            RETURN_IF_NOT_LOADED;

            if (menu->get_selected() != -2) {
                prev_selected = menu->get_selected();
                menu->select(-2);
            }
            menu->menu_w = menu_w_deselected;
            draw_list();
        }

        void on_enter() {
            RETURN_IF_NOT_LOADED;
            menu->on_enter();
        }
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("main-menu");
        }

        void on_numpad(int number) {
            RETURN_IF_NOT_LOADED;
            if (number == 0) {
                number = 10;
            }
            menu->select(number - 1);
        }


        void set_blank_space(int index, int blank_space) {
            menu->add_blank(blank_space, index);
        }

        void reset_list() {
            menu->remove_all_blanks();
        }

        void draw_list()
        {
            int stop_index = min(dodobot::robot_fn_list_len, dodobot::max_robot_fns_len);
            for (int index = 0; index < stop_index; index++) {
                String text = dodobot::robot_functions[index];
                menu->set_entry(index, text.c_str());
            }

            RETURN_IF_NOT_LOADED;
            topbar()->fillBottomScreen();
            menu->draw();
            draw_map();
        }

    private:
        int16_t border = 5;
        int16_t menu_w_selected = 120;
        int16_t menu_w_deselected = 50;

        int16_t map_x, map_y;
        uint16_t map_w, map_h;

        int prev_selected = 0;

        void draw_map()
        {
            map_x = menu->x0 + menu->menu_w;
            map_y = topbar()->get_height();
            map_w = tft.width() - map_x;
            map_h = tft.height() - map_y;
            tft.fillRect(map_x, map_y, map_w, map_h, ST77XX_GRAY);
        }

        ScrollingMenu* menu;
    };

    void load_system_screen(String name, int index) {
        dodobot_serial::println_info("Loading %s", name);
        switch (index) {
            case 0:  load("drive-system");  break;
            case 1:  load("linear-system");  break;
            case 2:  load("gripper-system");  break;
            case 3:  load("camera-system");  break;
            case 4:  load("info-system");  break;
        }
    }

    class SystemScreenController : public ViewWithOverlayController
    {
    public:
        SystemScreenController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            menu = new ScrollingMenu();
            menu->show_numbers = true;
            menu->add_entry("Drive motors", load_system_screen);
            menu->add_entry("Linear stepper", load_system_screen);
            menu->add_entry("Gripper", load_system_screen);
            menu->add_entry("Camera", load_system_screen);
            menu->add_entry("System info", load_system_screen);
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay() {
        }
        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
            int border = 5;
            menu->x0 = border;
            menu->y0 = topbar()->get_height() + border;
            menu->menu_w = tft.width() - menu->x0 - border;
            menu->entry_h = 15;
            menu->on_load();
        }

        void on_unload_with_overlay() {
            menu->on_unload();
        }
        void on_up() {
            RETURN_IF_NOT_LOADED;
            menu->on_up();
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            menu->on_down();
        }
        void on_left()  {}
        void on_right()  {}
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("main-menu");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            menu->on_enter();
        }
        void on_numpad(int number)
        {
            RETURN_IF_NOT_LOADED;
            if (number == 0) {
                number = 10;
            }
            menu->select(number - 1);
        }

    private:
        ScrollingMenu* menu;
    };

    void shutdown_screen_callback(String name, int index)
    {
        switch (index) {
            case 0:  notify(INFO, "Shutting down", 1000);  dodobot::signal_shutdown();  break;
            case 1:  notify(INFO, "Rebooting", 1000);  dodobot::signal_reboot();  break;
            case 2:  notify(INFO, "Relaunch ROS", 1000);  dodobot::signal_restart_ros();  break;
            case 3:  notify(INFO, "Relaunch client", 1000);  dodobot::signal_restart_client();  break;
            case 4:  notify(INFO, "Relaunch system", 1000);  dodobot::signal_restart_microcontroller();  break;
        }
    }

    class ShutdownScreenController : public ViewWithOverlayController
    {
    public:
        ShutdownScreenController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            menu = new ScrollingMenu();
            menu->show_numbers = true;
            menu->add_entry("Shutdown", shutdown_screen_callback);
            menu->add_entry("Reboot", shutdown_screen_callback);
            menu->add_entry("Relaunch ROS", shutdown_screen_callback);
            menu->add_entry("Relaunch client", shutdown_screen_callback);
            menu->add_entry("Relaunch system", shutdown_screen_callback);
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()  {}
        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
            int border = 5;
            menu->x0 = border;
            menu->y0 = topbar()->get_height() + border;
            menu->menu_w = tft.width() - menu->x0 - border;
            menu->entry_h = 15;
            menu->on_load();
        }

        void on_unload_with_overlay() {
            menu->on_unload();
        }
        void on_up() {
            RETURN_IF_NOT_LOADED;
            menu->on_up();
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            menu->on_down();
        }
        void on_left()  {}
        void on_right()  {}
        void on_back()  {
            load("main-menu");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            menu->on_enter();
        }
        void on_numpad(int number) {
            RETURN_IF_NOT_LOADED;
            if (number == 0) {
                number = 10;
            }
            menu->select(number - 1);
        }
    private:
        ScrollingMenu* menu;
    };

    const int zero_enc_seq_len = 2;
    int zero_enc_sequence[breakout_seq_len] = {0, 0};

    class DriveSystemController : public ViewWithOverlayController
    {
    public:
        DriveSystemController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            number_sequence = new NumberSequence(num_seq_len);
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()
        {
            int y_offset = topbar()->get_height() + 5;
            tft.setCursor(border_offset_w, y_offset); tft.println("A: " + String(dodobot_chassis::encA_pos) + "       ");
            tft.setCursor(border_offset_w + drive_column_offset, y_offset); tft.println("B: " + String(dodobot_chassis::encB_pos) + "       "); y_offset += row_size * 2;

            tft.setCursor(border_offset_w, y_offset); tft.println("c A: " + String(dodobot_chassis::motorA.getSpeed()) + "       ");
            tft.setCursor(border_offset_w + drive_column_offset, y_offset); tft.println("B: " + String(dodobot_chassis::motorB.getSpeed()) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("t A: " + String((int)dodobot_speed_pid::motorA_pid.get_target()) + "       ");
            tft.setCursor(border_offset_w + drive_column_offset, y_offset); tft.println("B: " + String((int)dodobot_speed_pid::motorB_pid.get_target()) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("m A: " + String((int)dodobot_chassis::enc_speedA) + "       ");
            tft.setCursor(border_offset_w + drive_column_offset, y_offset); tft.println("B: " + String((int)dodobot_chassis::enc_speedB) + "       "); y_offset += row_size;

            tft.setCursor(border_offset_w, y_offset); tft.println("L: " + String(dodobot_chassis::is_bump2_active()) + "       ");
            tft.setCursor(border_offset_w + drive_column_offset, y_offset); tft.println("R: " + String(dodobot_chassis::is_bump1_active()) + "       "); y_offset += row_size;
        }
        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
        }

        void on_unload_with_overlay()  {}
        void on_up() {
            RETURN_IF_NOT_LOADED;
            drive_robot_forward(3000.0);
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            drive_robot_forward(-3000.0);
        }
        void on_left() {
            RETURN_IF_NOT_LOADED;
            rotate_robot(-2000.0);
        }
        void on_right() {
            RETURN_IF_NOT_LOADED;
            rotate_robot(2000.0);
        }
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("system");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            drive_robot_forward(0.0);
        }
        void on_numpad(int number)
        {
            RETURN_IF_NOT_LOADED;
            dodobot_serial::println_info("number: %d", number);
            if (number_sequence->full()) {
                number_sequence->deque();
            }
            number_sequence->queue(number);
            if (number_sequence->check_sequence(zero_enc_sequence, zero_enc_seq_len)) {
                for (int i = 0; i < num_seq_len; i++) {
                    number_sequence->deque();
                }
                // zero encoders
                dodobot_chassis::reset_encoders();
            }
        }

        void drive_robot_forward(double speed_tps)   // ticks per s
        {
            dodobot_speed_pid::update_setpointA(speed_tps);
            dodobot_speed_pid::update_setpointB(speed_tps);
        }

        void rotate_robot(double speed_tps)   // ticks per s
        {
            dodobot_speed_pid::update_setpointA(speed_tps);
            dodobot_speed_pid::update_setpointB(-speed_tps);
        }
    private:
        int16_t border_offset_w = 3;
        int16_t drive_column_offset = 80;
        uint16_t row_size = 10;
        double speed_tps = 3000.0;

        int num_seq_len = 10;
        NumberSequence* number_sequence;
    };

    class LinearSystemController : public ViewWithOverlayController
    {
    public:
        LinearSystemController(ViewController* topbar) : ViewWithOverlayController(topbar) {

        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()
        {
            int y_offset = topbar()->get_height() + 5;
            tft.setCursor(border_offset_w, y_offset); tft.println("step: " + String(dodobot_linear::stepper_pos) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("enc: " + String(dodobot_linear::raw_encoder_pos) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("enc as step: " + String(dodobot_linear::encoder_pos) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("diff: " + String(dodobot_linear::stepper_pos - dodobot_linear::encoder_pos) + ", " + String(dodobot_linear::ENCODER_POSITION_ERROR) + "       "); y_offset += row_size;

            tft.setCursor(border_offset_w, y_offset); tft.println("linear pos mm: " + String(dodobot_linear::to_linear_pos(dodobot_linear::stepper_pos)) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("vel: " + String(dodobot_linear::stepper_vel) + "            "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("error: " + String(dodobot_linear::is_errored()) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("moving: " + String(dodobot_linear::is_moving) + "       "); y_offset += row_size;

            tft.setCursor(border_offset_w, y_offset);
            switch (dodobot_linear::planning_mode) {
                case TicPlanningMode::Off: tft.println("mode: Off       "); break;
                case TicPlanningMode::TargetPosition: tft.println("mode: Position       "); break;
                case TicPlanningMode::TargetVelocity: tft.println("mode: Velocity       "); break;
            }
            y_offset += row_size;

            tft.setCursor(border_offset_w, y_offset); tft.println("homed: " + String(dodobot_linear::is_homed) + "       "); y_offset += row_size;
            // tft.setCursor(border_offset_w, y_offset); tft.println("microstep: " + String(dodobot_linear::microsteps) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("limit: " + String(dodobot_linear::is_home_pin_active()) + "       "); y_offset += row_size;
        }
        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
        }

        void on_unload_with_overlay()  {}

        void on_up() {
            RETURN_IF_NOT_LOADED;
            dodobot_linear::set_position(dodobot_linear::target_position + 625 * dodobot_linear::microsteps);
        }

        void on_down() {
            RETURN_IF_NOT_LOADED;
            dodobot_linear::set_position(dodobot_linear::target_position - 625 * dodobot_linear::microsteps);
        }
        void on_left()  {}
        void on_right()  {}
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("system");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            dodobot_linear::home_stepper();
        }
    private:
        int16_t border_offset_w = 3;
        uint16_t row_size = 8;
    };

    class GripperSystemController : public ViewWithOverlayController
    {
    public:
        GripperSystemController(ViewController* topbar) : ViewWithOverlayController(topbar) {

        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()
        {
            int y_offset = topbar()->get_height() + 5;
            tft.setCursor(border_offset_w, y_offset); tft.println("L: " + String(dodobot_gripper::get_left_fsr()) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("R: " + String(dodobot_gripper::get_right_fsr()) + "       "); y_offset += row_size;
            tft.setCursor(border_offset_w, y_offset); tft.println("pos: " + String(dodobot_gripper::gripper_pos) + "       "); y_offset += row_size;
        }
        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
        }

        void on_unload_with_overlay()  {}
        void on_up() {
            RETURN_IF_NOT_LOADED;
            dodobot_gripper::close_gripper(100, dodobot_gripper::gripper_pos + 10);
        }

        void on_down() {
            RETURN_IF_NOT_LOADED;
            dodobot_gripper::open_gripper(dodobot_gripper::gripper_pos - 10);
        }

        void on_left() {
            RETURN_IF_NOT_LOADED;
            dodobot_gripper::open_gripper(dodobot_gripper::gripper_pos - 1);
        }
        void on_right() {
            RETURN_IF_NOT_LOADED;
            dodobot_gripper::close_gripper(100, dodobot_gripper::gripper_pos + 1);
        }
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("system");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            dodobot_gripper::toggle_gripper(100);
        }
    private:
        int16_t border_offset_w = 3;
        uint16_t row_size = 10;
    };

    class CameraSystemController : public ViewWithOverlayController
    {
    public:
        CameraSystemController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            x0 = 0;
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()
        {
            int y_offset = topbar()->get_height() + 5;
            // tft.setTextColor(ST77XX_BLACK, ST77XX_BLACK);
            tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
            tft.setCursor(border_offset_w, y_offset); tft.println("pos: " + String(dodobot_tilter::tilter_pos) + "  "); y_offset += row_size;


            if (!image_updated) {
                return;
            }
            image_updated = false;
            dodobot_sd::drawJPEG("CAMERA.JPG", x0, y0);
        }

        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();
            y0 = topbar()->get_height();
            on_update_image();
            DODOBOT_SERIAL_WRITE_BOTH("recvimage", "d", 1);
        }

        void on_unload_with_overlay() {
            DODOBOT_SERIAL_WRITE_BOTH("recvimage", "d", 0);
        }
        void on_up() {
            RETURN_IF_NOT_LOADED;
            dodobot_tilter::set_tilter(dodobot_tilter::tilter_pos + 40);
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            dodobot_tilter::set_tilter(dodobot_tilter::tilter_pos - 40);
        }
        void on_left() {
            RETURN_IF_NOT_LOADED;
            dodobot_tilter::set_tilter(dodobot_tilter::tilter_pos + 5);
        }
        void on_right() {
            RETURN_IF_NOT_LOADED;
            dodobot_tilter::set_tilter(dodobot_tilter::tilter_pos - 5);
        }
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("system");
        }
        void on_enter() {
            RETURN_IF_NOT_LOADED;
            dodobot_tilter::tilter_toggle();
        }
        void on_update_image() {
            image_updated = true;
        }
        void on_file(String filename)
        {
            if (img_filename.equals(filename)) {
                on_update_image();
            }
        }

    private:
        bool image_updated = false;
        int16_t x0, y0;
        int16_t border_offset_w = 3;
        uint16_t row_size = 10;
        String img_filename = "CAMERA.JPG";
    };

    class InfoSystemController : public ViewWithOverlayController
    {
    public:
        InfoSystemController(ViewController* topbar) : ViewWithOverlayController(topbar) {

        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay() {
            tft.setCursor(0, topbar()->get_height() + 5);
            tft.print("Serial: " + ROBOT_NAME); tft.print("\n");
            tft.print("Author: " + ROBOT_AUTHOR); tft.print("\n");
            tft.print(DODOBOT_VERSION); tft.print("\n");
            dodobot_sd::drawGIFframe();
        }
        void on_load_with_overlay()
        {
            UI_DELAY_MS = 110;
            topbar()->fillBottomScreen();
            dodobot_sd::loadGIF("CHANSEY.GIF");
            dodobot_sd::setGIFoffset(
                tft.width() - dodobot_sd::gif.getCanvasWidth(),
                tft.height() - dodobot_sd::gif.getCanvasHeight());
        }

        void on_unload_with_overlay() {
            UI_DELAY_MS = UI_DELAY_MS_DEFAULT;
        }
        void on_up()  {}
        void on_down()  {}
        void on_left()  {}
        void on_right()  {}
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            dodobot_sd::closeGIF();
            load("system");
        }
        void on_enter()  {}
    private:
    };

    void refresh_network_list()
    {
        dodobot_serial::println_info("Requesting network list refresh");
        dodobot_serial::info->write("network", "d", 2);
    }

    void refresh_network_list_callback(String name, int index) {
        refresh_network_list();
    }

    void request_network_connect(String name, int index)
    {
        if (name.length() == 0) {
            return;
        }
        if (name.charAt(0) == '*') {
            notify(INFO, "Already connected", 1000);
            return;
        }

        // dodobot_serial::println_info("Requesting network connect to %s", name.c_str());
        // dodobot_serial::info->write("network", "ds", 3, name.c_str());
    }

    class ConnectNetworkController : public ViewWithOverlayController
    {
    public:
        ConnectNetworkController(ViewController* topbar) : ViewWithOverlayController(topbar) {
            menu = new ScrollingMenu();
            menu->add_entry("Refresh", refresh_network_list_callback);
            menu->add_blank(6);
            for (int index = 0; index < dodobot::max_networks_len; index++) {
                menu->add_entry("", request_network_connect);
            }
        }

        TopbarController* topbar() {
            return (TopbarController*) overlay;
        }
        void draw_with_overlay()  {}
        void on_load_with_overlay()
        {
            topbar()->fillBottomScreen();

            int border = 5;
            menu->x0 = border;
            menu->y0 = topbar()->get_height() + border;
            menu->menu_w = tft.width() - menu->x0;
            menu->entry_h = 11;

            refresh_network_list();
            menu->on_load();
        }

        void draw_list()
        {
            RETURN_IF_NOT_LOADED;

            int stop_index = min(dodobot::network_list_len, dodobot::max_networks_len);
            for (int index = 0; index < stop_index; index++) {
                String text = dodobot::network_list_names[index];
                // int bars = dodobot::network_list_signals[index];
                menu->set_entry(index + 1, text.c_str());
            }

            menu->draw();
        }

        // String bars_to_str(int bars) {
        //
        // }

        void on_unload_with_overlay() {
            menu->on_unload();
        }
        void on_up() {
            RETURN_IF_NOT_LOADED;
            menu->on_up();
        }
        void on_down() {
            RETURN_IF_NOT_LOADED;
            menu->on_down();
        }
        void on_left()  {}
        void on_right()  {}
        void on_back()  {
            RETURN_IF_NOT_LOADED;
            load("network");
        }
        void on_enter() {
            menu->on_enter();
        }
        void on_numpad(int number) {
            RETURN_IF_NOT_LOADED;
            if (number == 0) {
                number = 10;
            }
            menu->select(number - 1);
        }

        void on_keyboard(EventType event, char c) {

        }

    private:
        ScrollingMenu* menu;
    };


    void add_view(String name, ViewController* view)
    {
        names[num_views] = name;
        views[num_views++] = view;
    }

    ViewController* view_from_name(String name) {
        for (size_t i = 0; i < num_views; i++) {
            if (names[i].equals(name)) {
                return views[i];
            }
        }
        return NULL;
    }

    void load(String name)
    {
        if (current_view != NULL) {
            current_view->on_unload();
        }
        current_view = view_from_name(name);

        if (current_view != NULL) {
            dodobot_serial::println_info("Loading view %s", name.c_str());
            current_view->on_load();
        }
        else {
            dodobot_serial::println_error("Loaded view %s is NULL", name.c_str());
        }
    }

    SplashScreenController* splash = new SplashScreenController();
    TopbarController* topbar = new TopbarController();
    MainMenuController* main_menu = new MainMenuController(topbar);
    NetworkScreenController* network_screen = new NetworkScreenController(topbar);
    RobotScreenController* robot_screen = new RobotScreenController(topbar);
    SystemScreenController* system_screen = new SystemScreenController(topbar);
    ShutdownScreenController* shutdown_screen = new ShutdownScreenController(topbar);
    BreakoutController* breakout_screen = new BreakoutController;
    DriveSystemController* drive_system_screen = new DriveSystemController(topbar);
    LinearSystemController* linear_system_screen = new LinearSystemController(topbar);
    GripperSystemController* gripper_system_screen = new GripperSystemController(topbar);
    CameraSystemController* camera_system_screen = new CameraSystemController(topbar);
    InfoSystemController* info_system_screen = new InfoSystemController(topbar);
    ConnectNetworkController* connect_network_screen = new ConnectNetworkController(topbar);

    void init()
    {
        add_view("splash", splash);

        add_view("main-menu", main_menu);
        add_view(main_menu->view_name(NETWORK), network_screen);
        add_view(main_menu->view_name(ROBOT), robot_screen);
        add_view(main_menu->view_name(SYSTEM), system_screen);
        add_view(main_menu->view_name(SHUTDOWN), shutdown_screen);

        add_view("breakout", breakout_screen);

        add_view("drive-system", drive_system_screen);
        add_view("linear-system", linear_system_screen);
        add_view("gripper-system", gripper_system_screen);
        add_view("camera-system", camera_system_screen);
        add_view("info-system", info_system_screen);

        add_view("connect-network", connect_network_screen);

        load("splash");
        dodobot_serial::println_info("UI ready");
    }

    void dim_display() {
        dodobot_display::set_display_brightness(0);
    }

    void wake_display() {
        dodobot_display::set_display_brightness(255);
    }

    void reset_display_brightness_timer() {
        ui_sleep_brightness_timer = CURRENT_TIME;
    }

    void draw()
    {
        if (CURRENT_TIME - ui_timer < UI_DELAY_MS) {
            return;
        }
        ui_timer = CURRENT_TIME;

        if (CURRENT_TIME - ui_sleep_brightness_timer > UI_BRIGHTNESS_TIMEOUT) {
            dim_display();
        }
        else {
            wake_display();
        }

        if (current_view == NULL)  return;
        current_view->draw();
    }

    void notify(NotificationLevel level, String text, uint32_t timeout) {
        topbar->notify(level, text, timeout);
    }

    EventType last_event;

    void on_up() {
        if (current_view == NULL)  return;
        current_view->on_up();
        last_event = UP_EVENT;
    }

    void on_down() {
        if (current_view == NULL)  return;
        current_view->on_down();
        last_event = DOWN_EVENT;
    }

    void on_left() {
        if (current_view == NULL)  return;
        current_view->on_left();
        last_event = LEFT_EVENT;
    }

    void on_right() {
        if (current_view == NULL)  return;
        current_view->on_right();
        last_event = RIGHT_EVENT;
    }

    void on_back() {
        if (current_view == NULL)  return;
        current_view->on_back();
        last_event = BACK_EVENT;
    }

    void on_enter() {
        if (current_view == NULL)  return;
        current_view->on_enter();
        last_event = ENTER_EVENT;
    }

    void on_repeat() {
        if (current_view == NULL)  return;
        current_view->on_repeat(last_event);
    }

    void on_numpad(int number) {
        if (current_view == NULL)  return;
        current_view->on_numpad(number);
        switch (number) {
            case 0: last_event = EVENT_0; break;
            case 1: last_event = EVENT_1; break;
            case 2: last_event = EVENT_2; break;
            case 3: last_event = EVENT_3; break;
            case 4: last_event = EVENT_4; break;
            case 5: last_event = EVENT_5; break;
            case 6: last_event = EVENT_6; break;
            case 7: last_event = EVENT_7; break;
            case 8: last_event = EVENT_8; break;
            case 9: last_event = EVENT_9; break;
        }
    }

    void on_keyboard(int event_code, char key)
    {
        if (current_view == NULL)  return;
        dodobot_ui::EventType event;
        switch (event_code) {
            case 0:  event = dodobot_ui::KEYBOARD_UP;  break;
            case 1:  event = dodobot_ui::KEYBOARD_DOWN;  break;
            case 2:  event = dodobot_ui::KEYBOARD_HOLD;  break;
            default:  event = dodobot_ui::NONE_EVENT;  break;
        }
        if (event == dodobot_ui::NONE_EVENT) {
            return;
        }

        current_view->on_keyboard(event, key);
        last_event = event;
    }

    void on_file(String filename)
    {
        if (current_view == NULL)  return;
        current_view->on_file(filename);
        last_event = FILE_EVENT;
    }
}
