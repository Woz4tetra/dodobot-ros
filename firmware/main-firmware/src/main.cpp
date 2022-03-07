#include <Arduino.h>
#include "dodobot.h"
#include "ir-remote-dodobot.h"
#include "i2c-dodobot.h"
#include "power-monitor-dodobot.h"
#include "display-dodobot.h"
#include "gripper-dodobot.h"
#include "tilter-dodobot.h"
#include "linear-dodobot.h"
#include "chassis-dodobot.h"
#include "speed-pid-dodobot.h"
#include "latch-circuit-dodobot.h"
#include "ui-dodobot.h"
#include "breakout-dodobot.h"
#include "sd-dodobot.h"


void set_active(bool state)
{
    dodobot::set_motors_active(state);
    dodobot_gripper::set_active(state);
    dodobot_tilter::set_active(state);
    dodobot_chassis::set_active(state);
    dodobot_speed_pid::set_speed_pid(state);
    dodobot_linear::set_active(state);
}

void set_ros_ready(bool state)
{
    dodobot::robot_state.is_ros_ready = state;
    dodobot_ui::main_menu->draw_all();
}

void dodobot_latch_circuit::shutdown_callback() {
    set_active(false);
}

void homing_routine()
{
    dodobot_chassis::stop_motors();
    dodobot_speed_pid::reset_pid();
    dodobot_linear::home_stepper();
    dodobot_serial::flush_read();
}

void dodobot_linear::send_event_callback(LinearEvent event) {
    switch (event) {
        case dodobot_linear::LinearEvent::HOMING_FAILED:  dodobot_ui::notify(dodobot_ui::ERROR, "Home failed", 5000); break;
        case dodobot_linear::LinearEvent::NOT_ACTIVE:  dodobot_ui::notify(dodobot_ui::ERROR, "Not active", 5000); break;
        case dodobot_linear::LinearEvent::NOT_HOMED:  dodobot_ui::notify(dodobot_ui::ERROR, "Not homed", 5000); break;
        case dodobot_linear::LinearEvent::POSITION_ERROR:  dodobot_ui::notify(dodobot_ui::ERROR, "Position error", 5000); break;
        default: break;
    }
}


void dodobot_serial::packet_callback(DodobotSerial* serial_obj, String category)
{
    // dodobot_serial::println_info("category: %s, packet #%d", category.c_str(), (int)serial_obj->get_read_packet_num());

    // get_ready
    if (category.equals("?")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, -1);
        if (serial_obj->get_segment().equals("dodobot")) {
            dodobot_serial::println_info("Received ready signal!");
            DODOBOT_SERIAL_WRITE_BOTH("ready", "us", CURRENT_TIME, ROBOT_NAME.c_str());
        }
        else {
            dodobot_serial::println_error("Invalid ready segment supplied: %s", serial_obj->get_segment().c_str());
        }
    }

    // toggle reporting
    else if (category.equals("[]")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, 4);
        int reporting_state = serial_obj->segment_as_int32();
        dodobot_serial::println_info("toggle_reporting %d", reporting_state);
        switch (reporting_state)
        {
            case 0: dodobot::robot_state.is_reporting_enabled = false; break;
            case 1: dodobot::robot_state.is_reporting_enabled = true; break;
            // case 2: reset(); break;
            default:
                dodobot_serial::println_error("Invalid reporting flag received: %d", reporting_state);
                break;
        }
    }

    // toggle motors active
    else if (category.equals("<>")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, 4);
        int active_state = serial_obj->segment_as_int32();
        // dodobot_serial::println_info("toggle_active %d", active_state);
        switch (active_state)
        {
            case 0: set_active(false); break;
            case 1: set_active(true); break;
            // case 2: dodobot::soft_restart(); break;
            default:
                break;
        }
    }

    // ROS ready flag
    else if (category.equals("ros")) {
        CHECK_SEGMENT(serial_obj, 4);  int ros_state = serial_obj->segment_as_int32();
        dodobot_serial::println_info("ROS ready state %d", ros_state);
        switch (ros_state)
        {
            case 0: set_ros_ready(false); break;
            case 1: set_ros_ready(true); break;
            default:
                break;
        }
    }

    // set gripper
    else if (category.equals("grip")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, 4);
        int gripper_state = serial_obj->segment_as_int32();
        int grip_threshold = -1;
        int pos_threshold = 0;
        switch (gripper_state)
        {
            case 0:
                if (serial_obj->next_segment(4)) {
                    pos_threshold = serial_obj->segment_as_int32();
                    dodobot_gripper::open_gripper(pos_threshold);
                }
                else {
                    dodobot_gripper::open_gripper();
                }
                break;
            case 1:
                CHECK_SEGMENT(serial_obj, 4);  grip_threshold = serial_obj->segment_as_int32();
                if (serial_obj->next_segment(4)) {
                    pos_threshold = serial_obj->segment_as_int32();
                    dodobot_gripper::close_gripper(grip_threshold, pos_threshold);
                }
                else {
                    dodobot_gripper::close_gripper(grip_threshold);
                }
                break;
            case 2:
                CHECK_SEGMENT(serial_obj, 4);  grip_threshold = serial_obj->segment_as_int32();
                dodobot_gripper::toggle_gripper(grip_threshold);
                break;
            default:
                break;
        }
    }

    // gripper settings
    else if (category.equals("gripcfg")) {
        CHECK_SEGMENT(serial_obj, 4); int open_pos = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, 4); int close_pos = serial_obj->segment_as_int32();
        dodobot_gripper::set_limits(open_pos, close_pos);
        dodobot_serial::println_info("Setting gripper limits: %d, %d", open_pos, close_pos);
    }

    // set tilter
    else if (category.equals("tilt")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, 4);
        int tilt_state = serial_obj->segment_as_int32();
        int tilt_pos = 0;
        switch (tilt_state)
        {
            case 0: dodobot_tilter::tilter_up(); break;
            case 1: dodobot_tilter::tilter_down(); break;
            case 2: dodobot_tilter::tilter_toggle(); break;
            case 3:
                CHECK_SEGMENT(serial_obj, 4);  tilt_pos = serial_obj->segment_as_int32();
                dodobot_tilter::set_tilter(tilt_pos);
                break;
            default:
                break;
        }
    }

    // set linear
    else if (category.equals("linear")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, 4);
        int linear_state = serial_obj->segment_as_int32();
        int linear_value = 0;
        switch (linear_state)
        {
            case 0:
                CHECK_SEGMENT(serial_obj, 4);  linear_value = serial_obj->segment_as_int32();
                dodobot_linear::set_position(linear_value);
                break;
            case 1:
                CHECK_SEGMENT(serial_obj, 4);  linear_value = serial_obj->segment_as_int32();
                dodobot_linear::set_velocity(linear_value);
                break;
            case 2: dodobot_linear::stop(); break;
            case 3: dodobot_linear::reset(); break;
            case 4: homing_routine(); break;
            default:
                break;
        }
    }

    // set linear configuration settings
    else if (category.equals("lincfg")) {
        CHECK_SEGMENT(serial_obj, 4); int config_type = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, 4); int value = serial_obj->segment_as_int32();

        switch (config_type) {
            case 0:  dodobot_linear::set_max_speed(value); break;
            case 1:  dodobot_linear::set_accel(value); break;
            default:
                break;
        }
    }

    // set pid ks
    else if (category.equals("ks")) {
        float k_value = 0;
        size_t index = 0;
        for (index = 0; index < dodobot_speed_pid::NUM_PID_KS; index++) {
            CHECK_SEGMENT_BREAK(serial_obj, 4); k_value = serial_obj->segment_as_float();
            dodobot_speed_pid::pid_Ks[index] = k_value;
            dodobot_serial::println_info("Set k %d: %.2f", index, k_value);
        }
        if (index == dodobot_speed_pid::NUM_PID_KS) {
            dodobot_speed_pid::set_Ks();
        }
        else {
            dodobot_speed_pid::failed_to_set_ks();
        }
    }

    // set chassis speed
    else if (category.equals("drive")) {
        dodobot_ui::reset_display_brightness_timer();
        CHECK_SEGMENT(serial_obj, 4); float setpointA = serial_obj->segment_as_float();
        CHECK_SEGMENT(serial_obj, 4); float setpointB = serial_obj->segment_as_float();
        dodobot_speed_pid::update_setpointA(setpointA);
        dodobot_speed_pid::update_setpointB(setpointB);
    }

    // shutdown signal
    else if (category.equals("shutdown")) {
        CHECK_SEGMENT(serial_obj, -1);
        if (serial_obj->get_segment().equals("dodobot")) {
            dodobot_serial::println_info("Received shutdown signal!");
            dodobot_latch_circuit::shutdown();
        }
        else {
            dodobot_serial::println_error("Invalid shutdown segment supplied: %s", serial_obj->get_segment().c_str());
        }
    }

    else if (category.equals("date")) {
        CHECK_SEGMENT(serial_obj, -1);
        dodobot_ui::update_date(serial_obj->get_segment());
    }

    else if (category.equals("network"))
    {
        CHECK_SEGMENT(serial_obj, 4);  int32_t wifi_state = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, 4);  int32_t hotspot_state = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, -1); dodobot::network_info = serial_obj->get_segment();
        dodobot_serial::println_info("Received network info. Wifi state: %d. Hotspot state: %d", wifi_state, hotspot_state);
        dodobot_ui::network_screen->draw_network_info();
        dodobot_ui::network_screen->set_wifi_state((bool)wifi_state);
        dodobot_ui::network_screen->set_hotspot_state((bool)hotspot_state);
    }
    else if (category.equals("netlist"))
    {
        CHECK_SEGMENT(serial_obj, 4);  int32_t index = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, -1); String network_name = String(serial_obj->get_segment());
        CHECK_SEGMENT(serial_obj, 4);  int32_t signal_strength = serial_obj->segment_as_int32();
        dodobot_serial::println_info("Received network list info: %s, %d", network_name.c_str(), signal_strength);
        dodobot::set_network_entry(index, network_name, signal_strength);
        dodobot_ui::connect_network_screen->draw_list();
    }

    // else if (category.equals("breakout")) {
    //     CHECK_SEGMENT(serial_obj, -1); dodobot_breakout::level_config = serial_obj->get_segment();
    // }

    else if (category.equals("setpath")) {
        CHECK_SEGMENT(serial_obj, -1);
        dodobot_sd::set_dest_path(String(serial_obj->get_segment()));
    }
    else if (category.equals("delete")) {
        CHECK_SEGMENT(serial_obj, -1);
        dodobot_sd::delete_file(String(serial_obj->get_segment()));
    }

    else if (category.equals("file"))
    {
        int32_t segment_index;
        int32_t num_segments;
        uint16_t file_len;
        char* file_bytes;
        if (!dodobot_serial::parse_large(serial_obj, segment_index, num_segments, file_len, dodobot_sd::prev_segment_index, &file_bytes)) {
            return;
        }
        if (segment_index == 0) {
            if (dodobot_sd::file_is_open) {
                dodobot_sd::close_file();
            }
            dodobot_sd::open_file();
        }

        dodobot_sd::prev_segment_index = segment_index;

        dodobot_serial::println_info("Received file. Segment %d of %d. Segment len: %d", segment_index + 1, num_segments, file_len);
        if (!dodobot_sd::append_to_buffer(file_bytes, (uint32_t)file_len)) {
            return;
        }
        if (segment_index == num_segments - 1)
        {
            dodobot_sd::close_file();
            dodobot_ui::on_file(dodobot_sd::dest_path);
        }
    }
    else if (category.equals("listdir"))
    {
        CHECK_SEGMENT(serial_obj, -1);
        dodobot_sd::list_dir(String(serial_obj->get_segment()));
    }

    else if (category.equals("key"))
    {
        CHECK_SEGMENT(serial_obj, -1); String str = String(serial_obj->get_segment());
        CHECK_SEGMENT(serial_obj, 4);  int32_t event_type = serial_obj->segment_as_int32();
        char c = str.charAt(0);
        dodobot_serial::println_info("Key: %d, %c", event_type, c);
        dodobot_ui::on_keyboard(event_type, c);
    }

    else if (category.equals("robotfn"))
    {
        CHECK_SEGMENT(serial_obj, 4);  int32_t index = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, 4);  int32_t num_functions = serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, -1); String function_name = String(serial_obj->get_segment());
        CHECK_SEGMENT(serial_obj, 4); int32_t blank_space = serial_obj->segment_as_int32();
        dodobot_serial::println_info("Received robot function: %s, %d", function_name.c_str(), index);
        if (index == 0) {
            dodobot_ui::robot_screen->reset_list();
        }

        dodobot::set_robot_fn_entry(index, function_name);
        if (blank_space > 0) {
            dodobot_serial::println_info("Adding blank space: %d @ %d", blank_space, index);
            dodobot_ui::robot_screen->set_blank_space(index, blank_space);
        }

        if (index == num_functions - 1) {
            dodobot_ui::robot_screen->draw_list();
        }
    }
    else if (category.equals("notify"))
    {
        CHECK_SEGMENT(serial_obj, 4);  dodobot_ui::NotificationLevel level = (dodobot_ui::NotificationLevel)serial_obj->segment_as_int32();
        CHECK_SEGMENT(serial_obj, -1); String text = String(serial_obj->get_segment());
        CHECK_SEGMENT(serial_obj, 4);  uint32_t timeout = serial_obj->segment_as_uint32();
        dodobot_serial::println_error("Received notification: %d, %s, %d", (int)level, text.c_str(), timeout);

        dodobot_ui::notify(level, text, timeout);
    }
    else if (category.equals("charge")) {
        CHECK_SEGMENT(serial_obj, 4);  bool is_charging = (bool)serial_obj->segment_as_int32();
        dodobot_ui::is_charging = is_charging;
    }
}

void dodobot_ir_remote::callback_ir(uint8_t remote_type, uint16_t value)
{
    dodobot_ui::reset_display_brightness_timer();
    switch (value) {
        case 0x00ff: dodobot_serial::println_info("IR: VOL-"); break;  // VOL-
        case 0x807f:
            dodobot_serial::println_info("IR: Play/Pause");
            set_active(!dodobot::robot_state.motors_active);
            dodobot_serial::println_info("motors_active: %d", dodobot::robot_state.motors_active);
            break;  // Play/Pause
        case 0x40bf: dodobot_serial::println_info("IR: VOL+"); break;  // VOL+
        case 0x20df: dodobot_serial::println_info("IR: SETUP");
            // dodobot_display::setup_display();  tft.print("Display ready\n");
            dodobot::robot_state.is_reporting_enabled = !dodobot::robot_state.is_reporting_enabled;
            dodobot_serial::println_info("is_reporting_enabled: %d", dodobot::robot_state.is_reporting_enabled);
            break;  // SETUP
        case 0xa05f:
            dodobot_serial::println_info("IR: ^");
            // dodobot_linear::set_position(dodobot_linear::tic.getCurrentPosition() + 10000);
            // dodobot_linear::set_velocity(dodobot_linear::MAX_SPEED);
            dodobot_ui::on_up();
            break;  // ^
        case 0x609f: dodobot_serial::println_info("IR: MODE"); break;  // MODE
        case 0x10ef:
            dodobot_serial::println_info("IR: <");
            dodobot_ui::on_left();
            break;  // <
        case 0x906f:
            dodobot_serial::println_info("IR: ENTER");
            dodobot_ui::on_enter();
            break;  // ENTER
        case 0x50af:
            dodobot_serial::println_info("IR: >");
            dodobot_ui::on_right();
            break;  // >
        case 0x30cf:
            dodobot_serial::println_info("IR: 0 10+");
            dodobot_ui::on_numpad(0);
            break;  // 0 10+
        case 0xb04f:
            dodobot_serial::println_info("IR: v");
            // dodobot_linear::set_position(dodobot_linear::tic.getCurrentPosition() - 10000);
            // dodobot_linear::set_velocity(-dodobot_linear::MAX_SPEED);
            dodobot_ui::on_down();
            break;  // v
        case 0x708f:
            dodobot_serial::println_info("IR: Del");
            dodobot_ui::on_back();
            break;  // Del
        case 0x08f7:
            dodobot_serial::println_info("IR: 1");
            // dodobot_gripper::toggle_gripper();
            dodobot_ui::on_numpad(1);
            break;  // 1
        case 0x8877:
            dodobot_serial::println_info("IR: 2");
            // dodobot_speed_pid::update_setpointA(3000);
            // dodobot_speed_pid::update_setpointB(3000);
            // dodobot_chassis::set_motorA(255);
            // dodobot_chassis::set_motorB(255);
            dodobot_ui::on_numpad(2);
            break;  // 2
        case 0x48B7: dodobot_serial::println_info("IR: 3");
            dodobot_ui::on_numpad(3);
            break;  // 3
        case 0x28D7:
            dodobot_serial::println_info("IR: 4");
            // dodobot_speed_pid::update_setpointA(-3000);
            // dodobot_speed_pid::update_setpointB(3000);
            dodobot_ui::on_numpad(4);
            break;  // 4
        case 0xA857:
            dodobot_serial::println_info("IR: 5");
            // dodobot_speed_pid::update_setpointA(0);
            // dodobot_speed_pid::update_setpointB(0);
            dodobot_ui::on_numpad(5);
            break;  // 5
        case 0x6897:
            dodobot_serial::println_info("IR: 6");
            // dodobot_speed_pid::update_setpointA(3000);
            // dodobot_speed_pid::update_setpointB(-3000);
            dodobot_ui::on_numpad(6);
            break;  // 6
        case 0x18E7:
            dodobot_serial::println_info("IR: 7");
            // dodobot_sd::list_all_files();
            dodobot_ui::on_numpad(7);
            break;  // 7
        case 0x9867:
            dodobot_serial::println_info("IR: 8");
            // dodobot_speed_pid::update_setpointA(-3000);
            // dodobot_speed_pid::update_setpointB(-3000);
            // dodobot_chassis::set_motorA(-255);
            // dodobot_chassis::set_motorB(-255);
            dodobot_ui::on_numpad(8);
            break;  // 8
        case 0x58A7:
            dodobot_serial::println_info("IR: 9");
            // dodobot_linear::home_stepper();
            dodobot_ui::on_numpad(9);
            break;  // 9
        case 0xffff:  // repeat last command
            dodobot_ui::on_repeat();
            break;
    }
}

void setup()
{
    dodobot::setup();
    dodobot_serial::setup_serial();
    dodobot_display::setup_display();  tft.print("Display ready\n");
    dodobot_ir_remote::setup_IR();  tft.print("IR ready\n");
    dodobot_i2c::setup_i2c();  tft.print("I2C ready\n");
    dodobot_power_monitor::setup_INA219();  tft.print("INA219 ready\n");
    dodobot_gripper::setup_gripper();  tft.print("Gripper ready\n");
    dodobot_tilter::setup_tilter();  tft.print("Tilter ready\n");
    dodobot_linear::setup_linear();  tft.print("Linear ready\n");
    dodobot_chassis::setup_chassis();  tft.print("Drive motors ready\n");
    dodobot_speed_pid::setup_pid();  tft.print("Speed PID ready\n");
    dodobot_latch_circuit::setup_latch();  tft.print("Latch ready\n");
    dodobot_sd::setup();  tft.print("SD card ready\n");
    tft.print("Dodobot is ready to go!\n");
    dodobot_serial::println_info("Dodobot is ready to go!");
    dodobot_ui::init();
}

void loop()
{
    dodobot_serial::data->read();
    dodobot_serial::info->read();

    dodobot::report_structs();

    if (dodobot_ir_remote::read_IR()) {
        dodobot_ir_remote::report_IR();
    }
    if (dodobot_power_monitor::read_INA219()) {
        dodobot_power_monitor::report_INA219();
    }
    if (dodobot_gripper::read_fsrs()) {
        dodobot_gripper::report_fsrs();
    }
    dodobot_gripper::update();
    dodobot_linear::update();
    dodobot_chassis::update();
    dodobot_speed_pid::update_speed_pid();
    dodobot_latch_circuit::update();
    dodobot_ui::draw();
}
