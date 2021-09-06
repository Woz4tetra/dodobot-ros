#include <db_power_box_parsing/db_power_box_parsing.h>


DodobotPowerBoxParsing::DodobotPowerBoxParsing(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ros::param::param<string>("~device_address", device_address, "");
    ros::param::param<int>("~device_baud", device_baud, 38400);
    ros::param::param<double>("~initial_connect_delay", initial_connect_delay, 2.0);
    ros::param::param<int>("~connection_attempts", connection_attempts, 5);
    ros::param::param<double>("~charge_threshold_mA", charge_threshold_mA, 100.0);

    if (connection_attempts <= 0) {
        connection_attempts = 1;
    }

    ROS_INFO_STREAM("Device address: " << device_address);
    ROS_INFO_STREAM("Device baud: " << device_baud);

    battery_msg.header.frame_id = "battery";
    battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    is_charging = false;

    battery_pub = nh.advertise<sensor_msgs::BatteryState>("charger", 50);
    is_charging_pub = nh.advertise<std_msgs::Bool>("is_charging", 10);

    is_charging_timer = nh.createTimer(ros::Duration(0.25), &DodobotPowerBoxParsing::is_charging_timer_callback, this);

    ring_light_sub = nh.subscribe<std_msgs::ColorRGBA>("ring_light", 50, &DodobotPowerBoxParsing::ring_light_callback, this);
    color_raw_sub = nh.subscribe<std_msgs::UInt32>("ring_light_raw", 50, &DodobotPowerBoxParsing::color_raw_callback, this);

    serial_interface = new DodobotSerial();

    ROS_INFO("Dodobot power box bridge init done");
}

void DodobotPowerBoxParsing::setup()
{
    int attempt = 0;
    while (true)
    {
        serial_interface->begin(device_address, device_baud);
        ros::Duration(initial_connect_delay).sleep();
        try {
            serial_interface->check_device_ready();
            break;
        }
        catch (ReadyTimeoutException_t) {
            attempt++;
            ROS_WARN("Connection failed. Attempt %d of %d", attempt, connection_attempts);
            serial_interface->close();
        }
        if (attempt == connection_attempts) {
            throw ReadyTimeoutException_t();
        }
    }
}

void DodobotPowerBoxParsing::loop()
{
    if (serial_interface->read()) {
        packet_callback(serial_interface->get_category());
    }
}

void DodobotPowerBoxParsing::stop()
{
    set_ring_color(0);
    serial_interface->close();
}

void DodobotPowerBoxParsing::packet_callback(string category)
{
    ROS_DEBUG_STREAM("Received package category: " << category);
    if (category.compare("power") == 0)
    {
        float shunt_voltage, bus_voltage, current_mA, power_mW, load_voltage;
        if (serial_interface->segment_as_float(shunt_voltage) && 
                serial_interface->segment_as_float(bus_voltage) && 
                serial_interface->segment_as_float(current_mA) && 
                serial_interface->segment_as_float(power_mW) && 
                serial_interface->segment_as_float(load_voltage))
        {
            power_packet_callback(shunt_voltage, bus_voltage, current_mA, power_mW, load_voltage);
        }
    }
}

int DodobotPowerBoxParsing::run()
{
    setup();

    ros::Rate clock_rate(60);  // run loop at 60 Hz

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

void DodobotPowerBoxParsing::ring_light_callback(const std_msgs::ColorRGBAConstPtr& msg) {
    set_ring_color(msg->r, msg->g, msg->b, msg->a);
}

void DodobotPowerBoxParsing::color_raw_callback(const std_msgs::UInt32ConstPtr& msg) {
    set_ring_color(msg->data);
}

void DodobotPowerBoxParsing::is_charging_timer_callback(const ros::TimerEvent& event)
{
    std_msgs::Bool msg;
    msg.data = is_charging;
    is_charging_pub.publish(msg);
}


void DodobotPowerBoxParsing::power_packet_callback(float shunt_voltage, float bus_voltage, float current_mA, float power_mW, float load_voltage)
{
    battery_msg.voltage = load_voltage;
    battery_msg.current = current_mA;
    is_charging = current_mA > charge_threshold_mA;
    battery_pub.publish(battery_msg);

    ROS_DEBUG(
    "\nshunt_voltage: %0.4f V\n" \
    "bus_voltage: %0.4f V\n" \
    "current_mA: %0.4f mA\n" \
    "power_mW: %0.4f mW\n" \
    "load_voltage: %0.4f V\n",
        shunt_voltage,
        bus_voltage,
        current_mA,
        power_mW,
        load_voltage
    );
}

void DodobotPowerBoxParsing::set_ring_color(double r_channel, double g_channel, double b_channel, double a_channel)
{
    uint8_t r, g, b, w;
    r = (uint8_t)(r_channel * 255.0);
    g = (uint8_t)(g_channel * 255.0);
    b = (uint8_t)(b_channel * 255.0);
    w = (uint8_t)(a_channel * 255.0);  // using alpha channel for white led channel
    uint32_t color = ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
    serial_interface->write("pix", "u", color);
}

void DodobotPowerBoxParsing::set_ring_color(uint32_t color) {
    serial_interface->write("pix", "u", color);
}