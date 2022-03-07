#include <Arduino.h>

#define COMM_SERIAL Serial

#define UNLATCH_PIN 24
#define LATCH_STAT_PIN 25
#define BUTTON_LED_PIN 14
#define USB_SENSE_PIN 7


bool usb_connected_once = false;
const uint32_t usb_check_interval_ms = 1000;
uint32_t usb_check_timer = 0;
#define CURRENT_TIME millis()

int led_val = 0;
bool prev_button_state = false;
bool is_shutting_down = false;


void unlatch() {
    digitalWrite(UNLATCH_PIN, HIGH);
}


bool is_usb_connected() {
    return digitalRead(USB_SENSE_PIN) == HIGH;
}

bool is_button_pushed() {
    return digitalRead(LATCH_STAT_PIN) == LOW;
}

void set_button_led(int val) {
    led_val = val;
    if (led_val < 0) {
        led_val = 0;
    }
    else if (led_val > 255) {
        led_val = 255;
    }
    analogWrite(BUTTON_LED_PIN, val);
}

void setup()
{
    COMM_SERIAL.begin(9600);

    pinMode(UNLATCH_PIN, OUTPUT);
    pinMode(LATCH_STAT_PIN, INPUT_PULLUP);
    pinMode(USB_SENSE_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LED_PIN, OUTPUT);

    digitalWrite(UNLATCH_PIN, LOW);
    set_button_led(255);

    prev_button_state = is_button_pushed();
}

bool is_incrementing = false;
void cycle_led()
{
    if (is_incrementing) {
        set_button_led(led_val + 1);
    }
    else {
        set_button_led(led_val - 1);
    }
    if (led_val == 0) {
        is_incrementing = true;
    }
    else if (led_val == 255) {
        is_incrementing = false;
    }
    delay(1);
}

void loop()
{
    if (COMM_SERIAL.available()) {
        String message = COMM_SERIAL.readStringUntil('\n');
        if (message.equals("shutdown")) {
            is_shutting_down = true;
        }
    }
    if (CURRENT_TIME - usb_check_timer > usb_check_interval_ms) {
        if (usb_connected_once) {
            if (!is_usb_connected()) {
                unlatch();
            }
        }
        else {
            if (is_usb_connected()) {
                usb_connected_once = true;
            }
        }
    }

    bool button_state = is_button_pushed();
    if (button_state) {
        if (!is_shutting_down) {
            cycle_led();            
        }
    }
    else {
        set_button_led(255);
    }
    if (prev_button_state != button_state) {
        COMM_SERIAL.println(button_state);
        prev_button_state = button_state;
    }
}
