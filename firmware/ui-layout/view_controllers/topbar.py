import time
from adafruit_gfx.color565 import *
from ui_elements.view_controller import ViewController

REPORTING_ENABLED = 0
REPORTING_DISABLED = 1

NO_USB_POWER = 0
USB_POWER_DETECTED = 1
USB_CONNECTION_STABLE = 2

MOTORS_INACTIVE = 0
MOTORS_ACTIVE = 1

FULL_VOLTAGE = 12.15
OK_VOLTAGE = 10.5
LOW_VOLTAGE = 9.25
CRITICAL_VOLTAGE = 8.0

ST77XX_DARKER_GREEN = Color565.from_rgb((2, 208, 2))


class TopbarController(ViewController):
    def __init__(self):
        super(TopbarController, self).__init__()
        self.height = 21
        self.width = 0

        self.report_icon_cx = 0
        self.report_icon_cy = 0

        self.connection_icon_cx = 0
        self.connection_icon_cy = 0

        self.top_icon_r = self.height // 2
        self.motor_icon_r = self.top_icon_r // 2

        self.battery_x = 0
        self.battery_V_cy = self.height // 3
        self.battery_mA_cy = 2 * self.height // 3

        self.notif_box_width = 60
        self.notif_height_offset = 3

        self.reporting_status = REPORTING_DISABLED
        self.connection_status = NO_USB_POWER
        self.motor_status = MOTORS_INACTIVE

        self.starting_text = "Connecting..."
        self.text = self.starting_text
        self.datetime_text = ""
        self.datetime_text_update = time.time()

        self.notification_queue = []
        self.notification = None
        self.notification_color = ST77XX_WHITE
        self.notification_timer = time.time()

        self.voltage_V = 0
        self.current_mA = 0

    def set_datetime_text(self, text):
        self.datetime_text = text
        self.datetime_text_update = time.time()

    def notify(self, notif_level, text, timeout):
        if self.notification is not None and self.notification[1] == text:
            return
        for notification in self.notification_queue:
            if notification[1] == text:
                return
        self.notification_queue.append((notif_level, text, timeout))

    def on_load(self):
        self.width = self.screen.width()

        self.report_icon_cx = self.top_icon_r + 2
        self.report_icon_cy = self.height // 2

        self.connection_icon_cx = self.report_icon_cx + 5
        self.connection_icon_cy = self.height // 2

        self.battery_x = self.width - 5

    def draw(self):
        self.draw_text()
        self.draw_status_icons()
        self.draw_battery()

        # self.screen.fillRect(0, 0, self.width, self.height, ST77XX_GRAY)
        self.screen.writeFastHLine(0, self.height, self.width, ST77XX_GRAY)

    def draw_text(self):
        if len(self.notification_queue) == 0 and self.notification is None:
            if self.connection_status == USB_CONNECTION_STABLE:
                self.text = self.datetime_text
            else:
                self.text = self.starting_text
            self.notification_color = ST77XX_WHITE
        else:
            if self.notification is None:
                self.notification = self.notification_queue.pop(0)
                self.notification_timer = time.time()

            notif_level = self.notification[0]
            self.text = self.notification[1]
            notif_time = self.notification[2]

            if notif_level == 0:
                self.notification_color = ST77XX_WHITE
            elif notif_level == 1:
                self.notification_color = ST77XX_YELLOW
            elif notif_level == 2:
                self.notification_color = ST77XX_RED

            if time.time() - self.notification_timer > notif_time:
                self.notification = None

        self.screen.setTextSize(1)
        w, h = self.screen.getTextBounds(self.text)
        x = (self.screen.width() - w) // 2
        y = (self.height - h) // 2
        self.screen.setCursor(x, y)

        self.screen.setTextColor(self.notification_color, self.notification_color)
        self.screen.fillRect(0, y, self.screen.width(), y + 8, ST77XX_BLACK)
        self.screen.print(self.text)
        self.screen.setTextColor(ST77XX_WHITE, ST77XX_BLACK)

        if self.notification is not None:
            x = (self.screen.width() - self.notif_box_width) // 2
            y = y + h + self.notif_height_offset

            notif_time = self.notification[2]
            dt = time.time() - self.notification_timer

            w = int(self.notif_box_width * (1.0 - dt / notif_time))
            h = 3

            self.screen.fillRect(x, y, self.notif_box_width, h, ST77XX_GRAY)
            self.screen.fillRect(x, y, w, h, self.notification_color)

    def draw_status_icons(self):
        if self.reporting_status == REPORTING_DISABLED:
            status_color = ST77XX_RED
        elif self.reporting_status == REPORTING_ENABLED:
            status_color = ST77XX_GREEN
        else:
            status_color = ST77XX_WHITE

        if self.connection_status == NO_USB_POWER:
            conn_color = ST77XX_WHITE
        elif self.connection_status == USB_POWER_DETECTED:
            conn_color = ST77XX_YELLOW
        elif self.connection_status == USB_CONNECTION_STABLE:
            conn_color = ST77XX_GREEN
        else:
            conn_color = ST77XX_RED

        if self.motor_status == MOTORS_INACTIVE:
            motor_color = ST77XX_GRAY
        elif self.motor_status == MOTORS_ACTIVE:
            motor_color = ST77XX_DARKER_GREEN
        else:
            motor_color = ST77XX_WHITE

        self.screen.fillArc(self.report_icon_cx, self.report_icon_cy, 90, 270, self.top_icon_r, self.top_icon_r,
                            status_color)
        self.screen.fillArc(self.connection_icon_cx, self.connection_icon_cy, 90, -90, self.top_icon_r, self.top_icon_r,
                            conn_color)

        self.screen.fillCircle(self.report_icon_cx, self.report_icon_cy, self.motor_icon_r, motor_color)
        self.screen.fillCircle(self.connection_icon_cx, self.connection_icon_cy, self.motor_icon_r, motor_color)

        w = self.connection_icon_cx - self.report_icon_cx
        h = 2 * self.motor_icon_r
        self.screen.fillRect(self.report_icon_cx, self.report_icon_cy - self.motor_icon_r, w, h, motor_color)

        # self.screen.fillCircle(self.report_icon_cx, self.report_icon_cy, self.top_icon_r, ST77XX_WHITE)
        # self.screen.fillCircle(self.connection_icon_cx, self.connection_icon_cy, self.top_icon_r, ST77XX_WHITE)

    def fillBelowScreen(self):
        self.screen.fillRect(0, self.height + 1, self.screen.width(), self.screen.height(), ST77XX_BLACK)

    def get_gauge_index(self, voltage_V):
        if voltage_V >= FULL_VOLTAGE:
            return 5
        elif voltage_V >= OK_VOLTAGE:
            return 4
        elif voltage_V > LOW_VOLTAGE:
            return 3
        elif voltage_V <= CRITICAL_VOLTAGE:
            return 1
        elif voltage_V <= LOW_VOLTAGE:
            return 2
        else:
            return 0

    def draw_battery(self):
        voltage_text = "%0.1fV" % self.voltage_V
        current_text = "%dmA" % self.current_mA

        self.screen.setTextSize(1)
        self.screen.setTextColor(ST77XX_WHITE, ST77XX_BLACK)

        w_V, h_V = self.screen.getTextBounds(voltage_text)
        single_char_w = w_V // len(voltage_text)
        x_V = self.battery_x - w_V
        y_V = self.battery_V_cy - h_V // 2

        w_mA, h_mA = self.screen.getTextBounds(current_text)
        x_mA = self.battery_x - w_mA
        y_mA = self.battery_mA_cy - h_mA // 2 + 1

        gauge_count = self.get_gauge_index(self.voltage_V)
        if gauge_count <= 1:
            battery_color = ST77XX_RED
        else:
            battery_color = ST77XX_DARKER_GREEN

        gauge_len = 5
        h_gauge = h_V + h_mA + 1

        batt_nub_h = 7
        batt_nub_y = (self.height - batt_nub_h) // 2 + 1
        self.screen.fillRect(self.battery_x, batt_nub_y, 4, batt_nub_h, ST77XX_GRAY)

        for index in range(max(len(voltage_text), len(current_text), gauge_len)):
            if index < gauge_len:
                gauge_x = self.battery_x - single_char_w * (index + 1)
                inv_index = gauge_len - index
                if inv_index <= gauge_count:
                    capacity_color = battery_color
                else:
                    capacity_color = ST77XX_GRAY

                self.screen.fillRect(gauge_x, y_V, single_char_w, h_gauge, capacity_color)

        self.screen.setTextColor(ST77XX_WHITE, ST77XX_WHITE)  # transparent text background
        # self.screen.fillRect(x_V - 12, y_V, w_V + 12, h_V, ST77XX_BLACK)
        self.screen.setCursor(x_V, y_V)
        self.screen.print(voltage_text)

        # self.screen.fillRect(self.battery_x - single_char_w * 6, y_mA, single_char_w * 6, h_mA, ST77XX_BLACK)
        self.screen.setCursor(x_mA, y_mA)
        self.screen.print(current_text)
        self.screen.setTextColor(ST77XX_WHITE, ST77XX_BLACK)
