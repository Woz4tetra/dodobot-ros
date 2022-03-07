import math
import pygame

from adafruit_gfx.color565 import *
from ui_elements.view_with_overlay_controller import ViewWithOverlayController

ST77XX_LIGHT_BLUE = Color565.from_rgb((201, 217, 248))
ST77XX_DARKER_BLUE = Color565.from_rgb((61, 120, 216))
ST77XX_LIGHT_PINK = Color565.from_rgb((255, 154, 159))


class MainMenuController(ViewWithOverlayController):
    def __init__(self, topbar):
        super(MainMenuController, self).__init__([topbar])
        self.topbar = topbar

        self.menu_entries = [2, 3, 4, 5]
        self.menu_names = ["Network", "Robot", "System Status", "Shutdown"]
        self.draw_slots = []
        self.draw_height = 0
        self.icon_text_height = 0
        self.icon_w = 50
        self.icon_h = 50
        self.icon_r = 10

        self.selected_w = 60
        self.selected_h = 60
        self.selected_r = 15

        self.overview_x = 0
        self.overview_y = 0
        self.overview_r = 5

        self.selected_index = 1

    def on_load_with_overlays(self):
        self.topbar.fillBelowScreen()
        self.draw_slots = [
            self.screen.width() // 4 - 30,
            self.screen.width() // 2,
            3 * self.screen.width() // 4 + 30
        ]
        self.draw_height = self.screen.height() // 2
        self.icon_text_height = self.draw_height + self.selected_h // 2 + 10

        self.overview_x = self.screen.width() // 2
        self.overview_y = self.draw_height + self.selected_h // 2 + 25

        self.draw_all()

    def draw_with_overlays(self):
        pass

    def draw_all(self):
        self.topbar.fillBelowScreen()
        for index, cx in enumerate(self.draw_slots):
            draw_index = (self.selected_index + index - 1) % len(self.menu_entries)
            self.draw_icon(cx, draw_index)

        cx = self.draw_slots[1]
        cy = self.draw_height
        x = cx - self.selected_w // 2
        y = cy - self.selected_h // 2
        self.screen.drawRoundRect(x, y, self.selected_w, self.selected_h, self.selected_r, ST77XX_WHITE)
        self.draw_selection_overview()

        # self.screen.writeFastVLine(self.screen.width() // 2, 0, self.screen.height(), ST77XX_WHITE)

    def draw_icon(self, cx, index):
        cy = self.draw_height
        x = cx - self.icon_w // 2
        y = cy - self.icon_h // 2

        menu_index = self.menu_entries[index]
        if menu_index == 2:
            self.draw_network_icon(x, y, cx, cy)
        elif menu_index == 3:
            self.draw_robot_status_icon(x, y, cx, cy)
        elif menu_index == 4:
            self.draw_system_icon(x, y, cx, cy)
        elif menu_index == 5:
            self.draw_shutdown_icon(x, y, cx, cy)
        else:
            self.screen.fillRoundRect(x, y, self.icon_w, self.icon_h, self.icon_r, ST77XX_WHITE)

        self.screen.setTextSize(1)
        text = self.menu_names[index]
        w, h = self.screen.getTextBounds(text)
        x = cx - w // 2
        y = self.icon_text_height - h // 2
        self.screen.setCursor(x, y)
        self.screen.print(text)

    def draw_robot_status_icon(self, x, y, cx, cy):
        self.screen.fillRoundRect(x, y, self.icon_w, self.icon_h, self.icon_r, ST77XX_LIGHT_BLUE)

        triangle_cx = cx + 2
        triangle_cy = cy + 5
        x0 = triangle_cx
        y0 = triangle_cy
        x1 = cx - self.icon_w // 2 + 7
        y1 = cy + self.icon_h // 2 - 15
        x2 = cx + self.icon_w // 2 - 15
        y2 = cy - self.icon_h // 2 + 7
        x3 = cx + self.icon_w // 2 - 15
        y3 = cy + self.icon_h // 2 - 7
        self.screen.fillTriangle(x0, y0, x1, y1, x2, y2, ST77XX_DARKER_BLUE)
        self.screen.fillTriangle(x0, y0, x2, y2, x3, y3, ST77XX_DARKER_BLUE)

    def draw_system_icon(self, x, y, cx, cy):
        self.screen.fillRoundRect(x, y, self.icon_w, self.icon_h, self.icon_r, ST77XX_GRAY)

        outer_r = self.icon_w // 2 - 7
        inner_r = self.icon_w // 7
        self.screen.drawCircle(cx, cy, outer_r, 5, ST77XX_BLACK)
        self.screen.fillCircle(cx, cy, inner_r, ST77XX_BLACK)

        center_angle = 0
        for index in range(8):
            tooth_cx = outer_r * math.cos(math.radians(center_angle)) + cx
            tooth_cy = outer_r * math.sin(math.radians(center_angle)) + cy

            self.screen.fillCircle(tooth_cx, tooth_cy, 5, ST77XX_BLACK)

            center_angle += 45

    def draw_network_icon(self, x, y, cx, cy):
        self.screen.fillRoundRect(x, y, self.icon_w, self.icon_h, self.icon_r, ST77XX_WHITE)
        fan_angle = 35
        center_angle = 270
        max_fan_w = self.icon_h - 10
        mid_fan_w = 2 * max_fan_w // 3 + 3
        low_fan_w = max_fan_w // 3 + 5
        thickness = 6
        x = cx
        y = cy + self.icon_h // 2 - 5
        self.screen.drawArc(x, y, center_angle - fan_angle, center_angle + fan_angle, max_fan_w, max_fan_w, thickness,
                            ST77XX_GRAY)
        self.screen.drawArc(x, y, center_angle - fan_angle, center_angle + fan_angle, mid_fan_w, mid_fan_w, thickness,
                            ST77XX_GRAY)
        self.screen.drawArc(x, y, center_angle - fan_angle, center_angle + fan_angle, low_fan_w, low_fan_w, thickness,
                            ST77XX_GRAY)
        self.screen.fillCircle(x, y - 3, 5, ST77XX_GRAY)

    def draw_shutdown_icon(self, x, y, cx, cy):
        self.screen.fillRoundRect(x, y, self.icon_w, self.icon_h, self.icon_r, ST77XX_LIGHT_PINK)

        outer_r = self.icon_w // 2 - 7
        rect_w = 5
        rect_h = 20
        rect_border = 6

        self.screen.drawCircle(cx, cy, outer_r, rect_w, ST77XX_BLACK)

        x = cx - rect_w // 2
        y = cy - outer_r - 3
        self.screen.fillRect(x - rect_border // 2, y - rect_border // 2, rect_w + rect_border, rect_h + rect_border, ST77XX_LIGHT_PINK)
        self.screen.fillRoundRect(x, y, rect_w, rect_h, 2, ST77XX_BLACK)

    def draw_selection_overview(self):
        num_entries = len(self.menu_entries)
        total_width = (self.overview_r * 2) * num_entries
        offset = self.overview_x - total_width // 2
        increment = total_width // (num_entries - 1)
        for index in range(num_entries):
            if index == self.selected_index:
                color = ST77XX_WHITE
            else:
                color = ST77XX_GRAY
            self.screen.fillCircle(offset, self.overview_y, self.overview_r, color)
            offset += increment

    def on_left(self):
        self.move_select_left()
        self.draw_all()

    def on_right(self):
        self.move_select_right()
        self.draw_all()

    def move_select_left(self):
        self.selected_index -= 1
        if self.selected_index < 0:
            self.selected_index = len(self.menu_entries) - 1

    def move_select_right(self):
        self.selected_index += 1
        if self.selected_index >= len(self.menu_entries):
            self.selected_index = 0

    def on_enter(self):
        if not (0 <= self.selected_index < len(self.menu_entries)):
            return

        self.screen.load_view(self.menu_entries[self.selected_index])
