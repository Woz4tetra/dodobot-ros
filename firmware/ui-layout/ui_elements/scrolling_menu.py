from adafruit_gfx.color565 import *


class MenuEntry:
    def __init__(self, name, callback):
        self.name = name
        self.callback = callback


class ScrollingMenu:
    def __init__(self, x=0, y=0, menu_w=None, entry_h=10, show_numbers=False, text_size=1):
        self.topbar = None

        self.x = x
        self.y = y
        self.menu_w = menu_w
        self.entry_h = entry_h
        self.show_numbers = show_numbers
        self.text_size = text_size

        self.entries = []
        self.blank_spaces = {}

        self.selected_index = 0
        self.prev_selected_index = -1
        self.screen = None

        self.in_view_start = 0
        self.in_view_stop = 0

        self.first_time_load = True

    def add_screen(self, screen):
        self.screen = screen

    def add_entry(self, name, callback):
        self.entries.append(MenuEntry(name, callback))
        return len(self.entries) - 1

    def add_blank(self, h, index=None):
        if index is None:
            index = len(self.entries) - 1
        else:
            if index < 0:
                return
            if index >= len(self.entries):
                return

        self.blank_spaces[index] = h

    def remove_entry(self, index):
        if len(self.entries) == 0:
            return
        while index < len(self.entries) - 1:
            self.entries[index] = self.entries[index + 1]
            index += 1
        self.entries.pop(index)

    def on_load(self):
        if self.menu_w is None:
            self.menu_w = self.screen.width()

        self.compute_stop_index()
        self.draw()

    def on_unload(self):
        pass

    def draw(self):
        self.topbar.fillBelowScreen()
        self.screen.setTextSize(self.text_size)
        if self.show_numbers:
            text_w, text_h = self.screen.getTextBounds("  ")
            text_w += 2
            draw_x = self.x + text_w
            draw_w = self.menu_w - text_w
        else:
            draw_x = self.x - 1
            draw_w = self.menu_w
        draw_y = self.y

        for index in range(self.in_view_start, self.in_view_stop):
            entry = self.entries[index]

            if index in self.blank_spaces:
                draw_y += self.blank_spaces[index]

            if self.show_numbers:
                self.screen.setCursor(self.x, draw_y)
                self.screen.print(str(index + 1))
            text = entry.name

            text_w, text_h = self.screen.getTextBounds(text)
            if self.selected_index != self.prev_selected_index and index == self.prev_selected_index:
                self.screen.drawRect(draw_x - 2, draw_y + (text_h - self.entry_h) // 2, draw_w, self.entry_h,
                                     ST77XX_BLACK)

            if index == self.selected_index:
                self.screen.drawRect(draw_x - 2, draw_y + (text_h - self.entry_h) // 2, draw_w, self.entry_h,
                                     ST77XX_WHITE)

            self.screen.setCursor(draw_x, draw_y)
            self.screen.print(text)
            draw_y += self.entry_h
        self.prev_selected_index = self.selected_index

    def on_up(self):
        self.select(self.selected_index - 1)

    def on_down(self):
        self.select(self.selected_index + 1)

    def on_enter(self):
        if 0 <= self.selected_index < len(self.entries):
            self.entries[self.selected_index].callback()

    def select(self, index):
        self.selected_index = index
        if self.selected_index < 0:
            self.selected_index = 0
        if self.selected_index >= len(self.entries):
            self.selected_index = len(self.entries) - 1

        if self.selected_index < self.in_view_start:
            # self.in_view_stop -= self.in_view_start - self.selected_index
            self.in_view_start = self.selected_index
        if self.selected_index >= self.in_view_stop:
            self.in_view_start += self.selected_index - (self.in_view_stop - 1)
            # self.in_view_stop = self.selected_index + 1
        self.compute_stop_index()
        self.draw()

    def compute_stop_index(self):
        draw_y = 0
        for index, entry in enumerate(self.entries):
            if index == self.in_view_start:
                draw_y = self.y
            draw_y += self.entry_h
            self.in_view_stop = index
            if draw_y > self.screen.height():
                break
            if index in self.blank_spaces:
                draw_y += self.blank_spaces[index]
        # self.in_view_start = 0
        self.in_view_stop += 1
