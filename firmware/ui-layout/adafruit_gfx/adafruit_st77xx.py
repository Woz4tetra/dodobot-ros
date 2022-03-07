import time
import math
import decimal
import pygame
import pygame.font
import pygame.locals

from . import classic_font
from .color565 import *


class Adafruit_ST77XX:
    def __init__(self, on_event):
        self.window_title = "Dodobot ST77XX layout tool"
        self.window = None  # type: pygame.Surface
        self.screen = None  # type: pygame.Surface
        self.clock = None  # type: pygame.time.Clock
        self.brightness_filter = None  # type: pygame.Surface

        self.on_event = on_event

        self.screen_width = 0
        self.screen_height = 0
        self._init_screen_width = 0
        self._init_screen_height = 0
        self.scale_factor = 1.0

        self.real_fps = 60.0
        self.sim_delay_ms = 1

        self.prev_sim_time = time.time()

        self.running = False
        self.initialized = False

        self.cursor_x = 0
        self.cursor_y = 0
        self.text_size = 1
        self.text_wrap = False
        self.text_width = 6
        self.text_height = 8
        self.text_foreground = ST77XX_WHITE
        self.text_background = ST77XX_BLACK

        self.controllers = {}
        self.view_index = 0

    def add_controller(self, controller, index=None):
        if index is None:
            index = 0
            while index in self.controllers:
                index += 1
        self.controllers[index] = controller

        controller._add_screen(self)

    def set_scale(self, scale):
        self.screen = pygame.Surface(self.size())
        self.scale_factor = scale
        self.window = pygame.display.set_mode(self.scaled_screen(), pygame.locals.RESIZABLE)
        self.brightness_filter = pygame.Surface(self.scaled_screen(), pygame.SRCALPHA)

    def set_delay(self, ms):
        self.sim_delay_ms = ms

    def set_display_brightness(self, brightness):
        brightness = max(0, min(255, 255 - brightness))
        self.brightness_filter.fill((0, 0, 0, brightness))
        self.draw()

    def size(self):
        return self.screen_width, self.screen_height

    def scaled_screen(self):
        return int(self.screen_width * self.scale_factor), int(self.screen_height * self.scale_factor)

    def init(self, w, h):
        if self.initialized:
            raise Exception(f"{self.__class__.__name__} is already initialized!")

        self.screen_width = w
        self.screen_height = h
        self._init_screen_width = w
        self._init_screen_height = h

        pygame.init()
        pygame.font.init()
        pygame.display.set_caption(self.window_title)

        self.clock = pygame.time.Clock()
        self.set_scale(self.scale_factor)
        self.view_index = 0
        self.initialized = True
        self.running = True

    def load_view(self, index):
        self.controllers[self.view_index].on_unload()
        self.view_index = index
        self.controllers[self.view_index].on_load()

    def draw(self):
        self.window.blit(pygame.transform.scale(self.screen, self.window.get_rect().size), (0, 0))
        self.window.blit(self.brightness_filter, (0, 0))
        pygame.display.update()

    def wait(self):
        self.clock.tick(int(self.real_fps))

    def update(self):
        self.run_events()
        self.draw()
        self.wait()

        current_time = time.time()
        dt = (current_time - self.prev_sim_time) * 1000.0
        if dt > self.sim_delay_ms:
            self.prev_sim_time = current_time

            self.controllers[self.view_index].draw()

            return True
        else:
            return False

    def run_events(self):
        for event in pygame.event.get():
            fn = None
            if event.type == pygame.QUIT:
                # change the value to False, to exit the main loop
                self.running = False
            elif event.type == pygame.VIDEORESIZE:
                self.set_scale(event.w / self.screen_width)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.controllers[self.view_index].on_up()
                elif event.key == pygame.K_DOWN:
                    self.controllers[self.view_index].on_down()
                elif event.key == pygame.K_LEFT:
                    self.controllers[self.view_index].on_left()
                elif event.key == pygame.K_RIGHT:
                    self.controllers[self.view_index].on_right()
                elif event.key == pygame.K_BACKSPACE:
                    self.controllers[self.view_index].on_back()
                elif event.key == pygame.K_RETURN:
                    self.controllers[self.view_index].on_enter()
                elif pygame.K_0 <= event.key <= pygame.K_9:
                    key_num = event.key - pygame.K_0
                    self.set_scale(key_num)
            self.on_event(self, event)

    # --- TFT and draw functions ---

    def width(self):
        return self.screen_width

    def height(self):
        return self.screen_height

    def setRotation(self, rotation):
        if rotation == 0 or rotation == 2:
            self.screen_width = self._init_screen_height
            self.screen_height = self._init_screen_width
        elif rotation == 1 or rotation == 3:
            self.screen_width = self._init_screen_width
            self.screen_height = self._init_screen_height

        self.set_scale(self.scale_factor)

    def fillScreen(self, color: Color565):
        self.screen.fill(color.to_rgb())

    def fillCircle(self, x, y, r, color: Color565):
        pygame.draw.circle(self.screen, color.to_rgb(), (x, y), r)

    def drawCircle(self, x, y, r, w, color: Color565):
        pygame.draw.circle(self.screen, color.to_rgb(), (x, y), r, w)

    def writeFastHLine(self, x, y, w, color: Color565):
        pygame.draw.line(self.screen, color.to_rgb(), (x, y), (x + w, y))

    def writeFastVLine(self, x, y, h, color: Color565):
        pygame.draw.line(self.screen, color.to_rgb(), (x, y), (x, y + h))

    def setCursor(self, x, y):
        self.cursor_x = int(x)
        self.cursor_y = int(y)

    def setTextSize(self, size):
        self.text_size = int(size)

    def setTextColor(self, foreground: Color565, background: Color565):
        self.text_foreground = foreground
        self.text_background = background

    def setTextWrap(self, wrap):
        self.text_wrap = bool(wrap)

    def print(self, text):
        for c in text:
            if c == '\n':
                self.cursor_x = 0
                self.cursor_y += self.text_height * self.text_size
            elif c == '\r':
                continue
            else:
                if self.text_wrap and (self.cursor_x + self.text_size * 6) > self.screen_width:
                    self.cursor_x = 0
                    self.cursor_y += self.text_height * self.text_size
                self.drawChar(self.cursor_x, self.cursor_y, c, self.text_foreground, self.text_background,
                              self.text_size, self.text_size)
                self.cursor_x += self.text_width * self.text_size

    def fillRect(self, x, y, w, h, color: Color565):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        pygame.draw.rect(self.screen, color.to_rgb(), (x, y, w, h))

    def drawRect(self, x, y, w, h, color: Color565):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        pygame.draw.rect(self.screen, color.to_rgb(), (x, y, w, h), width=1)

    def writePixel(self, x, y, color: Color565):
        x = int(x)
        y = int(y)
        self.screen.set_at((x, y), color.to_rgb())

    def drawChar(self, x, y, c: str, color: Color565, bg: Color565, size_x=1, size_y=1):
        x = int(x)
        y = int(y)
        size_x = int(size_x)
        size_y = int(size_y)

        if len(c) > 1:
            c = c[0]
        c = ord(c)
        if ((x >= self.screen_width) or  # Clip right
                (y >= self.screen_height) or  # Clip bottom
                ((x + 6 * size_x - 1) < 0) or  # Clip left
                ((y + 8 * size_y - 1) < 0)):  # Clip top
            return

        # if c >= 176:
        #     c += 1  # Handle 'classic' charset behavior

        for i in range(self.text_width - 1):  # Char bitmap = 5 columns
            line = classic_font.font[c * (self.text_width - 1) + i]
            for j in range(8):
                if line & 1:
                    if size_x == 1 and size_y == 1:
                        self.writePixel(x + i, y + j, color)
                    else:
                        self.fillRect(x + i * size_x, y + j * size_y, size_x, size_y, color)
                elif bg != color:
                    if size_x == 1 and size_y == 1:
                        self.screen.set_at((x + i, y + j), bg.to_rgb())
                    else:
                        self.fillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg)

                line >>= 1
        if bg != color:  # If opaque, draw vertical line for last column
            if size_x == 1 and size_y == 1:
                self.writeFastVLine(x + self.text_width - 1, y, self.text_height, bg)
            else:
                self.fillRect(x + self.text_width - 1 * size_x, y, size_x, self.text_height * size_y, bg)

    def getTextBounds(self, string):
        string = string.replace("\r", "")
        newlines = string.count("\n")
        str_width = len(string) * self.text_width * self.text_size
        str_height = (str_width // self.screen_width + newlines + 1) * self.text_height * self.text_size

        return str_width, str_height

    def fillTriangle(self, x0, y0, x1, y1, x2, y2, color: Color565):
        pygame.draw.polygon(self.screen, color.to_rgb(), [(x0, y0), (x1, y1), (x2, y2)])

    def drawArc(self, x, y, start_angle, stop_angle, rx, ry, w, color: Color565, increment=3):
        seg = 0

        sx = math.cos(math.radians(start_angle))
        sy = math.sin(math.radians(start_angle))

        x0 = self.round(sx * (rx - w) + x)
        y0 = self.round(sy * (ry - w) + y)
        x1 = self.round(sx * rx + x)
        y1 = self.round(sy * ry + y)
        i = start_angle
        while True:
            if stop_angle >= start_angle:
                i += increment
                if i > stop_angle:
                    break

            if stop_angle <= start_angle:
                i -= increment
                if i < stop_angle:
                    break

            sx2 = math.cos(math.radians(i + seg))
            sy2 = math.sin(math.radians(i + seg))

            x2 = self.round(sx2 * (rx - w) + x)
            y2 = self.round(sy2 * (ry - w) + y)
            x3 = self.round(sx2 * rx + x)
            y3 = self.round(sy2 * ry + y)

            self.fillTriangle(x0, y0, x1, y1, x2, y2, color)
            self.fillTriangle(x1, y1, x2, y2, x3, y3, color)

            x0 = x2
            y0 = y2
            x1 = x3
            y1 = y3

    def fillArc(self, x, y, start_angle, stop_angle, rx, ry, color: Color565, increment=2):
        sx = math.cos(math.radians(start_angle))
        sy = math.sin(math.radians(start_angle))

        x0 = self.round(x)
        y0 = self.round(y)
        x1 = self.round(sx * rx + x)
        y1 = self.round(sy * ry + y)
        i = start_angle
        while True:
            if stop_angle >= start_angle:
                i += increment
                if i > stop_angle:
                    break

            if stop_angle <= start_angle:
                i -= increment
                if i < stop_angle:
                    break

            sx2 = math.cos(math.radians(i))
            sy2 = math.sin(math.radians(i))

            x2 = self.round(sx2 * rx + x)
            y2 = self.round(sy2 * ry + y)

            self.fillTriangle(x0, y0, x1, y1, x2, y2, color)

            x1 = x2
            y1 = y2

    def round(self, x):
        return int(decimal.Decimal(x).quantize(decimal.Decimal(1)))

    def fillRoundRect(self, x, y, w, h, r, color: Color565):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        r = int(r)
        pygame.draw.rect(self.screen, color.to_rgb(), (x, y, w, h), border_radius=r)

    def drawRoundRect(self, x, y, w, h, r, color: Color565):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        r = int(r)
        pygame.draw.rect(self.screen, color.to_rgb(), (x, y, w, h), width=1, border_radius=r)
