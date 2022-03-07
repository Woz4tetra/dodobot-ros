import os
import time
import pygame
import random
from datetime import datetime

from gif import *
from adafruit_gfx import Adafruit_ST77XX
from adafruit_gfx.color565 import *

GIF_DIR = "5th Gen Animated Sprites"
GIF = None
GIF_INDEX = 0

IS_REPORTING_ENABLED = True
MOTORS_ACTIVE = True
USB_CONNECTED_ONCE = False
USB_VOLTAGE_STATE = False
USB_SHUTDOWN_TIMER = None
DATE_STRING = "00:00:00AM"

PREV_DISPLAYED_MENU = -1
DISPLAYED_MENU = 0

SCREEN_MID_W = 0
TOP_BAR_H = 20
TOPBAR_ICON_X = 5
TOPBAR_ICON_Y = TOP_BAR_H // 4
TOPBAR_ICON_R = 4

TOPBAR_ACTIVE_ICON_X = 5
TOPBAR_ACTIVE_ICON_Y = 3 * TOP_BAR_H // 4
TOPBAR_ACTIVE_ICON_R = TOPBAR_ICON_R

TOPBAR_LATCH_USB_ICON_X = 15
TOPBAR_LATCH_USB_ICON_Y = TOP_BAR_H // 2
TOPBAR_LATCH_USB_ICON_R = 4

PREV_DATE_STR_UPDATE = 0

MAIN_MENU_SELECT_INDEX = 0
PREV_MAIN_MENU_SELECT_INDEX = -1

BORDER_OFFSET_W = 3
BORDER_OFFSET_H = 1
ROW_SIZE = 10

MENUS = [
    "MAIN_MENU",
    "NETWORK_MENU",
    "DRIVE_MENU",
    "LINEAR_MENU",
    "TILTER_MENU",
    "GRIPPER_MENU",
    "BREAKOUT_MENU",
    "SHUTDOWN_MENU",
    "IMAGE_MENU",
]

MAIN_MENU_ENTRIES = [
    "Networking",
    "Drive Motors",
    "Linear",
    "Camera Tilter",
    "Gripper",
    "Breakout",
    "Image",
    "Shutdown/restart"
]


def current_time():
    return int(time.time() * 1000)


def down_menu_event(tft):
    print("down")


def up_menu_event(tft):
    print("up")

def left_menu_event(tft):
    print("left")


def right_menu_event(tft):
    print("right")


def load_random_gif():
    global GIF, GIF_INDEX
    filename = random.choice(os.listdir(GIF_DIR))
    path = os.path.join(GIF_DIR, filename)
    GIF = load_gif(path)
    GIF_INDEX = 0


def enter_menu_event(tft):
    print("enter")
    load_random_gif()


def back_menu_event(tft):
    print("back")


def repeat_key_event(tft):
    pass


def screen_change_event(tft):
    pass


def draw_gif(tft, gif_array, index, x, y):
    frame, duration = gif_array[index]
    tft.screen.blit(frame, (x, y))
    pygame.time.wait(duration)


def draw_battery(tft):
    battery_mA_menu_str = "  " + str(random.randint(500, 600)) + "mA"
    battery_V_menu_str = "  " + str("%0.2f" % (random.random() + 11.0)) + "V"
    w, h = tft.getTextBounds(battery_mA_menu_str)
    tft.setCursor(tft.width() - w - 2, TOP_BAR_H // 2 - h)
    tft.print(battery_mA_menu_str)

    w, h = tft.getTextBounds(battery_V_menu_str)
    tft.setCursor(tft.width() - w - 2, TOP_BAR_H // 2)
    tft.print(battery_V_menu_str)


def draw_reporting_icon(tft):
    if IS_REPORTING_ENABLED:
        tft.fillCircle(TOPBAR_ICON_X, TOPBAR_ICON_Y, TOPBAR_ICON_R, ST77XX_GREEN)
    else:
        tft.fillCircle(TOPBAR_ICON_X, TOPBAR_ICON_Y, TOPBAR_ICON_R, ST77XX_RED)


def draw_active_icon(tft):
    if IS_REPORTING_ENABLED:
        tft.fillCircle(TOPBAR_ACTIVE_ICON_X, TOPBAR_ACTIVE_ICON_Y, TOPBAR_ACTIVE_ICON_R, ST77XX_GREEN)
    else:
        tft.fillCircle(TOPBAR_ACTIVE_ICON_X, TOPBAR_ACTIVE_ICON_Y, TOPBAR_ACTIVE_ICON_R, ST77XX_RED)


def update_date_string():
    global DATE_STRING, PREV_DATE_STR_UPDATE
    DATE_STRING = datetime.now().strftime("%I:%M:%S%p")
    PREV_DATE_STR_UPDATE = current_time()


def draw_datestr(tft):
    w, h = tft.getTextBounds(DATE_STRING)
    x1 = SCREEN_MID_W - w // 2
    y1 = TOP_BAR_H // 2 - h // 2
    if not USB_VOLTAGE_STATE:
        tft.fillRect(x1, y1, w, h, ST77XX_BLACK)
        return

    tft.setCursor(x1, y1)
    if current_time() - PREV_DATE_STR_UPDATE > 1000:
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK)
        if current_time() - PREV_DATE_STR_UPDATE > 5000:
            tft.setTextColor(ST77XX_RED, ST77XX_BLACK)

    tft.print(DATE_STRING)
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK)


def draw_latch_debug(tft):
    if USB_CONNECTED_ONCE:
        if USB_VOLTAGE_STATE:
            tft.fillCircle(TOPBAR_LATCH_USB_ICON_X, TOPBAR_LATCH_USB_ICON_Y, TOPBAR_LATCH_USB_ICON_R, ST77XX_GREEN)
        else:
            tft.fillCircle(TOPBAR_LATCH_USB_ICON_X, TOPBAR_LATCH_USB_ICON_Y, TOPBAR_LATCH_USB_ICON_R, ST77XX_YELLOW)
    else:
        tft.fillCircle(TOPBAR_LATCH_USB_ICON_X, TOPBAR_LATCH_USB_ICON_Y, TOPBAR_LATCH_USB_ICON_R, ST77XX_RED)

    if USB_CONNECTED_ONCE and not USB_VOLTAGE_STATE and USB_SHUTDOWN_TIMER is not None:
        seconds = "%0.2f" % (USB_SHUTDOWN_TIMER * 1E-3)
        w, h = tft.getTextBounds(seconds)
        tft.setCursor(SCREEN_MID_W - w // 2, (TOP_BAR_H - h) // 2)
        tft.print(seconds)


def draw_topbar(tft):
    draw_battery(tft)

    draw_reporting_icon(tft)
    draw_active_icon(tft)

    draw_datestr(tft)
    draw_latch_debug(tft)


def draw_main_menu(tft):
    global MAIN_MENU_SELECT_INDEX, PREV_MAIN_MENU_SELECT_INDEX
    if MAIN_MENU_SELECT_INDEX < 0:
        MAIN_MENU_SELECT_INDEX = len(MAIN_MENU_ENTRIES) - 1
    if MAIN_MENU_SELECT_INDEX >= len(MAIN_MENU_ENTRIES):
        MAIN_MENU_SELECT_INDEX = 0

    # if PREV_MAIN_MENU_SELECT_INDEX == MAIN_MENU_SELECT_INDEX:
    #     return
    if PREV_MAIN_MENU_SELECT_INDEX >= 0:
        tft.drawRect(
            BORDER_OFFSET_W - 1,
            ROW_SIZE * PREV_MAIN_MENU_SELECT_INDEX + BORDER_OFFSET_H - 1 + TOP_BAR_H,
            tft.width() - BORDER_OFFSET_W - 1,
            ROW_SIZE - BORDER_OFFSET_H + 1, ST77XX_BLACK
        )

    for i in range(len(MAIN_MENU_ENTRIES)):
        tft.setCursor(BORDER_OFFSET_W, ROW_SIZE * i + BORDER_OFFSET_H + TOP_BAR_H)
        tft.print(MAIN_MENU_ENTRIES[i])

    tft.drawRect(
        BORDER_OFFSET_W - 1,
        ROW_SIZE * MAIN_MENU_SELECT_INDEX + BORDER_OFFSET_H - 1 + TOP_BAR_H,
        tft.width() - BORDER_OFFSET_W - 1,
        ROW_SIZE - BORDER_OFFSET_H + 1, ST77XX_WHITE
    )

    PREV_MAIN_MENU_SELECT_INDEX = MAIN_MENU_SELECT_INDEX


def draw_network_menu(tft):
    pass


def draw_drive_menu(tft):
    pass


def draw_linear_menu(tft):
    pass


def draw_tilter_menu(tft):
    pass


def draw_gripper_menu(tft):
    pass


def dodobot_breakout(tft):
    pass


def draw_shutdown_menu(tft):
    pass


def draw_image_menu(tft):
    pass


def other_keys_event(tft, event):
    global USB_CONNECTED_ONCE, USB_VOLTAGE_STATE
    if event.key == pygame.K_c:
        USB_CONNECTED_ONCE = True
        USB_VOLTAGE_STATE = True


def main():
    global GIF, GIF_INDEX, PREV_DISPLAYED_MENU, DISPLAYED_MENU, SCREEN_MID_W, MENUS

    event_mapping = {
        "down"         : down_menu_event,
        "up"           : up_menu_event,
        "left"         : left_menu_event,
        "right"        : right_menu_event,
        "enter"        : enter_menu_event,
        "back"         : back_menu_event,
        "repeat_key"   : repeat_key_event,
        "screen_change": screen_change_event,
        "other"        : other_keys_event,
    }
    GIF = load_gif("gifs/empoleon.gif")
    GIF_INDEX = 0

    tft = Adafruit_ST77XX(event_mapping)
    tft.init(160, 128)
    tft.set_delay(300)
    tft.set_scale(3.0)

    tft.set_display_brightness(255)
    tft.fillScreen(ST77XX_BLACK)
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK)

    tft.setTextWrap(False)
    tft.setTextSize(1)
    tft.setRotation(3)

    tft.print("Hello!\n")
    print("Display ready")

    menu_mapping = {
        "MAIN_MENU"    : draw_main_menu,
        "NETWORK_MENU" : draw_network_menu,
        "DRIVE_MENU"   : draw_drive_menu,
        "LINEAR_MENU"  : draw_linear_menu,
        "TILTER_MENU"  : draw_tilter_menu,
        "GRIPPER_MENU" : draw_gripper_menu,
        "BREAKOUT_MENU": dodobot_breakout,
        "SHUTDOWN_MENU": draw_shutdown_menu,
        "IMAGE_MENU"   : draw_image_menu,
    }

    SCREEN_MID_W = tft.width() // 2

    prev_date_update = time.time()
    while tft.running:
        if not tft.update():
            continue

        if time.time() - prev_date_update > 0.5:
            update_date_string()
            prev_date_update = time.time()

        # tft.fillScreen(ST77XX_BLACK)
        # draw_gif(tft, GIF, GIF_INDEX, 0, 0)
        # GIF_INDEX += 1
        # if GIF_INDEX >= len(GIF):
        #     GIF_INDEX = 0

        if PREV_DISPLAYED_MENU != DISPLAYED_MENU:
            tft.fillScreen(ST77XX_BLACK)
            screen_change_event(tft)
        menu_mapping[MENUS[DISPLAYED_MENU]](tft)

        PREV_DISPLAYED_MENU = DISPLAYED_MENU

        draw_topbar(tft)

    pygame.quit()


if __name__ == '__main__':
    main()
