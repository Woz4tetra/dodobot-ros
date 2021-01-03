from evdev import ecodes

SHIFT_KEYS = [ecodes.KEY_LEFTSHIFT, ecodes.KEY_RIGHTSHIFT]

UPPERCASE_LETTERS = {
    ecodes.KEY_Q: "Q",
    ecodes.KEY_W: "W",
    ecodes.KEY_E: "E",
    ecodes.KEY_R: "R",
    ecodes.KEY_T: "T",
    ecodes.KEY_Y: "Y",
    ecodes.KEY_U: "U",
    ecodes.KEY_I: "I",
    ecodes.KEY_O: "O",
    ecodes.KEY_P: "P",
    ecodes.KEY_A: "A",
    ecodes.KEY_S: "S",
    ecodes.KEY_D: "D",
    ecodes.KEY_F: "F",
    ecodes.KEY_G: "G",
    ecodes.KEY_H: "H",
    ecodes.KEY_J: "J",
    ecodes.KEY_K: "K",
    ecodes.KEY_L: "L",
    ecodes.KEY_Z: "Z",
    ecodes.KEY_X: "X",
    ecodes.KEY_C: "C",
    ecodes.KEY_V: "V",
    ecodes.KEY_B: "B",
    ecodes.KEY_N: "N",
    ecodes.KEY_M: "M",
}

LOWERCASE_LETTERS = {
    ecodes.KEY_Q: "q",
    ecodes.KEY_W: "w",
    ecodes.KEY_E: "e",
    ecodes.KEY_R: "r",
    ecodes.KEY_T: "t",
    ecodes.KEY_Y: "y",
    ecodes.KEY_U: "u",
    ecodes.KEY_I: "i",
    ecodes.KEY_O: "o",
    ecodes.KEY_P: "p",
    ecodes.KEY_A: "a",
    ecodes.KEY_S: "s",
    ecodes.KEY_D: "d",
    ecodes.KEY_F: "f",
    ecodes.KEY_G: "g",
    ecodes.KEY_H: "h",
    ecodes.KEY_J: "j",
    ecodes.KEY_K: "k",
    ecodes.KEY_L: "l",
    ecodes.KEY_Z: "z",
    ecodes.KEY_X: "x",
    ecodes.KEY_C: "c",
    ecodes.KEY_V: "v",
    ecodes.KEY_B: "b",
    ecodes.KEY_N: "n",
    ecodes.KEY_M: "m",
}

assert len(UPPERCASE_LETTERS) == len(LOWERCASE_LETTERS)

NUMBERS = {
    ecodes.KEY_1: "1",
    ecodes.KEY_2: "2",
    ecodes.KEY_3: "3",
    ecodes.KEY_4: "4",
    ecodes.KEY_5: "5",
    ecodes.KEY_6: "6",
    ecodes.KEY_7: "7",
    ecodes.KEY_8: "8",
    ecodes.KEY_9: "9",
    ecodes.KEY_0: "0",
}

NONSHIFT_KEYS = {
    ecodes.KEY_TAB: "\t",
    ecodes.KEY_ENTER: "\n",
    ecodes.KEY_ESC: "\x1b",
    ecodes.KEY_BACKSPACE: "\x08",
    ecodes.KEY_DELETE: "\x7f",
    ecodes.KEY_SPACE: " ",

    ecodes.KEY_KPENTER: "\n",
    ecodes.KEY_KPASTERISK: "*",
    ecodes.KEY_KPPLUS: "+",
    ecodes.KEY_KP1: "1",
    ecodes.KEY_KP2: "2",
    ecodes.KEY_KP3: "3",
    ecodes.KEY_KP4: "4",
    ecodes.KEY_KP5: "5",
    ecodes.KEY_KP6: "6",
    ecodes.KEY_KP7: "7",
    ecodes.KEY_KP8: "8",
    ecodes.KEY_KP9: "9",
    ecodes.KEY_KP0: "0",
    ecodes.KEY_KPSLASH: "/",
    ecodes.KEY_KPMINUS: "-",
    ecodes.KEY_KPDOT: ".",
}

UPPERCASE_KEYS = {
    ecodes.KEY_1: "!",
    ecodes.KEY_2: "@",
    ecodes.KEY_3: "#",
    ecodes.KEY_4: "$",
    ecodes.KEY_5: "%",
    ecodes.KEY_6: "^",
    ecodes.KEY_7: "&",
    ecodes.KEY_8: "*",
    ecodes.KEY_9: "(",
    ecodes.KEY_0: ")",
    ecodes.KEY_MINUS: "_",
    ecodes.KEY_EQUAL: "+",
    ecodes.KEY_LEFTBRACE: "{",
    ecodes.KEY_RIGHTBRACE: "}",
    ecodes.KEY_BACKSLASH: "|",
    ecodes.KEY_SEMICOLON: ":",
    ecodes.KEY_APOSTROPHE: "\"",
    ecodes.KEY_GRAVE: "~",
    ecodes.KEY_COMMA: "<",
    ecodes.KEY_DOT: ">",
    ecodes.KEY_SLASH: "?",

}
UPPERCASE_KEYS.update(UPPERCASE_LETTERS)
UPPERCASE_KEYS.update(NONSHIFT_KEYS)

CAPSLOCK_KEYS = {}
CAPSLOCK_KEYS.update(NUMBERS)
CAPSLOCK_KEYS.update(UPPERCASE_LETTERS)
CAPSLOCK_KEYS.update(NONSHIFT_KEYS)


LOWERCASE_KEYS = {
    ecodes.KEY_LEFTBRACE: "[",
    ecodes.KEY_RIGHTBRACE: "]",
    ecodes.KEY_BACKSLASH: "\\",
    ecodes.KEY_MINUS: "-",
    ecodes.KEY_EQUAL: "=",
    ecodes.KEY_SEMICOLON: ";",
    ecodes.KEY_APOSTROPHE: "'",
    ecodes.KEY_COMMA: ",",
    ecodes.KEY_DOT: ".",
    ecodes.KEY_SLASH: "/",
    ecodes.KEY_GRAVE: "`",
}
LOWERCASE_KEYS.update(NUMBERS)
LOWERCASE_KEYS.update(LOWERCASE_LETTERS)
LOWERCASE_KEYS.update(NONSHIFT_KEYS)
