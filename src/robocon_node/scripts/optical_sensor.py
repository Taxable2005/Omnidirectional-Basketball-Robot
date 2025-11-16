#!/usr/bin/env python3

from evdev import InputDevice, ecodes

mouse = InputDevice('/dev/input/by-id/usb-1ea7_2.4G_Mouse-if01-event-mouse')

x = y = 0
for event in mouse.read_loop():
    if event.type == ecodes.EV_REL:
        if event.code == ecodes.REL_X:
            x += event.value
        elif event.code == ecodes.REL_Y:
            y += event.value
        print(f"X: {x}, Y: {y}")
