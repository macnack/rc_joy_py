#!/usr/bin/env python3

# Copyright 2024 Maciej Krupka
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import serial


def open_serial_port(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate)
        print(f"Successfully opened the port {port}")
        return ser
    except serial.serialutil.SerialException as e:
        print(f"Error opening port {port}: {e}")
        # Here, you could try opening a different port or take other recovery actions.
        return None


def convert_to_float(value):
    # Convert to range [-1, 1]
    return value / 32767.0


def convert_to_one(value):
    # Convert to range [0, 1]
    return (value + 32767) / 65535.0


def convert_to_button(value):
    # Convert to boolean
    return value > 0


def convert_to_buttons(value):
    # Convert to
    if value < -int(32767 / 2):
        return [True, False, False]
    elif value > int(32767 / 2):
        return [False, False, True]
    else:
        return [False, True, False]


class RcJoy:

    def __init__(self, port) -> None:
        self.ser = open_serial_port(port=port, baud_rate=115200)
        self.init = 0

    def Connected(self):
        if self.ser is None:
            return False
        return True

    def read(self):
        try:
            line = self.ser.readline().decode().strip()

            if line:
                try:
                    values = [int(val) for val in line.split(',')]

                    if len(values) == 5:
                        joystick = {
                            'throttle': convert_to_float(values[4]),
                            'button': convert_to_button(values[1]),
                            'left_gain': convert_to_one(values[2]),
                            'trigger': convert_to_buttons(values[3]),
                            'steering': convert_to_float(values[0])}
                        return joystick
                    else:
                        if self.init < 10:
                            print("Sync received values.")
                            self.init += 1
                        else:
                            print("Received incorrect number of values.")
                        return None
                except ValueError:
                    print("Error converting values to integers.")
                    return None
            else:
                print("No data received.")
                return None
        except serial.serialutil.SerialException:
            # Handle serial exception (e.g., log, retry, or exit)
            print("Serial exception occurred.")
            return None
