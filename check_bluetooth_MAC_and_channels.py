#!/usr/bin/env python3

# Copyright 2021 Morten Melby Dahl.
# Copyright 2021 Norwegian University of Science and Technology.
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

import os
import socket


hciconfig_output = os.popen("hciconfig")

output = hciconfig_output.readlines()

# Takes the first available bluetooth adapter and gets the MAC address.
# If you want to use a different bluetooth adapter, manually run
# 'hciconfig' in the linux terminal and extract the correct MAC.

MAC = output[1].strip().split()[2]

min_port = 1
max_port = 30

ports = list(range(min_port, max_port + 1))

available_ports = []
busy_ports = []


def main():
    for port in ports:
        try:
            _socket = socket.socket(
                socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
            )
            _socket.bind((MAC, port))
            available_ports.append(port)
            _socket.close()
        except OSError:
            busy_ports.append(port)
    print("-----------------------------------------")
    print("Your Bluetooth adapters MAC address is:\n", MAC)
    print("-----------------------------------------")
    print("Busy Bluetooth channels [{}-{}]:\n".format(min_port, max_port), busy_ports)
    print("-----------------------------------------")


if __name__ == "__main__":
    main()