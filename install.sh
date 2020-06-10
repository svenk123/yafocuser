#!/bin/sh
#
# File: install.sh
# Project: yafocuser
#
# Created by Sven Kreiensen on 06.11.19.
# Copyright © 2019 Sven Kreiensen. All rights reserved.
#

echo "Press RESET on teensy2.0 board..."

./teensy_loader_cli_macos -v --mcu=atmega32u4 -w yafocuser.hex
