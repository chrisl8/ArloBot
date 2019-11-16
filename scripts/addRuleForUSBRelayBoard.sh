#!/usr/bin/env bash

echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="*", GROUP="dialout", MODE="0660"' >/etc/udev/rules.d/99-libftdi.rules
