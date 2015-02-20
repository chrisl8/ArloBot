#!/bin/bash
# The 'courtesy loader' is required for Python to run these directly
find ~/.arlobot/rosmaps/ -name *.yaml -type f -printf "%f\n"|sed -e "s/.yaml//"

