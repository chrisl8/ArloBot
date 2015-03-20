#!/bin/bash
# The 'courtesy loader' is required for Python to run these directly
SCRIPTDIR=$(cd $(dirname "$0") && pwd)
cd ${SCRIPTDIR}/../node
./arloBehavior.sh

