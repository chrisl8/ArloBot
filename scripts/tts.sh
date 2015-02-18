#!/bin/bash
# NOTE: The curtesy loader above is REQUIRED for python called shell scripts!
SCRIPTDIR=$(cd $(dirname "$0") && pwd)
node ${SCRIPTDIR}/tts.js "${1}"
