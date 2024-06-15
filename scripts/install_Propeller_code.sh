#!/usr/bin/env bash
# shellcheck disable=SC2059

if [[ $(jq '.hasActivityBoard' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  check_hardware.sh
  cd "${HOME}/dev_ws/src/ArloBot/PropellerCodeForArloBot/ROSInterfaceForArloBot/bin" || exit
  rm -rf ./*
  cmake -G "Unix Makefiles" ..
  PROPELLER_LOAD_PORT=$(find_ActivityBoard.sh) make run
else
  YELLOW='\033[1;33m'
  BLUE='\033[0;34m'
  NC='\033[0m' # NoColor
  printf "\n${YELLOW}Cannot load Propeller Activity Board code without an Activity board.!${NC}\n"
  printf "${BLUE}If you do have a board, have you edited ~/.arlobot/personalDataForBehavior.json yet?${NC}\n"
fi
