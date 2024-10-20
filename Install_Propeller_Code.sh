#!/usr/bin/env bash
# shellcheck disable=SC2059

set -e

if [[ $(arch) = "aarch64" ]];then
  printf "\n${YELLOW}The Propeller code cannot be built on a Pi! It has to be done on an x86_64 system!${NC}\n"
  printf "${LIGHTBLUE}You will need to build this code on an x86_64 system and  load the code onto your Propeller Activity Board before this code can work on your robot!${NC}\n"
  exit 1
fi

function finish() {
  if [[ -z ${INSTALL_FINISHED} ]]; then
    printf "\n"
    printf "${RED}INSTALL FAILURE!!!${NC}\n"
    printf "${RED}The Install Script has failed. Please investigate the cause, correct, and run again before proceeding.${NC}\n"
    printf "\n"
    printf "${YELLOW}If this was a transient error, such as a network failure connecting to something, you may just need to run it again.${NC}\n"
    printf "\n"
    printf "${YELLOW}If this persists, please file an issue against the repository at https://github.com/chrisl8/ArloBot/issues${NC}\n"
    printf "\n"
    exit 1
  fi
}
trap finish EXIT

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
#echo "${SCRIPT_DIR}" # For debugging

printf "\n${YELLOW}[Installing Ubuntu Dependencies]${NC}\n"

DOCKER_TEST_INSTALL=false
if [[ ! -e /etc/localtime ]]; then
  # These steps are to allow this script to work in a minimal Docker container for testing.
  printf "${YELLOW}[This looks like a Docker setup.]${NC}\n"
  printf "${LIGHTBLUE}Adding settings and basic packages for Docker based Ubuntu images.${NC}\n"
  apt update # Docker doesn't have the latest package lists by default.
  apt install -y sudo
  # Now the rest of the script should work as if it was in a normal Ubuntu install.
  DOCKER_TEST_INSTALL=true
fi

if ! (command -v git >/dev/null); then
  sudo apt install -y git
fi
if ! (command -v cmake >/dev/null); then
  sudo apt install -y cmake
fi
if ! (command -v wget >/dev/null); then
  sudo apt install -y wget
fi

printf "\n${YELLOW}[Cloning or Updating Arlobot git repositories]${NC}\n"
if ! [[ -d "${HOME}/ArloBot" ]]; then
  cd "${HOME}"
  git clone -b jazzy https://github.com/chrisl8/ArloBot.git
else
  cd "${HOME}/ArloBot"
  git fetch
  git checkout jazzy
  git pull
fi

ARLO_HOME=${HOME}/.arlobot
if ! [[ -d "${ARLO_HOME}" ]]; then
  mkdir "${ARLO_HOME}"
fi

for i in "${HOME}/ArloBot/PropellerCodeForArloBot/dotfiles/"*; do
  [[ -e "${i}" ]] || break # handle the case of no files
  # https://stackoverflow.com/a/9011264/4982408
  if [[ -e ${ARLO_HOME}/${i##*/} ]]; then
    if ! (diff "${i}" "${ARLO_HOME}/${i##*/}" >/dev/null); then
      printf "\n${GREEN}The ${RED}${i##*/}${GREEN} file in the repository is different from the one${NC}\n"
      printf "${GREEN}in your local settings.${NC}\n"
      printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
      printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
      diff "${i}" "${ARLO_HOME}/${i##*/}" || true
      # cp -i "${i}" "${ARLO_HOME}/"
    fi
  else
    printf "\n"
    cp "${i}" "${ARLO_HOME}/"
    printf "${GREEN}A brand new ${RED}${ARLO_HOME}/${i##*/}${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot and re-run this script!${NC}\n\n"
    printf "${LIGHT_PURPLE}The following code build will probably be incorrect for your setup.${NC}\n"
  fi
done

if ! [[ -e /usr/share/PropWare/include/arlodrive.h ]]; then
  printf "\n${YELLOW}[Setting up PropWare and PropGCC for putting code on Propeller Activity Board.]${NC}\n"
  printf "${LIGHTBLUE}Parallax no longer supports Linux so we are using some third party tools.${NC}\n"
  printf "${LIGHTBLUE}https://david.zemon.name/PropWare${NC}\n"
  cd /tmp
  # NOTE: This is the original location, but it has gone dark
  # wget -O propware_3.0.0.224-1_all.deb https://ci.zemon.name/repository/download/PropWare_Develop/3817:id/propware_3.0.0.224-1_all.deb?guest=1
  wget -O propware_3.0.0.224-1_all.deb https://github.com/chrisl8/propeller_binary_archive/raw/refs/heads/main/propware_3.0.0.224-1_all.deb?download=1
  sudo dpkg -i /tmp/propware_3.0.0.224-1_all.deb
  rm /tmp/propware_3.0.0.224-1_all.deb
fi

if ! [[ -d /opt/parallax ]]; then
  printf "\n${YELLOW}[Installing PropGCC, which is required by PropWare.]${NC}\n"
  cd /tmp
  # NOTE: This is the original location, but it has gone dark
  # wget -O propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz https://ci.zemon.name/repository/download/PropGCC5_Gcc4linuxX64/3620:id/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz?guest=1
  wget -O propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz https://github.com/chrisl8/propeller_binary_archive/raw/refs/heads/main/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz?download=1
  sudo cp /tmp/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz /opt
  rm /tmp/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz
  cd /opt
  sudo tar xf propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz
  sudo rm /opt/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz
  cd # Get out of /opt just for safety
fi

if ! [[ -e ${ARLO_HOME}/per_robot_settings_for_propeller_c_code.h ]]; then
  cp "${HOME}/ArloBot/PropellerCodeForArloBot/dotfiles/per_robot_settings_for_propeller_c_code.h" "${ARLO_HOME}"
fi

if ! [[ -e ${ARLO_HOME}/per_robot_settings_for_propeller_2nd_board.h ]]; then
  cp "${HOME}/ArloBot/PropellerCodeForArloBot/dotfiles/per_robot_settings_for_propeller_2nd_board.h" "${ARLO_HOME}"
fi

printf "\n${YELLOW}[Compiling Propeller Code.]${NC}\n"
printf "${LIGHTBLUE}You will need to load this code onto your Propeller board after the setup is done.${NC}\n"

function testMake() {
  printf " ${LIGHTBLUE}- ${1}${NC}\n"
  cd "${HOME}/ArloBot/PropellerCodeForArloBot/${1}"
  rm -rf bin
  if ! [[ -d bin ]]; then
    mkdir bin
  fi
  cd bin
  cmake -DCMAKE_MODULE_PATH=/usr/share/PropWare/CMakeModules -G "Unix Makefiles" ..
  make

}
testMake ROSInterfaceForArloBot
testMake MotorResponseTesting
testMake 2ndBoardCode
testMake Calib

if ! [[ ${DOCKER_TEST_INSTALL=true} == "true" ]]; then # This does not work on Docker
  "${SCRIPT_DIR}/scripts/check_hardware.sh"
  cd "${SCRIPT_DIR}/PropellerCodeForArloBot/ROSInterfaceForArloBot/bin"
  rm -rf ./*
  cmake -G "Unix Makefiles" ..
  PROPELLER_LOAD_PORT=$(find_ActivityBoard.sh) make run
fi
INSTALL_FINISHED="true"
