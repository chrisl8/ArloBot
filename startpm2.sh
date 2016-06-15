#!/usr/bin/env bash
export NVM_DIR="${HOME}/.nvm"
[ -s "$NVM_DIR/nvm.sh"  ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
${HOME}/.nvm/versions/node/$(nvm current)/bin/pm2 start ${HOME}/catkin_ws/src/ArloBot/node/pm2Config.json

