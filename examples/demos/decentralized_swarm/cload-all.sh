#!/usr/bin/env bash
# Reset
COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow
RED='\033[0;31m'            # Red

# Bash ranges can be continious like in {1..9} or non continious like in 2 {4..5}
for i in {1..6}; do
    make -j 8
    printf "${YELLOW}Flashing ${RED}CF${YELLOW} 0${i}${COLOR_RESET}\n"
    CLOAD_CMDS="-w radio://0/90/2M/E7E7E7E70${i}" make cload
done

for i in {7..9}; do
    make -j 8 BRUSHLESS=1
    printf "${YELLOW}Flashing ${RED}CF BRUSHLESS${YELLOW} 0${i}${COLOR_RESET}\n"
    CLOAD_CMDS="-w radio://0/90/2M/E7E7E7E70${i}" make cload
done
