#!/usr/bin/env bash
# Reset
COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow
RED='\033[0;31m'            # Red

# Bash ranges can be continious like in {1..9} or non continious like in 2 {4..5}
# Change the defconfig to match the platform
for i in {1..9}; do
    make cf21bl_defconfig
    make -j 8 BRUSHLESS=1
    printf "${YELLOW}Flashing ${RED}CF BRUSHLESS${YELLOW} 0${i}${COLOR_RESET}\n"
    CLOAD_CMDS="-w radio://0/80/2M/ABAD1DEA0${i}" make cload
done
