#!/usr/bin/env bash
# Reset
COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow

make -j 12

# Bash ranges can be continious like in {1..9} or non continious like in 2 {4..5}
for i in {5..6}; do
    printf "${YELLOW}Flashing CF 0${i}${COLOR_RESET}\n"
    CLOAD_CMDS="-w radio://0/120/2M/E7E7E7E70${i}" make cload
done
