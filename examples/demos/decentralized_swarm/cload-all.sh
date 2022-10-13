#!/usr/bin/env bash
# Reset
COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow

make -j 12

printf "${YELLOW}Flashing CF 01${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/120/2M/E7E7E70101" make cload
printf "${YELLOW}Flashing CF 02${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/120/2M/E7E7E70102" make cload
printf "${YELLOW}Flashing CF 03${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/120/2M/E7E7E70103" make cload
printf "${YELLOW}Flashing CF 04${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/120/2M/E7E7E70104" make cload
printf "${YELLOW}Flashing CF 05${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/120/2M/E7E7E70105" make cload
printf "${YELLOW}Flashing CF 06${COLOR_RESET}\n"
CLOAD_CMDS="-w radio://0/120/2M/E7E7E70106" make cload
printf "${YELLOW}Flashing CF 07${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/120/2M/E7E7E70107" make cload
# printf "${YELLOW}Flashing CF 08${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/120/2M/E7E7E70108" make cload
# printf "${YELLOW}Flashing CF 09${COLOR_RESET}\n"
# CLOAD_CMDS="-w radio://0/120/2M/E7E7E70109" make cload
