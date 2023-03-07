#!/usr/bin/env bash
COLOR_RESET='\033[0m'       # Text Reset
YELLOW='\033[0;33m'         # Yellow

# define array of uris
uris=(
    "radio://0/70/2M/E7E7E7E701"
    "radio://0/70/2M/E7E7E7E702"
    "radio://0/70/2M/E7E7E7E703"
    "radio://0/70/2M/E7E7E7E704"
    "radio://0/70/2M/E7E7E7E705"
    "radio://0/70/2M/E7E7E7E706"
    "radio://0/70/2M/E7E7E7E707"
    "radio://0/70/2M/E7E7E7E708"
    "radio://0/70/2M/E7E7E7E709"
)

for uri in "${uris[@]}" ; do
    printf "${YELLOW}Flashing CF $uri ${COLOR_RESET}\n"
    CLOAD_CMDS="-w ${uri}" make cload
done
