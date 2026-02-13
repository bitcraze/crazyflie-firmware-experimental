#!/usr/bin/env bash
echo "Flasing 1"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA01
echo "Flasing 2"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA02
echo "Flasing 3"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA03
echo "Flasing 4"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA04
echo "Flasing 5"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA05
echo "Flasing 6"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA06
echo "Flasing 7"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA07
echo "Flasing 8"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA08
echo "Flasing 9"
python3 -m cfloader flash build/cf21bl.bin stm32-fw -w radio://0/90/2M/ABAD1DEA09

