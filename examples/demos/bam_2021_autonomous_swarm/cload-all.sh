#!/usr/bin/env bash
file="cf2.bin"
baseUri="radio://0/80/2M/E7E7E7E7"

echo "Flasing 1"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}00
echo "Flasing 2"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}01
echo "Flasing 3"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}02
echo "Flasing 4"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}03
echo "Flasing 5"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}04
echo "Flasing 6"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}05
echo "Flasing 7"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}06
echo "Flasing 8"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}07
echo "Flasing 9"
python3 -m cfloader flash ${file} stm32-fw -w ${baseUri}08
