#!/bin/sh

# Runs for CAN ID 0x141 to 0x15B
for i in $(seq 321 347); do
    can_id=$(printf '%03X' $i)
    cansend can0 ${can_id}#9200000000000000
    sleep 0.1
    # candump can0 -n 1
done
