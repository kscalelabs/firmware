#!/bin/sh

# Runs for CAN ID 0x141 to 0x15B
# for i in $(seq 321 347); do
#     can_id=$(printf '%03X' $i)
#     cansend can0 ${can_id}#7600000000000000
#     sleep 0.1
# done

# Runs for arms.
# for i in 2 3 4 5 6 7 8 9 10 11 12 13; do
#     can_id=$(printf '%03X' $((320 + i)))
#     cansend can0 ${can_id}#7600000000000000
#     sleep 0.1
# done

# # Runs for calfs.
# for i in 20 21 26 27; do
#     can_id=$(printf '%03X' $((320 + i)))
#     cansend can0 ${can_id}#7600000000000000
#     sleep 0.1
# done

# # Runs for rest of legs.
# for i in 1 16 17 18 19 22 23 24 25; do
#     can_id=$(printf '%03X' $((320 + i)))
#     cansend can0 ${can_id}#7600000000000000
#     sleep 0.1
# done

# Runs for head.
for i in 14 15; do
    can_id=$(printf '%03X' $((320 + i)))
    cansend can0 ${can_id}#7600000000000000
    sleep 0.1
done
