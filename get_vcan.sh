#!/usr/bin/env bash

for i in $(seq 0 6); do
    sudo ip link add vcan${i} type vcan
    sudo ip link set vcan${i} up
    sleep 0.5
done
