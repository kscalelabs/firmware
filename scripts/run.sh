./start.sh can0
./start.sh can1
python3 can_motor_emulator.py
./teardown.sh can0
./teardown.sh can1