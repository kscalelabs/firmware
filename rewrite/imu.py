#!/usr/bin/env python3
import serial
import struct
import time
import mmap
import os

class IMUReader:
    def __init__(self, device='/dev/ttyUSB0', baudrate=230400, shm_path='/tmp/imu_shm'):
        # Serial setup
        self.serial = serial.Serial(device, baudrate, timeout=0)
        
        # Shared memory setup (36 bytes: timestamp + 3 gyro + 4 quaternion floats)
        self.shm_path = shm_path
        self.shm_size = 36
        
        # Create shared memory file if it doesn't exist
        if not os.path.exists(shm_path):
            with open(shm_path, 'wb') as f:
                f.write(b'\x00' * self.shm_size)
        
        # Map shared memory
        with open(shm_path, 'r+b') as f:
            self.shm = mmap.mmap(f.fileno(), self.shm_size)
        
        # Track last values to avoid unnecessary writes
        self.last_gyro = None
        self.last_quaternion = None
        
    def _parse_gyro(self, data):
        """Parse gyroscope data from IMU packet"""
        gx = struct.unpack('<h', data[2:4])[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
        gy = struct.unpack('<h', data[4:6])[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
        gz = struct.unpack('<h', data[6:8])[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
        return (gx, gy, gz)
    
    def _parse_quaternion(self, data):
        """Parse quaternion data from IMU packet"""
        qw = struct.unpack('<h', data[2:4])[0] / 32768.0
        qx = struct.unpack('<h', data[4:6])[0] / 32768.0
        qy = struct.unpack('<h', data[6:8])[0] / 32768.0
        qz = struct.unpack('<h', data[8:10])[0] / 32768.0
        return (qw, qx, qy, qz)
    
    def _update_shared_memory(self, timestamp, gyro_data=None, quaternion_data=None):
        """Update shared memory only if data has changed"""
        data_changed = False
        
        if gyro_data and gyro_data != self.last_gyro:
            self.last_gyro = gyro_data
            data_changed = True
            
        if quaternion_data and quaternion_data != self.last_quaternion:
            self.last_quaternion = quaternion_data
            data_changed = True
            
        if data_changed:
            # Pack data: timestamp + gyro (3 floats) + quaternion (4 floats)
            gyro = gyro_data or (0.0, 0.0, 0.0)
            quaternion = quaternion_data or (0.0, 0.0, 0.0, 0.0)
            
            packed_data = struct.pack('<dfffffff', timestamp, *gyro, *quaternion)
            
            # Write to shared memory
            self.shm.seek(0)
            self.shm.write(packed_data)
    
    def run(self):
        """Main IMU reading loop"""
        last_update = time.time()
        
        while True:
            time.sleep(0.0001)  # 100us delay to decrease CPU usage from 100% to 7%
            
            # Read byte-by-byte until we find sync byte
            if self.serial.read(1) == b'\x55':
                # Found sync, read remaining 10 bytes
                data = b'\x55' + self.serial.read(10)
                
                if len(data) == 11 and (sum(data[:10]) & 0xFF) == data[10]:
                    now = time.time()
                    
                    if data[1] == 0x52:  # Gyro
                        gyro_data = self._parse_gyro(data)
                        self._update_shared_memory(now, gyro_data=gyro_data)
                        last_update = now
                        
                    elif data[1] == 0x59:  # Quaternion
                        quaternion_data = self._parse_quaternion(data)
                        self._update_shared_memory(now, quaternion_data=quaternion_data)
                        last_update = now

    def test(self):
        """Test function that runs the IMU reader and prints data"""
        last_print = time.time()
        
        while True:
            time.sleep(0.0001)  # 100us delay to decrease CPU usage from 100% to 7%
            
            # Read byte-by-byte until we find sync byte
            if self.serial.read(1) == b'\x55':
                # Found sync, read remaining 10 bytes
                data = b'\x55' + self.serial.read(10)
                
                if len(data) == 11 and (sum(data[:10]) & 0xFF) == data[10]:
                    now = time.time()
                    dt = now - last_print
                    
                    if data[1] == 0x52:  # Gyro
                        gyro_data = self._parse_gyro(data)
                        print(f"dt={dt:.3f} gyro: x={gyro_data[0]:.3f} y={gyro_data[1]:.3f} z={gyro_data[2]:.3f}")
                        self._update_shared_memory(now, gyro_data=gyro_data)
                        last_print = now
                        
                    elif data[1] == 0x59:  # Quaternion
                        quaternion_data = self._parse_quaternion(data)
                        print(f"dt={dt:.3f} quat: w={quaternion_data[0]:.3f} x={quaternion_data[1]:.3f} y={quaternion_data[2]:.3f} z={quaternion_data[3]:.3f}")
                        self._update_shared_memory(now, quaternion_data=quaternion_data)
                        last_print = now

# Usage example (for testing)
if __name__ == "__main__":
    reader = IMUReader()
    reader.test()