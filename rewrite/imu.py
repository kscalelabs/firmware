import serial
import struct
import time
import mmap
import os
import multiprocessing
from multiprocessing import Process
import atexit
import signal


def quaternion_conjugate(q):
    """Compute quaternion conjugate"""
    qw, qx, qy, qz = q
    return (qw, -qx, -qy, -qz)

def quaternion_multiply(q1, q2):
    """Multiply two quaternions"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return (w, x, y, z)

def rotate_vector_by_quaternion(v, q, inverse=False):
    """Rotate vector v by quaternion q"""
    vx, vy, vz = v

    # Convert vector to quaternion (0, vx, vy, vz)
    v_quat = (0.0, vx, vy, vz)

    if inverse:
        # For inverse rotation, use conjugate of q
        q_conj = quaternion_conjugate(q)
        # Rotate: v' = q_conj * v_quat * q
        temp = quaternion_multiply(q_conj, v_quat)
        result_quat = quaternion_multiply(temp, q)
    else:
        # Rotate: v' = q * v_quat * q_conj
        q_conj = quaternion_conjugate(q)
        temp = quaternion_multiply(q, v_quat)
        result_quat = quaternion_multiply(temp, q_conj)

    # Extract vector part from result quaternion
    return (result_quat[1], result_quat[2], result_quat[3])


def parse_gyro(data):
    """Parse gyroscope data from IMU packet"""
    gx = struct.unpack('<h', data[2:4])[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
    gy = struct.unpack('<h', data[4:6])[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
    gz = struct.unpack('<h', data[6:8])[0] / 32768.0 * 2000.0 * 3.14159 / 180.0
    return (gx, gy, gz)

def parse_quaternion(data):
    """Parse quaternion data from IMU packet"""
    qw = struct.unpack('<h', data[2:4])[0] / 32768.0
    qx = struct.unpack('<h', data[4:6])[0] / 32768.0
    qy = struct.unpack('<h', data[6:8])[0] / 32768.0
    qz = struct.unpack('<h', data[8:10])[0] / 32768.0
    return (qw, qx, qy, qz)


def update_shared_memory(shm, shm_lock, timestamp, gyro, quaternion):
    """Write a complete record to shared memory (timestamp, gyro, quaternion)."""
    packed_data = struct.pack(
        '<dfffffff',
        timestamp,
        gyro[0], gyro[1], gyro[2],
        quaternion[0], quaternion[1], quaternion[2], quaternion[3],
    )
    if shm_lock is not None:
        shm_lock.acquire()
        try:
            shm.seek(0)
            shm.write(packed_data)
        finally:
            shm_lock.release()
    else:
        shm.seek(0)
        shm.write(packed_data)


class IMUReader:
    """Reads IMU data from a serial port in a separate process and shares via shared memory"""
    def __init__(self, device='/dev/ttyUSB0', baudrate=230400, shm_path='/tmp/imu_shm'):
        # Serial configuration (child process will open the port)
        self.device = device
        self.baudrate = baudrate

        # Shared memory setup (36 bytes: timestamp + 3 gyro + 4 quaternion floats)
        self.shm_path = shm_path
        self.shm_size = 36
        self.shm = self.create_shared_memory()

        # Process management
        self.process = None
        self.running = multiprocessing.Event()

        # Shared memory synchronization
        self.shm_lock = multiprocessing.Lock()

        # standard gravity
        self.gravity = (0.0, 0.0, -9.81)

        # Register clean shutdown hooks only for the main process
        self._register_shutdown_handlers()

    def _register_shutdown_handlers(self):
        def _safe_stop(*_args, **_kwargs):
            # Ensure handlers only run in the main process
            if multiprocessing.current_process().name != 'MainProcess':
                return
            try:
                self.stop()
            except Exception:
                pass

        try:
            atexit.register(_safe_stop)
        except Exception:
            pass

        try:
            signal.signal(signal.SIGINT, lambda s, f: _safe_stop())
            signal.signal(signal.SIGTERM, lambda s, f: _safe_stop())
        except Exception:
            # Signal setup may fail in some contexts (e.g., not main thread)
            pass

    def create_shared_memory(self):
        if not os.path.exists(self.shm_path):
            with open(self.shm_path, 'wb') as f:
                f.write(b'\x00' * self.shm_size)

        # Map shared memory
        with open(self.shm_path, 'r+b') as f:
            shm = mmap.mmap(f.fileno(), self.shm_size)
        return shm

    def __del__(self):
        """Destructor to clean up resources"""
        self.stop()
        if hasattr(self, 'shm'):
            self.shm.close()
        

    @staticmethod
    def _imu_reading_loop(device, baudrate, shm_path, shm_size, running_event, shm_lock):
        """Standalone IMU reading loop that runs in a separate process"""
        # Serial setup
        serial_conn = serial.Serial(device, baudrate, timeout=0)

        # Shared memory setup (child creates/opens and maps its own view)
        if not os.path.exists(shm_path):
            with open(shm_path, 'wb') as f:
                f.write(b'\x00' * shm_size)
        with open(shm_path, 'r+b') as f:
            shm = mmap.mmap(f.fileno(), shm_size)

        last_gyro = (0.0, 0.0, 0.0)
        last_quaternion = (0.0, 0.0, 0.0, 0.0)

        while running_event.is_set():
            time.sleep(0.0001)

            # Read byte-by-byte until we find sync byte
            if serial_conn.read(1) == b'\x55':
                # Found sync, read remaining 10 bytes
                data = b'\x55' + serial_conn.read(10)

                if len(data) == 11 and (sum(data[:10]) & 0xFF) == data[10]:
                    now = time.time()
                    if data[1] == 0x52:  # Gyro
                        last_gyro = parse_gyro(data)
                    elif data[1] == 0x59:  # Quaternion
                        last_quaternion = parse_quaternion(data)
                    
                    update_shared_memory(shm, shm_lock, now, last_gyro, last_quaternion)

        # Cleanup
        try:
            shm.close()
        except Exception:
            pass
        serial_conn.close()

    def start(self):
        """Start the IMU reading process"""
        if self.process is None or not self.process.is_alive():
            self.running.set()
            self.process = Process(
                target=self._imu_reading_loop,
                args=(self.device, self.baudrate, self.shm_path, self.shm_size, self.running, self.shm_lock))
            # Ensure child exits if parent dies unexpectedly
            self.process.daemon = True
            self.process.start()

    def stop(self):
        """Stop the IMU reading process"""
        if self.process is not None and self.process.is_alive():
            self.running.clear()
            self.process.join(timeout=1.0)
            if self.process.is_alive():
                self.process.terminate()
            self.process = None

    def test(self):
        """Test function that runs the IMU reader and prints data"""
        self.start()
        start_time = time.time()
        while True:
            time.sleep(0.1)
            projgrav, gyro, timestamp = self.get_projected_gravity_and_gyroscope()
            print(f"projected_gravity: (\033[94m{projgrav[0]:.4f}\033[0m, \033[94m{projgrav[1]:.4f}\033[0m, \033[94m{projgrav[2]:.4f}\033[0m), gyro: (\033[92m{gyro[0]:.4f}\033[0m, \033[92m{gyro[1]:.4f}\033[0m, \033[92m{gyro[2]:.4f}\033[0m), timestamp: {timestamp - start_time:.3f}")

    def get_projected_gravity_and_gyroscope(self):
        """Get the latest projected gravity and gyroscope data from shared memory

        Returns:
            tuple: (projected_gravity, gyro, timestamp) where projected_gravity and gyro are tuples of 3 floats, timestamp is float
        """
        try:
            # Read from shared memory with lock
            with self.shm_lock:
                self.shm.seek(0)
                packed_data = self.shm.read(self.shm_size)

            if len(packed_data) == self.shm_size:
                # Unpack data: timestamp + gyro (3 floats) + quaternion (4 floats)
                unpacked_data = struct.unpack('<dfffffff', packed_data)
                timestamp = unpacked_data[0]
                gyro = unpacked_data[1:4]
                quaternion = unpacked_data[4:8]

                # Calculate projected gravity from quaternion
                proj_grav = rotate_vector_by_quaternion(self.gravity, quaternion, inverse=True)

                return proj_grav, gyro, timestamp
            else:
                # Return default values if shared memory is not properly initialized
                return (0.0, 0.0, -9.81), (0.0, 0.0, 0.0), 0.0

        except Exception as e:
            # Handle any errors gracefully
            print(f"Error reading from shared memory: {e}")
            return (0.0, 0.0, -9.81), (0.0, 0.0, 0.0), 0.0



# for testing
if __name__ == "__main__":
    reader = IMUReader()
    reader.test()

