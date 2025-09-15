import math
import time
import traceback
import socket
import struct
from typing import Dict

class CANInterface:
    """ Communication only """
    def __init__(self):
        self.FRAME_FMT = '<IBBBB8s' # <I = little-endian u32; 4B = len, pad, res0, len8_dlc; 8s = 8 data bytes
        self.FRAME_SIZE = struct.calcsize(self.FRAME_FMT)
        self.host_id = 0xFD
        self.canbus_range = range(0, 7)
        self.actuator_range = range(10, 50)

        self.MUX_PING = 0x00
        self.MUX_CONTROL = 0x01
        self.MUX_FEEDBACK = 0x02
        self.MUX_MOTOR_ENABLE = 0x03
        self.MUX_READ_PARAM = 0x11


        self.EFF = 0x8000_0000

        self.sockets = {}
        self.actuators = {}
        self._scan()

    def _scan(self):
        for canbus in self.canbus_range:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            try:
                sock.bind((f"can{canbus}",))
                self.sockets[canbus] = sock
                self.actuators[canbus] = []
            except Exception as e:
                print(f"bus {canbus} not available: {e}")
                continue

            print(f"Scanning bus {canbus}")
            for actuator_id in self.actuator_range:
                if self._ping_actuator(canbus, actuator_id):
                    self.actuators[canbus].append(actuator_id)

        print("\033[1;36m🔍 CAN scan complete\033[0m")
        total_actuators = sum(len(actuators) for actuators in self.actuators.values())
        print(f"\033[1;32mFound {total_actuators} actuators on {len(self.sockets)} sockets\033[0m")
        for canbus, actuators in self.actuators.items():
            print(f"\033[1;34m{canbus}\033[0m: \033[1;35m{actuators}\033[0m")

    def _ping_actuator(self, canbus: str, actuator_can_id: int):
        try:
            frame = self._build_ping_frame(actuator_can_id)
            self.sockets[canbus].send(frame)
            self.sockets[canbus].settimeout(0.01)
            resp_frame = self.sockets[canbus].recv(self.FRAME_SIZE)
            _ = struct.unpack(self.FRAME_FMT, resp_frame)
            return True
        except socket.timeout:
            return False
        except Exception as e:
            print(f"Error pinging actuator {actuator_can_id} on {canbus}: {e}")
            traceback.print_exc()
            return False

    def _build_ping_frame(self, actuator_can_id: int) -> bytes:
        can_id = (actuator_can_id & 0xFF) | ((self.host_id & 0xFFFF) << 8) | ((self.MUX_PING & 0x1F) << 24)
        can_id |= 0x8000_0000 # set EFF flag
        length = 8
        payload = b'\x00' * length # empty payload
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)


    def enable_motor(self, canbus: str, actuator_can_id: int):
        frame = self._build_motor_enable_frame(actuator_can_id)
        self.sockets[canbus].send(frame)
        print(f"Sent motor enable command to {canbus} actuator {actuator_can_id}")

    def _build_motor_enable_frame(self, actuator_can_id: int) -> bytes:
        can_id = ((actuator_can_id & 0xFF) | (self.host_id << 8) | ((self.MUX_MOTOR_ENABLE & 0x1F) << 24))
        can_id |= self.EFF
        length = 8
        payload = b'\x00' * length # empty payload
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)


    def get_actuator_feedback(self) -> Dict[str, int]:
        for can, sock in self.sockets.items():
            for actuator_id in self.actuators[can]:
                frame = self._build_feedback_request(actuator_id)
                sock.send(frame)
                resp_frame = sock.recv(self.FRAME_SIZE)
                result = self._parse_feedback_response(resp_frame)
                print(f"can{can}: act {actuator_id}: {result}")

    def _build_feedback_request(self, actuator_can_id: int) -> bytes:
        can_id = ((actuator_can_id & 0xFF) | (self.host_id << 8) | ((self.MUX_FEEDBACK & 0x1F) << 24))
        can_id |= self.EFF
        length = 8
        payload = b'\x00' * length # empty payload
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)

    def _parse_feedback_response(self,frame: bytes) -> Dict[str, int]:
        if len(frame) != 16:
            raise ValueError("frame must be exactly 16 bytes")

        can_id, _length, _pad, _res0, _len8, payload = struct.unpack("<IBBBB8s", frame)
        b0 = (can_id >> 0)  & 0xFF  # host_id (u8)
        b1 = (can_id >> 8)  & 0xFF  # actuator_can_id (u8)
        b2 = (can_id >> 16) & 0xFF  # fault_flags (u8)
        b3 = (can_id >> 24) & 0xFF  # mux + EFF-in-byte
        mux = b3 & 0x1F

        if mux != self.MUX_FEEDBACK:
            raise ValueError(f"unexpected mux 0x{mux:02X} in feedback response")

        angle_be, ang_vel_be, torque_be, temp_be = struct.unpack(">HHHH", payload)
        angle = self._raw_to_rad(angle_be)
        ang_vel = self._raw_to_rad(ang_vel_be)
        torque = torque_be # TODO
        temp = temp_be / 10 

        return {
            "host_id": b0,
            "actuator_can_id": b1,
            "fault_flags": b2,
            "angle_raw": angle,
            "angular_velocity_raw": ang_vel,
            "torque_raw": torque,
            "temperature_raw": temp,
        }


    def read_actuator_param(self, param_index: int) -> Dict[str, int]:
        for can, sock in self.sockets.items():
            for actuator_id in self.actuators[can]:
                frame = self._build_read_param_request(actuator_id, param_index)
                sock.send(frame)
                resp_frame = sock.recv(self.FRAME_SIZE)
                result = self._parse_read_param_response(resp_frame)
                print(f"can{can}: act {actuator_id}: param 0x{param_index:04X}: {result}")

    def read_actuator_params(self) -> Dict[str, int]:
        """Read ALL available parameters from all actuators"""
        params_to_read = {
            # Control/Operational parameters (0x7000 range)
            "run_mode": 0x7005,
            "iq_ref": 0x7006,
            "spd_ref": 0x700A,
            "limit_torque": 0x700B,
            "cur_kp": 0x7010,
            "cur_ki": 0x7011,
            "cur_filt_gain": 0x7014,
            "loc_ref": 0x7016,
            "limit_spd": 0x7017,
            "limit_cur": 0x7018,
            "mech_pos": 0x7019,
            "iqf": 0x701A,
            "mech_vel": 0x701B,
            "vbus": 0x701C,
            "loc_kp": 0x701E,
            "spd_kp": 0x701F,
            "spd_ki": 0x7020,
            "spd_filt_gain": 0x7021,
            "acc_rad": 0x7022,
            "vel_max": 0x7024,
            "acc_set": 0x7025,
            "ep_scan_time": 0x7026,
            "can_timeout": 0x7028,
            "zero_sta": 0x7029,

            # Fault/Diagnostic parameters (0x3000 range)
            "motor_fault": 0x3022,
            "warn_status": 0x3023,
            "drv_fault1": 0x3024,
            "drv_fault2": 0x3025,
        }

        print(f"Reading {len(params_to_read)} parameters from all actuators...")
        for param_name, param_index in params_to_read.items():
            print(f"Reading param 0x{param_index:04X} ({param_name})")
            self.read_actuator_param(param_index)

    def _build_read_param_request(self, actuator_can_id: int, param_index: int) -> bytes:
        can_id = ((actuator_can_id & 0xFF) | (self.host_id << 8) | ((self.MUX_READ_PARAM & 0x1F) << 24))
        can_id |= self.EFF
        length = 8
        payload = struct.pack("<HHI", param_index & 0xFFFF, 0, 0)  # index (u16), reserved (u16), reserved (u32)
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)

    def _parse_read_param_response(self, frame: bytes) -> Dict[str, int]:
        if len(frame) != 16:
            raise ValueError("frame must be exactly 16 bytes")

        can_id, _length, _pad, _res0, _len8, payload = struct.unpack("<IBBBB8s", frame)
        b0 = (can_id >> 0)  & 0xFF  # host_id (u8)
        b1 = (can_id >> 8)  & 0xFF  # actuator_can_id (u8)
        b2 = (can_id >> 16) & 0xFF  # fault_flags (u8)
        b3 = (can_id >> 24) & 0xFF  # mux + EFF-in-byte
        mux = b3 & 0x1F

        if mux != self.MUX_READ_PARAM:
            raise ValueError(f"unexpected mux 0x{mux:02X} in read param response")

        index, res1, value = struct.unpack("<HHI", payload)

        return {
            "host_id": b0,
            "actuator_can_id": b1,
            "fault_flags": b2,
            "param_index": index,
            "param_value": value,
        }


    @staticmethod
    def _raw_to_rad(x: int) -> float:
        return (x - 32768) / 16384 * (2 * math.pi)

    @staticmethod
    def _rad_to_raw(x: float) -> int:
        return int(x / (2 * math.pi) * 16384 + 32768)

    @staticmethod
    def _u16(x: float | int) -> int:
        return int(x) & 0xFFFF


    @staticmethod
    def _clamp_u16(x: float) -> int:
        return max(0, min(65535, int(round(x))))


    def set_pd_target(self, canbus: str, actuator_can_id: int, angle: float, angular_vel: float, kp: float, kd: float):
        torque_scale = 0
        angle_scale = self._rad_to_raw(angle)
        angular_vel_scale = self._rad_to_raw(angular_vel)
        print(f"Angle scale: {angle_scale}, Angular vel scale: {angular_vel_scale}")
        kp_scale = 1
        kd_scale = 0.1
        frame = self.build_pd_command(actuator_can_id, torque_scale, angle_scale, angular_vel_scale, kp_scale, kd_scale)
        self.sockets[canbus].send(frame)

        self.sockets[canbus].settimeout(0.25)
        raw = self.sockets[canbus].recv(16)
        print(f"Raw response: {raw.hex()}")
        fb = self.parse_feedback_response_pd_command(raw)
        print(f"Feedback: {fb}")
        if fb["mux"] == self.MUX_CONTROL and fb["actuator_can_id"] == actuator_can_id:
            phys = self.feedback_to_physical_pd_command(fb)
            return raw, fb, phys


    def build_pd_command(
        self,
        actuator_can_id: int,
        torque_scale: int,
        angle_scale: int,
        angular_vel_scale: int,
        kp_scale: int,
        kd_scale: int,
    ) -> bytes:
        can_id = ((actuator_can_id & 0xFF) | (self._clamp_u16(torque_scale) << 8) | ((self.MUX_CONTROL & 0x1F) << 24))
        can_id |= self.EFF

        payload = struct.pack(">HHHH",
                              self._clamp_u16(angle_scale),
                              self._clamp_u16(angular_vel_scale),
                              self._clamp_u16(kp_scale),
                              self._clamp_u16(kd_scale))
        length = 8
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)



    def parse_feedback_response_pd_command(self, frame: bytes):
        if len(frame) != 16:
            raise ValueError("expected 16-byte CAN frame")
        can_id, length, _pad, _res0, _len8, payload = struct.unpack("<IBBBB8s", frame)
        b0 = (can_id >> 0)  & 0xFF   # host_id (u8)
        b1 = (can_id >> 8)  & 0xFF   # actuator_can_id (u8)
        b2 = (can_id >> 16) & 0xFF   # fault_flags (u8)
        b3 = (can_id >> 24) & 0xFF   # mux|eff-byte
        mux = b3 & 0x1F
        angle_be, ang_vel_be, torque_be, temp_be = struct.unpack(">HHHH", payload)
        return {
            "eff": 1 if (can_id & self.EFF) else 0,
            "length": length,
            "mux": mux,
            "host_id": b0,
            "actuator_can_id": b1,
            "fault_flags": b2,
            "angle_raw": angle_be,
            "angular_velocity_raw": ang_vel_be,
            "torque_raw": torque_be,
            "temperature_raw": temp_be,
        }

    def feedback_to_physical_pd_command(self, fb: dict):
        return {
            "actuator_can_id": fb["actuator_can_id"],
            "angle_rad": self._raw_to_rad(fb["angle_raw"]),
            "angular_velocity_rad_s": self._raw_to_rad(fb["angular_velocity_raw"]),
            "torque_Nm": fb["torque_raw"],
            "temperature_C": fb["temperature_raw"],
            "fault_flags": fb["fault_flags"],
        }


class MotorDriver:
    """ Driver logic """
    def __init__(self):
        self.ci = CANInterface()

        self.ci.read_actuator_params()

        # set all act to some safe normal operating point
        # self.ci.set_pd_target()

                    # Enable motor for actuator 21 before controlling it
                    # can_interface.enable_motor("can0", 21)

                    # make act 21 move to 1 rad
                    # can_interface.set_pd_target(
                    #     canbus="can0",
                    #     actuator_can_id=21,
                    #     angle=1, # rad
                    #     angular_vel=0,
                    #     kp=2 * 13, # not scaled / 13
                    #     kd=1 * 650, # not scaled / 650
                    # )

        # self.ci.enable_all_actuators()

        # forever loop
        # self._loop()

    def _loop(self):
        while True:
            self.ci.get_actuator_feedback()
            time.sleep(0.1)



def main():
    driver = MotorDriver()


if __name__ == "__main__":
    exit(0 if main() else 1)


# todo: 
# - poll at high freq. see what happens. 
# poll all acts
# set all acts
# calibrate value scalings
# done
# clean up and simplify
