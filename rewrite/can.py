import math
from string import printable
import time
import traceback
import socket
import struct
from typing import Dict

from robot import RobotConfig


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
                sock.settimeout(0.01)
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
            resp_frame = self.sockets[canbus].recv(self.FRAME_SIZE)
            _ = struct.unpack(self.FRAME_FMT, resp_frame)
            return True
        except socket.timeout:
            return False
        except Exception as e:
            # print(f"Error pinging actuator {actuator_can_id} on {canbus}: {e}")
            # traceback.print_exc()
            return False

    def _build_ping_frame(self, actuator_can_id: int) -> bytes:
        can_id = (actuator_can_id & 0xFF) | ((self.host_id & 0xFFFF) << 8) | ((self.MUX_PING & 0x1F) << 24)
        can_id |= 0x8000_0000 # set EFF flag
        length = 8
        payload = b'\x00' * length # empty payload
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)


    def enable_motors(self):
        for canbus in self.sockets.keys():
            for actuator_id in self.actuators[canbus]:
                self._enable_motor(canbus, actuator_id)
        print(f"✅ Motors enabled")

    def _enable_motor(self, canbus: int, actuator_can_id: int):
        frame = self._build_motor_enable_frame(actuator_can_id)
        self.sockets[canbus].send(frame)
        raw = self.sockets[canbus].recv(16) # receive response to keep can buffer clear

    def _build_motor_enable_frame(self, actuator_can_id: int) -> bytes:
        can_id = ((actuator_can_id & 0xFF) | (self.host_id << 8) | ((self.MUX_MOTOR_ENABLE & 0x1F) << 24))
        can_id |= self.EFF
        length = 8
        payload = b'\x00' * length # empty payload
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)


    def get_actuator_feedback(self) -> Dict[str, int]:
        results = {}
        for can, sock in self.sockets.items():
            for actuator_id in self.actuators[can]:
                frame = self._build_feedback_request(actuator_id)
                sock.send(frame)
                resp_frame = sock.recv(self.FRAME_SIZE)
                result = self._parse_feedback_response(resp_frame)
                assert result['actuator_can_id'] == actuator_id, f"mismatch in actuator id -- probably missed a response: {result}"
                results[actuator_id] = result
        return results

    def _build_feedback_request(self, actuator_can_id: int) -> bytes:
        can_id = ((actuator_can_id & 0xFF) | (self.host_id << 8) | ((self.MUX_FEEDBACK & 0x1F) << 24))
        can_id |= self.EFF
        length = 8
        payload = b'\x00' * length # empty payload
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)

    def _parse_feedback_response(self, frame: bytes) -> Dict[str, int]:
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

        return {
            "host_id": b0,
            "actuator_can_id": b1,
            "fault_flags": b2,
            "angle_raw": angle_be,
            "angular_velocity_raw": ang_vel_be,
            "torque_raw": torque_be,
            "temperature_raw": temp_be,
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



    def set_pd_targets(self, actions: dict[int, float], robotcfg: RobotConfig, scaling: float = 1.0):
        for canbus in self.sockets.keys():
            for actuator_id in self.actuators[canbus]:
                self._set_pd_target(canbus, actuator_id, actions[actuator_id], robotcfg, scaling)

    def _set_pd_target(self, canbus: int, actuator_can_id: int, angle: float, robotcfg: RobotConfig, scaling: float = 1.0):
        assert 0.0 <= scaling <= 1.0
        frame = self._build_pd_command(
            actuator_can_id, 
            int(robotcfg.actuators[actuator_can_id].physical_to_can_torque(0)), 
            int(robotcfg.actuators[actuator_can_id].physical_to_can_angle(angle)),
            int(robotcfg.actuators[actuator_can_id].physical_to_can_velocity(0)),
            int(robotcfg.actuators[actuator_can_id].raw_kp*scaling), 
            int(robotcfg.actuators[actuator_can_id].raw_kd*scaling)
        )
        self.sockets[canbus].send(frame)
        raw = self.sockets[canbus].recv(16) # just drop response
        # fb = self._parse_feedback_response_pd_command(raw)
        # if fb["mux"] == self.MUX_CONTROL and fb["actuator_can_id"] == actuator_can_id:
        #     phys = self.feedback_to_physical_pd_command(fb)
        #     return raw, fb, phys


    def _build_pd_command(
        self,
        actuator_can_id: int,
        raw_torque: int,
        raw_angle: int,
        raw_angular_vel: int,
        raw_kp: int,
        raw_kd: int,
    ) -> bytes:
        assert isinstance(raw_torque, int) and isinstance(raw_angle, int) and isinstance(raw_angular_vel, int) and isinstance(raw_kp, int) and isinstance(raw_kd, int)
        can_id = ((actuator_can_id & 0xFF) | (raw_torque << 8) | ((self.MUX_CONTROL & 0x1F) << 24))
        can_id |= self.EFF
        payload = struct.pack(">HHHH", raw_angle, raw_angular_vel, raw_kp, raw_kd)
        length = 8
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)


    # def _parse_feedback_response_pd_command(self, frame: bytes):
    #     if len(frame) != 16:
    #         raise ValueError("expeccommandted 16-byte CAN frame")
    #     can_id, length, _pad, _res0, _len8, payload = struct.unpack("<IBBBB8s", frame)
    #     b0 = (can_id >> 0)  & 0xFF   # host_id (u8)
    #     b1 = (can_id >> 8)  & 0xFF   # actuator_can_id (u8)
    #     b2 = (can_id >> 16) & 0xFF   # fault_flags (u8)
    #     b3 = (can_id >> 24) & 0xFF   # mux|eff-byte
    #     mux = b3 & 0x1F
    #     angle_be, ang_vel_be, torque_be, temp_be = struct.unpack(">HHHH", payload)
    #     return {
    #         "eff": 1 if (can_id & self.EFF) else 0,
    #         "length": length,
    #         "mux": mux,
    #         "host_id": b0,
    #         "actuator_can_id": b1,
    #         "fault_flags": b2,
    #         "angle_raw": angle_be,
    #         "angular_velocity_raw": ang_vel_be,
    #         "torque_raw": torque_be,
    #         "temperature_raw": temp_be,
    #     }

    # def feedback_to_physical_pd_command(self, fb: dict):
    #     return {
    #         "actuator_can_id": fb["actuator_can_id"],
    #         "angle_rad": self._raw_to_rad(fb["angle_raw"]),
    #         "angular_velocity_rad_s": self._raw_to_rad(fb["angular_velocity_raw"]),
    #         "torque_Nm": fb["torque_raw"],
    #         "temperature_C": fb["temperature_raw"],
    #         "fault_flags": fb["fault_flags"],
    #     }


class MotorDriver:
    """ Driver logic """
    def __init__(self):
        self.robotcfg = RobotConfig()
        self.ci = CANInterface()

        self.acts = sum([self.ci.actuators[canbus] for canbus in self.ci.sockets.keys()], [])

        # # TODO check all actuators dont return errors, else stop and print error
        # self.ci.read_actuator_params()
        # self.ci.check_errors() 

        # slowly set all acts to 0
        self.ci.enable_motors()
        self.ci.set_pd_targets({k: 0.0 for k in self.acts},robotcfg=self.robotcfg, scaling=0.005)
        time.sleep(2)
        self.ci.set_pd_targets({k: 0.0 for k in self.acts},robotcfg=self.robotcfg, scaling=0.05)
        time.sleep(2)


        # forever loop
        self._loop()

    def _loop(self):
        # while True:
            # before = time.perf_counter()
            # fb = self.ci.get_actuator_feedback()
            # after = time.perf_counter()
            # # print(f"Time taken: {(after - before)*1000:.3f} ms")
        


        t0 = time.perf_counter()
        while True:
            angle = 3.14158/2 * math.sin(2 * math.pi * 0.5 * (time.perf_counter() - t0))
            action = {k: angle for k in self.acts}
            self.ci.set_pd_targets(action, robotcfg=self.robotcfg, scaling=0.1)
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
# deal with uncalled messages
