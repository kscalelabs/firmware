import math
import socket
import struct
from typing import Dict

class CANInterface:
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
        self.EFF = 0x8000_0000

        self.sockets = {}
        self.actuators = {}
        self.scan()

    def scan(self):
        for canbus in self.canbus_range:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            try:
                sock.bind((f"can{canbus}",))
                self.sockets[f"can{canbus}"] = sock
                self.actuators[f"can{canbus}"] = []
            except Exception as e:
                print(f"bus {canbus} not available: {e}")
                continue

            print(f"Scanning bus {canbus}")
            for actuator_id in self.actuator_range:
                if self.ping_actuator(canbus, actuator_id):
                    self.actuators[f"can{canbus}"].append(actuator_id)

        print("\033[1;36m🔍 CAN scan complete\033[0m")
        total_actuators = sum(len(actuators) for actuators in self.actuators.values())
        print(f"\033[1;32mFound {total_actuators} actuators on {len(self.sockets)} sockets\033[0m")
        for canbus, actuators in self.actuators.items():
            print(f"\033[1;34m{canbus}\033[0m: \033[1;35m{actuators}\033[0m")

    def ping_actuator(self, canbus: str, actuator_can_id: int):
        try:
            frame = self._build_ping_frame(actuator_can_id)
            self.sockets[canbus].send(frame)
            self.sockets[canbus].settimeout(0.01)
            resp_frame = self.sockets[canbus].recv(self.FRAME_SIZE)
            _ = struct.unpack(self.FRAME_FMT, resp_frame)
            return True
        except:
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

    def _build_motor_enable_frame(self, actuator_can_id: int, length: int = 8) -> bytes:
        can_id = ((actuator_can_id & 0xFF)
                  | (self.host_id << 8)
                  | ((self.MUX_MOTOR_ENABLE & 0x1F) << 24))
        can_id |= self.EFF
        payload = b"\x00" * 8
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)


    def get_actuator_feedback(self) -> Dict[str, int]:
        for can, sock in self.sockets.items():
            for actuator_id in self.actuators[can]:
                frame = self._build_feedback_request(actuator_id)
                sock.send(frame)
                resp_frame = sock.recv(self.FRAME_SIZE)
                result = self._parse_feedback_response(resp_frame)
                print(f"Feedback from {can} actuator {actuator_id}: {result}")



    def _build_feedback_request(self, actuator_can_id: int, length: int = 8, eff: bool = True) -> bytes:
        """
        Compose a FeedbackRequest frame (mux 0x02).
        Identifier packing: [b0=actuator_can_id][b1..b2=host_id LE][b3=mux|EFFbit]
        """
        can_id = ((actuator_can_id & 0xFF)
                  | (self.host_id << 8)
                  | ((self.MUX_FEEDBACK & 0x1F) << 24))
        if eff:
            can_id |= self.EFF
        payload = b"\x00" * 8  # request carries no data
        return struct.pack(self.FRAME_FMT, can_id, length & 0xFF, 0, 0, 0, payload)

    def _parse_feedback_response(self,frame: bytes) -> Dict[str, int]:
        """
        Parse a 16-byte FeedbackResponse frame.
        Response can_id bytes (little-endian u32 reinterpreted as 4 bytes):
          b0=host_id, b1=actuator_can_id, b2=fault_flags, b3=mux|EFFbit
        Data payload (8 bytes) is four big-endian u16 words:
          angle_be, angular_vel_be, torque_be, temp_be
        """
        if len(frame) != 16:
            raise ValueError("frame must be exactly 16 bytes")

        can_id, length, _pad, _res0, _len8, payload = struct.unpack("<IBBBB8s", frame)
        b0 = (can_id >> 0)  & 0xFF  # host_id (u8)
        b1 = (can_id >> 8)  & 0xFF  # actuator_can_id (u8)
        b2 = (can_id >> 16) & 0xFF  # fault_flags (u8)
        b3 = (can_id >> 24) & 0xFF  # mux + EFF-in-byte
        mux = b3 & 0x1F
        eff = bool(can_id & self.EFF)

        if mux != self.MUX_FEEDBACK:
            raise ValueError(f"unexpected mux 0x{mux:02X} in feedback response")

        angle_be, ang_vel_be, torque_be, temp_be = struct.unpack(">HHHH", payload)
        angle = self._raw_to_rad(angle_be)
        ang_vel = self._raw_to_rad(ang_vel_be)
        torque = torque_be # TODO
        temp = temp_be / 10 

        return {
            "eff": int(eff),
            "length": length,
            "mux": mux,
            "host_id": b0,
            "actuator_can_id": b1,
            "fault_flags": b2,
            "angle_raw": angle,
            "angular_velocity_raw": ang_vel,
            "torque_raw": torque,
            "temperature_raw": temp,
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
        length: int = 8,
        eff: bool = True
    ) -> bytes:
        """
        Build ControlCommandRequest frame (mux 0x01).
        - torque_scale goes into identifier bytes [1..2] (little-endian u16)
        - angle/angular_vel/kp/kd go into payload as big-endian u16s
        """
        can_id = ((actuator_can_id & 0xFF)
                  | (self._clamp_u16(torque_scale) << 8)
                  | ((self.MUX_CONTROL & 0x1F) << 24))
        if eff:
            can_id |= self.EFF

        payload = struct.pack(">HHHH",
                              self._clamp_u16(angle_scale),
                              self._clamp_u16(angular_vel_scale),
                              self._clamp_u16(kp_scale),
                              self._clamp_u16(kd_scale))
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



def main():
    can_interface = CANInterface()
    can_interface.get_actuator_feedback()

    # Enable motor for actuator 21 before controlling it
    can_interface.enable_motor("can0", 21)

    # make act 21 move to 1 rad
    can_interface.set_pd_target(
        canbus="can0",
        actuator_can_id=21,
        angle=1, # rad
        angular_vel=0,
        kp=2 * 13, # not scaled / 13
        kd=1 * 650, # not scaled / 650
    )


if __name__ == "__main__":
    exit(0 if main() else 1)



