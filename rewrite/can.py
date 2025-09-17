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
                assert result['actuator_can_id'] == actuator_id, f"mismatch in actuator id -- probably missed a response earlier: {result}"
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



class MotorDriver:
    """ Driver logic """
    def __init__(self):
        self.robot = RobotConfig()
        self.ci = CANInterface()

        states = self.ci.get_actuator_feedback()

        print("\033[1;36mActuator states:\033[0m")
        print("ID  | Nam | Angle | Velocity | Torque | Temp  | Faults")
        print("----|-----|-------|----------|--------|-------|-------")
        for act_id, state in states.items():
            name = self.robot.actuators[act_id].name[:3]
            fault_color = "\033[1;31m" if state['fault_flags'] > 0 else "\033[1;32m"
            angle = self.robot.actuators[act_id].can_to_physical_angle(state['angle_raw'])
            velocity = self.robot.actuators[act_id].can_to_physical_velocity(state['angular_velocity_raw'])
            torque = self.robot.actuators[act_id].can_to_physical_torque(state['torque_raw'])
            temp = self.robot.actuators[act_id].can_to_physical_temperature(state['temperature_raw'])
            print(f"{act_id:3d} | {name:3s} | \033[1;34m{angle:5.2f}\033[0m | \033[1;35m{velocity:8.2f}\033[0m | \033[1;33m{torque:6.2f}\033[0m | \033[1;36m{temp:5.1f}\033[0m | {fault_color}{state['fault_flags']:3d}\033[0m")
        
        if any(state['fault_flags'] > 0 for state in states.values()):
            print("\033[1;31m❌ Actuator faults detected\033[0m")
            # exit(1) # TODO for some reason we get 128 uncalibrated faults

        input("Press Enter to proceed to enable motors...")
        self.ci.enable_motors()
        print(f"✅ Motors enabled")

        home_targets = {id: self.robot.actuators[id].joint_bias for id in self.robot.actuators.keys()}
        print("\nHoming...")
        for scale in [math.exp(math.log(0.001) + (math.log(1.0) - math.log(0.001)) * i / 29) for i in range(30)]:  # Logarithmic interpolation from 0.001 to 1.0 in 30 steps
            print(f"PD ramp: {scale:.3f}")
            self.ci.set_pd_targets(home_targets, robotcfg=self.robot, scaling=scale)
            time.sleep(.1)
        print("✅ Homing complete")

        input("Press Enter to start policy...")
        print("🤖 Running policy...")

    def sine_wave(self):
        t0 = time.perf_counter()
        while True:
            angle = 3.14158/2 * math.sin(2 * math.pi * 0.5 * (time.perf_counter() - t0))
            action = {id: angle + self.robot.actuators[id].joint_bias for id in self.robot.actuators.keys()}
            self.ci.set_pd_targets(action, robotcfg=self.robot, scaling=0.1)
            time.sleep(0.1)

    def get_joint_angles_and_velocities(self, joint_order: list[str]) -> tuple[list[float], list[float]]:
        fb = self.ci.get_actuator_feedback()

        # TODO testbench stuff
        joint_angles_raw = {id: fb[id]['angle_raw'] if id in fb else 0 for id in self.robot.actuators.keys()}
        joint_velocities_raw = {id: fb[id]['angular_velocity_raw'] if id in fb else 0 for id in self.robot.actuators.keys()}

        joint_angles = {id: self.robot.actuators[id].can_to_physical_angle(joint_angles_raw[id]) for id in self.robot.actuators.keys()}
        joint_angles_ordered = [joint_angles[self.robot.full_name_to_actuator_id[name]] for name in joint_order]

        joint_velocities = {id: self.robot.actuators[id].can_to_physical_velocity(joint_velocities_raw[id]) for id in self.robot.actuators.keys()}
        joint_velocities_ordered = [joint_velocities[self.robot.full_name_to_actuator_id[name]] for name in joint_order]

        return joint_angles_ordered, joint_velocities_ordered

    def take_action(self, action: list[float], joint_order: list[str]):
        action = {self.robot.actuators[self.robot.full_name_to_actuator_id[name]].can_id: action for name, action in zip(joint_order, action)}
        self.ci.set_pd_targets(action, robotcfg=self.robot, scaling=1.0)


def main():
    driver = MotorDriver()
    driver.sine_wave()


if __name__ == "__main__":
    exit(0 if main() else 1)
