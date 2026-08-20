"""Control a ToF-equipped LiteWing from the XIAO/BNO085 glove.

The glove sends: raw_roll,raw_pitch,button_state over UDP port 4210.

Controls:
  - Keep the glove level and the button released while auto-tare completes.
  - Hold the button to take off and maintain TARGET_HEIGHT_M.
  - While held, tilt the glove to command roll and pitch.
  - Release the button to level the drone and perform a controlled landing.
  - Loss of glove packets also starts a controlled landing.

Always validate the complete path with propellers removed first.
"""

from __future__ import annotations

import enum
import socket
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


# Network settings -----------------------------------------------------------

DRONE_URI = "udp://192.168.43.42:2390"
GLOVE_LISTEN_IP = "0.0.0.0"
GLOVE_UDP_PORT = 4210


# Glove mapping --------------------------------------------------------------

# The BNO085 is mounted with its axes rotated relative to the hand, so raw
# pitch becomes flight roll and raw roll becomes flight pitch.
SWAP_ROLL_PITCH = True

# Both axes are inverted as requested. Change one sign to +1.0 if physical
# testing shows that particular direction should not be inverted.
ROLL_SIGN = -1.0
PITCH_SIGN = -1.0

TARE_SAMPLES = 50
DEAD_ZONE_DEG = 4.0
MAX_GLOVE_ANGLE_DEG = 30.0
MAX_COMMAND_ANGLE_DEG = 12.0
FILTER_ALPHA = 0.20


# Height and safety ----------------------------------------------------------

TARGET_HEIGHT_M = 0.50
MIN_ACTIVE_HEIGHT_M = 0.12
TAKEOFF_RATE_MPS = 0.30
LANDING_RATE_MPS = 0.22
CONTROL_RATE_HZ = 50.0
GLOVE_TIMEOUT_S = 0.30
BUTTON_DEBOUNCE_PACKETS = 3

# Keep this True for the first propeller-off test. The entire glove/UDP/CRTP
# path runs, but the aircraft is never given a non-zero height command.
PROPS_OFF_TEST = True


class FlightState(enum.Enum):
    IDLE = "IDLE"
    TAKING_OFF = "TAKING_OFF"
    FLYING = "FLYING"
    LANDING = "LANDING"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_dead_zone(value: float) -> float:
    if abs(value) <= DEAD_ZONE_DEG:
        return 0.0

    # Remove the dead-zone step so the command grows smoothly from zero.
    return value - DEAD_ZONE_DEG if value > 0.0 else value + DEAD_ZONE_DEG


def glove_angle_to_command(angle_deg: float) -> float:
    angle_deg = apply_dead_zone(angle_deg)
    usable_range = MAX_GLOVE_ANGLE_DEG - DEAD_ZONE_DEG
    if usable_range <= 0.0:
        raise ValueError("MAX_GLOVE_ANGLE_DEG must exceed DEAD_ZONE_DEG")

    command = angle_deg * MAX_COMMAND_ANGLE_DEG / usable_range
    return clamp(command, -MAX_COMMAND_ANGLE_DEG, MAX_COMMAND_ANGLE_DEG)


def parse_packet(data: bytes) -> tuple[float, float, bool]:
    text = data.decode("ascii").strip()
    parts = text.split(",")
    if len(parts) != 3:
        raise ValueError(f"expected 3 fields, received {len(parts)}")

    raw_roll = float(parts[0])
    raw_pitch = float(parts[1])
    button = int(parts[2])
    if button not in (0, 1):
        raise ValueError("button must be 0 or 1")

    return raw_roll, raw_pitch, bool(button)


def send_stop(cf: Crazyflie, repeats: int = 1) -> None:
    for _ in range(repeats):
        cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
        if repeats > 1:
            time.sleep(0.02)


def main() -> None:
    period_s = 1.0 / CONTROL_RATE_HZ

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver.bind((GLOVE_LISTEN_IP, GLOVE_UDP_PORT))
    receiver.setblocking(False)

    tare_roll_values: list[float] = []
    tare_pitch_values: list[float] = []
    roll_offset = 0.0
    pitch_offset = 0.0
    tare_complete = False

    filtered_roll = 0.0
    filtered_pitch = 0.0
    button_raw = False
    button_debounced = False
    previous_button_raw = False
    button_stable_packets = 0
    last_packet_time = 0.0
    last_status_time = 0.0

    state = FlightState.IDLE
    commanded_height = 0.0

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")

    print(f"Listening for glove packets on UDP {GLOVE_UDP_PORT}")
    print(f"Connecting to LiteWing at {DRONE_URI}")

    with SyncCrazyflie(DRONE_URI, cf=cf):
        # Required motor-lock handshake. It is repeated again on every landing.
        send_stop(cf, repeats=5)
        print("Connected to LiteWing")
        print("PROPS_OFF_TEST =", PROPS_OFF_TEST)
        print("Keep the glove level with the button released for auto-tare")

        next_tick = time.monotonic()
        try:
            while True:
                now = time.monotonic()

                # Drain the UDP queue and retain the newest valid packet.
                while True:
                    try:
                        packet, _sender = receiver.recvfrom(128)
                    except BlockingIOError:
                        break

                    try:
                        raw_roll, raw_pitch, new_button_raw = parse_packet(packet)
                    except (UnicodeDecodeError, ValueError) as error:
                        print("Ignored malformed glove packet:", error)
                        continue

                    last_packet_time = now
                    button_raw = new_button_raw

                    # Tare only while the trigger is released. Averaging several
                    # samples is more stable than using the first packet alone.
                    if not tare_complete and not button_raw:
                        tare_roll_values.append(raw_roll)
                        tare_pitch_values.append(raw_pitch)
                        if len(tare_roll_values) >= TARE_SAMPLES:
                            roll_offset = sum(tare_roll_values) / len(tare_roll_values)
                            pitch_offset = sum(tare_pitch_values) / len(tare_pitch_values)
                            tare_complete = True
                            print(
                                f"Auto-tare complete: raw roll={roll_offset:.2f}, "
                                f"raw pitch={pitch_offset:.2f}"
                            )

                    corrected_roll = raw_roll - roll_offset
                    corrected_pitch = raw_pitch - pitch_offset

                    if SWAP_ROLL_PITCH:
                        corrected_roll, corrected_pitch = (
                            corrected_pitch,
                            corrected_roll,
                        )

                    roll_command = ROLL_SIGN * glove_angle_to_command(corrected_roll)
                    pitch_command = PITCH_SIGN * glove_angle_to_command(corrected_pitch)

                    filtered_roll += FILTER_ALPHA * (roll_command - filtered_roll)
                    filtered_pitch += FILTER_ALPHA * (pitch_command - filtered_pitch)

                    if button_raw == previous_button_raw:
                        button_stable_packets += 1
                    else:
                        previous_button_raw = button_raw
                        button_stable_packets = 1

                    if button_stable_packets >= BUTTON_DEBOUNCE_PACKETS:
                        button_debounced = button_raw

                if now < next_tick:
                    time.sleep(min(next_tick - now, 0.002))
                    continue
                next_tick += period_s
                if now - next_tick > period_s:
                    next_tick = now + period_s

                packet_fresh = now - last_packet_time <= GLOVE_TIMEOUT_S
                control_enabled = (
                    tare_complete
                    and packet_fresh
                    and button_debounced
                    and not PROPS_OFF_TEST
                )

                if state is FlightState.IDLE and control_enabled:
                    state = FlightState.TAKING_OFF
                    commanded_height = MIN_ACTIVE_HEIGHT_M
                    print("State -> TAKING_OFF")

                elif state in (FlightState.TAKING_OFF, FlightState.FLYING):
                    if not control_enabled:
                        state = FlightState.LANDING
                        print("State -> LANDING")

                elif state is FlightState.LANDING and control_enabled:
                    # Re-pressing during descent resumes controlled flight.
                    state = FlightState.TAKING_OFF
                    commanded_height = max(commanded_height, MIN_ACTIVE_HEIGHT_M)
                    print("State -> TAKING_OFF")

                if PROPS_OFF_TEST:
                    state = FlightState.IDLE
                    commanded_height = 0.0
                    send_stop(cf)
                elif state is FlightState.IDLE:
                    commanded_height = 0.0
                    send_stop(cf)
                elif state is FlightState.TAKING_OFF:
                    commanded_height = min(
                        TARGET_HEIGHT_M,
                        commanded_height + TAKEOFF_RATE_MPS * period_s,
                    )
                    if commanded_height >= TARGET_HEIGHT_M:
                        state = FlightState.FLYING
                        print("State -> FLYING")

                    cf.commander.send_zdistance_setpoint(
                        filtered_roll,
                        filtered_pitch,
                        0.0,
                        commanded_height,
                    )
                elif state is FlightState.FLYING:
                    cf.commander.send_zdistance_setpoint(
                        filtered_roll,
                        filtered_pitch,
                        0.0,
                        TARGET_HEIGHT_M,
                    )
                elif state is FlightState.LANDING:
                    # Level the aircraft while reducing the ToF height target.
                    filtered_roll *= 0.75
                    filtered_pitch *= 0.75
                    commanded_height = max(
                        MIN_ACTIVE_HEIGHT_M,
                        commanded_height - LANDING_RATE_MPS * period_s,
                    )

                    if commanded_height <= MIN_ACTIVE_HEIGHT_M:
                        send_stop(cf, repeats=5)
                        state = FlightState.IDLE
                        commanded_height = 0.0
                        print("State -> IDLE")
                    else:
                        cf.commander.send_zdistance_setpoint(
                            filtered_roll,
                            filtered_pitch,
                            0.0,
                            commanded_height,
                        )

                if now - last_status_time >= 0.25:
                    print(
                        f"{state.value:10s} "
                        f"roll={filtered_roll:6.2f} "
                        f"pitch={filtered_pitch:6.2f} "
                        f"height={commanded_height:.2f}m "
                        f"button={int(button_debounced)} "
                        f"link={'OK' if packet_fresh else 'LOST'}"
                    )
                    last_status_time = now

        except KeyboardInterrupt:
            print("\nKeyboard interrupt: stopping motors")
        finally:
            send_stop(cf, repeats=10)
            receiver.close()


if __name__ == "__main__":
    main()
