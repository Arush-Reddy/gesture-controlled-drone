# Gesture-Controlled LiteWing Drone

A wearable controller built with a Seeed Studio XIAO ESP32-C3, BNO085 IMU,
and a dead-man button. A laptop receives glove orientation over UDP and sends
height-controlled roll/pitch commands to a VL53L1X-equipped LiteWing using the
Crazyflie Python library.

## System

```text
Glove / XIAO ESP32-C3
  BNO085 + button
        |
        | UDP: raw roll,pitch,button on port 4210
        v
Laptop / Python bridge
  tare + swap + invert + filter + safety state machine
        |
        | Crazyflie CRTP over UDP
        v
LiteWing 192.168.43.42
  VL53L1X height hold + onboard stabilization
```

## Files

- `glove_litewing/glove_litewing.ino` - firmware for the XIAO ESP32-C3 glove.
- `glove_litewing/litewing_hover_bridge.py` - recommended ToF height-hold
  controller with auto-tare, swapped/inverted axes, takeoff, landing, and
  link-loss handling.
- `glove_litewing/README.md` - installation, configuration, and test steps.
- `Gesture-Controlled-Drone-Project-Plan-Updated.docx` - current component and
  development plan.
- `Gesture-Controlled-Drone-Project-Plan.docx` - original plan retained for
  comparison.

## Required hardware

- LiteWing with working VL53L1X downward ToF sensor
- Seeed Studio XIAO ESP32-C3
- BNO085 at I2C address `0x4B`
- Momentary button connected between `D3` and ground
- Laptop with Wi-Fi and Python

## Quick start

1. Confirm that the LiteWing flies and height hold works using its official app.
2. Remove all propellers.
3. Open `glove_litewing/glove_litewing.ino`, enter the LiteWing Wi-Fi SSID and
   password, and upload it to the XIAO ESP32-C3.
4. Connect the laptop to the same LiteWing Wi-Fi network.
5. Install the LiteWing-compatible Crazyflie Python library:

   ```powershell
   git clone https://github.com/jobitjoseph/crazyflie-clients-python.git
   cd crazyflie-clients-python
   python -m pip install -e .
   ```

6. Run the controller:

   ```powershell
   cd glove_litewing
   python litewing_hover_bridge.py
   ```

7. Keep the glove level with the button released until auto-tare completes.
8. Verify directions, button release, packet loss, and motor stop while
   `PROPS_OFF_TEST = True`.

## Control behavior

- Roll and pitch are swapped to match the physical sensor mounting and both are
  inverted in the Python bridge.
- Hold the D3 button to request takeoff and hover at 0.5 m.
- Release the button to level the aircraft and land.
- Missing glove packets initiate landing.
- `Ctrl+C` sends repeated zero-thrust stop commands.

## Safety

The repository defaults to `PROPS_OFF_TEST = True`; the drone cannot take off
until this is deliberately changed in `litewing_hover_bridge.py`. Validate the
official app, ToF sensor, control directions, and all stop/failsafe paths before
installing propellers. Use an open test area, eye protection, a spotter, and
experienced adult or mentor supervision.
