# Glove and LiteWing software

Use these two files together:

- `glove_litewing.ino` runs on the Seeed Studio XIAO ESP32-C3 and transmits raw
  BNO085 roll, pitch, and D3 button state at 50 Hz over UDP port 4210.
- `litewing_hover_bridge.py` runs on the laptop. It performs auto-tare, swaps
  and inverts the mounted sensor axes, filters and limits commands, and controls
  the ToF-equipped LiteWing through Crazyflie CRTP.

## Installation

1. Confirm the LiteWing and VL53L1X height hold work with the official app.
2. Remove all propellers.
3. Connect the XIAO and laptop to the LiteWing Wi-Fi access point.
4. Replace the Wi-Fi placeholders in `glove_litewing.ino`, select
   `XIAO_ESP32C3` in Arduino IDE, and upload the sketch.
5. Install the Arduino `Adafruit BNO08x` library if it is not already present.
6. Install the LiteWing-compatible Crazyflie Python package:

   ```powershell
   git clone https://github.com/jobitjoseph/crazyflie-clients-python.git
   cd crazyflie-clients-python
   python -m pip install -e .
   ```

7. Return to this repository and run:

   ```powershell
   cd glove_litewing
   python litewing_hover_bridge.py
   ```

8. Keep the glove level and the button released until the 50-sample auto-tare
   completes.

## Controls

- Hold D3: take off, maintain a 0.5 m ToF height target, and accept hand tilt.
- Release D3: level and land.
- Lost glove packets: level and land.
- `Ctrl+C`: send repeated motor-stop commands and exit.

The bridge defaults to `PROPS_OFF_TEST = True`, so it will not take off. Verify
the corrected roll/pitch directions and every stop path before deliberately
changing that setting.
