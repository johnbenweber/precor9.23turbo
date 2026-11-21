# Precor 9.23 Turbo

ESP32 firmware that can either broadcast conditioned treadmill-harness
readings or run the original sonar-based free-run mockup over Bluetooth LE.
Neither mode drives the treadmill's physical speed or incline controls.

## Harness telemetry mode

| Harness signal | ESP32 pin | Firmware interpretation |
| --- | --- | --- |
| Pin 1, belt-moving status | GPIO 26 | Low means the belt is moving (opto-inverted) |
| Pin 8, speed signal | GPIO 27 | Rising edge after optocoupler on each low pulse |
| Pin 11, incline output | ADC1 GPIO 34 | 4.14 V = 0%; 0.914 V = 15% |

These are ESP32-side connections **after** appropriate electrical
conditioning. Never connect harness pins directly to ESP32 pins: ESP32 GPIOs
are not 5 V tolerant, and the harness supply and signal references have not
been fully verified.

- Use appropriately rated isolation/conditioning for digital signals. Pin 8
  appears to be 5 V logic, normally high and low during rotation; pin 9 is the
  suspected signal return. Confirm the waveform across pins 8 and 9 before
  connecting an input stage. The recommended optocoupler input is on while
  pin 8 is high, so its ESP32-side output rises when pin 8 drops low. That
  inversion matches the firmware's rising-edge interrupt. A non-inverting
  conditioner would require changing the interrupt to `FALLING`. Use an
  external 3.3 V pull-up on the isolated side. Check that pin 8 can supply the
  optocoupler's continuous idle current; use a higher-impedance isolated
  interface if it cannot.
- Pin 1 may reach 20 V. Use an optocoupler or a suitably rated isolated input
  module, and verify its return and actual high-state voltage before selecting
  the input resistor. As an example only, a 10 kOhm resistor at 20 V with a
  1.2 V optocoupler LED gives about 1.9 mA and dissipates about 0.04 W. Check
  that the selected optocoupler works at that LED current and that the treadmill
  output can supply it. The isolated ESP32-side output must be limited to
  3.3 V; an optocoupler transistor output with a pull-up to 3.3 V is suitable.
  This firmware expects the optocoupler output to pull GPIO 26 low while the
  belt is moving. Use an external 3.3 V pull-up on the isolated side.
- Pin 11's observed range exceeds the ESP32 ADC range. Use a protected analog
  front end that keeps ADC GPIO 34 within the board's specified range under
  faults. Firmware assumes that the ADC-side voltage is half the measured
  harness voltage; adjust `kInclineAdcToHarnessVoltage` in
  `src/harness_telemetry.cpp` for the installed front end.
- A simple divider shares grounds and is suitable only if that is verified
  safe. Do not assume harness pin 7, pin 9, or pin 12 is a safe common ground.
- Leave pins 2, 3, 5, and 6 disconnected. They are control signals and this
  firmware does not drive them. Pins 4, 10, and 12 are not used by the
  telemetry firmware.

Speed is calculated assuming two pulses per rotation and the 64.59 mm roller
diameter. A moving belt with no observed pulses is reported as unavailable
speed, not zero speed. Incline is linearly mapped from the observed endpoint
voltages; if its voltage is outside the expected range, incline is omitted
from the FTMS data.

## Free-run mockup mode

The original HC-SR04 distance-based speed mockup remains available. Connect
the sonar trigger to ESP32 GPIO 5 and echo to GPIO 18. The Echo signal must be
within the ESP32 input voltage rating; level-shift it if necessary.

Set `kFreeRunModeEnabled` to `true` in `src/main.cpp` to opt in. This selects
the sonar/free-run speed and simulated incline path instead of harness
telemetry. It also exposes the FTMS control point and advertises the Wahoo Run
service for incline writes. Set it to `false` (the default) for measured,
read-only FTMS data with no Wahoo control service. These are mutually exclusive
modes.

Free-run mode only changes the values advertised over BLE; it does not drive
the treadmill's harness command signals. The Wahoo notification payload is
not implemented.

## Build and flash

Open this directory in PlatformIO and build/upload the `esp32doit-devkit-v1`
environment. The project uses the NimBLE-Arduino dependency declared in
`platformio.ini`. Set the Serial Monitor to 115200 baud.
