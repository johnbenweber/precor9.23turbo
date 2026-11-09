# Precor 9.23 Turbo

ESP32 test firmware that reads hand distance with an HC-SR04 sensor and
advertises a simulated treadmill over Bluetooth LE. It does not control a
treadmill motor or incline mechanism.

## Hardware

| Component | ESP32 connection |
| --- | --- |
| HC-SR04 trigger | GPIO 5 |
| HC-SR04 echo | GPIO 18 |
| HC-SR04 ground | GND |
| HC-SR04 power | 3.3 V |

The HC-SR04 Echo output must not exceed the ESP32's input voltage rating.
Verify the sensor's output level and use a level shifter or divider if needed.

## Behavior

- Samples the sensor every 100 ms and smooths the most recent five valid
  readings.
- Updates simulated speed every second. Hand distance from 4 to 10 inches is
  the control range, with a neutral zone centered at 7 inches (6.5–7.5 inches).
  Moving closer accelerates; moving farther decelerates. Outside the control
  range or when an echo is missing, speed is held.
- Advertises standard FTMS treadmill data and control, plus the observed Wahoo
  Run service used by MyWhoosh.
- Prints elapsed milliseconds, simulated speed, control zone, and simulated
  incline every 250 ms at 115200 baud.

## Build and flash

Open this directory in PlatformIO and build/upload the `esp32doit-devkit-v1`
environment. The project uses the NimBLE-Arduino dependency declared in
`platformio.ini`. Set the Serial Monitor to 115200 baud.

## BLE incline inputs

Standard FTMS incline control and Wahoo Run slope writes both update the same
simulated incline, which is reported in FTMS treadmill-data notifications. The
observed Wahoo `E03E` payload begins with `0x11`, followed by a signed
little-endian slope in hundredths of a percent. Incline is rounded to tenths of
a percent and clamped to 0–15% to match the advertised supported range.

The Wahoo payload interpretation is based on observed MyWhoosh traffic and the
QZ handling of the characteristic; the Wahoo UUID reference is in
[QZ's virtual treadmill source](https://github.com/cagnulein/qdomyos-zwift/blob/master/src/virtualdevices/virtualtreadmill.cpp#L168).
The run-speed control concept is also related to
[US patent 10,016,656](https://patents.google.com/patent/US10016656B2/en?q=(automated+treadmill)&oq=automated+treadmill).
The firmware does not drive physical treadmill speed or incline controls.
