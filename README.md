# Arduino Mavlink Simulator

## Introduction

This is an Arduino Sketch for simulating MAVLink telemetry data that can be used by common groundstation applications such as Mission Planner or APM Planner 2.0.
The primary idea behind this is to serve as a base for using an Arduino to convert other telemetry protocols (OpenTelemetry, BST, S.Port) to the very common MAVLink format. This then allows the use of MAVLink-based groundstation applications with non-supported telemetry protocols.

The Sketch can also be used to simulate any type of MAVLink vehicle from an Arduino, as the parsed telemetry data can be sourced from any source connected to the Arduino (for example, this would allow the use of Mission Planner to view telemetry from a simple Arduino-based rover).

Note that in its current state, the sketch does **NOT** convert any telemetry protocol or provide a data source - it only acts as an interface for converting raw input variables into MAVLink format.

## Installation & Testing

Simply flash the .ino file on any Arduino (tested on Arduino Nano), don't forget to include the Mavlink library.
The Arduino will then provide telemetry data through its USB port at 57600 bauds. The code can be adapted relatively easily to provide data through other outputs (such as a bluetooth module) by changing the output functions to output on another Serial interface.

From your groundstation application, simply choose the appropriate COM port, set baud rate to 57600 and hit "connect".
If you are using Mission Planner, you might have to skip parameter fetching, as the Arduino does not output any parameter data.

## Currently supported applications

The following groundstation applications have been tested and are known to be working with this sketch:

- [x] Mission Planner (Windows)
- [x] APM Planner 2.0 (macOS/Windows)
- [ ] QGroundcontrol (macOS/Windows) -> coming soon

## Known issues

- Climb rate and acceleration are currently unsupported


If you find any further problems, feel free to open an issue!

## Future plans

- Create separate versions for converting OpenTelemetry, BST and S.Port telemetry data into MAVLink format
- Add support for QGroundcontrol


#### (c) 2019 David Wyss

Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink
