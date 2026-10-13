# Multi Point Dual Mode Spraying System

Spraying-control documentation for the agricultural robot project by Akinsanya Ezra Oluwatobi.

## Scope

This repository contains extracts of the ESP32-S3 spraying functions from the integrated robot controller. It covers the pump relay, servo-operated spray rail, liquid-level state machine, Raspberry Pi heartbeat supervision, spray-command lease and spraying telemetry. It does not contain the collaborators' LiDAR navigation, obstacle avoidance, motor-drive, odometry, image inference or training implementations.

## Files

- `spraying_control_excerpt.cpp`: selected configuration, state and function definitions, copied from the supplied integrated source.
- `spraying_command_branches.inc`: spraying-related branches from the existing `handlePiCommand` function. These belong inside that handler, not at file scope.
- `provenance.json`: source fingerprint and extraction record.

These files are documentation excerpts, **not a complete Arduino sketch and not replacement robot firmware**. They have not been independently compiled or hardware-tested. Removing the surrounding firmware would also remove essential mode, communication and motor-safety coordination.

## Retained interfaces

The complete firmware supplies `PiSerial`, `writePulse`, `stopMotors`, acknowledgement helpers and the mode/inhibit command paths. Its `setup()` initialises the relay OFF, UART, level input and servo channel. Its repeated service loop polls commands and calls the fluid, servo, safety, spray-output and telemetry services. Those full entry points are not reproduced here because they also contain navigation implementation.

| Connection | ESP32-S3 GPIO |
|---|---:|
| Pi UART RX | 18 |
| Pi UART TX | 17 |
| Pump relay control | 13 |
| Rail servo control | 14 |
| Level-sensor input | 12 |

UART baud rate is 230400. GPIO numbers are chip GPIO identifiers, not Raspberry Pi physical pin numbers. Supply compatibility, relay polarity and mechanical servo limits must be checked against the assembled hardware before using the complete firmware.

## Configuration preserved from the supplied version

| Setting | Value |
|---|---:|
| Spray-command lease | 500 ms |
| Pi heartbeat timeout | 1000 ms |
| Rail horizontal command | 97.6 degrees |
| Rail vertical command | 7.6 degrees |
| Rail settling interval | 1000 ms |
| Level sampling interval | 20 ms |
| Level debounce | 300 ms |
| Dry-state confirmation | 20000 ms |
| Fluid recovery stability | 2000 ms |
| Fluid sensor enabled at startup | false |

The level sensor is bypassed by default in this supplied revision. When enabled, a debounced dry reading causes pumping to stop in the EMPTY_PENDING state; the 20-second timer confirms/latches empty status. It is not an instruction to keep the pump running dry for 20 seconds. Do not describe this configuration as the same test configuration that produced the earlier reported 1.45–2.00-second cutoff result.

The pump requires an unexpired request, an active mode, cleared inhibition, a healthy heartbeat, operational fluid status, a settled servo and no PWM fault. The application-level MANUAL enum does not itself describe the physical RC relay position. This excerpt does not implement the physical mode-selection relay wiring.

## Commands represented

`HB`, `SPRAY,ON`, `SPRAY,OFF`, `TARGET,WEED`, `TARGET,MAIZE`, `FLUID,SENSOR,1`, `FLUID,SENSOR,0`, and `FLUID,RECOVER`.

The complete controller additionally handles mode and inhibit commands. A documented spray lease is not evidence of treatment accuracy; experimental results must remain tied to their actual tested revision.

## Attribution

The extracts were separated from an integrated team controller for documentation of the spraying contribution. Shared interfaces remain dependencies; this repository does not claim sole authorship of the complete robot software. No separate software licence has been selected.
