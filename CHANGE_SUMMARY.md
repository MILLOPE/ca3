# CA3 Current Change Summary

This note summarizes the current observable changes in the project based on the present workspace state, especially the changes used to get the baseline build and UART output running.

## Scope

- Project: `CA3`
- Main file: `D:\code\ceg_sem2\ceg5202\CA3\Core\Src\main.c`
- Summary basis: current source code and runtime output

## Current Functional State

- The project builds successfully in STM32CubeIDE and produces `CA3.elf`.
- The board runs successfully after flashing.
- `LED2` blinks during runtime, which confirms the scheduler loop is active.
- UART output is visible on `STMicroelectronics STLink Virtual COM Port (COM3)` at `115200 8-N-1`.
- Sensor initialization succeeds:
  - `sensor_init_fail=0`
- AI model initialization succeeds:
  - `ai_init_fail=0`
- Periodic status messages are transmitted once per second.
- HAR inference is running and currently reports valid classifications such as `Stationary`.
- Measured runtime metrics currently show:
  - `0.00%` miss rate for all six sensor tasks
  - inference latency around `4 ms`
  - UART latency around `23 ms`

## Changes/Behaviors Confirmed in `main.c`

### 1. UART startup test message added

A direct UART test line is emitted during boot so serial output can be verified before the rest of the application logic is considered.

Observed runtime message:

```text
UART boot ok
```

Purpose:

- Confirms `USART1` and the ST-LINK virtual COM port path are working
- Separates UART problems from scheduler/sensor/AI problems

### 2. Baseline run currently uses Option 1 disabled

The current run output shows:

```text
option1=0
```

This indicates the external blocking task used for the optimization path is disabled for the current baseline test run.

Purpose:

- Keeps the run closer to a clean baseline configuration
- Avoids interference from the urgent external task during basic validation

### 3. UART reporting is active and periodic

The application sends these boot/status messages:

- `CA3 start ...`
- `ODR[Hz] ...`
- one runtime status line per second

Purpose:

- Confirms the board is alive after reset
- Exposes scheduler state, sensor values, inference result, and timing metrics
- Provides raw evidence for later report writing

### 4. Baseline metrics are already exposed over UART

Current status lines include:

- timestamp
- accelerometer, gyroscope, humidity, temperature, magnetometer, pressure
- classification result
- probabilities
- inference latency
- UART latency
- miss rate for all six sensor tasks
- inference queue/drop counters

Purpose:

- Makes baseline evaluation measurable without extra debug tools
- Matches the assignment's need for quantitative evidence

## Important Current Limitation

The current runtime configuration does not yet fully match the assignment baseline requirements.

Current printed ODRs:

```text
acc=26 gyro=12.5 hum=1 temp=1 mag=1.25 prs=1
```

Assignment baseline requires:

- accelerometer: `52 Hz`
- all other 5 sensors: `1 Hz`

So the following still need adjustment:

- accelerometer: `26 -> 52 Hz`
- gyroscope: `12.5 -> 1 Hz`
- magnetometer: `1.25 -> 1 Hz`

## Known Model/Input Mismatch Risk

The current AI model report indicates an input shape of `1x26x3`, while the assignment text asks for buffering `52` accelerometer readings per inference window.

This means one of the following must be clarified in the final implementation/report:

- use a model that truly expects `52x3`, or
- keep sampling at `52 Hz` and explicitly explain how the model input window is derived from that stream

## Recommended Next Steps

1. Adjust task periods and ODR-related constants to match the assignment baseline exactly.
2. Keep `option1=0` while validating the strict baseline.
3. Re-run for at least `60 s` and save the UART log.
4. After the baseline is stable, re-enable and evaluate the selected optimization option.

## Runtime Evidence Snapshot

Observed boot/status output includes:

```text
UART boot ok
CA3 start t=1 ms, sensor_init_fail=0, ai_init_fail=0, option1=0
ODR[Hz] acc=26 gyro=12.5 hum=1 temp=1 mag=1.25 prs=1; polling cyclic scheduler, no FIFO
t=1s ... miss% a/g/h/t/m/p=0.00/0.00/0.00/0.00/0.00/0.00 ...
```

This confirms that the current codebase is runnable and measurable, but still needs frequency alignment to satisfy the assignment baseline strictly.
