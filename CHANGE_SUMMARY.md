# CA3 Current Change Summary

This note summarizes the current observable changes in the project based on the present workspace state, especially the changes used to get the baseline build, UART output, and assignment-aligned polling configuration running.

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
  - inference latency around `4 ms`
  - UART latency around `9-23 ms`
  - `0.00%` miss rate for the five 1 Hz sensor tasks
  - non-zero accelerometer miss rate after moving to `52 Hz`, currently around `1.0-1.5%`

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

### 3. Polling rates were updated toward the assignment baseline

The current code now advertises and uses:

```text
acc=52 gyro=1 hum=1 temp=1 mag=1 prs=1
```

Implemented changes:

- accelerometer polling increased from `26 Hz` to `52 Hz`
- gyroscope polling reduced from `12.5 Hz` to `1 Hz`
- magnetometer polling reduced from `1.25 Hz` to `1 Hz`
- Option 1 remains disabled during baseline validation

Purpose:

- aligns the polling schedule with the assignment baseline requirements
- keeps the optimization path separated from baseline testing

### 4. 52-sample accelerometer capture is now bridged to the current 26-sample HAR model

The generated AI model still expects `26x3` input, but the baseline scheduler now captures `52` accelerometer samples per second.

Current implementation:

- capture `52` accelerometer samples per one-second window
- downsample the captured window from `52 -> 26`
- feed the reduced `26x3` window into the current HAR-Net model

Purpose:

- respects the assignment's `52 Hz` accelerometer sampling requirement
- keeps the existing generated model usable without regenerating X-CUBE-AI immediately

### 5. UART reporting is active and periodic

The application sends these boot/status messages:

- `CA3 start ...`
- `ODR[Hz] ...`
- one runtime status line per second

Purpose:

- Confirms the board is alive after reset
- Exposes scheduler state, sensor values, inference result, and timing metrics
- Provides raw evidence for later report writing

### 6. Baseline metrics are already exposed over UART

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

## Current Baseline Status

The polling configuration now matches the assignment target rates at the scheduler level:

- accelerometer: `52 Hz`
- gyroscope: `1 Hz`
- humidity: `1 Hz`
- temperature: `1 Hz`
- magnetometer: `1 Hz`
- pressure: `1 Hz`

This means the project has moved from "frequency mismatch" into "performance tuning under the correct rates".

## Important Current Limitation

The main remaining baseline issue is accelerometer miss rate under the new `52 Hz` schedule.

Observed behavior:

- five low-rate sensors stay at `0.00%` miss rate
- accelerometer miss rate rises to about `1.0-1.5%`
- inference remains stable at about `4 ms`
- UART transmission can take about `23 ms`, which is longer than a single `52 Hz` accelerometer period (`~19.23 ms`)

Most likely cause:

- blocking UART transmission occasionally delays the next accelerometer poll enough to count as a missed update

Implication:

- the system is much closer to the assignment baseline now
- but it still needs UART/reporting optimization before the accelerometer miss-rate target of `0%` is satisfied

## Known Model/Input Mismatch Risk

The current AI model report indicates an input shape of `1x26x3`, while the assignment text asks for buffering `52` accelerometer readings per inference window.

This means one of the following must be clarified in the final implementation/report:

- use a model that truly expects `52x3`, or
- keep sampling at `52 Hz` and explicitly explain how the model input window is derived from that stream

## Recommended Next Steps

1. Shorten the periodic UART status line to reduce blocking time.
2. Re-test the `52 Hz` baseline and verify whether accelerometer miss rate returns to `0.00%`.
3. Save a continuous UART log for at least `60 s`.
4. After the baseline is stable, re-enable and evaluate the selected optimization option.

## Runtime Evidence Snapshot

Observed boot/status output includes:

```text
UART boot ok
CA3 start t=1 ms, sensor_init_fail=0, ai_init_fail=0, option1=0
ODR[Hz] acc=26 gyro=12.5 hum=1 temp=1 mag=1.25 prs=1; polling cyclic scheduler, no FIFO
t=1s ... miss% a/g/h/t/m/p=0.00/0.00/0.00/0.00/0.00/0.00 ...
```

Updated runtime evidence now includes:

```text
UART boot ok
CA3 start t=1 ms, sensor_init_fail=0, ai_init_fail=0, option1=0
ODR[Hz] acc=52 gyro=1 hum=1 temp=1 mag=1 prs=1; polling cyclic scheduler, HAR input downsampled 52->26
t=1s ... miss% a/g/h/t/m/p=0.00/0.00/0.00/0.00/0.00/0.00 ...
t=9s ... miss% a/g/h/t/m/p=1.47/0.00/0.00/0.00/0.00/0.00 ...
```

This confirms that the current codebase is runnable, measurable, and scheduler-aligned with the assignment frequencies, with the remaining issue narrowed down to accelerometer misses under UART blocking.
