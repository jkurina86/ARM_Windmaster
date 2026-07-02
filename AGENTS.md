# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

ARM_Windmaster is an STM32L476RG-based embedded data acquisition system that records synchronized sensor data at 50 Hz with microsecond-precision timestamps for environmental research. It integrates:
- **WindMaster** sonic anemometer (3D wind speed via USART2)
- **VectorNav VN-300** GPS/IMU (orientation, position, velocity via USART3+DMA)
- **AB-RTCMC RTC** external real-time clock (SPI1)
- **SD Card** data logging (SPI2 with FatFS)

## Build Commands

```bash
cmake --preset Debug          # Configure (Ninja + arm-none-eabi-gcc toolchain)
cmake --build build/Debug     # Build → produces .elf, .bin, .hex
cmake --build build/Debug --clean-first  # Clean rebuild
cmake --build build/Debug --target flash # Flash via STM32CubeProgrammer CLI + ST-Link
```

**Adding new source files**: Add `.c` files to `target_sources()` in the root `CMakeLists.txt`. CubeMX-generated files go in `cmake/stm32cubemx/CMakeLists.txt`.

**Toolchain** (`cmake/gcc-arm-none-eabi.cmake`): Cortex-M4 hard float (fpv4-sp-d16), C11 with GNU extensions. Debug: `-O0 -g3`, Release: `-Os -g0`. Linker uses `--specs=nano.specs -u _printf_float` (required for Telus CSV float output). Linker script: `STM32L476XX_FLASH.ld`.

**CubeMX**: The `.ioc` file contains hardware config. Regenerating updates files in `cmake/stm32cubemx/`. When editing CubeMX-generated files (`main.c`, `stm32l4xx_it.c`, `gpio.c`, `usart.c`, `dma.c`, `spi.c`, `tim.c`, `rtc.c`, `stm32l4xx_hal_msp.c`), all custom code **must** go between `/* USER CODE BEGIN */` and `/* USER CODE END */` comment pairs or it will be overwritten on regeneration.

**IDE**: Build first to generate `compile_commands.json` in `build/Debug/`. The `.clangd` config filters ARM-specific flags for clangd indexing. VSCode tasks and Cortex-Debug launch configs are in `.vscode/`.

## Architecture

```
Shell (USART1)  ──>  Tasker Queue  ──>  Task Handlers
                          │
                          v
     ┌──────────────────────────────────────┐
     │           Recorder Module            │
     │  (dual queues, time-sync, buffering) │
     └──────────────────────────────────────┘
          │                    │
          v                    v
   VectorNav Driver      WindMaster Driver     ──>  Calculations  ──>  Telus (CSV over UART4)
   (USART3 + DMA)         (USART2 + DMA)           (1-min stats)
```

### Key Modules (all in `Core/Src/` and `Core/Inc/`)

| Module | Purpose |
|--------|---------|
| `recorder.c/h` | Dual 16-entry queues (power-of-2), 32-record double-buffered write, time-syncs VN+WM packets within 25ms, 20MB log rollover |
| `vectornav.c/h` | VN-300 binary protocol (0xFA sync, 86-byte packets, CRC16), GPS time extraction |
| `windmaster.c/h` | Mode 10 binary packets (0xB4B4 header, 23 bytes, XOR checksum), U/V/W in cm/s |
| `systime.c/h` | Epoch timekeeping (base 2000-01-01), PPS-disciplined TIM2 microsecond timestamps |
| `calculations.c/h` | 1-minute Earth-frame wind stats at 20 Hz with motion correction (lever arm + ZYX Euler rotation) |
| `telus.c/h` | UART4 telemetry - responds to `idata\r\n` with CSV calculation reports via DMA |
| `shell.c/h` | Interactive CLI on USART1 (type `help` for commands) |
| `tasker.c/h` | 16-entry deferred task queue (function pointer + 16B inline arg, moves work from ISR to main loop) |
| `filesystem.c/h` | FatFS wrapper for SD card |
| `ab-rtcmc-rtc.c/h` | SPI driver for AB-RTCMC external RTC |

### Task Handlers

- `task_gen.c` - System commands (reset, status, version, help, snooze)
- `task_rec.c` - Recording commands (start, stop, stats, queue-debug)
- `task_fs.c` - File operations (mount, unmount, df, ls, cat, write, rm, mkdir, rmdir, cp)
- `task_rtc.c` - RTC commands (settime, gettime, temp, timer-set/stop/status)

## Main Loop and Initialization

System blocks on startup until GPS fix is acquired for time sync (`main.c`):
1. HAL + peripheral init (80 MHz from HSE+PLL)
2. Module init: systime, filesystem, tasker, calc, RS232 transceivers (`transceiver_init()` in `gpio.c`), UART ISRs, sensors, telus
3. **Wait for GPS lock**, sync system epoch
4. Enter main loop:

```c
while (1) {
    recorder_service();              // Process queues, write to SD
    calc_service();                  // Finalize statistics (one state machine step per call)
    telus_service();                 // Process Telus commands
    bad_packet_recorder_service();   // Flush bad WM packets to SD
    shell_task();                    // Process shell input
    tasker_run();                    // Execute one deferred task
}
```

## DMA Channel Assignments

ISR handlers for these are in `stm32l4xx_it.c`. Know these when modifying sensor drivers or adding DMA-based peripherals.

| DMA Channel | Peripheral | Direction | Notes |
|-------------|-----------|-----------|-------|
| DMA1 Ch2 | USART3 (VectorNav) | TX | Polling TX, DMA ISR just clears TC flag |
| DMA1 Ch3 | USART3 (VectorNav) | RX | 1KB circular buffer in RAM2_IMU |
| DMA1 Ch6 | USART2 (WindMaster) | RX | 1KB circular buffer in RAM2_WM |
| DMA1 Ch7 | USART2 (WindMaster) | TX | Used for config commands |
| DMA2 Ch2 | UART5 | RX | RS232 transceiver on PB4/PB5 |
| DMA2 Ch3 | UART4 (Telus) | TX | 6KB buffer in RAM2_TELUS, calls `telus_tx_complete()` on TC |
| DMA2 Ch4 | SPI1 (RTC) | TX | HAL-managed |
| DMA2 Ch5 | UART4 (Telus) | RX | 64-byte circular buffer |

## Memory Layout (RAM2 Sections in Linker Script)

Real-time buffers are placed in specific RAM2 (32KB at 0x10000000) regions via `__attribute__((section(...)))`:

| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| RAM2_WM | 0x10000000 | 1KB | WindMaster DMA circular buffer |
| RAM2_IMU | 0x10000400 | 1KB | VectorNav DMA circular buffer |
| RAM2_REC | 0x10000800 | 8KB | Dual record buffers (4KB each) |
| RAM2_QUEUES | 0x10002800 | 2KB | WM/VN paired queues |
| RAM2_CALC | 0x10003000 | 3KB | Calculation report circular buffer (30 reports) |
| RAM2_TELUS | 0x10003C00 | 6KB | UART4 TX DMA buffer |
| RAM2_BAD | 0x10005400 | 2KB | Bad WindMaster packet double buffers (32 entries each) |

## Data Formats

**Recorder output** (`Recorder_Data_t`): 128-byte fixed records with extracted sensor fields. Layout: magic 0xFACEFACE, log_index, epoch_seconds, ms, GPS time (uint64), VN attitude (yaw/pitch/roll floats), VN gyro (xyz floats), VN position (lat/lon/alt doubles), VN velocity (NED floats), VN acceleration (xyz floats), WM wind (U/V/W int16 cm/s), SoS, Temp, timing_offset_ms, 18B padding.

**Telus CSV**: ISO 8601 timestamps, lat/lon (6dp), wind values (3dp). Format: `timestamp,latitude,longitude,u_mean,u_std,v_mean,v_std,w_mean,w_std,wind_speed_mean,wind_speed_std,wind_from_mean,wind_from_std,gust_mean,gust_std`

## Hardware Peripherals

| Peripheral | Function |
|------------|----------|
| USART1 (115200) | Shell/debug (HAL interrupt RX) |
| USART2 (57600, DMA) | WindMaster (LL driver) |
| USART3 (DMA) | VectorNav VN-300 (LL driver) |
| UART4 (115200, DMA RX/TX) | Telus telemetry (LL driver) |
| UART5 (DMA) | RS232 transceiver (enable: PB4 high, PB5 low) |
| SPI1 (CS: PA4) | AB-RTCMC RTC |
| SPI2 (CS: PB12) | SD card |
| TIM2 | 1 MHz free-running microsecond counter (systime) |
| TIM3 CC2 | Captures RTC CLKOUT rising edge → `systime_pps_event()` |
| TIM4 | 20 Hz timer (currently disabled in main) |

**RS232 transceivers** are enabled by GPIO in `transceiver_init()` (`gpio.c`): PB0 for USART3, PB1 for USART2, PB2 for SD card power, PB4/PB5 for UART5.

## Critical Patterns and Invariants

- **128-byte record invariant**: `Recorder_Data_t` must stay exactly 128 bytes. Enforced with `_Static_assert`.
- **DMA circular buffers** handle all sensor UART input. Check buffer sizes when modifying baud rates.
- **Non-blocking design**: ISRs only enqueue data or clear flags. All processing (file I/O, stats) happens in main loop. Never add blocking calls to ISR context.
- **Paired queue model**: VN and WM packets are time-matched within 25ms window before writing. Queue lengths must be power of 2 (index masking).
- **Double-buffered recording**: Active buffer fills while flush buffer writes to SD.
- **Volatile queue indices**: Head/tail pointers marked `volatile` for ISR-safe access.
- **Motion correction**: `U_earth = T(roll,pitch,yaw) × [U_observed + Ω×R]` where R = 0.8m lever arm.
- **Calculations state machine**: `COLLECTING → READY → FINALIZE_SPEED → FINALIZE_DIRECTION → FINALIZE_GUST → DONE` (one stage per `calc_service()` call, non-blocking).
- **System clock**: 80 MHz from 4 MHz HSE with PLL (PLLM=1, PLLN=20, PLLR=2).
- **LL vs HAL**: Sensor UARTs (USART2, USART3, UART4) use LL drivers for performance. Shell (USART1) and SPI use HAL. Don't mix LL/HAL calls on the same peripheral.
- **Version string**: Hardcoded in `task_gen.c` `handle_version()`.

## Scripts

Located in `Scripts/`. Require `pyserial` (`pip install pyserial`).

```bash
# Parse binary logs to CSV/TXT
python Scripts/parse_log.py log_12345678.bin --csv output.csv
python Scripts/parse_log.py ./logs/ --csv --txt --output-dir ./output

# Test Telus interface
python Scripts/test_telus.py COM6                  # Windows
python Scripts/test_telus.py /dev/ttyUSB0          # Linux
python Scripts/test_telus_periodic.py              # Long-duration polling
```

`anomaly.py` - Wind speed anomaly detection (>30 m/s threshold). Requires `pandas`.
