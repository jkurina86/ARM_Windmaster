# ARM_Windmaster Technical Reference Manual

**Document Version:** 1.0
**Hardware Platform:** STM32L476RG Nucleo-64
**Firmware Version:** See `version` shell command

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Memory Map](#3-memory-map)
4. [Sensor Interfaces](#4-sensor-interfaces)
   - 4.1 [VectorNav VN-300 GPS/IMU](#41-vectornav-vn-300-gpsimu)
   - 4.2 [WindMaster Sonic Anemometer](#42-windmaster-sonic-anemometer)
   - 4.3 [AB-RTCMC Real-Time Clock](#43-ab-rtcmc-real-time-clock)
5. [Data Recording System](#5-data-recording-system)
6. [Wind Calculations Engine](#6-wind-calculations-engine)
7. [Telemetry Interface (Telus)](#7-telemetry-interface-telus)
8. [System Timing](#8-system-timing)
9. [Shell Command Reference](#9-shell-command-reference)
10. [Software Architecture](#10-software-architecture)
11. [Configuration & Calibration](#11-configuration--calibration)
12. [Troubleshooting](#12-troubleshooting)
13. [Appendices](#13-appendices)

---

## 1. System Overview

### 1.1 Purpose

ARM_Windmaster is an embedded data acquisition system designed for environmental research applications requiring high-precision, motion-corrected wind measurements. The system synchronizes data from a 3D sonic anemometer with GPS/IMU measurements to produce Earth-frame wind vectors corrected for platform motion.

### 1.2 Key Capabilities

| Capability | Specification |
|------------|---------------|
| Wind Measurement Rate | 20 Hz (WindMaster native) |
| IMU/GPS Rate | 50 Hz (VectorNav) |
| Recording Rate | 20 Hz (paired records) |
| Timestamp Precision | Microsecond (TIM2 interpolation) |
| Time Synchronization | GPS-disciplined via VN-300 |
| Statistics Period | 1-minute averages |
| Gust Detection | 3-second sliding window |
| Storage | SD card via SPI (FatFS) |
| External Telemetry | UART4 CSV output |

### 1.3 Functional Block Diagram

```
                                    ┌─────────────────────┐
                                    │   External Host     │
                                    │  (Telus Interface)  │
                                    └──────────┬──────────┘
                                               │ UART4
                                               │ 115200 baud
┌─────────────┐                    ┌───────────▼───────────┐
│  VectorNav  │  USART3 + DMA      │                       │
│   VN-300    │───────────────────►│                       │
│  GPS/IMU    │  Binary @ 50 Hz    │                       │
└─────────────┘                    │                       │
                                   │    STM32L476RG        │
┌─────────────┐                    │                       │
│ WindMaster  │  USART2 + DMA      │    Main Processor     │
│             │───────────────────►│                       │
│ Anemometer  │  Binary @ 20 Hz    │                       │
└─────────────┘                    │                       │
                                   │                       │
┌─────────────┐                    │                       │       ┌──────────┐
│  AB-RTCMC   │  SPI1              │                       │ SPI2  │ SD Card  │
│    RTC      │◄──────────────────►│                       │◄─────►│ (FatFS)  │
│  32.768kHz  │                    │                       │       └──────────┘
└─────────────┘                    └───────────┬───────────┘
                                               │ USART1
                                               │ 115200 baud
                                    ┌──────────▼──────────┐
                                    │   Debug Terminal    │
                                    │   (Shell Interface) │
                                    └─────────────────────┘
```

---

## 2. Hardware Architecture

### 2.1 Microcontroller Specifications

| Parameter | Value |
|-----------|-------|
| MCU | STM32L476RG |
| Core | ARM Cortex-M4 with FPU |
| Clock Speed | 80 MHz (HSE + PLL) |
| Flash | 1024 KB |
| SRAM | 96 KB (main) + 32 KB (RAM2) |
| Package | LQFP64 |

### 2.2 Peripheral Assignments

| Peripheral | Function | Pins | Configuration |
|------------|----------|------|---------------|
| USART1 | Shell/Debug | PA9 (TX), PA10 (RX) | 115200 8N1, Polling |
| USART2 | WindMaster | PA2 (TX), PA3 (RX) | 57600 8N1, DMA |
| USART3 | VectorNav | PC4 (TX), PC5 (RX) | Variable baud, DMA |
| UART4 | Telus | PA0 (TX), PA1 (RX) | 115200 8N1, DMA |
| UART5 | Unused | PC12 (TX), PD2 (RX) | Configured but unused |
| SPI1 | External RTC | PA5 (SCK), PA6 (MISO), PA7 (MOSI), PA4 (CS) | 312.5 kHz |
| SPI2 | SD Card | PB13 (SCK), PB14 (MISO), PB15 (MOSI), PB12 (CS) | Variable |
| TIM2 | Timestamp Counter | Internal | 1 MHz, 32-bit |
| TIM3 | PPS Input | PC7 (CH2) | 1 MHz, Input Capture |
| TIM4 | 20 Hz Tick | Internal | Disabled |

### 2.3 DMA Channel Assignments

```
DMA1 Controller:
├── Channel 2: USART3 TX (VectorNav commands)
├── Channel 3: USART3 RX (VectorNav data) ─► 1024-byte circular buffer
└── Channel 6: USART2 RX (WindMaster data) ─► 1024-byte circular buffer

DMA2 Controller:
├── Channel 3: UART4 TX (Telus response) ─► 6144-byte buffer
├── Channel 4: SPI1 TX (RTC)
└── Channel 5: UART4 RX (Telus command) ─► 64-byte circular buffer
```

### 2.4 GPIO Configuration

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| PB0 | USART3 Transceiver Enable | Output | RS-232 level shifter (VectorNav) |
| PB1 | USART2 Transceiver Enable | Output | RS-232 level shifter (WindMaster) |
| PB2 | SD Card Power | Output | Active high |
| PB4 | UART5 Transceiver Enable 1 | Output | Unused |
| PB5 | UART5 Transceiver Enable 2 | Output | Unused |
| PA4 | RTC SPI Chip Select | Output | Active low |
| PB12 | SD Card SPI Chip Select | Output | Active low |
| PC7 | PPS Input | Input | TIM3 CH2 capture |

### 2.5 Clock Tree

```
HSE (8 MHz) ──► PLL ──► SYSCLK (80 MHz)
                  │
                  ├──► AHB (80 MHz) ──► DMA1, DMA2
                  │
                  ├──► APB1 (80 MHz) ──► USART2, USART3, UART4, UART5
                  │                      SPI2, TIM2, TIM3, TIM4
                  │
                  └──► APB2 (80 MHz) ──► USART1, SPI1, ADC1

LSE (32.768 kHz) ──► RTC (Internal)
                 ──► TIM3 (1 Hz output to external RTC)
```

---

## 3. Memory Map

### 3.1 Flash Layout

| Region | Start | Size | Contents |
|--------|-------|------|----------|
| Vector Table | 0x08000000 | 0x1C0 | Exception handlers |
| Code | 0x080001C0 | ~64 KB | Application firmware |
| Constants | Variable | Variable | String literals, lookup tables |

### 3.2 Main SRAM (96 KB @ 0x20000000)

| Section | Approximate Size | Contents |
|---------|------------------|----------|
| .data | Variable | Initialized globals |
| .bss | Variable | Zero-initialized globals |
| Heap | 0x200 bytes | Dynamic allocation (minimal) |
| Stack | 0x400 bytes | Call stack (grows down) |

### 3.3 RAM2 Allocation (32 KB @ 0x10000000)

RAM2 is specifically allocated for DMA buffers and large data structures to avoid conflicts with the main stack/heap.

| Section | Start Address | Size | Purpose |
|---------|---------------|------|---------|
| `.dma_buffer_wm` | 0x10000000 | 1,024 bytes | WindMaster DMA circular buffer |
| `.dma_buffer_imu` | 0x10000400 | 1,024 bytes | VectorNav DMA circular buffer |
| `.record_buffer_a` | 0x10000800 | 4,096 bytes | Recording buffer A (32 records) |
| `.record_buffer_b` | 0x10001800 | 4,096 bytes | Recording buffer B (32 records) |
| `.queue_wm` | 0x10002800 | 1,024 bytes | WindMaster packet queue (16 entries) |
| `.queue_vn` | 0x10002C00 | 1,024 bytes | VectorNav packet queue (16 entries) |
| `.calc_reports` | 0x10003000 | 3,072 bytes | Calculation report buffer (30 reports) |
| `.telus_tx` | 0x10003C00 | 6,144 bytes | Telus TX buffer (CSV response) |
| **Total Used** | | **22,528 bytes** | 69% of 32 KB |
| **Available** | 0x10005800 | 9,472 bytes | Future expansion |

### 3.4 Linker Script Sections

```ld
MEMORY
{
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
  RAM2   (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
}

SECTIONS
{
  .dma_buffer_wm (NOLOAD) : { *(.dma_buffer_wm) } >RAM2
  .dma_buffer_imu (NOLOAD) : { *(.dma_buffer_imu) } >RAM2
  .record_buffer_a (NOLOAD) : { *(.record_buffer_a) } >RAM2
  .record_buffer_b (NOLOAD) : { *(.record_buffer_b) } >RAM2
  .queue_wm (NOLOAD) : { *(.queue_wm) } >RAM2
  .queue_vn (NOLOAD) : { *(.queue_vn) } >RAM2
  .calc_reports (NOLOAD) : { *(.calc_reports) } >RAM2
  .telus_tx (NOLOAD) : { *(.telus_tx) } >RAM2
}
```

---

## 4. Sensor Interfaces

### 4.1 VectorNav VN-300 GPS/IMU

#### 4.1.1 Overview

The VectorNav VN-300 is a dual-antenna GPS-aided inertial navigation system providing position, velocity, and attitude data. It connects via USART3 using a binary protocol at 50 Hz.

#### 4.1.2 Physical Connection

| VN-300 Pin | STM32 Pin | Signal |
|------------|-----------|--------|
| TX1 | PC5 | USART3 RX |
| RX1 | PC4 | USART3 TX |
| GND | GND | Common ground |

#### 4.1.3 Binary Packet Format (86 bytes)

```
Offset  Size    Type      Field               Description
──────────────────────────────────────────────────────────────────
0       1       uint8     sync                0xFA (sync byte)
1       1       uint8     groups              0x01 (group field)
2       2       uint16    group_field_common  Group field bitmask
4       8       uint64    time_gps            GPS time (nanoseconds since 1980-01-06)
12      4       float     yaw                 Heading (degrees, ±180)
16      4       float     pitch               Pitch angle (degrees, ±90)
20      4       float     roll                Roll angle (degrees, ±180)
24      4       float     gyro_x              Angular rate X (rad/s)
28      4       float     gyro_y              Angular rate Y (rad/s)
32      4       float     gyro_z              Angular rate Z (rad/s)
36      8       double    latitude            WGS84 latitude (degrees)
44      8       double    longitude           WGS84 longitude (degrees)
52      8       double    altitude            Altitude above ellipsoid (m)
60      4       float     vel_n               Velocity North (m/s)
64      4       float     vel_e               Velocity East (m/s)
68      4       float     vel_d               Velocity Down (m/s)
72      4       float     accel_x             Acceleration X (m/s²)
76      4       float     accel_y             Acceleration Y (m/s²)
80      4       float     accel_z             Acceleration Z (m/s²)
84      2       uint16    checksum            CRC-16 (big-endian)
──────────────────────────────────────────────────────────────────
Total: 86 bytes
```

#### 4.1.4 Data Structure

```c
typedef struct {
    uint8_t sync;           // 0xFA
    uint8_t groups;         // 0x01
    uint16_t group_field;   // Bitmask
    uint64_t timegps;       // GPS nanoseconds
    float yaw, pitch, roll; // Attitude (degrees)
    float gyro_x, gyro_y, gyro_z;       // Angular rates (rad/s)
    double latitude, longitude, altitude; // Position
    float vel_n, vel_e, vel_d;          // NED velocity (m/s)
    float accel_x, accel_y, accel_z;    // Acceleration (m/s²)
    uint16_t checksum;      // CRC-16
} VN_Packet_t;
```

#### 4.1.5 Communication Protocol

**Initialization Sequence:**
1. Disable async output: `$VNWRG,75,0,20,00*F819\r\n`
2. Configure DMA circular buffer (1024 bytes)
3. Flush any pending RX data

**Start 50 Hz Output:**
```
Command: $VNWRG,75,1,8,01,01EA*07CF
         │      │ │ │  └─────── Output fields bitmask
         │      │ │ └────────── Rate divisor (800/8 = 50 Hz)
         │      │ └──────────── Async mode enabled
         │      └────────────── Register 75 (async config)
         └───────────────────── Write register command
```

**Stop Output:**
```
Command: $VNWRG,75,0,20,00*F819
```

**GPS Fix Query:**
```
Command: $VNRRG,63*XX  (Read GNSS Solution LLA register)
Response: $VNRRG,63,<time>,<week>,<fix>,<sats>,<lat>,<lon>,<alt>,...*XX
```

#### 4.1.6 Checksum Calculation

The VN-300 uses CRC-16 with polynomial 0x8005, but stored in **big-endian** format:

```c
uint16_t vn_calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = (crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    return crc;
}

// Validation (packet checksum is big-endian)
uint16_t received = (packet[84] << 8) | packet[85];
uint16_t computed = vn_calculate_crc16(&packet[1], 83);
bool valid = (received == computed);
```

#### 4.1.7 GPS Time Conversion

```c
// GPS epoch: January 6, 1980 00:00:00 UTC
// Leap seconds: 18 (as of 2017)
#define GPS_EPOCH_UNIX      315964800  // Unix timestamp of GPS epoch
#define GPS_LEAP_SECONDS    18

RTC_DateTime_t gps_ns_to_datetime(uint64_t gps_ns) {
    uint64_t gps_seconds = gps_ns / 1000000000ULL;
    uint32_t unix_seconds = GPS_EPOCH_UNIX + gps_seconds - GPS_LEAP_SECONDS;
    // Convert unix_seconds to year/month/day/hour/minute/second
    return datetime;
}
```

---

### 4.2 WindMaster Sonic Anemometer

#### 4.2.1 Overview

The Gill WindMaster is a 3-axis ultrasonic anemometer that measures wind speed and direction by timing ultrasonic pulses between three pairs of transducers. The system uses Mode 10 (binary output) at 57,600 baud.

#### 4.2.2 Physical Connection

| WindMaster Pin | STM32 Pin | Signal |
|----------------|-----------|--------|
| TX | PA3 | USART2 RX |
| RX | PA2 | USART2 TX |
| GND | GND | Common ground |

#### 4.2.3 Mode 10 Binary Packet Format (23 bytes)

```
Offset  Size    Type      Field           Description
──────────────────────────────────────────────────────────────────
0       2       uint16    header          0xB4B4 (sync pattern)
2       2       int16     status          Status/error code
4       2       int16     u_speed         U-axis wind speed (mm/s)
6       2       int16     v_speed         V-axis wind speed (mm/s)
8       2       int16     w_speed         W-axis wind speed (mm/s)
10      2       int16     sos             Speed of Sound (m/s × 10)
12      2       int16     analog_1        Analog input 1
14      2       int16     analog_2        Analog input 2
16      2       int16     analog_3        Analog input 3
18      2       int16     analog_4        Analog input 4
20      2       int16     temperature     PRT temperature
22      1       uint8     checksum        XOR of bytes 2-21
──────────────────────────────────────────────────────────────────
Total: 23 bytes
```

#### 4.2.4 Data Structure

```c
#define WM_PACKET_SIZE      23
#define WM_HEADER           0xB4B4

typedef struct {
    uint16_t header;        // 0xB4B4
    int16_t status;         // Status word
    int16_t U_axis_speed;   // U-axis (mm/s)
    int16_t V_axis_speed;   // V-axis (mm/s)
    int16_t W_axis_speed;   // W-axis (mm/s)
    int16_t SoS;            // Speed of sound (m/s × 10)
    int16_t analog_inputs[4]; // Analog channels
    int16_t Temp;           // Temperature
    uint8_t checksum;       // XOR checksum
} WM_Packet_t;
```

#### 4.2.5 Coordinate System

The WindMaster uses a right-handed coordinate system:

```
        W (+)
         │
         │    V (+)
         │   /
         │  /
         │ /
         │/_________ U (+)
        Origin

U-axis: Positive toward North reference mark
V-axis: Positive 90° clockwise from U (toward East)
W-axis: Positive upward (vertical)
```

#### 4.2.6 Unit Conversions

| Measurement | Raw Unit | Conversion |
|-------------|----------|------------|
| U, V, W Speed | mm/s | ÷ 1000 → m/s |
| Speed of Sound | 0.1 m/s | ÷ 10 → m/s |
| Temperature | Raw ADC | Device-specific |

#### 4.2.7 Communication Protocol

**Enter Configuration Mode:**
```
Command: *\r
Response: (echo) *\r\n
          Mode 10 output stops
```

**Exit Configuration Mode (Start Measurements):**
```
Command: Q\r
Response: <CR><LF> (then binary packets begin)
```

**Checksum Calculation:**
```c
uint8_t wm_calculate_checksum(const uint8_t *data) {
    uint8_t checksum = 0;
    for (int i = 2; i < 22; i++) {  // Bytes 2-21
        checksum ^= data[i];
    }
    return checksum;
}
```

#### 4.2.8 Status Word Interpretation

| Bit | Meaning |
|-----|---------|
| 0-3 | Error code (0 = OK) |
| 4 | Path 1 error |
| 5 | Path 2 error |
| 6 | Path 3 error |
| 7 | Reserved |
| 8-15 | Sample counter |

---

### 4.3 AB-RTCMC Real-Time Clock

#### 4.3.1 Overview

The AB-RTCMC-32.768kHz-B5ZE-S3 is a temperature-compensated real-time clock with SPI interface. It provides battery-backed timekeeping and a programmable clock output.

#### 4.3.2 Physical Connection

| RTC Pin | STM32 Pin | Signal |
|---------|-----------|--------|
| MOSI | PA7 | SPI1 MOSI |
| MISO | PA6 | SPI1 MISO |
| SCK | PA5 | SPI1 SCK |
| CS | PA4 | GPIO (active low) |
| CLKOUT | PC7 | TIM3 CH2 (1 Hz PPS) |

#### 4.3.3 SPI Configuration

| Parameter | Value |
|-----------|-------|
| Mode | Master |
| Data Size | 8 bits |
| Clock Polarity | Low (CPOL=0) |
| Clock Phase | 1st Edge (CPHA=0) |
| Prescaler | 256 (80 MHz ÷ 256 = 312.5 kHz) |
| Bit Order | MSB First |

#### 4.3.4 Register Map

**Control Registers (0x00-0x04):**

| Address | Register | Description |
|---------|----------|-------------|
| 0x00 | Control_1 | Main control (WE, TE, TAR, EERE, SRON, TD[1:0], CLK_INT) |
| 0x01 | Control_INT | Interrupt enables (AIE, TIE, V1IE, V2IE, SRIE) |
| 0x02 | Control_INT_FLAG | Interrupt flags (AF, TF, V1IF, V2IF, SRF) |
| 0x03 | Control_STATUS | Status (V1F, V2F, SR, PON, EEBUSY) |
| 0x04 | Control_RESET | System reset (SYSR) |

**Clock Registers (0x08-0x0E, BCD format):**

| Address | Register | Range |
|---------|----------|-------|
| 0x08 | Seconds | 00-59 |
| 0x09 | Minutes | 00-59 |
| 0x0A | Hours | 00-23 (24h) or 01-12 + AM/PM |
| 0x0B | Days | 01-31 |
| 0x0C | Weekdays | 0-6 |
| 0x0D | Months | 01-12 |
| 0x0E | Years | 00-99 |

**Timer Registers (0x18-0x19):**

| Address | Register | Description |
|---------|----------|-------------|
| 0x18 | Timer_Low | Timer value bits 0-7 |
| 0x19 | Timer_High | Timer value bits 8-15 |

**Temperature Register (0x20):**

| Address | Register | Description |
|---------|----------|-------------|
| 0x20 | Temperature | Signed 8-bit, 1°C per LSB |

#### 4.3.5 SPI Protocol

**Write Operation:**
```
CS Low → [0x00 | reg] → [data] → CS High
         │
         └── Bit 7 = 0 for write
```

**Read Operation:**
```
CS Low → [0x80 | reg] → [0x00 (dummy)] → CS High
         │                    │
         │                    └── Returns data on MISO
         └── Bit 7 = 1 for read
```

#### 4.3.6 Timer Configuration

```c
typedef enum {
    RTC_TIMER_DIV_4096HZ = 0x00,  // 4096 Hz tick
    RTC_TIMER_DIV_64HZ   = 0x20,  // 64 Hz tick
    RTC_TIMER_DIV_1HZ    = 0x40,  // 1 Hz tick
    RTC_TIMER_DIV_1_60HZ = 0x60   // 1/60 Hz tick (1 minute)
} RTC_TimerDivision_t;

// For N seconds at 1 Hz: timer_value = N
// For N minutes at 1/60 Hz: timer_value = N
```

#### 4.3.7 CLKOUT Configuration

The RTC CLKOUT pin is configured to output 1 Hz for PPS synchronization:

```c
// Control_1 register: TD[1:0] = 01 for 1 Hz output
RTC_WriteRegister(RTC_REG_CONTROL1, 0x20);  // TD0=1, TD1=0 → 1 Hz
```

---

## 5. Data Recording System

### 5.1 Architecture Overview

The recording system uses a dual-queue, double-buffered architecture to ensure continuous data capture without blocking the main loop.

```
   VectorNav Driver              WindMaster Driver
         │                              │
         │ recorder_queue_vn()          │ recorder_queue_wm()
         ▼                              ▼
  ┌──────────────┐              ┌──────────────┐
  │   VN Queue   │              │   WM Queue   │
  │  16 entries  │              │  16 entries  │
  └──────┬───────┘              └──────┬───────┘
         │                              │
         └──────────┬───────────────────┘
                    │ recorder_service()
                    ▼
           ┌────────────────┐
           │ Time Matching  │
           │ (Pairing)      │
           └────────┬───────┘
                    │
                    ▼
           ┌────────────────┐
           │ Active Buffer  │
           │ (32 records)   │
           └────────┬───────┘
                    │ Buffer full
                    ▼
           ┌────────────────┐
           │ Flush Buffer   │
           │ (SD Write)     │
           └────────────────┘
```

### 5.2 Record Format (128 bytes)

```c
typedef struct {
    // Header (10 bytes)
    uint32_t magic_number;      // 0xFACEFACE
    uint32_t log_index;         // Sequential record number
    uint32_t epoch_seconds;     // Seconds since 2000-01-01
    uint16_t ms;                // Milliseconds (0-999)

    // VectorNav Data (76 bytes)
    uint64_t timegps;           // GPS nanoseconds
    float yaw, pitch, roll;     // Attitude (degrees)
    float gyro_x, gyro_y, gyro_z;       // Angular rates (rad/s)
    double latitude, longitude, altitude; // Position
    float vel_n, vel_e, vel_d;          // NED velocity (m/s)
    float acc_x, acc_y, acc_z;          // Acceleration (m/s²)

    // WindMaster Data (12 bytes)
    int16_t U_axis_speed;       // U-axis (mm/s)
    int16_t V_axis_speed;       // V-axis (mm/s)
    int16_t W_axis_speed;       // W-axis (mm/s)
    int16_t SoS;                // Speed of Sound (0.1 m/s)
    int16_t Temp;               // Temperature
    int16_t timing_offset_ms;   // WM_time - VN_time (±25 ms)

    // Padding (18 bytes)
    uint8_t footer_padding[18];
} Recorder_Data_t;

_Static_assert(sizeof(Recorder_Data_t) == 128, "Record must be 128 bytes");
```

### 5.3 Queue Structures

```c
typedef struct {
    uint32_t timestamp_s;       // Seconds from systime
    uint16_t timestamp_ms;      // Milliseconds (0-999)
    WM_Packet_t wm_packet;      // WindMaster data
} WM_QueueEntry_t;

typedef struct {
    uint32_t timestamp_s;       // Seconds from systime
    uint16_t timestamp_ms;      // Milliseconds (0-999)
    VN_Packet_t vn_packet;      // VectorNav data
} VN_QueueEntry_t;

#define WM_Q_LEN    16          // ~800 ms at 20 Hz
#define VN_Q_LEN    16          // ~320 ms at 50 Hz
```

### 5.4 Time-Matching Algorithm

The pairing algorithm matches WindMaster packets (20 Hz) with the nearest VectorNav packet (50 Hz):

```c
// For each WM packet, find VN packet with minimum time delta
int find_nearest_vn(WM_QueueEntry_t *wm, VN_Queue_t *vn_queue) {
    int best_idx = -1;
    int32_t best_delta = INT32_MAX;

    for (int i = 0; i < vn_queue->count; i++) {
        VN_QueueEntry_t *vn = &vn_queue->entries[i];

        // Calculate time delta in milliseconds
        int32_t delta_s = (int32_t)vn->timestamp_s - (int32_t)wm->timestamp_s;
        int32_t delta_ms = delta_s * 1000 + (int32_t)vn->timestamp_ms - (int32_t)wm->timestamp_ms;

        if (abs(delta_ms) < abs(best_delta)) {
            best_delta = delta_ms;
            best_idx = i;
        }
    }

    // Reject if offset exceeds tolerance
    if (abs(best_delta) > MAX_OFFSET_MS) {
        return -1;
    }

    return best_idx;
}
```

### 5.5 Buffer Management

| Parameter | Value |
|-----------|-------|
| Buffer Size | 4,096 bytes |
| Records per Buffer | 32 |
| Buffer Count | 2 (double-buffered) |
| Log Rollover | 20 MB |

**Buffer Swap Logic:**
```c
void recorder_service(void) {
    // Pair packets and fill active buffer
    while (can_pair_packets() && active_buffer_count < 32) {
        pair_and_store();
    }

    // Swap buffers when active is full
    if (active_buffer_count >= 32) {
        swap_buffers();
        schedule_sd_write();
    }

    // Write flush buffer to SD
    if (flush_pending) {
        write_to_sd();
    }
}
```

### 5.6 File Naming Convention

Log files are named using the epoch timestamp at creation:

```
log_<epoch_seconds>.bin

Example: log_789012345.bin
```

### 5.7 Statistics Tracking

```c
typedef struct {
    bool recording;                 // Currently recording
    uint32_t records_written;       // Total records to SD
    uint32_t active_buffer_records; // Records in current buffer
    uint8_t wm_queue_count;         // Current WM queue depth
    uint8_t vn_queue_count;         // Current VN queue depth
    uint8_t wm_queue_max;           // Peak WM queue depth
    uint8_t vn_queue_max;           // Peak VN queue depth
    uint32_t wm_drops;              // WM overflow drops
    uint32_t vn_drops;              // VN overflow drops
    uint32_t vn_discards;           // VN discards (pairing)
    char filename[64];              // Current log filename
} recorder_stats_t;
```

---

## 6. Wind Calculations Engine

### 6.1 Overview

The calculations module computes 1-minute Earth-frame wind statistics with motion correction. It processes paired VN/WM data at 20 Hz and produces statistical reports every 60 seconds.

### 6.2 Motion Correction Pipeline

#### 6.2.1 Lever Arm Correction

The VN-300 IMU is mounted 0.8 m below the WindMaster anemometer. Platform rotation creates apparent wind that must be subtracted:

```
Physical Setup:
    ┌─────────────┐ ◄── WindMaster (origin)
    │             │
    │     │       │     R = [0, 0, 0.8] m
    │     │ 0.8m  │
    │     ▼       │
    │  ┌─────┐    │ ◄── VN-300 IMU
    │  │ IMU │    │
    │  └─────┘    │
    └─────────────┘

Correction: Ω × R where Ω = [ωx, ωy, ωz] (angular velocity)
```

```c
// Cross product: Ω × R = [ωx, ωy, ωz] × [0, 0, 0.8]
float lever_correction_x = gyro_y * 0.8f;   // ωy × Rz
float lever_correction_y = -gyro_x * 0.8f;  // -ωx × Rz
float lever_correction_z = 0.0f;            // Rx = Ry = 0

// Apply correction to observed wind
u_corrected = u_observed + lever_correction_x;
v_corrected = v_observed + lever_correction_y;
w_corrected = w_observed + lever_correction_z;
```

#### 6.2.2 Platform-to-Earth Frame Transformation

The corrected wind vector is rotated from platform frame to Earth frame using a ZYX Euler rotation matrix:

```c
// Convert angles to radians
float phi   = roll  * DEG_TO_RAD;  // Roll (X rotation)
float theta = pitch * DEG_TO_RAD;  // Pitch (Y rotation)
float psi   = yaw   * DEG_TO_RAD;  // Yaw (Z rotation)

// Precompute trig functions
float cp = cosf(phi),   sp = sinf(phi);
float ct = cosf(theta), st = sinf(theta);
float cy = cosf(psi),   sy = sinf(psi);

// ZYX Rotation Matrix T (platform → Earth)
// T = Rz(yaw) × Ry(pitch) × Rx(roll)
float T[3][3] = {
    { cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp },
    { sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp },
    { -st,    ct*sp,             ct*cp            }
};

// Transform: U_earth = T × U_platform
u_earth = T[0][0]*u + T[0][1]*v + T[0][2]*w;
v_earth = T[1][0]*u + T[1][1]*v + T[1][2]*w;
w_earth = T[2][0]*u + T[2][1]*v + T[2][2]*w;
```

### 6.3 Statistical Computations

#### 6.3.1 Linear Statistics (U, V, W, Speed)

For N samples:

```
Mean:     μ = Σxᵢ / N
Variance: σ² = (Σxᵢ²/N) - μ²
Std Dev:  σ = √σ²
```

```c
// Running accumulation (Welford-style)
sum += x;
sum_sq += x * x;

// Finalization
float mean = sum / n;
float variance = (sum_sq / n) - (mean * mean);
float stddev = sqrtf(fmaxf(0.0f, variance));  // Guard against negative
```

#### 6.3.2 Circular Statistics (Wind Direction)

Wind direction requires circular mean calculation:

```c
// Accumulation
float dir_rad = atan2f(v_earth, u_earth);  // Wind TO direction
float from_rad = dir_rad + M_PI;           // Convert to FROM direction
sin_sum += sinf(from_rad);
cos_sum += cosf(from_rad);

// Finalization
float mean_sin = sin_sum / n;
float mean_cos = cos_sum / n;
float mean_rad = atan2f(mean_sin, mean_cos);
float mean_deg = mean_rad * RAD_TO_DEG;
if (mean_deg < 0) mean_deg += 360.0f;

// Circular standard deviation
float R = sqrtf(mean_sin*mean_sin + mean_cos*mean_cos);  // Resultant length
float stddev_deg;
if (R >= 1.0f) {
    stddev_deg = 0.0f;
} else if (R <= 0.001f) {
    stddev_deg = 180.0f;  // Uniform distribution
} else {
    stddev_deg = sqrtf(-2.0f * logf(R)) * RAD_TO_DEG;
}
```

#### 6.3.3 Gust Detection (3-Second Window)

Gust is defined as the maximum 3-second mean wind speed:

```c
#define GUST_WINDOW 60  // 3 seconds at 20 Hz

// Sliding window ring buffer
float gust_ring[GUST_WINDOW];
int gust_idx = 0;
float window_sum = 0;
float window_sum_sq = 0;
float max_mean = 0;
float max_mean_sum_sq = 0;

// Add new sample
window_sum -= gust_ring[gust_idx];      // Remove oldest
window_sum_sq -= gust_ring[gust_idx] * gust_ring[gust_idx];
gust_ring[gust_idx] = wind_speed;       // Add newest
window_sum += wind_speed;
window_sum_sq += wind_speed * wind_speed;
gust_idx = (gust_idx + 1) % GUST_WINDOW;

// Track maximum
float current_mean = window_sum / GUST_WINDOW;
if (current_mean > max_mean) {
    max_mean = current_mean;
    max_mean_sum_sq = window_sum_sq;
}

// Finalization
gust_mean = max_mean;
gust_std = sqrtf((max_mean_sum_sq / GUST_WINDOW) - (max_mean * max_mean));
```

### 6.4 Report Structure

```c
typedef struct {
    uint32_t timestamp_s;           // Report timestamp (epoch)
    double latitude, longitude;     // Position at end of period

    // Earth-frame wind components (m/s)
    float u_mean, u_std;            // East-West component
    float v_mean, v_std;            // North-South component
    float w_mean, w_std;            // Vertical component

    // Scalar statistics
    float wind_speed_mean, wind_speed_std;  // Horizontal speed (m/s)
    float wind_from_mean, wind_from_std;    // Direction FROM (degrees, 0-360)
    float gust_mean, gust_std;              // 3-sec max (m/s)
} CalcReport_t;
```

### 6.5 State Machine

The finalization process is split across multiple service calls to avoid blocking:

```
┌─────────────┐
│  COLLECTING │ ◄── Normal state, accumulating samples
└──────┬──────┘
       │ 1200 samples reached
       ▼
┌─────────────┐
│    READY    │ ◄── Period complete, copy accumulators
└──────┬──────┘
       │ calc_service() call
       ▼
┌─────────────┐
│ FIN_LINEAR  │ ◄── Compute U/V/W and speed statistics
└──────┬──────┘
       │ calc_service() call
       ▼
┌─────────────┐
│  FIN_CIRC   │ ◄── Compute circular direction statistics
└──────┬──────┘
       │ calc_service() call
       ▼
┌─────────────┐
│  FIN_GUST   │ ◄── Finalize gust statistics
└──────┬──────┘
       │ calc_service() call
       ▼
┌─────────────┐
│    DONE     │ ◄── Report stored in buffer
└──────┬──────┘
       │ Auto-transition
       ▼
┌─────────────┐
│  COLLECTING │ ◄── Begin next period
└─────────────┘
```

### 6.6 Configuration Constants

| Constant | Value | Description |
|----------|-------|-------------|
| CALC_PERIOD_SAMPLES | 1200 | 60 seconds at 20 Hz |
| CALC_GUST_WINDOW | 60 | 3 seconds at 20 Hz |
| CALC_REPORT_BUFFER_SIZE | 30 | 30 minutes of reports |
| LEVER_ARM_Z | 0.8 | IMU offset below anemometer (m) |

---

## 7. Telemetry Interface (Telus)

### 7.1 Overview

The Telus module provides an external interface for retrieving calculation reports via UART4. An external host can send a command and receive CSV-formatted wind statistics.

### 7.2 Physical Layer

| Parameter | Value |
|-----------|-------|
| UART | UART4 |
| Baud Rate | 115,200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| TX Pin | PA0 |
| RX Pin | PA1 |

### 7.3 Protocol

**Request:**
```
idata\r\n
```

**Response (CSV format):**
```
timestamp,latitude,longitude,u_mean,u_std,v_mean,v_std,w_mean,w_std,wind_speed_mean,wind_speed_std,wind_from_mean,wind_from_std,gust_mean,gust_std\r\n
2025-01-14T12:00:00Z,45.123456,-122.654321,1.234,0.123,2.345,0.234,0.012,0.045,2.678,0.267,225.5,12.3,3.456,0.345\r\n
2025-01-14T12:01:00Z,45.123457,-122.654320,...\r\n
...
```

### 7.4 CSV Field Definitions

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| timestamp | ISO 8601 | - | YYYY-MM-DDTHH:MM:SSZ |
| latitude | float | degrees | 6 decimal places |
| longitude | float | degrees | 6 decimal places |
| u_mean | float | m/s | East-West component mean |
| u_std | float | m/s | East-West component std dev |
| v_mean | float | m/s | North-South component mean |
| v_std | float | m/s | North-South component std dev |
| w_mean | float | m/s | Vertical component mean |
| w_std | float | m/s | Vertical component std dev |
| wind_speed_mean | float | m/s | Horizontal speed mean |
| wind_speed_std | float | m/s | Horizontal speed std dev |
| wind_from_mean | float | degrees | Direction FROM (0-360) |
| wind_from_std | float | degrees | Direction std dev |
| gust_mean | float | m/s | 3-second max mean |
| gust_std | float | m/s | 3-second max std dev |

### 7.5 State Machine

```
┌───────────┐
│   IDLE    │ ◄── Waiting for command
└─────┬─────┘
      │ "idata\r\n" received
      ▼
┌───────────┐
│  SENDING  │ ◄── DMA transfer in progress
└─────┬─────┘
      │ DMA complete interrupt
      ▼
┌───────────┐
│   IDLE    │ ◄── Reports cleared, ready for next
└───────────┘
```

### 7.6 Testing

Use the provided test script:

```bash
# Interactive port selection
python Scripts/test_telus.py

# Specific port
python Scripts/test_telus.py COM6          # Windows
python Scripts/test_telus.py /dev/ttyUSB0  # Linux
```

---

## 8. System Timing

### 8.1 Architecture

The timing system provides microsecond-precision timestamps synchronized to GPS time via the VN-300's 1 PPS output.

```
                     ┌──────────────────┐
     GPS Satellites  │    VN-300        │
          │          │    GPS/IMU       │
          ▼          │                  │
     ┌────────┐      │  ┌────────────┐  │
     │ GNSS   │──────┼─►│ GPS Time   │  │
     └────────┘      │  │ (ns since  │  │
                     │  │  1980)     │  │
                     │  └─────┬──────┘  │
                     │        │         │
                     │  ┌─────▼──────┐  │
                     │  │ 1 PPS Out  │──┼──► TIM3 CH2 (PC7)
                     │  └────────────┘  │        │
                     └──────────────────┘        │
                                                 ▼
┌─────────────────────────────────────────────────────────────┐
│                       STM32L476RG                           │
│                                                             │
│  ┌──────────────┐      ┌──────────────┐                    │
│  │    TIM3      │      │    TIM2      │                    │
│  │ Input Capture│      │ Free-running │                    │
│  │   (1 PPS)    │      │   1 MHz      │                    │
│  └──────┬───────┘      └──────┬───────┘                    │
│         │                     │                            │
│         ▼                     ▼                            │
│  ┌─────────────────────────────────────┐                   │
│  │         systime Module              │                   │
│  │  • g_epoch_sec (seconds since 2000) │                   │
│  │  • g_pps_t2 (TIM2 count at PPS)     │                   │
│  │  • g_tps_est (ticks per second)     │                   │
│  └─────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 Timestamp Components

| Component | Source | Resolution | Range |
|-----------|--------|------------|-------|
| Epoch Seconds | PPS counter | 1 second | 2000-01-01 to 2136 |
| Milliseconds | TIM2 interpolation | ~1 µs | 0-999 |
| GPS Time | VN-300 packet | 1 ns | Full GPS epoch |

### 8.3 PPS Synchronization

```c
void systime_pps_event(void) {
    uint32_t now_t2 = LL_TIM_GetCounter(TIM2);

    // Calculate actual ticks since last PPS
    uint32_t delta = now_t2 - g_pps_t2;

    // Update estimated ticks-per-second with EMA filter
    // (15 × old + 1 × new) / 16
    g_tps_est = (15 * g_tps_est + delta) >> 4;

    // Apply pending time set on PPS edge
    if (g_set_pending) {
        g_epoch_sec = g_set_epoch_sec;
        g_set_pending = false;
    } else {
        g_epoch_sec++;
    }

    g_pps_t2 = now_t2;
    g_pps_count++;
}
```

### 8.4 Snapshot Function

```c
void systime_snapshot(uint32_t *sec, uint16_t *ms) {
    __disable_irq();

    uint32_t s = g_epoch_sec;
    uint32_t t2 = LL_TIM_GetCounter(TIM2);
    uint32_t pps = g_pps_t2;
    uint32_t tps = g_tps_est;

    __enable_irq();

    // Calculate sub-second offset
    uint32_t delta = t2 - pps;
    uint32_t ms_offset = (delta * 1000) / tps;

    // Handle rollover
    if (ms_offset >= 1000) {
        s++;
        ms_offset -= 1000;
    }

    *sec = s;
    *ms = (uint16_t)ms_offset;
}
```

### 8.5 Epoch Definition

| System | Epoch | Relationship |
|--------|-------|--------------|
| Internal (systime) | 2000-01-01 00:00:00 UTC | Base |
| GPS Time | 1980-01-06 00:00:00 UTC | +630720000 seconds |
| Unix | 1970-01-01 00:00:00 UTC | +946684800 seconds |

### 8.6 GPS Time Conversion

```c
// GPS nanoseconds to systime epoch
uint32_t gps_to_systime(uint64_t gps_ns) {
    // GPS to Unix (Jan 6, 1980 → Jan 1, 1970)
    uint64_t gps_sec = gps_ns / 1000000000ULL;
    uint32_t unix_sec = gps_sec + 315964800 - 18;  // Subtract leap seconds

    // Unix to systime (Jan 1, 1970 → Jan 1, 2000)
    uint32_t systime_sec = unix_sec - 946684800;

    return systime_sec;
}
```

---

## 9. Shell Command Reference

### 9.1 Connection Settings

| Parameter | Value |
|-----------|-------|
| Port | USART1 |
| Baud Rate | 115,200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Line Ending | CR/LF |

### 9.2 System Commands

#### `help`
Display list of available commands with descriptions.

#### `clear`
Clear terminal screen (sends ANSI escape sequence).

#### `status`
Display system status including:
- Recording state (active/stopped)
- GPS fix status
- Queue depths (WM/VN)
- Buffer utilization

**Example output:**
```
=== System Status ===
Recording: ACTIVE
GPS Fix: YES (3D)
WM Queue: 2/16 (max: 5)
VN Queue: 3/16 (max: 7)
Records Written: 12,345
```

#### `reset`
Perform software reset of the microcontroller.

#### `version`
Display firmware version information.

#### `snooze <seconds>`
Enter low-power Stop Mode 2 for specified duration.

**Parameters:**
- `seconds`: Sleep duration (1-65535)

**Notes:**
- Cannot be used while recording is active
- All peripherals are disabled during sleep
- System time is restored from RTC on wakeup

#### `systime`
Display current system time in both epoch and ISO 8601 format.

**Example output:**
```
Epoch: 789012345 (since 2000-01-01)
ISO:   2025-01-14T12:30:45Z
PPS:   1234567 pulses received
```

### 9.3 Recording Commands

#### `rec-start`
Start data recording to SD card. Creates new log file with timestamped name.

**Prerequisites:**
- SD card must be mounted
- GPS fix must be acquired

#### `rec-stop`
Stop data recording and close current log file.

#### `rec-stats`
Display recording statistics.

**Example output:**
```
=== Recorder Statistics ===
Recording: YES
Filename: log_789012345.bin
Records Written: 12,345
Active Buffer: 17/32
WM Queue: 2/16 (drops: 0, max: 5)
VN Queue: 3/16 (drops: 0, discards: 42, max: 7)
```

#### `queue-debug`
Display detailed queue timing information for debugging pairing issues.

### 9.4 Filesystem Commands

#### `fs-mount`
Mount SD card filesystem.

#### `fs-unmount`
Unmount SD card filesystem safely.

#### `fs-df`
Display filesystem free space.

**Example output:**
```
Filesystem: FAT32
Total: 7,634 MB
Used:  1,234 MB
Free:  6,400 MB (83%)
```

#### `fs-ls [path]`
List directory contents.

**Parameters:**
- `path`: Directory path (default: root `/`)

**Example:**
```
> fs-ls /logs
drw-  2025-01-14 12:00  .
drw-  2025-01-14 11:00  ..
-rw-  2025-01-14 12:30  log_789012345.bin  (15.2 MB)
-rw-  2025-01-14 11:00  log_789000000.bin  (20.0 MB)
```

#### `fs-cat <file>`
Display file contents to terminal.

**Parameters:**
- `file`: File path to read

#### `fs-write <file> <data>`
Write data to file (creates or overwrites).

**Parameters:**
- `file`: Target file path
- `data`: Data string to write

#### `fs-rm <file>`
Delete a file.

**Parameters:**
- `file`: File path to delete

#### `fs-mkdir <dir>`
Create a directory.

**Parameters:**
- `dir`: Directory path to create

#### `fs-rmdir <dir>`
Remove an empty directory.

**Parameters:**
- `dir`: Directory path to remove

#### `fs-cp <src> <dst>`
Copy a file.

**Parameters:**
- `src`: Source file path
- `dst`: Destination file path

### 9.5 RTC Commands

#### `rtc-time`
Display current RTC date and time.

**Example output:**
```
RTC Time: 2025-01-14 12:30:45
```

#### `rtc-settime <YYYY> <MM> <DD> <HH> <MM> <SS>`
Set RTC date and time.

**Parameters:**
- `YYYY`: Year (2000-2099)
- `MM`: Month (01-12)
- `DD`: Day (01-31)
- `HH`: Hour (00-23)
- `MM`: Minute (00-59)
- `SS`: Second (00-59)

**Example:**
```
> rtc-settime 2025 01 14 12 30 45
RTC set to: 2025-01-14 12:30:45
```

#### `rtc-temp`
Display RTC temperature sensor reading.

**Example output:**
```
RTC Temperature: 23°C
```

#### `rtc-timer-set <seconds>`
Set and start RTC countdown timer.

**Parameters:**
- `seconds`: Timer duration (1-65535)

#### `rtc-timer-stop`
Stop the RTC countdown timer.

#### `rtc-timer-status`
Display RTC timer status and remaining time.

---

## 10. Software Architecture

### 10.1 Module Hierarchy

```
main.c
├── HAL Initialization
├── SystemClock_Config()
├── Peripheral Init (MX_*_Init)
└── Main Loop
    ├── recorder_service()      ──► recorder.c
    │   ├── vn_drain_and_queue() ──► vectornav.c
    │   ├── wm_drain_and_queue() ──► windmaster.c
    │   └── filesystem_*()      ──► filesystem.c
    │
    ├── calc_service()          ──► calculations.c
    │
    ├── telus_service()         ──► telus.c
    │
    ├── shell_task()            ──► shell.c
    │   ├── task_gen.c          (system commands)
    │   ├── task_rec.c          (recording commands)
    │   ├── task_fs.c           (filesystem commands)
    │   └── task_rtc.c          (RTC commands)
    │
    └── tasker_run()            ──► tasker.c
```

### 10.2 Task Queue System

The tasker module provides deferred execution for operations that shouldn't run in interrupt context:

```c
typedef void (*task_fn_t)(int argc, char **argv);

typedef struct {
    task_fn_t fn;                   // Function to execute
    uint8_t arg[TASK_ARG_SIZE];     // Inline argument storage (16 bytes)
} queued_task_t;

#define TASK_QUEUE_LEN  16          // Must be power of 2

// Circular queue
static queued_task_t task_queue[TASK_QUEUE_LEN];
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;

// Enqueue (called from shell parser)
bool tasker_enqueue(task_fn_t fn, const void *arg, size_t arg_size);

// Dequeue and execute (called from main loop)
void tasker_run(void);
```

### 10.3 Interrupt Handlers

| Interrupt | Priority | Handler | Purpose |
|-----------|----------|---------|---------|
| DMA1_CH3 | 5 | `DMA1_Channel3_IRQHandler` | VN-300 RX complete |
| DMA1_CH6 | 5 | `DMA1_Channel6_IRQHandler` | USART2 RX (WindMaster) complete |
| DMA2_CH3 | 5 | `DMA2_Channel3_IRQHandler` | Telus TX complete |
| DMA2_CH5 | 5 | `DMA2_Channel5_IRQHandler` | Telus RX complete |
| TIM3 | 10 | `TIM3_IRQHandler` | PPS input capture |
| USART1 | 6 | `USART1_IRQHandler` | Shell RX |

### 10.4 Initialization Sequence

```c
int main(void) {
    // 1. Core initialization
    HAL_Init();
    SystemClock_Config();           // 80 MHz from HSE+PLL

    // 2. Peripheral initialization (STM32CubeMX generated)
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();                 // RTC
    MX_SPI2_Init();                 // SD Card
    MX_UART4_Init();                // Telus
    MX_FATFS_Init();
    MX_USART2_UART_Init();          // WindMaster
    MX_TIM2_Init();                 // Timestamp
    MX_TIM3_Init();                 // PPS input
    MX_USART1_UART_Init();          // Shell
    MX_USART3_UART_Init();          // VectorNav

    // 3. Start timestamp counter
    LL_TIM_EnableCounter(TIM2);

    // 4. Module initialization
    systime_startup();              // RTC → systime sync
    filesystem_init();
    tasker_init();
    calc_init();
    transceiver_init();             // RS-232 level shifters
    init_uart_interrupts();
    wm_init();                      // WindMaster on USART2 (no output yet)
    vn_init();                      // VectorNav (no output yet)
    telus_init();

    // 5. Wait for GPS fix (blocking)
    while (!vn_gps_fix()) {
        HAL_Delay(5000);
    }

    // 6. Sync time from GPS
    gps_time_sync();                // RTC ← GPS, systime ← RTC

    // 7. Enter main loop
    shell_init();                   // Print prompt
    while (1) {
        recorder_service();
        calc_service();
        telus_service();
        shell_task();
        tasker_run();
    }
}
```

### 10.5 Data Flow

```
                                TIME SYNC
                                    │
VN-300 ──► DMA Buffer ──► vn_drain_and_queue() ──► VN Queue ──┐
 50 Hz      1 KB              │                    16 entries │
                              │                               │
                         calc_add_sample()                    │
                              │                    PAIRING    │
                              ▼                       │       │
                        Accumulators ◄────────────────┤       │
                              │                       │       │
                         calc_service()               ▼       │
                              │                 Recorder_Data_t
                              ▼                       │       │
                        CalcReport_t                  ▼       │
                          30 buf              Active Buffer   │
                              │                 32 records    │
                         telus_service()              │       │
                              │                       ▼       │
                              ▼               Flush Buffer ───┼──► SD Card
                         UART4 TX                             │    FatFS
                              │                               │
                              ▼                               │
                         External Host                        │
                                                              │
WM 1000 ──► DMA Buffer ──► wm_drain_and_queue() ──► WM Queue ─┘
 20 Hz      1 KB                                    16 entries
```

---

## 11. Configuration & Calibration

### 11.1 Lever Arm Configuration

The lever arm offset must be configured based on physical mounting:

```c
// In calculations.c
#define LEVER_ARM_X  0.0f   // X offset (m) - typically 0
#define LEVER_ARM_Y  0.0f   // Y offset (m) - typically 0
#define LEVER_ARM_Z  0.8f   // Z offset (m) - IMU below anemometer
```

**Measurement Procedure:**
1. Measure vertical distance from WindMaster transducer center to VN-300 IMU center
2. Positive Z = IMU below anemometer
3. Update LEVER_ARM_Z constant
4. Rebuild firmware

### 11.2 GPS Leap Seconds

GPS time does not include leap seconds. Update when new leap seconds are added:

```c
// In vectornav.c and parse_log.py
#define GPS_LEAP_SECONDS  18  // Current as of 2017
```

### 11.3 VectorNav Configuration

The VN-300 output configuration is set via register 75:

```c
// Register 75: Async Data Output Configuration
// $VNWRG,75,<async_mode>,<rate_divisor>,<output_group>,<output_fields>*XX

// Current configuration (50 Hz, selected fields)
"$VNWRG,75,1,8,01,01EA*07CF"
//        │ │  │  └──── Fields: TimeGps, YPR, Gyro, Pos, Vel, Accel
//        │ │  └─────── Group 1 (Common)
//        │ └────────── 800 Hz / 8 = 50 Hz
//        └──────────── Async mode enabled
```

### 11.4 WindMaster Configuration

The WindMaster is factory-configured for Mode 10 binary output at 57,600 baud. Configuration changes require entering config mode:

```
*\r              Enter config mode
MODE 10\r        Set binary output mode
BAUD 57600\r     Set baud rate
Q\r              Exit config mode
```

### 11.5 Calculation Period

Modify statistics averaging period:

```c
// In calculations.c
#define CALC_PERIOD_SAMPLES  1200  // 60 seconds at 20 Hz
#define CALC_GUST_WINDOW     60    // 3 seconds at 20 Hz
```

---

## 12. Troubleshooting

### 12.1 System Won't Start

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| No shell output | USART1 not connected | Check TX/RX wiring |
| Stuck on "Getting GPS fix" | VN-300 no satellites | Move to open sky |
| Stuck on "Getting GPS fix" | VN-300 not powered | Check power connections |
| Immediate reset | Stack overflow | Increase stack size |
| Immediate reset | Hard fault | Check memory access |

### 12.2 Recording Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| "SD not mounted" | Card not inserted | Insert SD card |
| "SD not mounted" | Card not formatted | Format as FAT32 |
| High drop count | SD write too slow | Use faster SD card |
| High drop count | Main loop blocked | Check for blocking calls |
| Pairing failures | Sensors not synced | Verify both sensors running |

### 12.3 Sensor Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| VN queue always empty | DMA not started | Call vn_start() |
| VN queue always empty | Wrong baud rate | Verify VN-300 config |
| WM queue always empty | DMA not started | Call wm_start() |
| WM queue always empty | WindMaster in config mode | Power cycle WindMaster |
| Invalid checksums | Electrical noise | Check cable shielding |
| Time offset > 25ms | Clock drift | Check PPS signal |

### 12.4 Telus Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| No response | Wrong baud rate | Verify 115200 8N1 |
| No response | TX/RX swapped | Check pin connections |
| Empty CSV | No reports available | Wait for full period |
| Truncated response | Buffer overflow | Reduce report count |

### 12.5 Debug Commands

```bash
# Check system status
> status

# Check recording statistics
> rec-stats

# Check queue state
> queue-debug

# Check filesystem
> fs-df
> fs-ls /

# Check timing
> systime
```

### 12.6 LED Indicators

| LED | State | Meaning |
|-----|-------|---------|
| LD2 (Green) | Blinking | Normal operation |
| LD2 (Green) | Solid | Recording active |
| LD2 (Green) | Off | Error state |

---

## 13. Appendices

### 13.1 Glossary

| Term | Definition |
|------|------------|
| DMA | Direct Memory Access - hardware data transfer without CPU |
| FatFS | FAT filesystem library for embedded systems |
| GPS | Global Positioning System |
| IMU | Inertial Measurement Unit |
| NED | North-East-Down coordinate frame |
| PPS | Pulse Per Second - timing signal |
| SoS | Speed of Sound |
| YPR | Yaw-Pitch-Roll attitude angles |

### 13.2 Physical Constants

| Constant | Value | Description |
|----------|-------|-------------|
| DEG_TO_RAD | 0.01745329252 | π/180 |
| RAD_TO_DEG | 57.29577951 | 180/π |
| MM_TO_M | 0.001 | Millimeters to meters |

### 13.3 Epoch Reference Dates

| Epoch | Date | Unix Timestamp |
|-------|------|----------------|
| Unix | 1970-01-01 00:00:00 UTC | 0 |
| GPS | 1980-01-06 00:00:00 UTC | 315964800 |
| systime | 2000-01-01 00:00:00 UTC | 946684800 |

### 13.4 Related Documents

- VectorNav VN-300 User Manual
- Gill WindMaster User Manual
- STM32L476RG Reference Manual (RM0351)
- STM32L476RG Datasheet
- FatFS Application Note

### 13.5 Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-21 | Initial release |

---

*End of Document*
