# Ableton Link Audio ESP32-S3-BOX-3 Example

This example demonstrates how to use Ableton Link with audio sharing support on the ESP32-S3-BOX-3.

## Features

- **Link Synchronization**: Synchronize tempo, beat, and phase with other Link peers over Wi-Fi.
- **Link Audio**:
  - **Sending**: Captures local microphone audio (ES7210) and shares it with the Link session.
  - **Receiving**: Automatically discovers and subscribes to remote Link Audio channels, playing them back through the speaker (ES8311).
- **Local Monitoring**: Hear a synchronized metronome mixed with the remote audio stream.
- **Low Latency**: Optimized networking and task priorities for stable real-time performance.
- **Hardware Integration**: Fully utilizes the `esp-box-3` Board Support Package (BSP) and `esp_codec_dev` drivers.

## Hardware Requirements

- **ESP32-S3-BOX-3**: The code is specifically tuned for the BOX-3 pinout and codecs.
- **Wi-Fi Connection**: A stable 2.4GHz or 5GHz Wi-Fi network.

## Pinout (ESP32-S3-BOX-3)

- **I2S**: MCLK (GPIO 2), BCK (GPIO 17), WS (GPIO 45), DO (GPIO 15), DI (GPIO 16)
- **I2C**: SCL (GPIO 18), SDA (GPIO 8)
- **PA Control**: GPIO 46 (Speaker Amplifier Enable)

## Building and Flashing

1.  **Configure the Project**:
    Set your Wi-Fi credentials in the configuration:
    ```bash
    idf.py menuconfig
    ```
    Navigate to `Example Connection Configuration` and enter your SSID and Password.

2.  **Build and Flash**:
    ```bash
    idf.py build flash monitor
    ```

## Usage

- Upon boot, the device will play a 2-second test beep to verify audio output.
- Once connected to Wi-Fi, it will join the Link session.
- The device automatically subscribes to the first available audio channel it finds.
- Local status (Peers, Tempo, Latency) is printed to the serial console.
- **Metronome**: Only heard locally (mixed with remote audio).
- **Microphone**: Raw input is sent to other peers but not monitored locally to prevent feedback.

## Implementation Details

- **Core Affinity**: Link networking runs on **Core 0**, while Audio processing is dedicated to **Core 1**.
- **Jitter Buffer**: A 256-slot queue in PSRAM provides a stable jitter buffer for received audio.
- **Optimization**: WiFi Power Save is disabled (`WIFI_PS_NONE`) to minimize network jitter.
