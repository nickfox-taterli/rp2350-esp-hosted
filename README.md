# RP2350 with ESP Hosted Communication

This project runs on an MCU using the FreeRTOS operating system and communicates with an ESP32 module via SPI using the ESP Hosted protocol. The communication is based on RPC (Remote Procedure Call) with data packed in protobuf format. The project includes tasks for SPI communication, RPC handling, Wi-Fi event processing, and TCP/IP networking using LWIP.

## Project Structure

```
├─Core                 # MCU-related code and headers
├─Driver
│  └─ethernetif       # LWIP porting layer
└─Middlewares
    ├─esp-hosted
    │  ├─common
    │  │  └─protobuf-c # Protobuf runtime library
    │  ├─drivers
    │  │  ├─mempool    # Memory pool for Wi-Fi communication (platform-independent)
    │  │  ├─rpc        # RPC communication (platform-independent)
    │  │  ├─transport  # Low-level driver (needs porting)
    │  ├─host
    │  │  ├─api        # Host API interface
    │  └─proto         # Protobuf protocol definitions from the original ESP Hosted firmware
    ├─FreeRTOS-Kernel  # FreeRTOS kernel
    ├─LwIP             # LWIP network stack
    └─protobuf-c       # Protobuf runtime library headers (not included in ESP Hosted library)
```

## Tasks Overview

| Task Name      | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| `spi_rx`       | Handles SPI data reception. Triggered by a semaphore when data is available.|
| `spi_tx`       | Pushes data to the SPI transmit queue when data needs to be sent.           |
| `rpc_tx`       | Sends control signals (e.g., Wi-Fi connection) via RPC.                     |
| `rpc_rx`       | Processes control data received from SPI and handles RPC-related tasks.     |
| `wifi_event`   | Handles asynchronous Wi-Fi events (e.g., scan completion).                 |
| `wifi_thread`  | Main thread controlling the network flow.                                   |
| `tcpip_thread` | LWIP main thread for TCP/IP networking.                                     |

## SPI Communication

The SPI communication between RP2350 and ESP Hosted is based on RPC with data packed in protobuf format. The SPI data packets include the following types:

- **STA**: Wireless station data.
- **AP**: Wireless access point data.
- **PRIV**: Control information.
- **VHCI**: Virtual HCI for Bluetooth.
- **TEST**: Test channel.

## GPIO Pin Configuration

The default GPIO pin configuration for RP2350 is defined in `transport.c`. The ESP Hosted slave-side configuration can be defined in the ESP-IDF `menuconfig`.

| Signal        | RP2350 Pin |
|---------------|------------|
| CLK           | 2          |
| MOSI          | 3          |
| MISO          | 4          |
| CS            | 8          |
| Handshake     | 6          |
| Data Ready    | 5          |
| Reset Out     | 7          |

## Example Application

The example application runs an `iperf2` server, which is built into LWIP. Testing with an ESP32-C3 at 50MHz shows stable communication with a throughput of approximately 4.3Mbps. Other protocols may yield higher performance, but they have not been tested.

## Build Information

This project is built using the PICO SDK for RP2350.

## Notes

- The project may contain undiscovered bugs as it has not been thoroughly tested. It is intended for educational purposes and experimentation.
- Logging can be configured by modifying `esp_wrapper.h`. Uncomment or redefine macros as needed.

## References

- ESP Hosted Project: [GitHub Repository](https://github.com/espressif/esp-hosted/tree/feature/esp_as_mcu_host)
- Pico SDK: [GitHub Repository](https://github.com/raspberrypi/pico-sdk)