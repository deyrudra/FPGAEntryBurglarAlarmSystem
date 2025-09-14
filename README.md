# FPGA Entry and Burglar Alarm System

An FPGA-based entry control and burglar alarm that pairs HDL logic with a microcontroller sidecar. The design targets a simple access workflow: read an RFID token, authorize, then arm or disarm zones and drive alarms.

## Highlights

- HDL design for the core security logic on FPGA  
- Microcontroller firmware for RFID via SPI and host control  
- Clean separation of concerns: FPGA handles timing-critical logic, MCU handles I/O and protocol glue  
- Verilog and C++ codebases in one repo  

## Repository layout

```
fpga/
  security_system/        # Verilog sources and project files
microcontroller/
  RFID_SPI_FPGA_ARDUINO/  # MCU firmware for RFID over SPI
.gitattributes
```

The top-level folders and file types are visible in the repo browser. Languages breakdown currently shows Verilog and C++.

## How it works

1. The microcontroller reads an RFID tag over SPI, performs simple checks, then exchanges control signals with the FPGA.  
2. The FPGA implements the security state machine: idle, verify, arm, alarm. It debounces inputs, times entry delays, and drives outputs to buzzers, LEDs, or relays.  
3. Authorized tokens transition the system to disarmed. Invalid tokens or sensor triggers escalate to alarm.  

## Hardware

- FPGA dev board that supports your chosen toolchain  
- RFID reader with SPI interface  
- Microcontroller board compatible with the firmware in `microcontroller/`  
- Basic peripherals: piezo buzzer or siren driver, LEDs, door sensor or PIR  

## Getting started

### FPGA side

1. Open the project under `fpga/security_system/` in your vendor tools.  
2. Synthesize, place and route, then program your dev board.  
3. Wire I/O:  
   - Inputs: sensor lines, MCU handshakes, reset  
   - Outputs: alarm driver, status LEDs  

### Microcontroller side

1. Open `microcontroller/RFID_SPI_FPGA_ARDUINO/` in your IDE.  
2. Adjust SPI pins and defines for your board and RFID module.  
3. Build and flash the firmware, then connect SPI and control lines to the FPGA headers.  

## Operation

- Present an RFID tag to authenticate.  
- Use a designated input to arm the system.  
- On entry, the FPGA runs a grace timer. Failure to authenticate within the window raises the alarm.  
- Visual and audible indicators reflect current state.  

## Testing tips

- Unit-test the RFID read path on the MCU with a serial console before wiring to the FPGA.  
- Simulate the FPGA state machine with test benches that toggle sensor and token-valid signals.  
- Add a mock token-valid switch to validate timing without a reader attached.  

## Branches

The repository currently has five branches:

- **main** — the primary working branch  
- **gate_level_sim** — for gate-level Verilog simulations  
- **dev** — for new features under development  
- **fpga_tests** — dedicated to FPGA test benches and verification  
- **arduino_driver_update** — updates to the microcontroller / Arduino driver side  

## Roadmap ideas

- Event logging over UART or USB CDC  
- Multi-zone sensors and entry delays  
- Configurable token list stored on MCU flash or external EEPROM  
- Keypad fallback and temporary PINs  
- Simple desktop utility to manage authorized tokens  
