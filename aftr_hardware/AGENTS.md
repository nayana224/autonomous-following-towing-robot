# aftr_hardware Instructions

## Scope

- This package owns the MD motor driver serial protocol and the `ros2_control` hardware interface.
- Treat changes here as hardware-sensitive.

## Reading Order

1. `docs/hardware_analysis.md`
2. `docs/md_packet_protocol.md`
3. `src/md_packet.cpp`
4. `src/md_driver.cpp`
5. `src/md_hardware_interface.cpp`

## Rules

- Keep packet structure, checksum behavior, byte order, and PID meanings explicit.
- Validate parameters and joint assumptions before using them.
- Do not change RPM conversion, timeout, or deactivate/release behavior without explaining hardware risk.
- Check packet parsing and lifecycle changes against the protocol notes; validate builds without actuating motors before hardware tests.

