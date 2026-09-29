# aftr_hardware Flow

## 1. Overall Flow
Controller → HardwareInterface → Driver → Packet → Serial → Motor

## 2. Command Flow
cmd_vel → write() → RPM → setVelocity() → packet → serial

## 3. Feedback Flow
serial → parse → readData() → read() → state update

## 4. Notes
- request-response structure
- `write()` and `read()` are executed periodically
