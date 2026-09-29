# md_packet_protocol

## 1. Packet Structure
[RMID][TMID][ID][PID][LEN][DATA][CHECKSUM]

## 2. Header
- transmit: 183 (MD400T), 184 (PC)
- receive: 184 (PC), 183 (MD400T)

## 3. Main PID Values
- 207: velocity command
- 210: status response

## 4. Checksum
Use the low 1 byte of the sum of all bytes.

## 5. Characteristics
- request + response structure
- velocity command and feedback request handled together
