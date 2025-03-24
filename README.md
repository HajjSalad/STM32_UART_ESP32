### STM32-ESP32 UART
This project established connection between STM32 and ESP32 boards. 

### Version v1.0 - Timeout Handshake Mechanism 
This version implements a timeout-based handshake to coordinate data transfer between the STM32 and ESP32:
1. ESP32 initiates communication by sending a "ready" message.
2. STM32 responds by transmitting data upon receiving the ready signal.
3. ESP32 acknowledges receipt of the data by sending an acknowledgment signal.
4. STM32 finalizes the handshake, completing the transmission cycle.

Limitations:
- Timing synchronization is required to align data transmission and reception correctly.
- Polling-based approach increases computational overhead due to constant status checks.

### Version v1.0 - Interrupt Driven Handshake 
Interrupt-driven handshake mechanism coming soon...

#### Version 1.0 Demo
![Demo](./uartVid.gif) 
