This a collaborative project of Utkarsh Chaurasia and Baibhav Giri.

This project implements a complete UART Transmitter and Receiver using Verilog and verifies their operation through a loopback-based testbench in Vivado. The design consists of an FSM-based UART transmitter, an FSM-based UART receiver, and a wrapper module that integrates both using a common baud-rate timing (CLKS_PER_BIT = 64) to ensure reliable communication. A self-checking testbench generates clocks and reset, waits for the transmitter to become idle before sending each byte, and feeds the transmitted serial output back into the receiver input. Behavioral simulation confirms correct UART framing (1 start bit, 8 data bits LSB first, 1 stop bit) and shows that the received data exactly matches the transmitted data (for example, TX = 0x6F and RX = 0x6F). This validates proper TX–RX synchronization, correct sampling in the receiver, and overall functional correctness of the UART design.


<img width="2048" height="1202" alt="image" src="https://github.com/user-attachments/assets/1faa4b50-bb41-4d66-abb4-ad945daa086f" />
