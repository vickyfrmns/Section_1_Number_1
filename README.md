# STM32F4 ADC and UART with Timer Interrupt

This project demonstrates how to interface an ADC (Analog to Digital Converter) with UART communication on an STM32F4 microcontroller. It utilizes a timer interrupt to periodically read ADC values and send the data over UART. It also supports UART data reception through interrupts.

## Features

- ADC reading from pin `PA1` (ADC Channel 1).
- UART communication over pins `PA2` (TX) and `PA3` (RX) at 9600 baud.
- Timer interrupt (TIM3) triggers every 500ms to:
  - Read the ADC value.
  - Transmit the ADC value over UART.
  - Receive data from UART using interrupts.
- Sleep mode is used to minimize power consumption when the system is idle.

## Hardware Requirements

- **STM32F4 Discovery board** or any STM32F4 series microcontroller.
- **ADC Pin**: PA1 (Analog input).
- **UART Pins**: PA2 (TX), PA3 (RX).

## Software Requirements

- Platform IO on Visual Studio Code

## Code Walkthrough

### Initialization Functions

1. **SystemClock_Config**: Configures the system clock using HSE and PLL to achieve a high-speed clock.
2. **GPIO_Init**: Configures GPIO pins for UART (TX and RX) and ADC (PA1 for analog input).
3. **UART2_Init**: Initializes the UART2 peripheral with 9600 baud rate, 8-bit data, and 1 stop bit.
4. **ADC1_Init**: Configures the ADC1 peripheral for single conversion mode on pin PA1 (ADC Channel 1).
5. **TIM3_Init**: Initializes Timer 3 with a prescaler of 8400, providing a 500Hz timer frequency (every 500ms).

### Interrupts

- **HAL_TIM_PeriodElapsedCallback**: This callback is invoked when the timer overflows every 500ms. It reads the ADC value and sends it via UART. It also triggers UART reception to receive incoming data.
- **UART_Receive_Data**: Receives UART data asynchronously via interrupts and prints the received data.

### Main Loop

The main loop utilizes the `__WFI()` instruction to put the MCU into sleep mode until an interrupt occurs. This helps reduce power consumption while the system is idle.

## How to Use

1. **Configure the project**:
   - Clone the example from this repository using :
   ```bash
    git clone --url_https/sshkey
    ```
   - Open on Microsoft Studio Code using Platform.IO

2. **Build the project**:
   - Compile and flash the firmware onto the STM32F4 board using PlatformIO.

## Example Output

The output printed on the terminal will look like this:
The ADC value will be updated every 500ms, and any data received over UART will be printed with the prefix `Data diterima:`.
