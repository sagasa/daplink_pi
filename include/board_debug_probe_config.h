/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef BOARD_DEBUG_PROBE_H_
#define BOARD_DEBUG_PROBE_H_

#define PROBE_IO_SWDI
#define PROBE_CDC_UART
// reset pin (target nRESET)
#define PROBE_PIN_RESET 10

// PIO config
#define PROBE_SM 0
#define PROBE_PIN_OFFSET 0
#define PROBE_PIN_SWDI (PROBE_PIN_OFFSET + 0)   // GPIO0
#define PROBE_PIN_SWDIO (PROBE_PIN_OFFSET + 1)  // GPIO1
#define PROBE_PIN_SWCLK 2                       // GPIO2

// UART config
#define PROBE_UART_TX 12   // MAIN UART TX GPIO12
#define PROBE_UART_RX 13   // MAIN UART RX GPIO13
#define PROBE_UART_INTERFACE uart1
#define PROBE_UART_BAUDRATE 115200

// UART2 config (second bridge)
#define PROBE_UART2_TX 8   // SUB UART TX GPIO8
#define PROBE_UART2_RX 9   // SUB UART RX GPIO9
#define PROBE_UART2_INTERFACE uart0
#define PROBE_UART2_BAUDRATE 115200

#define PROBE_USB_CONNECTED_LED 25
// DAP LEDs omitted on this board
// #define PROBE_DAP_CONNECTED_LED 19
// #define PROBE_DAP_RUNNING_LED 18
// LED pins (swap Main/Sub assignments)
#define PROBE_UART_TX_LED 5    // MAIN_TX_LED GPIO5
#define PROBE_UART_RX_LED 6    // MAIN_RX_LED GPIO6

// LEDs for second UART bridge (avoid conflicts)
// SUB_TX_LED omitted to free GPIO7 for analog switch select
#define PROBE_UART2_TX_LED 3   // SUB_TX_LED GPIO3
#define PROBE_UART2_RX_LED 4   // SUB_RX_LED GPIO4

#define PROBE_PRODUCT_STRING "Debug Probe FS (CMSIS-DAP)"

// Target select switch & status LED
// GPIO27: slide switch to GND when Main selected (active low)
// GPIO3 : status LED (on when Main selected)
#define PROBE_TARGET_SELECT_PIN 27
#define PROBE_TARGET_STATUS_LED 26
// Keep a temporary test LED within unused 16-25 range
#define PROBE_TARGET_TEST_LED 24

// Main UART DTR output (active low for Arduino auto-reset)
#define PROBE_UART_DTR 23

// Treat this CDC interface index as "Main" (0 or 1)
// In your current enumeration, Sub appears as index 0, so set to 1.
#define PROBE_MAIN_CDC_INDEX 1

// Analog switch control for SWD/NRESET selection
// GPIO11: shutdown (active low to enable), drive Low permanently
// GPIO7 : select signal, mirrors status LED polarity
#define PROBE_ANALOG_SW_SHDN 11
#define PROBE_ANALOG_SW_SEL 7
#endif
