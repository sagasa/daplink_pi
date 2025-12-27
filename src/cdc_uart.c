/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
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

#include <pico/stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"

#include "probe_config.h"
#include "probe.h"

TaskHandle_t uart_taskhandle;
TickType_t last_wake, interval = 100;

// Per-port state
typedef struct {
  volatile TickType_t break_expiry;
  volatile bool timed_break;
  int was_connected;
  uint cdc_tx_oe;
#ifdef PROBE_UART_TX_LED
  uint tx_led_debounce;
#endif
#ifdef PROBE_UART_RX_LED
  uint rx_led_debounce;
#endif
} cdc_port_state_t;

#define CDC_PORTS CFG_TUD_CDC
static cdc_port_state_t ports[CDC_PORTS];

/* Max 1 FIFO worth of data per port */
static uint8_t tx_buf[CDC_PORTS][32];
static uint8_t rx_buf[CDC_PORTS][32];
// Actually s^-1 so 25ms
#define DEBOUNCE_MS 40
static uint debounce_ticks = 5;

#ifdef PROBE_UART_DTR
// DTR pulse generation state
static volatile bool main_dtr_state;
static volatile bool main_dtr_prev;
static volatile bool dtr_pulse_active;
static volatile TickType_t dtr_pulse_expiry;
static volatile TickType_t dtr_last_accept_tick;
static TaskHandle_t dtr_taskhandle;
static volatile bool dtr_state[CDC_PORTS];
static volatile bool dtr_prev[CDC_PORTS];
static volatile bool reset_pulse_active;
static volatile TickType_t reset_pulse_expiry;
#endif

// Map config to arrays for simplicity
static uart_inst_t *uart_if[CDC_PORTS] = {
  PROBE_UART_INTERFACE,
#if CFG_TUD_CDC > 1
  PROBE_UART2_INTERFACE,
#endif
};

static const uint uart_tx_pin[CDC_PORTS] = {
  PROBE_UART_TX,
#if CFG_TUD_CDC > 1
  PROBE_UART2_TX,
#endif
};

static const uint uart_rx_pin[CDC_PORTS] = {
  PROBE_UART_RX,
#if CFG_TUD_CDC > 1
  PROBE_UART2_RX,
#endif
};

static const uint uart_baud[CDC_PORTS] = {
  PROBE_UART_BAUDRATE,
#if CFG_TUD_CDC > 1
  PROBE_UART2_BAUDRATE,
#endif
};

#ifdef PROBE_UART_TX_LED
static const uint uart_tx_led[CDC_PORTS] = {
  PROBE_UART_TX_LED,
#if CFG_TUD_CDC > 1
#ifdef PROBE_UART2_TX_LED
  PROBE_UART2_TX_LED,
#else
  (uint)-1,
#endif
#endif
};
#endif

#ifdef PROBE_UART_RX_LED
static const uint uart_rx_led[CDC_PORTS] = {
  PROBE_UART_RX_LED,
#if CFG_TUD_CDC > 1
  PROBE_UART2_RX_LED,
#endif
};
#endif

void cdc_uart_init(void) {
  for (int i = 0; i < CDC_PORTS; i++) {
    gpio_set_function(uart_tx_pin[i], GPIO_FUNC_UART);
    gpio_set_function(uart_rx_pin[i], GPIO_FUNC_UART);
    gpio_set_pulls(uart_tx_pin[i], 1, 0);
    gpio_set_pulls(uart_rx_pin[i], 1, 0);
    uart_init(uart_if[i], uart_baud[i]);
#ifdef PROBE_UART_TX_LED
    ports[i].tx_led_debounce = 0;
    if (uart_tx_led[i] != (uint)-1) {
      gpio_init(uart_tx_led[i]);
      gpio_set_dir(uart_tx_led[i], GPIO_OUT);
    }
#endif
#ifdef PROBE_UART_RX_LED
    ports[i].rx_led_debounce = 0;
    gpio_init(uart_rx_led[i]);
    gpio_set_dir(uart_rx_led[i], GPIO_OUT);
#endif
    ports[i].was_connected = 0;
    ports[i].cdc_tx_oe = 0;
    ports[i].timed_break = false;
    ports[i].break_expiry = 0;
  }

// Flow control config applies to first port only unless duplicated
#ifdef PROBE_UART_HWFC
    /* HWFC implies that hardware flow control is implemented and the
     * UART operates in "full-duplex" mode (See USB CDC PSTN120 6.3.12).
     * Default to pulling in the active direction, so an unconnected CTS
     * behaves the same as if CTS were not enabled. */
    gpio_set_pulls(PROBE_UART_CTS, 0, 1);
    gpio_set_function(PROBE_UART_RTS, GPIO_FUNC_UART);
    gpio_set_function(PROBE_UART_CTS, GPIO_FUNC_UART);
    uart_set_hw_flow(uart_if[0], true, true);
#else
#ifdef PROBE_UART_RTS
    gpio_init(PROBE_UART_RTS);
    gpio_set_dir(PROBE_UART_RTS, GPIO_OUT);
    gpio_put(PROBE_UART_RTS, 1);
#endif
#endif

#ifdef PROBE_UART_DTR
  gpio_init(PROBE_UART_DTR);
  gpio_set_dir(PROBE_UART_DTR, GPIO_OUT);
  gpio_put(PROBE_UART_DTR, 0);
#endif
}

bool cdc_task(void)
{
    bool keep_alive_any = false;

    for (int i = 0; i < CDC_PORTS; i++) {
        uint rx_len = 0;
        // Drain UART RX FIFO even if not connected
        while (uart_is_readable(uart_if[i]) && (rx_len < sizeof(rx_buf[i]))) {
            rx_buf[i][rx_len++] = uart_getc(uart_if[i]);
        }

        if (tud_cdc_n_connected(i)) {
            ports[i].was_connected = 1;
            int written = 0;
            if (rx_len) {
#ifdef PROBE_UART_RX_LED
                gpio_put(uart_rx_led[i], 1);
                ports[i].rx_led_debounce = debounce_ticks;
#endif
                written = MIN(tud_cdc_n_write_available(i), rx_len);
                if (rx_len > (uint)written) ports[i].cdc_tx_oe++;
                if (written > 0) {
                    tud_cdc_n_write(i, rx_buf[i], written);
                    tud_cdc_n_write_flush(i);
                }
            } else {
#ifdef PROBE_UART_RX_LED
                if (ports[i].rx_led_debounce)
                    ports[i].rx_led_debounce--;
                else
                    gpio_put(uart_rx_led[i], 0);
#endif
            }

            size_t watermark = MIN(tud_cdc_n_available(i), sizeof(tx_buf[i]));
            if (watermark > 0) {
                size_t tx_len;
#ifdef PROBE_UART_TX_LED
                if (uart_tx_led[i] != (uint)-1) {
                  gpio_put(uart_tx_led[i], 1);
                  ports[i].tx_led_debounce = debounce_ticks;
                }
#endif
                watermark = MIN(watermark, 16);
                tx_len = tud_cdc_n_read(i, tx_buf[i], watermark);
                uart_write_blocking(uart_if[i], tx_buf[i], tx_len);
            } else {
#ifdef PROBE_UART_TX_LED
        if (uart_tx_led[i] != (uint)-1) {
          if (ports[i].tx_led_debounce)
            ports[i].tx_led_debounce--;
          else
            gpio_put(uart_tx_led[i], 0);
        }
#endif
            }

            if (ports[i].timed_break) {
                if (((int)ports[i].break_expiry - (int)xTaskGetTickCount()) < 0) {
                    ports[i].timed_break = false;
                    uart_set_break(uart_if[i], false);
#ifdef PROBE_UART_TX_LED
                    ports[i].tx_led_debounce = 0;
#endif
                } else {
                    keep_alive_any = true;
                }
            }
        } else if (ports[i].was_connected) {
            tud_cdc_n_write_clear(i);
            uart_set_break(uart_if[i], false);
            ports[i].timed_break = false;
            ports[i].was_connected = 0;
#ifdef PROBE_UART_TX_LED
            ports[i].tx_led_debounce = 0;
#endif
            ports[i].cdc_tx_oe = 0;
        }
    }
    return keep_alive_any;
}

void cdc_thread(void *ptr)
{
  BaseType_t delayed;
  last_wake = xTaskGetTickCount();
  bool keep_alive;
  /* Threaded with a polling interval that scales according to linerate */
  while (1) {
    keep_alive = cdc_task();
    if (!keep_alive) {
      delayed = xTaskDelayUntil(&last_wake, interval);
        if (delayed == pdFALSE)
          last_wake = xTaskGetTickCount();
    }
  }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* line_coding)
{
  uart_parity_t parity;
  uint data_bits, stop_bits;
  /* Set the tick thread interval to the amount of time it takes to
   * fill up half a FIFO. Millis is too coarse for integer divide.
   */
  uint32_t micros = (1000 * 1000 * 16 * 10) / MAX(line_coding->bit_rate, 1);
  /* Adjust polling interval; keep minimal changes */
  interval = MAX(1, micros / ((1000 * 1000) / configTICK_RATE_HZ));
  debounce_ticks = MAX(1, configTICK_RATE_HZ / (interval * DEBOUNCE_MS));
  probe_info("New baud rate %ld micros %ld interval %lu\n",
                  line_coding->bit_rate, micros, interval);
  uart_deinit(uart_if[itf]);
  tud_cdc_n_write_clear(itf);
  tud_cdc_n_read_flush(itf);
  uart_init(uart_if[itf], line_coding->bit_rate);

  switch (line_coding->parity) {
  case CDC_LINE_CODING_PARITY_ODD:
    parity = UART_PARITY_ODD;
    break;
  case CDC_LINE_CODING_PARITY_EVEN:
    parity = UART_PARITY_EVEN;
    break;
  default:
    probe_info("invalid parity setting %u\n", line_coding->parity);
    /* fallthrough */
  case CDC_LINE_CODING_PARITY_NONE:
    parity = UART_PARITY_NONE;
    break;
  }

  switch (line_coding->data_bits) {
  case 5:
  case 6:
  case 7:
  case 8:
    data_bits = line_coding->data_bits;
    break;
  default:
    probe_info("invalid data bits setting: %u\n", line_coding->data_bits);
    data_bits = 8;
    break;
  }

  /* The PL011 only supports 1 or 2 stop bits. 1.5 stop bits is translated to 2,
   * which is safer than the alternative. */
  switch (line_coding->stop_bits) {
  case CDC_LINE_CONDING_STOP_BITS_1_5:
  case CDC_LINE_CONDING_STOP_BITS_2:
    stop_bits = 2;
  break;
  default:
    probe_info("invalid stop bits setting: %u\n", line_coding->stop_bits);
    /* fallthrough */
  case CDC_LINE_CONDING_STOP_BITS_1:
    stop_bits = 1;
  break;
  }

  uart_set_format(uart_if[itf], data_bits, stop_bits, parity);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  // Apply hardware handshake lines only for Main CDC interface
#ifndef PROBE_MAIN_CDC_INDEX
#define PROBE_MAIN_CDC_INDEX 0
#endif
#ifdef PROBE_UART_RTS
  if (itf == PROBE_MAIN_CDC_INDEX) gpio_put(PROBE_UART_RTS, !rts);
#endif
#ifdef PROBE_UART_DTR
  // Record DTR state; pulse generation is handled in a separate task
  dtr_state[itf] = dtr;
  if (itf == PROBE_MAIN_CDC_INDEX) {
    main_dtr_state = dtr;
  }
#endif

  /* CDC drivers use linestate as a bodge to activate/deactivate the interface.
   * Light LEDs accordingly; do not suspend global thread to keep other ports alive */
#ifdef PROBE_UART_RX_LED
  if (!dtr) {
    gpio_put(uart_rx_led[itf], 0);
    ports[itf].rx_led_debounce = 0;
  }
#endif
#ifdef PROBE_UART_TX_LED
  if (!dtr) {
    if (uart_tx_led[itf] != (uint)-1) {
      gpio_put(uart_tx_led[itf], 0);
      ports[itf].tx_led_debounce = 0;
    }
  }
#endif
}

#ifdef PROBE_UART_DTR
static void dtr_pulse_thread(void *ptr)
{
  TickType_t wake = xTaskGetTickCount();
  // Ensure idle low
  gpio_put(PROBE_UART_DTR, 0);
  while (1) {
    TickType_t now = xTaskGetTickCount();
    // Determine selected interface from target select switch
#ifdef PROBE_TARGET_SELECT_PIN
    bool main_selected = (gpio_get(PROBE_TARGET_SELECT_PIN) != 0);
#else
    bool main_selected = true;
#endif
    // Gate behavior by select state
    bool fell_main = (dtr_prev[PROBE_MAIN_CDC_INDEX] == true) && (dtr_state[PROBE_MAIN_CDC_INDEX] == false);
    bool fell_sub  = (dtr_prev[PROBE_MAIN_CDC_INDEX ^ 1] == true) && (dtr_state[PROBE_MAIN_CDC_INDEX ^ 1] == false);
    if (main_selected) {
      // LED pulse + reset assert on main DTR falling
      if (!dtr_pulse_active && fell_main) {
        if ((int)(now - dtr_last_accept_tick) >= (int)pdMS_TO_TICKS(200)) {
          gpio_put(PROBE_UART_DTR, 1);
          probe_assert_reset(0);
          dtr_pulse_active = true;
          reset_pulse_active = true;
          dtr_pulse_expiry = now + pdMS_TO_TICKS(10);
          reset_pulse_expiry = dtr_pulse_expiry;
          dtr_last_accept_tick = now;
        }
      }
    } else {
      // Only reset assert on sub DTR falling; keep LED idle low
      if (!reset_pulse_active && fell_sub) {
        if ((int)(now - dtr_last_accept_tick) >= (int)pdMS_TO_TICKS(200)) {
          probe_assert_reset(0);
          reset_pulse_active = true;
          reset_pulse_expiry = now + pdMS_TO_TICKS(10);
          dtr_last_accept_tick = now;
        }
      }
    }
    // Handle expiries
    if (dtr_pulse_active && ((int)(now - dtr_pulse_expiry) >= 0)) {
      gpio_put(PROBE_UART_DTR, 0);
      dtr_pulse_active = false;
    }
    if (reset_pulse_active && ((int)(now - reset_pulse_expiry) >= 0)) {
      probe_assert_reset(1);
      reset_pulse_active = false;
    }
    // Update previous states
    for (int i = 0; i < CDC_PORTS; i++) dtr_prev[i] = dtr_state[i];
    main_dtr_prev = main_dtr_state;
    xTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}

void cdc_start_dtr_pulse_task(void)
{
  // Reset state
  main_dtr_state = false;
  main_dtr_prev = false;
  dtr_pulse_active = false;
  reset_pulse_active = false;
  dtr_last_accept_tick = 0;
  for (int i = 0; i < CDC_PORTS; i++) { dtr_state[i] = false; dtr_prev[i] = false; }
  UBaseType_t prio = (uart_taskhandle ? uxTaskPriorityGet(uart_taskhandle) : (tskIDLE_PRIORITY + 3));
  xTaskCreate(dtr_pulse_thread, "DTR", configMINIMAL_STACK_SIZE, NULL, prio, &dtr_taskhandle);
}
#endif

void tud_cdc_send_break_cb(uint8_t itf, uint16_t wValue) {
  switch(wValue) {
    case 0:
  uart_set_break(uart_if[itf], false);
  ports[itf].timed_break = false;
#ifdef PROBE_UART_TX_LED
  ports[itf].tx_led_debounce = 0;
#endif
    break;
    case 0xffff:
  uart_set_break(uart_if[itf], true);
  ports[itf].timed_break = false;
#ifdef PROBE_UART_TX_LED
  if (uart_tx_led[itf] != (uint)-1) {
    gpio_put(uart_tx_led[itf], 1);
    ports[itf].tx_led_debounce = 1 << 30;
  }
#endif
    break;
    default:
  uart_set_break(uart_if[itf], true);
  ports[itf].timed_break = true;
#ifdef PROBE_UART_TX_LED
  if (uart_tx_led[itf] != (uint)-1) {
    gpio_put(uart_tx_led[itf], 1);
    ports[itf].tx_led_debounce = 1 << 30;
  }
#endif
  ports[itf].break_expiry = xTaskGetTickCount() + (wValue * (configTICK_RATE_HZ / 1000));
    break;
  }
}
