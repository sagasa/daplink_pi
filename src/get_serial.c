/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Federico Zuccardi Merli
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

#include <stdint.h>
#include "pico.h"
#include "pico/unique_id.h"
#include "get_serial.h"

#include "hardware/flash.h"
#include "hardware/sync.h"

#include "hardware/adc.h"
#include "hardware/timer.h"
#include "pico/time.h"

/* C string for iSerialNumber in USB Device Descriptor, two chars per byte + terminating NUL */
char usb_serial[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

/* Why a uint8_t[8] array inside a struct instead of an uint64_t an inquiring mind might wonder */
static pico_unique_board_id_t uID;

#ifndef PICO_FLASH_SIZE_BYTES
#warning "PICO_FLASH_SIZE_BYTES が未定義です。ボード設定を確認してください。"
#endif
#define SERIAL_SECTOR_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define SERIAL_XIP_ADDR        (XIP_BASE + SERIAL_SECTOR_OFFSET)
#define SERIAL_MAGIC           (0x534E4F31u) /* 'SNO1' */

typedef struct {
    uint32_t magic;
    uint8_t  id[8];
    uint32_t reserved; // アラインメント用
} stored_serial_t;

static bool read_stored_serial(uint8_t out[8]) {
    const stored_serial_t *s = (const stored_serial_t *)SERIAL_XIP_ADDR;
    if (s->magic == SERIAL_MAGIC) {
        for (int i = 0; i < 8; ++i) out[i] = s->id[i];
        return true;
    }
    return false;
}


static inline uint32_t rotl32(uint32_t x, int r) {
    return (x << r) | (x >> (32 - r));
}

// xorshift128+ の 32bit 版に近い軽量PRNG（状態2ワード）
typedef struct { uint32_t s0, s1; } prng32_t;

static uint32_t prng32_next(prng32_t *st) {
    uint32_t x = st->s0;
    uint32_t y = st->s1;
    st->s0 = y;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    st->s1 = x;
    return x + y;
}

// 簡易ミキサ（SplitMix系の一部を32bit化）
static uint32_t mix32(uint32_t v) {
    v ^= v >> 16;
    v *= 0x7feb352dU;
    v ^= v >> 15;
    v *= 0x846ca68bU;
    v ^= v >> 16;
    return v;
}

// ADCからノイズを少し集めてシードに混ぜる
static void gather_entropy32(uint32_t *e0, uint32_t *e1) {
    // ADC 初期化
    adc_init();

    // 内部温度センサ(ADC4)のゆらぎ
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    busy_wait_us(11);
    uint16_t t0 = adc_read();
    busy_wait_us(17);
    uint16_t t1 = adc_read();

    // 外部ADC0 (GPIO26) — 未接続だとよりノイジー。接続済みでも微小ゆらぎはある
    adc_gpio_init(26);
    adc_select_input(0);
    busy_wait_us(13);
    uint16_t a0 = adc_read();
    busy_wait_us(19);
    uint16_t a1 = adc_read();

    // タイマ類
    uint32_t tu0 = time_us_32();
    busy_wait_us(7);
    uint32_t tu1 = time_us_32();

    // 混合
    uint32_t m0 = (tu0 ^ (tu1 << 1)) ^ ((uint32_t)t0 << 16) ^ (uint32_t)a0;
    uint32_t m1 = (rotl32(tu1, 7) ^ (uint32_t)t1) ^ ((uint32_t)a1 << 16) ^ 0xA5A5F00Du;

    // 温度センサ無効化（省電力）
    adc_set_temp_sensor_enabled(false);

    *e0 = mix32(m0);
    *e1 = mix32(m1);
}

static void gen_random_id(uint8_t out[8]) {
    uint32_t e0, e1;
    gather_entropy32(&e0, &e1);

    // どちらかが0にならないよう最低限のガード
    if (e0 == 0) e0 = 0x6d253f1bu ^ time_us_32();
    if (e1 == 0) e1 = 0x9e3779b9u ^ (uint32_t)to_us_since_boot(get_absolute_time());

    prng32_t rng = { .s0 = e0, .s1 = e1 };

    // 8バイト生成
    uint32_t r0 = prng32_next(&rng);
    uint32_t r1 = prng32_next(&rng);
    memcpy(out + 0, &r0, 4);
    memcpy(out + 4, &r1, 4);

    // 退屈パターン（全0/全FF/全同値）ガード
    bool all0 = true, allff = true, allsame = true;
    for (int i = 0; i < 8; ++i) {
        if (out[i] != 0x00) all0 = false;
        if (out[i] != 0xFF) allff = false;
        if (out[i] != out[0]) allsame = false;
    }
    if (all0 || allff || allsame) {
        // 追加で撹拌
        uint32_t r2 = prng32_next(&rng);
        uint32_t r3 = prng32_next(&rng);
        r0 ^= mix32(r2);
        r1 ^= rotl32(mix32(r3), 13);
        memcpy(out + 0, &r0, 4);
        memcpy(out + 4, &r1, 4);
    }
}

static bool write_stored_serial(const uint8_t in[8]) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));

    stored_serial_t *rec = (stored_serial_t *)page;
    rec->magic = SERIAL_MAGIC;
    memcpy(rec->id, in, 8);
    rec->reserved = 0xFFFFFFFFu;

    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(SERIAL_SECTOR_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SERIAL_SECTOR_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);

    const stored_serial_t *s = (const stored_serial_t *)SERIAL_XIP_ADDR;
    return (s->magic == SERIAL_MAGIC) && (memcmp(s->id, in, 8) == 0);
}

static bool is_all_same(const uint8_t *p, uint8_t v, int len) {
    for (int i = 0; i < len; ++i) if (p[i] != v) return false;
    return true;
}

void usb_serial_init(void)
{
    pico_get_unique_board_id(&uID);

    bool invalid = is_all_same(uID.id, 0xFF, PICO_UNIQUE_BOARD_ID_SIZE_BYTES) ||
                   is_all_same(uID.id, 0x00, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);

    if (invalid) {
        // MX25R4035F は RDUID 非対応 → フォールバック
        uint8_t id[8];
        if (!read_stored_serial(id)) {
            gen_random_id(id);
            (void)write_stored_serial(id);
        }
        for (int i = 0; i < 8; ++i) uID.id[i] = id[i];
    }

    // 16進文字列化（必ずNUL終端を入れる）
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2; i++)
    {
        /* Byte index inside the uid array */
        int bi = i / 2;
        /* Use high nibble first to keep memory order (just cosmetics) */
        uint8_t nibble = (uID.id[bi] >> 4) & 0x0F;
        uID.id[bi] <<= 4;
        /* Binary to hex digit */
        usb_serial[i] = nibble < 10 ? (char)(nibble + '0') : (char)(nibble + 'A' - 10);
    }
    usb_serial[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2] = '\0';
}