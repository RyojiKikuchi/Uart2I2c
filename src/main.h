/**
 * @file main.h
 * @brief PIC16F15313 Serial-to-I2C Converter
 *
 * PIC16F15313を使用したシリアル-I2C変換プログラム ヘッダファイル
 *
 * Pin Assignment:
 *   RA0 - EUSART TX  : Serial transmit (hardware EUSART, via PPS)
 *   RA1 - EUSART RX  : Serial receive  (hardware EUSART, via PPS)
 *   RA2 - RESET      : I2C device reset (active LOW, 0.5s pulse)
 *   RA3 - MCLR       : Master Clear Reset (MCLRE=ON)
 *   RA4 - SCL        : I2C clock (MSSP hardware I2C, open-drain)
 *   RA5 - SDA        : I2C data  (MSSP hardware I2C, open-drain)
 */

#ifndef MAIN_H
#define MAIN_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "version.h"

/* -----------------------------------------------------------------------
 * Oscillator / Timing
 * ----------------------------------------------------------------------- */
#define _XTAL_FREQ  32000000UL   /* 32 MHz internal oscillator */

#define I2C_KHZ_DEFAULT 100U
#define LOOP_CYCLE_1MS  (uint16_t)(_XTAL_FREQ / 4000U)

/* -----------------------------------------------------------------------
 * Hardware EUSART baud rate selection
 * FOSC = 32 MHz, BRG16 = 1, BRGH = 1
 * Formula: SPBRGx = (FOSC / (4 * BaudRate)) - 1
 * ----------------------------------------------------------------------- */
static const uint16_t UART_BAUD[8] = {
    96U,    /* actual: 9604 bps,   error: 0.04% */
    192U,   /* actual: 19185 bps,  error: 0.08% */
    384U,   /* actual: 38462 bps,  error: 0.16% */
    576U,   /* actual: 57554 bps,  error: 0.08% */
    1152U,  /* actual: 115942 bps, error: 0.64% */
    2304U,  /* actual: 228571 bps, error: 0.79% */
    4608U,  /* actual: 470588 bps, error: 2.12% */
    9216U   /* actual: 888888 bps, error: 3.55% */
};

/* -----------------------------------------------------------------------
 * Serial command buffer
 * CMD_BUF_SIZE must hold the longest possible command line:
 *   "SND,7F," (7) + I2C_DATA_MAX*2 hex chars + ",XX"  checksum (3) + ",NS" (3) + null = 146 bytes.
 * 147 provides one byte of slack — CMD_BUF_SIZE is kept at 147.
 * ----------------------------------------------------------------------- */
#define CMD_BUF_SIZE   147

/* -----------------------------------------------------------------------
 * I2C data payload limit
 * PIC16F15313 has 256 bytes RAM. With the streaming SND implementation
 * (no separate data[] buffer), the binding constraint is CMD_BUF_SIZE.
 * Overhead per SND line: "SND,7F," (7) + ",XX" checksum (3) + null (1) = 11.
 * I2C_DATA_MAX = floor((CMD_BUF_SIZE - 11) / 2) = floor(133/2) = 66 bytes.
 * ----------------------------------------------------------------------- */
#define I2C_DATA_MAX   66        /* max bytes per SND command */
#define RCV_BYTES_MAX  8192U     /* max bytes per RCV command */

/* -----------------------------------------------------------------------
 * Pin helpers (LATA bits)
 * ----------------------------------------------------------------------- */
#define PIN_RESET_LOW()   do { LATAbits.LATA2 = 0; } while(0)
#define PIN_RESET_HIGH()  do { LATAbits.LATA2 = 1; } while(0)



#define CMD_CALC(a, b, c) \
                (uint16_t)(((uint16_t)a & 0x1FU) | \
                          (((uint16_t)b & 0x1FU) << 5) | \
                          (((uint16_t)c & 0x1FU) << 10))

#define CMD_RST     CMD_CALC('R', 'S', 'T')
#define CMD_SND     CMD_CALC('S', 'N', 'D')
#define CMD_RCV     CMD_CALC('R', 'C', 'V')
#define CMD_VER     CMD_CALC('V', 'E', 'R')
#define CMD_NOP     CMD_CALC('N', 'O', 'P')

#define CRC8_POLY  0x07U

/* Watchdog configuration note:
 * Do not define WDT_ENABLED in this header. main.c checks WDT_ENABLED for
 * configuration pragmas before including main.h, so defining it here would
 * only affect later #ifdef WDT_ENABLED code paths and not the WDTE config.
 * Enable it via the project/compiler preprocessor settings so it is visible
 * before the configuration block in main.c.
 */

#endif /* MAIN_H */
