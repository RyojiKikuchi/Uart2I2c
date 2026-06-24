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
 * Configuration bits  (XC8 pragma style)
 * ----------------------------------------------------------------------- */

// PIC16F15313 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config LPBOREN = ON     // Low-Power BOR enable bit (ULPBOR enabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) is set to 2.7V)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/* -----------------------------------------------------------------------
 * Oscillator / Timing
 * ----------------------------------------------------------------------- */
#define _XTAL_FREQ  32000000UL   /* 32 MHz internal oscillator */

#define I2C_KHZ_DEFAULT 10U     /* 100KHz */
#define LOOP_CYCLE_1MS  (uint16_t)(_XTAL_FREQ / 4000U)

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
#define RCV_BYTES_MAX  4096U     /* max bytes per RCV command */

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
#define CMD_SNT     CMD_CALC('S', 'N', 'T')
#define CMD_RCV     CMD_CALC('R', 'C', 'V')
#define CMD_VER     CMD_CALC('V', 'E', 'R')
#define CMD_NOP     CMD_CALC('N', 'O', 'P')

#define CRC8_INIT  0x00U
#define CRC8_POLY  0x07U
#define CRC16_INIT  0xFFU
#define CRC16_POLY_HIGH 0x10
#define CRC16_POLY_LOW  0x21

#define I2C_START_RETRY_COUNT 2U

#define NEWLINE "\r\n"

//#define CRC8
#define CRC16

/* Checksum selection: define exactly one of CRC8 or CRC16 */
#if defined(CRC8) && defined(CRC16)
#error "Define only one of CRC8 or CRC16"
#elif !defined(CRC8) && !defined(CRC16)
#error "Define CRC8 or CRC16"
#endif

#endif /* MAIN_H */
