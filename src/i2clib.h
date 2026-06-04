/* 
 * File:   i2clib.h
 * Author: r-kik
 *
 * Created on 2026/04/21, 15:40
 */

#ifndef I2CLIB_H
#define	I2CLIB_H

#include <xc.h>
#include <stdbool.h>

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
 * Clock Settings
 * ----------------------------------------------------------------------- */

#define _XTAL_FREQ  32000000UL   /* 32 MHz internal oscillator */
#define LOOP_CYCLE_1MS  (uint16_t)(_XTAL_FREQ / 4000U)

/* -----------------------------------------------------------------------
 * I2C speed limits in khz (uint16_t) — used for 16-bit-only arithmetic
 * ----------------------------------------------------------------------- */
#define I2C_KHZ_MIN      30U
#define I2C_KHZ_MAX     400U

#define I2C_PIN         0b00110000  // RA4,RA5
#define I2C_PIN_SCL     0b00010000  // SCL=RA4
#define I2C_PIN_SDA     0b00100000  // SDA=RA5

/* -----------------------------------------------------------------------
 * I2C address/direction bit constants
 * ----------------------------------------------------------------------- */
#define I2C_WRITE_BIT  0U
#define I2C_READ_BIT   1U

#define I2C_TIMEOUT_MS  10U

void i2c_init(uint16_t speed_khz);
bool i2c_wait(bool isRead);
bool i2c_start(void);
bool i2c_restart(void);
bool i2c_stop(void);
bool i2c_write(uint8_t data);
bool i2c_read(uint8_t *data, uint8_t ack);
void i2c_recovery(void);
void i2c_abort(void);
bool i2c_putstart(void);
bool i2c_putrestart(void);

#endif	/* I2CLIB_H */
