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
 * Clock Settings
 * ----------------------------------------------------------------------- */

#define _XTAL_FREQ  32000000UL   /* 32 MHz internal oscillator */
#define LOOP_CYCLE_READ_1MS  (uint16_t)(_XTAL_FREQ / 2000U)
#define LOOP_CYCLE_WRITE_1MS  (uint16_t)(_XTAL_FREQ / 4000U)

/* -----------------------------------------------------------------------
 * I2C speed limits in khz (uint16_t) — used for 16-bit-only arithmetic
 * ----------------------------------------------------------------------- */
#define I2C_KHZ_MIN      30U
#define I2C_KHZ_MAX     400U

/* -----------------------------------------------------------------------
 * I2C Pin Configuration & PPS Configuration
 * ----------------------------------------------------------------------- */

#define I2C_PIN_SCL     0b00010000  // SCL=RA4
#define I2C_PIN_SDA     0b00100000  // SDA=RA5

#define I2C_PORT_SCL    PORTA   // read: PORTx
#define I2C_PORT_SDA    PORTA   // read: PORTx

#define I2C_LAT_SCL     LATA    // write: LATx
#define I2C_LAT_SDA     LATA    // write: LATx

#define I2C_TRIS_SCL    TRISA 
#define I2C_TRIS_SDA    TRISA

#define I2C_INLVL_SCL   INLVLA
#define I2C_INLVL_SDA   INLVLA

#define I2C_SLRCON_SCL  SLRCONA
#define I2C_SLRCON_SDA  SLRCONA

// PPS INPUT SIGNAL ROUTING
#define I2C_PPSIN_CLK   0x04    // RA4
#define I2C_PPSIN_DAT   0x05    // RA5

// PPS OUTPUT SIGNAL ROUTING
#define I2C_PPSOUT_SCL  0x15    // SCL1
#define I2C_PPSOUT_SDA  0x16    // SDA1

/* -----------------------------------------------------------------------
 * I2C address/direction bit constants
 * ----------------------------------------------------------------------- */
#define I2C_WRITE_BIT  0U
#define I2C_READ_BIT   1U

#define I2C_TIMEOUT_MS  10U

void i2c_init(uint8_t speed_khz);
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
