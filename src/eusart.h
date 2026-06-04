/* 
 * File:   eusart.h
 * Author: r-kik
 *
 * Created on 2026/04/22, 0:50
 */

#ifndef EUSART_H
#define	EUSART_H

#include <xc.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Hardware EUSART baud rate selection
 * FOSC = 32 MHz, BRG16 = 1, BRGH = 1
 * Formula: SPBRGx = (FOSC / (4 * BaudRate)) - 1
 * ----------------------------------------------------------------------- */
static const uint16_t UART_BAUD_SPBRG[8] = {
    0x340U,  /*   9600bps, actual:   9604 bps, error: 0.04% */
    0x1A0U,  /*  19200bps, actual:  19185 bps, error: 0.08% */
    0x0CFU,  /*  38400bps, actual:  38462 bps, error: 0.16% */
    0x08AU,  /*  57600bps, actual:  57554 bps, error: 0.08% */
    0x044U,  /* 115200bps, actual: 115942 bps, error: 0.64% */
    0x022U,  /* 230400bps, actual: 228571 bps, error: 0.79% */
    0x010U,  /* 460800bps, actual: 470588 bps, error: 2.12% */
    0x008U   /* 921600bps, actual: 888888 bps, error: 3.55% */
};

#define _XTAL_FREQ          32000000UL   /* 32 MHz internal oscillator */

void uart_init(uint16_t baud_spbrg);
void uart_putch(uint8_t c);
void uart_puts(const char *s);
void uart_putbyte_hex(uint8_t b);
uint8_t uart_getch(void);
void uart_read_line(char *buf, uint8_t buf_size);

#endif	/* EUSART_H */

