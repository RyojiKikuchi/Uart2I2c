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

#define _XTAL_FREQ          32000000UL   /* 32 MHz internal oscillator */
#define SPBRG_CALC_NUMBER   (uint16_t)(_XTAL_FREQ / 800U) // 16bitでspbrg計算のために事前にクロックを800で割っておく

void uart_init(uint16_t baud_spbrg);
void uart_putch(uint8_t c);
void uart_puts(const char *s);
void uart_putbyte_hex(uint8_t b);
uint8_t uart_getch(void);
void uart_read_line(char *buf, uint8_t buf_size);

#endif	/* EUSART_H */

