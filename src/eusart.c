/* -----------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------- */
#include "eusart.h"

/* -----------------------------------------------------------------------
 * UART helpers (hardware EUSART)
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * Hardware EUSART initialisation
 * Configures the EUSART for 8N1 async mode at the selected baud rate.
 * Must be called after system_init() has configured TRIS and PPS.
 * uart_bps は通信速度を100で割った数値を設定する。 9600bps=96, 115200bps = 1152
 * ----------------------------------------------------------------------- */
void uart_init(uint16_t uart_bps) {

    /* Switch UART baud rate: wait for TX shift register to drain then reconfigure EUSART.
     * TRMT (not TX1IF) is used here because the shift register must be fully empty before
     * changing the baud rate; TX1IF only indicates TXREG is ready for the next byte.
     * OK response will be sent at the new baud rate. */
    while (!TXSTAbits.TRMT) {
#ifdef WDT_ENABLED
        CLRWDT();
#endif
    }

    uint16_t b = uart_bps / 2;
    uint16_t spbrg = ((SPBRG_CALC_NUMBER + (b / 2)) / b) - 1;
    SPBRGH = (uint8_t) ((spbrg >> 8U) & 0xFFU);
    SPBRGL = (uint8_t) (spbrg & 0xFFU);
    BAUDCON = 0x08U; /* BRG16=1 (16-bit BRG), ABDEN=0, WUE=0 */
    TXSTA = 0x24U; /* TXEN=1, BRGH=1, SYNC=0 (async), TX9=0 */
    RCSTA = 0x90U; /* SPEN=1 (enables TX/RX pins), CREN=1 (continuous receive), RX9=0 */
}

/**
 * Transmit one byte via hardware EUSART.
 * Blocks until TXREG is empty (TX1IF=1) before loading the next byte,
 * utilising the EUSART double buffer (TXREG + TSR).
 * When WDT_ENABLED is defined, CLRWDT is called each polling iteration
 * to keep the watchdog alive during large RCV output sequences.
 */
void uart_putch(uint8_t c) {
    while (!PIR3bits.TX1IF) {
#ifdef WDT_ENABLED
        CLRWDT();
#endif
    }
    TXREG = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putch((uint8_t) * s++);
}

void uart_putbyte_hex(uint8_t b) {
    static const char hex_chars[] = "0123456789ABCDEF";
    uart_putch((uint8_t) hex_chars[(b >> 4) & 0x0F]);
    uart_putch((uint8_t) hex_chars[b & 0x0F]);
}

/**
 * Receive one byte via hardware EUSART.
 * Busy-waits for a received byte; when WDT is enabled, calls CLRWDT()
 * to keep the watchdog alive.
 * Clears overrun error (OERR) if set by toggling CREN.
 */
uint8_t uart_getch(void) {
    while (!PIR3bits.RC1IF) {
#ifdef WDT_ENABLED
        CLRWDT(); /* pet watchdog while waiting for data */
#endif
    }
    uint8_t data = RCREG; // 先にFIFOを読む
    if (RCSTAbits.OERR) { // その後でOERRをクリア
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    return data;
}

/**
 * Read one line terminated by CR or CR LF.
 * Stores up to (buf_size-1) characters, null-terminated.
 * Leading CR/LF characters are skipped so that the LF following a CR+LF
 * sequence does not produce a spurious empty command on the next call.
 * Empty lines (lone CR or LF with no preceding data) are silently ignored.
 */
void uart_read_line(char *buf, uint8_t buf_size) {
    uint8_t idx = 0;
    char c;

    while (true) {
        c = (char) uart_getch();
        switch (c) {
            case '\r':
                /* Skip CR */
                continue;
            case '\n':
                /*  LF ends the line */
                if (idx == 0) continue; /* skip leading LF     */
                buf[idx] = '\0';
                return;
        }
        if (idx < (uint8_t) (buf_size - 1U)) {
            buf[idx++] = c;
        }
    }
}

