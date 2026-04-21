/**
 * @file main.c
 * @brief PIC16F15313 Serial-to-I2C Converter
 *
 * シリアルポートからのコマンドでI2CマスタとしてI2Cデバイスを制御する変換プログラム。
 *
 * Pin Assignment:
 *   RA0 - EUSART TX  : Serial transmit (hardware EUSART, via PPS)
 *   RA1 - EUSART RX  : Serial receive  (hardware EUSART, via PPS)
 *   RA2 - RESET      : I2C device reset (active LOW, 0.5s pulse)
 *   RA3 - MCLR       : Master Clear Reset (MCLRE=ON)
 *   RA4 - SCL        : I2C clock (MSSP hardware I2C, open-drain)
 *   RA5 - SDA        : I2C data  (MSSP hardware I2C, open-drain)
 *
 * Commands:
 *   VER<CRLF>                               - Return ROM version string
 *   NOP<CRLF>                               - No operation (returns OK)
 *   RST[,<opt1>[@<opt2>]]<CRLF>         - Reset I2C device
 *     Options (separated by '@', any order, all optional):
 *       <number> : I2C speed in kbps (30-400); retains current speed if omitted
 *       NR       : skip RESET(RA2) LOW pulse; 500ms wait still performed
 *       S1       : set UART to   9600 bps (applied before OK response)
 *       S2       : set UART to  19200 bps (applied before OK response)
 *       S3       : set UART to  38400 bps (applied before OK response)
 *       S4       : set UART to  57600 bps (applied before OK response)
 *       S5       : set UART to 115200 bps (applied before OK response)
 *       S6       : set UART to 230400 bps (applied before OK response)
 *     Unknown option → no reset, returns NG,CM
 *   SND,<addr_hex>,<hexdata>[,<NS|checksum>]<CRLF> - I2C write (NS=no-stop; optional CRC-8 checksum)
 *   RCV,<addr_hex>,<nbytes><CRLF>             - I2C read (response includes CRC-8 checksum)
 *
 * Compiler: XC8
 * Target  : PIC16F15313
 */

/* -----------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------- */
#include "main.h"
#include "i2clib.h"
#include "eusart.h"

/* -----------------------------------------------------------------------
 * Configuration bits  (XC8 pragma style)
 * ----------------------------------------------------------------------- */
// CONFIG1
#pragma config FEXTOSC = OFF     // External oscillator disabled
#pragma config RSTOSC  = HFINT32 // 32 MHz internal oscillator
#pragma config CLKOUTEN = OFF    // CLKOUT disabled
#pragma config CSWEN   = ON      // Clock switch enabled
#pragma config FCMEN   = OFF     // Fail-safe clock monitor disabled

// CONFIG2
#pragma config MCLRE   = ON      // MCLR enabled (RA3 = MCLR)
#pragma config PWRTE   = ON      // Power-up timer enabled
#ifdef WDT_ENABLED
#pragma config WDTE    = ON      // Watchdog timer enabled (always on)
#else
#pragma config WDTE    = OFF     // Watchdog timer disabled
#endif
#pragma config LPBOREN = ON      // Low-power BOR enabled
#pragma config BOREN   = ON      // Brown-out reset enabled
#pragma config BORV    = HI      // BOR voltage high
#pragma config ZCD     = OFF     // Zero-cross detect disabled
#pragma config PPS1WAY = OFF      /* PPS multiple-write enabled (required for I2C bus recovery) */
#pragma config STVREN  = ON      // Stack overflow reset enabled

// CONFIG3
#ifdef WDT_ENABLED
#pragma config WDTCPS  = WDTCPS_11 // WDT period ~2 s (CLRWDT called in uart_putch/uart_getch and I2C_Wait/I2C_ReadWait polling loops)
#pragma config WDTCWS  = WDTCWS_7  // WDT window (don't care)
#pragma config WDTCCS  = LFINTOSC  // WDT clock (don't care)
#endif

// CONFIG4
#pragma config BBSIZE  = BB512   // Boot block size
#pragma config BBEN    = OFF     // Boot block disabled
#pragma config SAFEN   = OFF     // SAF disabled
#pragma config WRTAPP  = OFF     // App memory write protected: off
#pragma config WRTB    = OFF     // Boot block write: off
#pragma config WRTC    = OFF     // Config registers write: off
#pragma config WRTSAF  = OFF     // SAF write: off
#pragma config LVP     = ON      // Low-voltage programming enabled

// CONFIG5
#pragma config CP      = OFF     // Code protection off

/* -----------------------------------------------------------------------
 * Module-level state
 * ----------------------------------------------------------------------- */
static uint16_t g_i2c_speed_khz = I2C_KHZ_DEFAULT; /* Current I2C speed in kbps */
static bool g_i2c_open = false; /* true while I2C bus is held open by NS flag; cleared on STOP or error */

/* -----------------------------------------------------------------------
 * Interrupt Service Routine
 * No interrupt sources are enabled; this handler is never executed.
 * ----------------------------------------------------------------------- */
void __interrupt() isr(void) {
}

/* -----------------------------------------------------------------------
 * System initialisation
 * ----------------------------------------------------------------------- */
static void system_init(void) {
    /* ----- Oscillator ----- */
    OSCCON1 = 0x60; /* HFINTOSC, no div */
    OSCFRQ = 0x06; /* 32 MHz */

    /* Initial pin states:
     * RA0=1: UART TX idle HIGH (before PPS/EUSART takes control)
     * RA2=0: RESET asserted LOW (startup_reset() controls this)
     * RA4=0: SCL LATA=0 (pin held HIGH by external pull-up; TRIS=1/MSSP controls the pin)
     * RA5=0: SDA LATA=0 (pin held HIGH by external pull-up; TRIS=1/MSSP controls the pin)
     */
    LATA = 0b00000001;

    /* ----- I/O direction -----
     * RA0 output       (EUSART TX, via PPS — driven by EUSART peripheral)
     * RA1 input        (EUSART RX, via PPS)
     * RA2 output       (RESET, active LOW, push-pull)
     * RA3 input        (MCLR, input-only pin)
     * RA4 input+OD     (SCL1, MSSP hardware I2C, open-drain — external pull-up required)
     * RA5 input+OD     (SDA1, MSSP hardware I2C, open-drain — external pull-up required)
     *
     * Open-drain operation (ODCONA bits 4,5 set):
     *   LATA=1 → open-drain releases line → external pull-up drives HIGH
     *   LATA=0 → open-drain pulls line LOW
     * Reading PORTA gives actual pin voltage regardless of TRIS or LATA.
     */
    TRISA = 0b00111010; /* RA1=input(RX), RA3=input(MCLR), RA4=input(SCL1), RA5=input(SDA1); others=output */
    ANSELA = 0x00; /* All pins digital */
    WPUA = 0x00; /* No weak pull-ups (external pull-ups used for I2C) */
    ODCONA = 0b00110000; /* RA4(SCL1) and RA5(SDA1): open-drain */

    //INLVLA &= ~0x30;    // RA4, RA5 を I2Cレベル(SMBus/I2C)しきい値に設定
    INLVLA |= 0x30;
    SLRCONA |= 0x30; // I2C規格に合わせ、スルーレートを制限（急峻な変化を抑える）

    /* PPS: route EUSART TX/RX to pins.
     * The unlock sequence must not be interrupted.
     * PPS1WAY is OFF, so multiple PPS writes are permitted if needed. */
    uint8_t saved_gie = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; /* unlock */

    // 1. PPS設定: ピン割り当て (RA4=SCL, RA5=SDA)
    // 入力設定 (デバイス側がピンを見るための設定)
    SSP1CLKPPS = 0x04; // SCL入力: RA4
    SSP1DATPPS = 0x05; // SDA入力: RA5

    // I2C設定
    RA4PPS = 0x15; // RA4出力: SCL1
    RA5PPS = 0x16; // RA5出力: SDA1

    // シリアル通信設定
    RA0PPS = 0x0F; /* RA0 → TX1 output */
    RX1DTPPS = 0x01; /* RA1 → RX1 input  */

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; /* lock */
    INTCONbits.GIE = saved_gie;

}

/* -----------------------------------------------------------------------
 * Reset sequence
 * RESET=LOW for 500ms, then HIGH, 200ms wait
 * ----------------------------------------------------------------------- */
static void i2c_device_reset(bool no_reset) {

    /* 1. RESET LOW (skipped when NR option given) */
    if (!no_reset) {
        PIN_RESET_LOW();
    }

    /* 2. 500ms wait */
#ifdef WDT_ENABLED
    CLRWDT();
#endif
    __delay_ms(500);

    /* 3. RESET HIGH (skipped when NR option given) */
    if (!no_reset) {
        PIN_RESET_HIGH();
    }

    /* 4. 200ms wait */
#ifdef WDT_ENABLED
    CLRWDT();
#endif
    __delay_ms(200);

}

/**
 * Release I2C bus and clear NS-chain state when g_i2c_open is true.
 * Call this before any error return that may occur while the bus is held open
 * by a preceding NS-flagged SND.
 */
static void I2C_AbortIfOpen(void) {
    if (g_i2c_open) {
        i2c_abort();
        g_i2c_open = false;
    }
}

static void send_ok(void) {
    uart_puts("OK\r\n");
}

static void send_ng_cm(void) {
    I2C_AbortIfOpen();
    uart_puts("NG,CM\r\n");
} /* command syntax error / out-of-range value */

static void send_ng_i2(void) {
    I2C_AbortIfOpen();
    uart_puts("NG,I2\r\n");
} /* I2C error */

static void send_ng_cs(void) {
    I2C_AbortIfOpen();
    uart_puts("NG,CS\r\n");
} /* SND checksum mismatch */

/* -----------------------------------------------------------------------
 * Hex helpers
 * ----------------------------------------------------------------------- */
static uint8_t hex2nibble(char c) {

    // 以下と同等。容量及び実行ステップ削減
    //if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    //if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);    
    uint8_t h = (uint8_t) c - '0';
    if (h <= 9) return h;
    h -= 0x11;
    if (h <= 5) return (uint8_t) (h + 0x0AU);

    return 0xFF; /* invalid */
}

/* -----------------------------------------------------------------------
 * Decimal string parser (lightweight replacement for strtol)
 * Parses an ASCII decimal string into a uint16_t using 16-bit arithmetic
 * only (avoids the 64-bit ___lldiv that strtol pulls in on XC8/PIC16).
 * Returns true if s is non-empty, all digits, and value fits in uint16_t.
 * ----------------------------------------------------------------------- */
static bool parse_dec_u16(const char *s, uint16_t *out) {
    if (s == NULL || *s == '\0') return false;
    uint16_t val = 0;
    while (*s != '\0') {

        // 0～9である事のチェック(容量削減版)
        // *s - '0' をすることで、'0'=>0x00、'9'=>0x09、'/'=>0xFF、':'=>0x0A となり、
        // d > 9 を判定することで数値であることが確認できる。
        uint8_t d = (uint8_t) (*s - '0');
        if (d > 9) return false;

        // 16bit(65535)に収まらない場合はoverflowとする
        if (val > 6553 || (val == 6553 && d > 5)) return false;

        // val = (uint16_t)(val * 10U + d); と同義。
        // 容量削減のため乗算をビットシフトと加算に変換
        val = (uint16_t) ((val << 3) + (val << 1) + d);
        s++;
    }
    *out = val;
    return true;
}

/* -----------------------------------------------------------------------
 * Hex byte parser
 * Parses s[0..1] (2 hex chars) into a uint8_t.
 * Returns true when both characters are valid hex digits.
 * This helper is used both for fixed-length fields and for streaming parses
 * inside longer strings; callers validate length/terminators as needed.
 * ----------------------------------------------------------------------- */
static bool parse_hex_u8(const char *s, uint8_t *out) {
    uint8_t hi = hex2nibble(s[0]);
    uint8_t lo = hex2nibble(s[1]);
    if (hi == 0xFFU || lo == 0xFFU) return false;
    *out = (uint8_t) ((hi << 4) | lo);
    return true;
}

/* -----------------------------------------------------------------------
 * CRC-8 checksum 
 * ----------------------------------------------------------------------- */
static uint8_t calc_crc8(uint8_t crc, uint8_t data) {
    uint8_t c = crc ^ data;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((c & 0x80U) != 0U) {
            c = (uint8_t) (((uint8_t) (c << 1U)) ^ CRC8_POLY);
        } else {
            c = (uint8_t) (c << 1U);
        }
    }
    return c;
}

/* -----------------------------------------------------------------------
 * Minimal string helpers (replaces <string.h> to save code space on XC8)
 * ----------------------------------------------------------------------- */

/* Find c in s, replace it with '\0', and set *next to the following token.
 * If c is not found, s is left unchanged and *next is set to NULL.
 * next must be non-NULL. */
static void scan_to(char *s, char c, char **next) {
    // 終端まで検索
    while (*s != '\0') {
        if (*s == c) {
            // hit
            // next にセパレーターの後に続く文字列のポインタを設定
            *next = s + 1U;
            // separator を\0に更新
            *s = '\0';
            return;
        }
        s++;
    }
    // 終端までhitしなければNULLを返却
    *next = NULL;
    return;
}

/* Slaveアドレスのparse 
 * rest はNULLではないこと */
static bool parse_slave_address(char *param, uint8_t *addr, char **rest) {

    /* パラメタ無し */
    if (param == NULL || *param == '\0') {
        send_ng_cm();
        return false;
    }

    /* Parse slave address */
    scan_to(param, ',', rest);
    if (*rest == NULL) {
        send_ng_cm();
        return false;
    }
    char *addr_str = param;

    /* アドレス範囲チェック */
    if (addr_str[2] != '\0' || !parse_hex_u8(addr_str, addr) || *addr > 0x7FU) {
        send_ng_cm();
        return false;
    }
    return true;
}

/* -----------------------------------------------------------------------
 * Command handlers
 * ----------------------------------------------------------------------- */

/**
 * RST[,<opt1>[@<opt2>]]
 * Resets the I2C device. Options separated by '@':
 *   <number> : I2C speed in kbps; retains current speed if omitted
 *   NR       : skip RESET(RA2) pulse (LOW→HIGH); 500ms wait still performed
 * Unknown option → NG,CM (no reset performed).
 */
static void cmd_rst(char *param) {
    uint16_t speed_khz = g_i2c_speed_khz;
    bool no_reset = false;
    uint16_t new_baud = 0;

    if (param != NULL && *param != '\0') {
        /* Parse '@'-separated tokens; modifies param in place */
        char *p = param;
        while (p != NULL) {
            char *next = NULL;
            scan_to(p, '@', &next);

            /* Distinguish numeric token (speed) from string option */
            uint16_t v;
            if (parse_dec_u16(p, &v)) {
                /* Numeric → treat as speed in kbps */
                if (v < I2C_KHZ_MIN || v > I2C_KHZ_MAX) {
                    send_ng_cm();
                    return;
                }
                speed_khz = v;
            } else {
                // UART Speed change
                uint8_t p0 = p[0];
                uint8_t p1 = p[1];
                if (p[2] != '\0') {
                    send_ng_cm();
                    return;
                } else if (p0 == 'N' && p1 == 'R') {
                    // NR
                    no_reset = true;
                } else {
                    uint8_t s = p1 - '1';
                    if (p0 == 'S' && s < 8U) {
                        // S1-S8
                        new_baud = UART_BAUD[s];
                    } else {
                        send_ng_cm();
                        return;
                    }
                }

            }
            p = next;
        }
    }

    // Set the reset pin from HIGH to LOW for 500 milliseconds
    i2c_device_reset(no_reset);

    /* Restart I2C at new speed */
    g_i2c_speed_khz = speed_khz;
    i2c_init(speed_khz);

    /* Bus recovery: flush any slave stuck mid-byte after the hardware reset */
    i2c_recovery();

    /* Changing the Serial Communication Speed */
    if (new_baud) {
        uart_init(new_baud);
    }

    /* OK — send response at the new baud rate */
    send_ok();
}

/**
 * SND,<addr_hex>,<hex_data>[,<NS|checksum_hex>]
 * Sends bytes to an I2C slave, optionally verifying the CRC-8 checksum.
 *
 * <addr_hex> is 2 hex digits representing the 7-bit slave address (00–7F).
 * <checksum_hex> is 2 hex digits representing the CRC-8 checksum
 * computed over the data bytes.  If omitted the checksum is not verified.
 * The command is rejected with NG if a checksum is provided but does not match.
 *
 * NS (No Stop) option: when the last field is exactly "NS", the I2C STOP
 * condition is suppressed and g_i2c_open is set.  The next SND command will
 * continue streaming data without issuing START or the slave address again.
 * This allows sending more than 66 bytes in one I2C transaction by splitting
 * the payload across multiple SND commands, e.g.:
 *   SND,7F,<up to 132 hex chars>,NS   (START + addr + 66 bytes, no STOP)
 *   SND,7F,<remaining hex chars>       (data only + STOP)
 *
 * CRC-8 accumulation across NS chunks: the running CRC state (g_i2c_crc) is
 * carried from one NS-chained SND to the next.  Only the final SND (no NS)
 * may include a checksum field, which is verified against the cumulative CRC
 * over all data bytes sent in the entire transaction (all chunks combined).
 * NS commands must not include a checksum field.  Examples:
 *   SND,7F,<66 bytes hex>,NS           → send 66 bytes, carry CRC, no STOP
 *   SND,7F,<64 bytes hex>,<cksum_hex>  → send 64 bytes, verify total CRC, STOP
 *
 * Bytes are streamed directly from the hex string in the command buffer to
 * the I2C bus without a separate data[] array, saving I2C_DATA_MAX bytes of
 * stack RAM.  The hex string is fully validated before the I2C transaction
 * starts so that malformed input never produces a partial transmission.
 */
static void cmd_snd(char *param) {

    /* Parse slave address */
    uint8_t addr;
    char* data_str;
    if (!parse_slave_address(param, &addr, &data_str)) {
        return;
    }

    /* Split rest into data and optional checksum at the last comma */
    char *cksum_str = NULL;
    bool has_cksum = false;
    bool no_stop = false;

    scan_to(data_str, ',', &cksum_str);

    if (cksum_str != NULL) {

        char *no_stop_str = NULL;
        scan_to(cksum_str, ',', &no_stop_str);

        /* NS (No Stop) option: keep I2C bus open after sending, skip STOP */
        if (cksum_str[0] == 'N' && cksum_str[1] == 'S' && cksum_str[2] == '\0') {
            no_stop = true;
            has_cksum = false;
        } else {
            has_cksum = true;

            if (no_stop_str != NULL) {
                if (no_stop_str[0] == 'N' && no_stop_str[1] == 'S' && no_stop_str[2] == '\0') {
                    no_stop = true;
                } else {
                    send_ng_cm();
                    return;
                }
            }

        }
    }

    /* Validate hex string before starting the I2C transaction.
     * Use uint8_t counter — buffers fit in 8 bits (CMD_BUF_SIZE = 144).
     *
     * Compute running CRC-8 over current chunk, starting from accumulated state.
     * g_i2c_crc is 0 for a fresh transaction and carries the accumulated value
     * across NS-chained SND commands.  This allows the final SND (without NS)
     * to verify the CRC over the entire payload in one checksum field. */
    uint8_t slen = 0;
    uint8_t run_crc = 0;
    uint8_t byte_val;
    while (data_str[slen] != '\0') {
        if (!(slen & 0x01U)) {
            if (!parse_hex_u8(&data_str[slen], &byte_val)) {
                send_ng_cm();
                return;
            }
            run_crc = calc_crc8(run_crc, byte_val);
        }
        slen++;
    }

    /* データ長が0、二文字区切りになっていない、データ長がI2C_DATA_MAXを
     * 超える場合はコマンドエラーとする  */
    if (slen == 0U || (slen & 1U) || slen > (uint8_t) (I2C_DATA_MAX * 2)) {
        send_ng_cm();
        return;
    }

    /* If checksum field present, validate format and verify cumulative CRC-8. */
    if (has_cksum) {
        uint8_t recv_cksum;
        if (!parse_hex_u8(cksum_str, &recv_cksum) ||
                cksum_str[2] != '\0') {
            send_ng_cm();
            return;
        }
        if (run_crc != recv_cksum) {
            send_ng_cs();
            return;
        }
    }

    /* Start I2C transaction */
    bool ok = true;

    if (!g_i2c_open) {
        /* I2C start (timeout-protected); retry once after bus recovery if bus locked */
        if (!i2c_putstart()) {
            send_ng_i2();
            return;
        }

        /* Send address + write bit */
        if (!i2c_write((uint8_t) (((unsigned int) addr << 1U) | I2C_WRITE_BIT))) {
            ok = false;
            goto snd_stop;
        }
    }
    /* When g_i2c_open is true (NS continuation): the bus is already held open from
     * the preceding NS-flagged SND.  The MSSP is still in master-transmit state, so
     * we continue writing data bytes directly — no Repeated START (RSEN) and no
     * address byte.  Sending RSEN here would cause the slave to expect a new address,
     * producing a NACK on the first data byte (NG,I2). */

    /* Stream: parse two hex chars at a time and send directly over I2C.
     * hex2nibble() is called again here rather than caching results from the
     * validation loop above, because storing I2C_DATA_MAX nibble-pairs in RAM
     * would cost more bytes than the stack frame we are trying to avoid. */
    for (uint8_t i = 0; i < slen && ok; i += 2U) {
        parse_hex_u8(&data_str[i], &byte_val);
        if (!i2c_write(byte_val)) {
            ok = false;
        }
    }

snd_stop:
    if (no_stop && ok) {
        g_i2c_open = true;
        send_ok();
        return;
    }
    g_i2c_open = false;
    if (!i2c_stop()) {
        ok = false;
    }

    if (ok) {
        send_ok();
    } else {
        send_ng_i2();
    }
}

/**
 * RCV,<addr_hex>,<nbytes>
 * Receives nbytes from an I2C slave, prints as hex to UART followed by the
 * CRC-8 checksum of the received bytes as 2 hex digits.
 */
static void cmd_rcv(char *param) {

    /* Parse slave address */
    uint8_t addr;
    char* nbytes_str;
    if (!parse_slave_address(param, &addr, &nbytes_str)) {
        return;
    }

    // 受信バイト数範囲チェック
    uint16_t nbytes;
    if (!parse_dec_u16(nbytes_str, &nbytes) || nbytes < 1U || nbytes > RCV_BYTES_MAX) {
        send_ng_cm();
        return;
    }

    /* Start I2C transaction */

    g_i2c_open = false;

    /* I2C start (timeout-protected); retry once after bus recovery if bus locked */
    if (!i2c_putstart()) {
        goto rcv_recovery;
    }

    /* Send address + read bit */
    if (!i2c_write((uint8_t) (((unsigned int) addr << 1U) | I2C_READ_BIT))) {
        /* NACK on address, or timeout (stuck bus) → send STOP and return NG,I2.
         * If STOP itself fails, fall back to bus recovery. */
        if (!i2c_stop()) {
            goto rcv_recovery;
        }
        goto rcv_stop;
    }

    /* Receive bytes, output each as 2 hex digits immediately.
     * CRC-8 is accumulated on the fly over each received byte. */
    uint8_t cksum = 0;
    uint8_t b;
    while (nbytes--) {
        uint8_t ack_nack = 0; // 0 = ACK (続けて読み取る), 1 = NACK (読み取り終了)
        if (nbytes == 0) {
            ack_nack = 1;
        }
        if (!i2c_read(&b, ack_nack)) {
            /* Timeout: stop I2C and return NG,I2 */
            goto rcv_recovery;
        }
        cksum = calc_crc8(cksum, b);
        uart_putbyte_hex(b);
    }

    /* Append comma separator and CRC-8 checksum as 2 hex digits */
    uart_putch(',');
    uart_putbyte_hex(cksum);

    if (!i2c_stop()) {
        goto rcv_recovery;
    }

    uart_puts("\r\n");
    send_ok();
    return;

rcv_recovery:
    i2c_recovery();
rcv_stop:
    send_ng_i2();
}

/* -----------------------------------------------------------------------
 * Command dispatcher
 * ----------------------------------------------------------------------- */
static void cmd_process(char *line) {
    if (line == NULL || *line == '\0') {
        send_ng_cm();
        return;
    }

    /* Split "CMD,param..." at first comma */
    char *param = NULL;
    scan_to(line, ',', &param);

    // 先頭3文字の下位5bitをシフトして15bit ＋ 最上位ビットに4バイト目のNULL判定
    // "VER"なら以下の計算結果 18614U となる。
    //   V: 0x56 & 0x1F = 0x16
    //   E: 0x45 & 0x1F = 0x05 << 5
    //   R: 0x52 & 0x1F = 0x12 << 10
    //  \0: \x0000 
    // "ver","6%2"でも同一結果となるが、メモリ容量削減のため許容する。 
    uint16_t cmd = (line[0] & 0x1F);
    cmd |= ((uint16_t) (line[1] & 0x1F) << 5);
    cmd |= ((uint16_t) (line[2] & 0x1F) << 10);
    cmd |= ((uint16_t) (line[3] != 0) << 15);

    /* Commands that call I2C_AbortIfOpen() below (all except SND/RCV) close an
     * NS-open transaction before processing so the next command starts clean. */
    switch (cmd) {
        case CMD_RST:
            I2C_AbortIfOpen();
            cmd_rst(param);
            break;
        case CMD_SND:
            cmd_snd(param);
            break;
        case CMD_RCV:
            cmd_rcv(param);
            break;
        case CMD_VER:
            I2C_AbortIfOpen();
            uart_puts(ROM_VERSION);
            uart_puts("\r\n");
            send_ok();
            break;
        case CMD_NOP:
            I2C_AbortIfOpen();
            send_ok();
            break;
        default:
            send_ng_cm();
            break;
    }
}

/* -----------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------- */
void main(void) {
    /* Initialise all peripherals */
    system_init();
    uart_init(UART_BAUD[0]);
    i2c_init(I2C_KHZ_DEFAULT);

    /* Startup reset sequence */
    i2c_device_reset(false);

    /* Bus recovery: flush any slave stuck mid-byte after the startup reset */
    i2c_recovery();

    /* Signal ready — 9600 bps */
    send_ok();

    /* Main command loop */
    static char cmd_buf[CMD_BUF_SIZE];
    for (;;) {
        uart_read_line(cmd_buf, CMD_BUF_SIZE);
        cmd_process(cmd_buf);
    }
}
