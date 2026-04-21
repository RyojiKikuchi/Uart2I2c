/* -----------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------- */
#include "i2clib.h"

/* -----------------------------------------------------------------------
 * I2C (MSSP) initialisation
 * Configures the MSSP peripheral for I2C Master mode at the given speed.
 * Caller must pass a validated speed (I2C_KHZ_MIN..I2C_KHZ_MAX).
 * ----------------------------------------------------------------------- */
void i2c_init(uint16_t speed_khz) {

    SSP1CON1 = 0x00; // リセット

    // 100KHz超の場合、SMP=0に設定する
    if (speed_khz > 100) {
        SSP1STAT = 0x00; // Fast Mode (SMP=0)
    } else {
        SSP1STAT = 0x80; // Standard Mode (SMP=1)
    }

    // ボーレート計算 (Fosc=32MHz)
    // 32000 / (4 * speed) - 1
    uint16_t add_val = (8000U / speed_khz) - 1;
    if (add_val > 255U) {
        SSP1ADD = 255U; // 最小速度制限(31.25KHz)
    } else {
        SSP1ADD = (uint8_t) add_val;
    }

    // 3. MSSP設定 (Master mode)
    SSP1CON1 = 0x28; // I2C Master mode, SSP enable

}

// 待機関数
// 戻り値: true = 成功, false = タイムアウト失敗
bool i2c_wait(bool isRead) {
    uint8_t timeout_ms = I2C_TIMEOUT_MS;
    while (timeout_ms > 0) {
        uint16_t count = LOOP_CYCLE_1MS; // 約1ms分のループ
        while (count--) {

            if (!isRead) {
                // 下記のフラグがすべて0になるのを待つ
                // SSP1STATbits.BF: バッファフル (送信中)
                // SSP1CON2 & 0x1F: SEN, RSEN, PEN, RCEN, ACKEN (各条件の実行中)
                if (!(SSP1STAT & 0x01) && !(SSP1CON2 & 0x1F)) {
                    return true; // 成功
                }
            } else {
                // 下記のフラグがすべて0になるのを待つ
                // SSP1CON2  RSEN
                if (!SSP1CON2bits.RCEN) {
                    return true; // 成功
                }
            }

#ifdef WDT_ENABLED
            CLRWDT(); // I2Cポーリング中にウォッチドッグタイマーをリセット
#endif
        }

        timeout_ms--; // 1ms経過（近似）
    }
    return false;
}

// 開始条件

bool i2c_start(void) {
    if (!i2c_wait(false)) return false;
    SSP1CON2bits.SEN = 1;
    return true;
}

// 停止条件

bool i2c_stop(void) {
    if (!i2c_wait(false)) return false;
    SSP1CON2bits.PEN = 1;
    return i2c_wait(false);
}

// データ送信

bool i2c_write(uint8_t data) {

    if (!i2c_wait(false)) return false;

    PIR3bits.SSP1IF = 0; // 送信前にフラグをクリア
    SSP1BUF = data;

    // BFは8ビット送信後に即クリアされるため、ACKSTAT（9クロック目のACK/NACK）が
    // 確定するSSP1IFのセットを待つ
    uint8_t timeout_ms = 10;
    while (timeout_ms--) {
        uint16_t count = LOOP_CYCLE_1MS;
        while (count--) {
            if (PIR3bits.SSP1IF) {
                PIR3bits.SSP1IF = 0;
                return (SSP1CON2bits.ACKSTAT == 0); // 0=ACK, 1=NACK
            }
#ifdef WDT_ENABLED
            CLRWDT(); // I2Cポーリング中にウォッチドッグタイマーをリセット
#endif
        }
    }
    return false;
}

// 1バイト受信
// ack: 0 = ACK (続けて読み取る), 1 = NACK (読み取り終了)

bool i2c_read(uint8_t *data, uint8_t ack) {
    if (!i2c_wait(false)) return false;

    SSP1CON2bits.RCEN = 1; // 受信開始
    if (!i2c_wait(true)) return false;

    *data = SSP1BUF; // データの取り出し

    // ACK/NACK応答の設定
    SSP1CON2bits.ACKDT = ack;
    SSP1CON2bits.ACKEN = 1; // 応答送信

    return i2c_wait(false);
}

// I2CバスとMSSPモジュールの強制リセット

void i2c_recovery(void) {

    // 1. MSSPモジュールを一旦OFF
    SSP1CON1bits.SSPEN = 0;

    // 保留中のSSP1CON2制御ビット（SEN/RSEN/PEN/RCEN/ACKEN）をクリア
    SSP1CON2 = 0x00;

    // 2. ピンを一時的に手動制御（GPIO）に切り替えてバスを解放
    // SDA, SCLを出力モードに設定
    TRISA &= ~I2C_PIN; // RA4, RA5を出力に

    // スレーブがSDAをLowに保持している場合、SCLを最大9回振って
    // スレーブの内部状態をリセットさせる（バス・クリア・シーケンス）
    for (uint8_t i = 0; i < 9; i++) {
        //RA4 = 0;
        PORTA &= ~I2C_PIN_SCL;
        __delay_us(5);
        //RA4 = 1;
        PORTA |= I2C_PIN_SCL;
        __delay_us(5);
        // もしSDAがHighに戻ったら（スレーブが解放したら）途中で抜けても良い
        //if (RA5 == 1) break;
        if ((PORTA & I2C_PIN_SDA) !=  0) break;
    }
//#define I2C_PIN_SCL     0b00010000  // SCL=RA4
//#define I2C_PIN_SDA     0b00100000  // SDA=RA5
    // 3. ストップ条件を擬似的に生成（SDAをLow→Highへ）
    //RA5 = 0;
    PORTA &= ~I2C_PIN_SDA;
    __delay_us(5);
    //RA4 = 1;
    PORTA |= I2C_PIN_SCL;
    __delay_us(5);
    //RA5 = 1;
    PORTA |= I2C_PIN_SDA;
    __delay_us(5);

    // 4. ピン設定をMSSP用に戻す
    TRISA |= I2C_PIN; // 再び入力(MSSP制御下)へ

    // 5. WCOL（書き込み衝突）とSSPOV（受信オーバーフロー）エラーフラグをクリア
    //    これらはソフトウェアで明示的にクリアしないとSSPEN ON/OFFを跨いで残留する
    SSP1CON1 &= ~0xC0;

    // 6. MSSPモジュールを再起動
    SSP1CON1bits.SSPEN = 1;

    // 7. BF（Buffer Full）フラグのクリア
    //    I2C受信完了直後にタイムアウトした場合、SSP1BUFにデータが残りBF=1のままになる。
    //    BF=1だとI2C_Wait()が永久にタイムアウトし、以降の操作がすべてNG,TOになる。
    //    SSP1BUFを読み捨てることでBFをクリアする。
    if (SSP1STATbits.BF) {
        (void) SSP1BUF;
    }
}

// I2C Abort

void i2c_abort(void) {
    if (!i2c_stop()) {
        i2c_recovery();
    }
}

// I2Cスタート送信

bool i2c_putstart() {
    if (!i2c_start()) {
        i2c_recovery();
        if (!i2c_start()) {
            return false;
        }
    }
    return true;
}

