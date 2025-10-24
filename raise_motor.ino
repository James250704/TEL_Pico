/*
 * 最終整合程式：平台馬達 H/L/S 控制
 * (v2.0 - 抗抖動/Jitter 優化版)
 *
 * 功能:
 * 1. 使用極速的 checkPicoCommands() 避免 String 緩存。
 * 2. 僅在狀態改變時才設定馬達方向，避免在 loop 中增加延遲。
 * 3. H/L/S 控制馬達，S 狀態保持鎖定。
 */

#include <SoftwareSerial.h>

// === 1. 序列埠設定 ===
const long DEBUG_BAUDRATE = 9600;
const long PICO_BAUDRATE = 9600;
const int PICO_RX_PIN = A0;
const int PICO_TX_PIN = A1;
SoftwareSerial picoSerial(PICO_RX_PIN, PICO_TX_PIN);

// === 2. 馬達腳位定義 ===
const int stepPinX = 2;
const int dirPinX = 5;
const int stepPinY = 3;
const int dirPinY = 6;
const int stepPinZ = 4;
const int dirPinZ = 7;
const int enablePin = 8;

// === 3. 馬達參數 ===
const int stepDelay = 500; // 來自您的 "可運作程式"
const int DIR_DOWN = HIGH; // H 指令 (下降)
const int DIR_UP = LOW;    // L 指令 (上升)

// === 4. 狀態變數 ===
char motorState = 'S';     // 當前馬達狀態 (H, L, S)
char lastMotorState = 'S'; // 上一次的馬達狀態，用於偵測變化

void setup() {
    // 設定所有腳位為 OUTPUT 模式
    pinMode(stepPinX, OUTPUT);
    pinMode(dirPinX, OUTPUT);
    pinMode(stepPinY, OUTPUT);
    pinMode(dirPinY, OUTPUT);
    pinMode(stepPinZ, OUTPUT);
    pinMode(dirPinZ, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // 預設 enablePin 為 LOW (啟用/通電)
    digitalWrite(enablePin, LOW);

    // 初始化 "除錯" 序列埠
    Serial.begin(DEBUG_BAUDRATE);
    while(!Serial)
        ;
    Serial.println("--- 平台馬達 H/L/S 控制已啟動 (v2.0 - 抗抖動) ---");
    Serial.println("馬達預設啟用 (通電鎖定)");
    Serial.println("等待來自 Pico (A0) 的 H/L/S 指令...");

    // 初始化 "Pico" 序列埠
    picoSerial.begin(PICO_BAUDRATE);
}

void loop() {
    // 1. 永遠先檢查指令 (這個函式現在超級快)
    checkPicoCommands();

    // 2. 執行馬達動作
    runMotors();
}

/**
 * @brief (優化版) 極速檢查 Pico 指令
 * - 不使用 String, 不使用 buffer
 * - 立即反應 H, L, S
 * - 忽略 '\n' 和其他字元
 */
void checkPicoCommands() {
    // 只要有資料就讀取
    if(picoSerial.available() > 0) {
        // 讀取一個字元
        char c = picoSerial.read();

        // 根據單一字元立即更新狀態
        if(c == 'H') {
            motorState = 'H';
        } else if(c == 'L') {
            motorState = 'L';
        } else if(c == 'S') {
            motorState = 'S';
        }
        // 我們完全忽略 'H' 'L' 'S' 以外的所有字元 (例如 '\n')
    }
}

/**
 * @brief (優化版) 根據 motorState 變數來驅動馬達
 * - 僅在 motorState 發生 "改變" 時才設定方向
 */
void runMotors() {

    // 偵測狀態是否 "剛剛" 發生改變
    if(motorState != lastMotorState) {
        // --- 狀態發生了變化 ---
        if(motorState == 'H') {
            // 剛切換到 H: 設定方向為 "下降"
            Serial.println("狀態變更 -> H (下降)"); // 在此處除錯是安全的
            digitalWrite(dirPinX, DIR_DOWN);
            digitalWrite(dirPinY, DIR_DOWN);
            digitalWrite(dirPinZ, DIR_DOWN);
        } else if(motorState == 'L') {
            // 剛切換到 L: 設定方向為 "上升"
            Serial.println("狀態變更 -> L (上升)");
            digitalWrite(dirPinX, DIR_UP);
            digitalWrite(dirPinY, DIR_UP);
            digitalWrite(dirPinZ, DIR_UP);
        } else if(motorState == 'S') {
            // 剛切換到 S: 停止
            Serial.println("狀態變更 -> S (停止)");
        }

        // 更新 "上一次的狀態"
        lastMotorState = motorState;
    }

    // --- 執行 "當前" 狀態的動作 ---
    // (這個 if 區塊會被重複執行)

    if(motorState == 'H' || motorState == 'L') {
        // 狀態是 H 或 L:
        // *** 這裡不再設定方向 ***
        // 這裡只做一件事: 產生脈衝 (就像您的獨立測試程式一樣)

        digitalWrite(stepPinX, HIGH);
        digitalWrite(stepPinY, HIGH);
        digitalWrite(stepPinZ, HIGH);
        delayMicroseconds(stepDelay); // 500us

        digitalWrite(stepPinX, LOW);
        digitalWrite(stepPinY, LOW);
        digitalWrite(stepPinZ, LOW);
        delayMicroseconds(stepDelay); // 500us

    } else {
        // 狀態是 S:
        // 不做任何事 (不發送脈衝)，馬達保持鎖定
    }
}