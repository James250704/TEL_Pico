/*
 * CNC Shield 平台升降控制程式
 * 版本: 2.0 (已整合 RC 遙控)
 *
 * 功能:
 * 1. 透過 Serial (UART) 接收來自遙控接收器Pico 的指令 (格式: "v0,v1,v2,v3,...")
 * 2. 解析 CH4 (SA) 的值 (字串中的第 4 個值，索引 3)
 * 3. 根據訊號值控制馬達:
 * - CH4 < -50 : 啟動馬達 (ENABLE_PIN = LOW)
 * - CH4 >= -50: 關閉馬達 (ENABLE_PIN = HIGH)
 */

// === 馬達引腳定義 ===
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define ENABLE_PIN 8 // CNC Shield 上的致動腳位 (Active LOW)

// === 馬達運轉設定 ===
#define MOTOR_DIRECTION HIGH // 設定馬達固定轉動方向
const int STEP_DELAY = 550;  // 微秒，步進速度

// === 馬達控制變數 ===
bool stepState = LOW;
unsigned long lastStepTime = 0;
bool motorActive = false; // 馬達是否啟動的狀態

// === RC 訊號接收設定 ===
const long BAUD_RATE = 115200; // 必須與 remote_receiver.py 的 CommConfig 匹配
String serialBuffer = "";

void setup() {
    // 初始化馬達引腳
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    // 設定初始狀態
    digitalWrite(X_DIR_PIN, MOTOR_DIRECTION);
    digitalWrite(ENABLE_PIN, HIGH); // 預設關閉馬達 (HIGH = 禁用)
    digitalWrite(X_STEP_PIN, LOW);

    lastStepTime = micros();

    // 初始化序列埠 (UART)
    Serial.begin(BAUD_RATE);
    serialBuffer.reserve(100); // 為序列緩衝區預留空間
}

/**
 * @brief 檢查並處理序列埠輸入
 * 從 remote_receiver.py 讀取一行指令
 */
void checkSerialInput() {
    while(Serial.available() > 0) {
        char inChar = (char)Serial.read();

        if(inChar == '\n') {
            // 收到一個完整的指令 (以換行符結尾)
            processRCCommand(serialBuffer);
            serialBuffer = ""; // 清空緩衝區
        } else {
            // 將字元添加到緩衝區
            serialBuffer += inChar;
        }
    }
}

/**
 * @brief 解析 RC 指令字串
 * @param cmd 完整的指令字串 (例如 "0,0,50,-100,0,...")
 */
void processRCCommand(String cmd) {
    // 我們需要找到第 4 個值 (索引 3)，代表 CH4 (SA)

    int commaIndex = -1;
    int lastCommaIndex = -1;
    String ch4_str = "";

    // 尋找第 3 個逗號 (索引 2)
    commaIndex = cmd.indexOf(','); // 第 1 個
    if(commaIndex == -1)
        return;                                    // 格式錯誤
    commaIndex = cmd.indexOf(',', commaIndex + 1); // 第 2 個
    if(commaIndex == -1)
        return;                                    // 格式錯誤
    commaIndex = cmd.indexOf(',', commaIndex + 1); // 第 3 個
    if(commaIndex == -1)
        return;                                    // 格式錯誤
    commaIndex = cmd.indexOf(',', commaIndex + 1); // 第 3 個
    if(commaIndex == -1)
        return; // 格式錯誤
    // 提取第 3 個和第 4 個逗號之間的子字串
    int startIndex = commaIndex + 1;
    int endIndex = cmd.indexOf(',', startIndex);

    if(endIndex == -1) {
        // 可能是字串結尾
        ch4_str = cmd.substring(startIndex);
    } else {
        ch4_str = cmd.substring(startIndex, endIndex);
    }

    // 將字串轉換為整數
    int ch4_val = ch4_str.toInt();

    // 應用控制邏輯
    // CH 4(SA) < -50 就啟動
    // > -50 (即 >= -50) 就關閉
    if(ch4_val > -50) {
        motorActive = true;
    } else {
        motorActive = false;
    }
}

/**
 * @brief 根據 motorActive 狀態驅動馬達
 */
void runMotor() {
    if(motorActive) {
        // 狀態: 啟動
        // 確保馬達驅動器已啟用 (Active LOW)
        digitalWrite(ENABLE_PIN, LOW);

        // 執行步進
        unsigned long now = micros();
        if(now - lastStepTime >= STEP_DELAY) {
            lastStepTime = now;
            stepState = !stepState;
            digitalWrite(X_STEP_PIN, stepState);
        }
    } else {
        // 狀態: 關閉
        // 設置 ENABLE HIGH 來釋放馬達 (節能/停止)
        digitalWrite(ENABLE_PIN, HIGH);

        // 重置步進狀態，確保下次啟動乾淨
        if(stepState == HIGH) {
            digitalWrite(X_STEP_PIN, LOW);
            stepState = LOW;
        }
    }
}

void loop() {
    // 1. 檢查並處理來自接收器 Pico 的序列訊號
    checkSerialInput();

    // 2. 根據 'motorActive' 狀態控制馬達
    runMotor();
}