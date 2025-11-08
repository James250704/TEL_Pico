// CNC Shield 平台升降控制程式 (最終修正版)

#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7
#define ENABLE_PIN 8 // Active LOW

#define LIMIT_PIN_TOP 11 // Z-Limit 接腳 (D11)

#define DIR_UP LOW
#define DIR_DOWN HIGH

const int STEP_DELAY = 500; // 微秒，步進速度
bool stepState = LOW;
unsigned long lastStepTime = 0;
char motorState = 'S'; // H (上) / S (停) / L (下)

String uartBuffer = "";
const int BUF_MAX = 60;

// --- *** 已修正 *** 抗 EMI 噪音的讀取函式 (邏輯更正) ---
bool isTopLimitPressed_Debounced() {
    if(digitalRead(LIMIT_PIN_TOP) != HIGH)
        return false; // 第一次不是 HIGH → 沒壓
    delayMicroseconds(150);
    return (digitalRead(LIMIT_PIN_TOP) == HIGH); // 第二次確認仍是 HIGH → 壓下
}
// -------------------------------------------------

void setup() {
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Z_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Z_DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    pinMode(LIMIT_PIN_TOP, INPUT_PULLUP);

    digitalWrite(ENABLE_PIN, HIGH); // 初始斷電
    Serial.begin(115200);
    // Serial.println("=== CNC Shield 自動控制啟動 (已修正) ===");
}

void loop() {
    readPico();
    runMotors();
}

// --- 從 RX0 讀取 Pico (*** 正確邏輯 ***) ---
void readPico() {
    while(Serial.available() > 0) {
        char c = Serial.read();
        if(c == '\n') {
            uartBuffer.trim();
            if(uartBuffer.length() > 0) {
                int ch6 = parseCH6(uartBuffer);

                // --- 邏輯修正：使用 "正確" 的函式檢查 ---
                bool topLimitPressed = isTopLimitPressed_Debounced();
                // Serial.println(topLimitPressed); // 除錯用，按壓時應顯示 1

                if(ch6 <= -41) { // 遙控器「往上」→ 平台往上
                    if(isTopLimitPressed_Debounced()) {
                        motorState = 'S'; // 壓到頂 → 停
                    } else {
                        motorState = 'H'; // 允許往上
                    }
                } else if(ch6 >= 41) { // 遙控器「往下」→ 平台往下
                    motorState = 'L';  // 往下永遠允許
                } else {
                    motorState = 'S';
                }
                // ----------------------------------------
            }
            uartBuffer = "";
        } else if(c >= 32 && c <= 126 && uartBuffer.length() < BUF_MAX) {
            uartBuffer += c;
        }
    }
}

// --- 簡單解析第六個逗號分隔值 (不變) ---
int parseCH6(String data) {
    int tokenIndex = 0;
    int lastComma = -1;
    for(int i = 0; i < data.length(); i++) {
        if(data[i] == ',') {
            tokenIndex++;
            lastComma = i;
            if(tokenIndex == 6)
                break;
        }
    }
    if(tokenIndex == 6) {
        String token = data.substring(lastComma + 1);
        return token.toInt();
    }
    return 0;
}

// --- 步進馬達控制 (*** 正確邏輯 ***) ---
void runMotors() {
    unsigned long now = micros();

    // --- 即時安全檢查：檢查 "運行中" 且 "向上 H" 時，是否撞到開關 ---
    if(motorState == 'H' && isTopLimitPressed_Debounced()) {
        motorState = 'S'; // 強制停止
        // Serial.println("!! 運行中觸發頂部限位，馬達停止 !!");
    }
    // ------------------------------------------------

    // 根據 "最終" 的 motorState 決定是否致能
    if(motorState == 'S') {
        digitalWrite(ENABLE_PIN, HIGH); // 斷電
        digitalWrite(X_STEP_PIN, LOW);
        digitalWrite(Y_STEP_PIN, LOW);
        digitalWrite(Z_STEP_PIN, LOW);
        return;
    } else {
        digitalWrite(ENABLE_PIN, LOW); // 致能
    }

    // --- 步進邏輯 (與原版相同) ---
    if(now - lastStepTime >= STEP_DELAY) {
        lastStepTime = now;
        stepState = !stepState;

        if(motorState == 'H') {
            digitalWrite(X_DIR_PIN, DIR_UP);
            digitalWrite(Y_DIR_PIN, DIR_UP);
            digitalWrite(Z_DIR_PIN, DIR_UP);
        } else if(motorState == 'L') {
            digitalWrite(X_DIR_PIN, DIR_DOWN);
            digitalWrite(Y_DIR_PIN, DIR_DOWN);
            digitalWrite(Z_DIR_PIN, DIR_DOWN);
        }

        digitalWrite(X_STEP_PIN, stepState);
        digitalWrite(Y_STEP_PIN, stepState);
        digitalWrite(Z_STEP_PIN, stepState);
    }
}