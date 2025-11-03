// CNC Shield 平台升降控制程式 (修正 EMI 噪音版)

#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7
#define ENABLE_PIN 8 // Active LOW

#define LIMIT_PIN_TOP 11 // Z-Limit 接腳 (D11)

#define DIR_UP HIGH
#define DIR_DOWN LOW

const int STEP_DELAY = 500; // 微秒，步進速度
bool stepState = LOW;
unsigned long lastStepTime = 0;
char motorState = 'S'; // H (上) / S (停) / L (下)

String uartBuffer = "";
const int BUF_MAX = 60;

// --- 新增：抗 EMI 噪音的讀取函式 ---
// 透過短暫延遲和二次確認來過濾噪音
bool isTopLimitPressed_Debounced() {
    // 第一次讀取
    if(digitalRead(LIMIT_PIN_TOP) == HIGH) {
        return false; // 如果是 HIGH，肯定未按壓
    }

    // 第一次讀到 LOW，可能是雜訊
    delayMicroseconds(150); // 關鍵：等待 150 微秒，讓噪音過去

    // 第二次讀取
    return (digitalRead(LIMIT_PIN_TOP) == LOW); // 以第二次的讀取為準
}
// ------------------------------------

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
    // Serial.println("=== CNC Shield 自動控制啟動 (抗EMI版) ===");
}

void loop() {
    readPico();
    runMotors();
}

// --- 從 RX0 讀取 Pico (*** 此函式已修改 ***) ---
void readPico() {
    while(Serial.available() > 0) {
        char c = Serial.read();
        if(c == '\n') {
            uartBuffer.trim();
            if(uartBuffer.length() > 0) {
                int ch6 = parseCH6(uartBuffer);

                // --- 邏輯修改：使用 "抗噪音" 函式檢查 ---
                bool topLimitPressed = isTopLimitPressed_Debounced();

                // 這是您加入的除錯行
                // Serial.println(topLimitPressed);

                if(ch6 <= -21) {
                    if(topLimitPressed) {
                        motorState = 'S';
                    } else {
                        motorState = 'H';
                    }
                } else if(ch6 >= 21) {
                    motorState = 'L';
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

// --- 步進馬達控制 (*** 此函式已修改 ***) ---
void runMotors() {
    unsigned long now = micros();

    // --- 即時安全檢查：使用 "抗噪音" 函式 ---
    if(motorState == 'L' && isTopLimitPressed_Debounced()) {
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