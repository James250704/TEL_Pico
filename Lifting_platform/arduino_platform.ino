// CNC Shield 平台升降控制程式

#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7
#define ENABLE_PIN 8 // Active LOW

#define DIR_UP HIGH
#define DIR_DOWN LOW

const int STEP_DELAY = 500; // 微秒，步進速度
bool stepState = LOW;
unsigned long lastStepTime = 0;
char motorState = 'S'; // H / S / L

String uartBuffer = "";
const int BUF_MAX = 60;

void setup() {
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Z_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Z_DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(ENABLE_PIN, HIGH); // 初始斷電

    Serial.begin(115200);
    while(!Serial)
        ;
    Serial.println("=== CNC Shield 自動控制啟動 ===");
}

void loop() {
    readPico();
    runMotors();
}

// --- 從 RX0 讀取 Pico 並解析 CH6 ---
void readPico() {
    while(Serial.available() > 0) {
        char c = Serial.read();
        if(c == '\n') {
            uartBuffer.trim();
            if(uartBuffer.length() > 0) {
                int ch6 = parseCH6(uartBuffer);
                // Serial.print("CH6=");
                // Serial.println(ch6);

                if(ch6 <= -21)
                    motorState = 'H';
                else if(ch6 >= 21)
                    motorState = 'L';
                else
                    motorState = 'S';
            }
            uartBuffer = "";
        } else if(c >= 32 && c <= 126 && uartBuffer.length() < BUF_MAX) {
            uartBuffer += c;
        }
    }
}

// --- 簡單解析第六個逗號分隔值 ---
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

// --- 步進馬達控制 ---
void runMotors() {
    unsigned long now = micros();

    if(motorState == 'S') {
        digitalWrite(ENABLE_PIN, HIGH);
        digitalWrite(X_STEP_PIN, LOW);
        digitalWrite(Y_STEP_PIN, LOW);
        digitalWrite(Z_STEP_PIN, LOW);
        return;
    } else {
        digitalWrite(ENABLE_PIN, LOW);
    }

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