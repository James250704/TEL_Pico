// CNC Shield 平台升降控制程式 (修改版：僅 X 軸連續旋轉)
// 尚未加上遙控

#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define ENABLE_PIN 8 // Active LOW

#define DIR_TEST_DIRECTION LOW // LOW = DIR_UP, HIGH = DIR_DOWN

const int STEP_DELAY = 550; // 微秒，步進速度 (已加速)

bool stepState = LOW;
unsigned long lastStepTime = 0;

void setup() {
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(X_DIR_PIN, DIR_TEST_DIRECTION);
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(X_STEP_PIN, LOW);
    lastStepTime = micros();
}

void loop() {
    unsigned long now = micros();

    if(now - lastStepTime >= STEP_DELAY) {
        lastStepTime = now;
        stepState = !stepState;
        digitalWrite(X_STEP_PIN, stepState);
    }
}