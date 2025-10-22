// 平台上升不進馬達程式

// 定義 XYZ 軸的 STEP 和 DIR 腳位
// CNC Shield V3 預設的腳位定義
const int stepPinX = 2; // X軸 步進腳位
const int dirPinX = 5;  // X軸 方向腳位
const int stepPinY = 3; // Y軸 步進腳位
const int dirPinY = 6;  // Y軸 方向腳位
const int stepPinZ = 4; // Z軸 步進腳位
const int dirPinZ = 7;  // Z軸 方向腳位
const int enablePin = 8;

// 設定步進脈衝的延遲時間（決定速度）
// 數值越小，速度越快。這裡設定 1000 微秒，即 1 毫秒
const int stepDelay = 500;

// 設定方向。HIGH 或 LOW 決定了馬達的旋轉方向。
// 您需要根據您的接線和馬達特性來測試哪個是「順時針」。
// 這裡先設為 HIGH，如果方向不對，請將此值改為 LOW。
const int motorDirection = HIGH; // HIGH 下降 LOW 上升

void setup() {
    // 設定所有腳位為 OUTPUT 模式
    pinMode(stepPinX, OUTPUT);
    pinMode(dirPinX, OUTPUT);
    pinMode(stepPinY, OUTPUT);
    pinMode(dirPinY, OUTPUT);
    pinMode(stepPinZ, OUTPUT);
    pinMode(dirPinZ, OUTPUT);
    pinMode(enablePin, OUTPUT);

    digitalWrite(enablePin, LOW);
    // 設定 XYZ 軸的旋轉方向
    digitalWrite(dirPinX, motorDirection);
    digitalWrite(dirPinY, motorDirection);
    digitalWrite(dirPinZ, motorDirection);

    // 初始化序列埠用於偵錯 (非必要，但建議)
    Serial.begin(9600);
    Serial.println("XYZ三軸馬達開始持續旋轉...");
}

void loop() {
    // 輸出一個脈衝（HIGH）
    digitalWrite(stepPinX, HIGH);
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinZ, HIGH);

    // 保持 HIGH 狀態一段時間
    delayMicroseconds(stepDelay);

    // 輸出第二個脈衝（LOW）
    digitalWrite(stepPinX, LOW);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinZ, LOW);

    // 保持 LOW 狀態一段時間，完成一個完整的步進脈衝
    delayMicroseconds(stepDelay);
}