#include <Arduino.h>
#include <SoftwareSerial.h>

/* 10.15c_keys.cpp —— 按键遥控板端【4 键哑板】（配主控 10.15e 起）
 * 代码与 10.15c_keys.ino 完全一致（仅补顶部 #include <Arduino.h>、注释精简过）。
 * 只发单字符 '1'~'4'（含义由主控端 F1/F2 模式决定，本板不解释）；上电发 '?' 自检，主控回 "+LINK OK" 即链路通。
 * 接线：D7(软串口 TX) → 主控 D10；GND ↔ GND ★必须共地。按键 1~4 = D2~D5 → GND（芯片内部上拉）。
 */

const int KEY_PIN[4]   = {2, 3, 4, 5};
const char KEY_CODE[4] = {'1', '2', '3', '4'};

const unsigned long DEBOUNCE_MS = 20;        // 去抖：电平稳定 20ms 才算数
SoftwareSerial armSerial(6, 7);              // RX=D6 悬空；TX=D7 → 主控 D10

struct KeyState {                            // 每个按键的去抖状态
  int  stable;
  int  lastRaw;
  unsigned long tChange;
};
KeyState gKey[4];

void setup() {
  Serial.begin(9600);
  armSerial.begin(9600);

  for (int i = 0; i < 4; i++) {
    pinMode(KEY_PIN[i], INPUT_PULLUP);       // INPUT_PULLUP：按下=LOW
    gKey[i].stable  = HIGH;
    gKey[i].lastRaw = HIGH;
    gKey[i].tChange = millis();
  }

  Serial.println("按键板上电：4 键哑板（含义由主控端 F1/F2 模式决定）");
  Serial.println("  D2 按键1   D3 按键2   D4 按键3   D5 按键4");
  Serial.println("  （各键当前含义去看主控串口监视器）");

  delay(300);                                // 等主控串口初始化完
  armSerial.write('?');
  Serial.println("已发送链路自检 ? —— 去主控的串口监视器看有没有 +LINK OK");
}

// 扫描按键：只在"按下"那一刻发一次字符（长按不连发）
void scanKey(int i) {
  int raw = digitalRead(KEY_PIN[i]);

  if (raw != gKey[i].lastRaw) {              // 电平刚变：重新计时
    gKey[i].lastRaw = raw;
    gKey[i].tChange = millis();
    return;
  }
  if (millis() - gKey[i].tChange < DEBOUNCE_MS) return;   // 还没稳
  if (raw == gKey[i].stable) return;                       // 无变化

  gKey[i].stable = raw;
  if (gKey[i].stable == LOW) {
    armSerial.write(KEY_CODE[i]);
    Serial.print("按键 ");
    Serial.print(KEY_CODE[i]);
    Serial.println(" 已发送");
  }
}

void loop() {
  for (int i = 0; i < 4; i++) scanKey(i);
}
