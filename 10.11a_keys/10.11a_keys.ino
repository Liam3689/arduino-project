/* 10.11a_keys.ino  ——  十二期二面 · 任务三 的【按键遥控板端】
 * 板子：UNO 兼容开发板（ATmega328P / ATmega328PB 均可，面包板搭建）
 *
 * ============================================================================
 * 2026-10-11 决策记录（为什么不叫 esp8266_keys 了）：
 *   新版考核 PDF（第 5 页，任务三共 25%）原文："借助一块电路组成员自制 Arduino 按键
 *   遥控板……注意：若硬件同学完成相应任务，则必须使用硬件同学画的板子；若硬件同学
 *   未能完成任务，则需自行购买现成开发板 使用面包板进行相应操作（见附件 1）。"
 *   → 电路方向未能完成自制板，按文档走"自行购买现成开发板 + 面包板"，
 *     新任务里已经没有 esp8266 了，所以旧版 esp8266_keys.ino（Serial1/NodeMCU 引脚）作废，
 *     本版按 UNO 兼容板重写。旧版保留在 10.7c/esp8266_keys/ 备查，勿再烧录。
 *
 * 【任务三要求原文】（新版 PDF 第 5 页，任务三 25%）
 *   遥控板负责捕获并解析 4 个按键的输入信号，将对应的控制指令通过串口发送至
 *   机械臂上的 Arduino；Arduino 收到指令后驱动机械臂执行相应动作：
 *     按键 1（10%）→ 循环夹取 A/B/C    按键 2 → 录制开/关（不单独计分，但键 3 依赖它）
 *     按键 3（10%）→ 播放              按键 4（5%）→ 回中
 *   原文说"相应引脚配置需询问电路方向的同学"——电路组未参与，引脚由本文件自定义。
 * ============================================================================
 *
 * 【两块板之间只有 2 根线】
 *      按键遥控板（UNO 兼容板）              Arduino Uno（主控 + 机械臂）
 *        D7 (软串口 TX)  ──────────────→      D10 (软串口 RX)
 *        GND ──────────────────────────→      GND     ★ 必须共地！不共地必然乱码
 *
 *   为什么用软串口不用硬件串口？
 *     UNO 兼容板只有一组硬件 UART（D0/D1），被 USB 下载/调试占着；
 *     拔了 USB 就没法看调试信息，所以链路走 SoftwareSerial(D6,D7)，
 *     主控那头本来就是 SoftwareSerial(D10)，两边对得上。9600 波特率足够稳。
 *
 * 【按键接线】（一端接 GPIO，另一端接 GND，用芯片内部上拉，面包板不用加电阻）
 *       按键 1 → D2      按键 2 → D3
 *       按键 3 → D4      按键 4 → D5
 *   ⚠ 不用 D0/D1（硬件串口）、不用 D6/D7（软串口占用）。
 *
 * 【协议：为什么只发一个字符，不发 "PICK_A\n" 这种长指令】
 *   1) 单字符不会被"粘包"：长指令要靠换行符判断边界，丢一个字节整条作废；
 *      单字符每次都是完整的一条，丢了就丢了，下一条不受影响。
 *   2) 两边代码都简单：这边一行 write('1')，那边一个分派就完事。
 *   3) 8 位机 + 9600 波特率，越短越省。
 *
 * 【上电自检】
 *   本板上电先发一个 '?' 给主控。主控收到后只回一句 "+LINK OK"、不动机械臂。
 *   → 先给主控上电、再给本板上电，看主控串口监视器出现 "+LINK OK" 即链路（含共地）通。
 */

#include <SoftwareSerial.h>

// ---------- 引脚 ----------
// 按键：一端接 GPIO，一端接 GND。INPUT_PULLUP，"按下"读出来是 LOW。
const int KEY_PIN[4]   = {2, 3, 4, 5};       // D2 / D3 / D4 / D5
const char KEY_CODE[4] = {'1', '2', '3', '4'};

// ---------- 去抖参数 ----------
// 机械按键按下/弹起瞬间会"弹跳"几下，几十微秒里电平乱跳，
// 不处理会一次按下发出好几个字符。做法：等电平稳定 20ms 才算数。
const unsigned long DEBOUNCE_MS = 20;

// ---------- 发射串口 ----------
// SoftwareSerial(RX脚, TX脚)：RX=D6 悬空不接（本板只发不收），TX=D7 → 主控 D10
SoftwareSerial armSerial(6, 7);

// 每个按键的去抖状态。用结构体把小账本捆在一起，比散着写 12 个变量清楚得多。
struct KeyState {
  int  stable;              // 已经确认过的电平
  int  lastRaw;             // 上一次读到的原始电平
  unsigned long tChange;    // 原始电平最后一次变化的时间
};
KeyState gKey[4];

void setup() {
  Serial.begin(9600);                        // USB 调试口（电脑串口监视器看）
  armSerial.begin(9600);                     // 和主控对话，接主控 D10；两边必须同波特率

  for (int i = 0; i < 4; i++) {
    pinMode(KEY_PIN[i], INPUT_PULLUP);       // 不按 = HIGH，按下 = LOW
    gKey[i].stable  = HIGH;
    gKey[i].lastRaw = HIGH;
    gKey[i].tChange = millis();
  }

  Serial.println("按键板上电：4 个按键就绪");
  Serial.println("  D2 按键1 循环夹取 A/B/C");
  Serial.println("  D3 按键2 录制 开/关");
  Serial.println("  D4 按键3 播放");
  Serial.println("  D5 按键4 回中");

  // 上电先喊一声，让主控回一句 "+LINK OK"，用来验证那根线和共地
  delay(300);                                // 等主控把自己的串口初始化完
  armSerial.write('?');
  Serial.println("已发送链路自检 ? —— 去主控的串口监视器看有没有 +LINK OK");
}

// 扫描一个按键：等电平稳定 DEBOUNCE_MS 之后，只在"按下"那一刻发一次字符。
// 长按不会连续触发，因为 stable 变了之后就不再进这个分支了。
void scanKey(int i) {
  int raw = digitalRead(KEY_PIN[i]);

  if (raw != gKey[i].lastRaw) {              // 电平刚变，重新开始计时
    gKey[i].lastRaw = raw;
    gKey[i].tChange = millis();
    return;
  }
  if (millis() - gKey[i].tChange < DEBOUNCE_MS) return;   // 还没稳，再等等
  if (raw == gKey[i].stable) return;                       // 稳定值和现在一样，没新鲜事

  gKey[i].stable = raw;                      // 确认了一个新的稳定电平
  if (gKey[i].stable == LOW) {               // 只认"按下"，不认"松手"
    armSerial.write(KEY_CODE[i]);
    Serial.print("按键 ");
    Serial.print(KEY_CODE[i]);
    Serial.println(" 已发送");
  }
}

void loop() {
  for (int i = 0; i < 4; i++) scanKey(i);
}
