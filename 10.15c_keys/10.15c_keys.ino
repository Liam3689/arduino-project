/* 10.15c_keys.ino  ——  十二期二面 · 按键遥控板端【4 键哑板】（配主控固件 10.15c 起；键位注释已同步 10.15e）
 * 板子：UNO 兼容开发板（ATmega328P / ATmega328PB 均可，面包板搭建）
 *
 * ============================================================================
 * 2026-10-15 定稿（用户拍板）：按键板保持 **4 个物理键的哑板** —— 只负责把按键
 *   变成单字符 '1'~'4' 发给主控，**没有任何模式逻辑**。
 *   模式切换在【主控板】的串口监视器里发 F1 / F2（主控 10.15e 起只有两档）：
 *     模式 1 = 任务三键位：按键1/2/3/4 = 夹取循环 / 录制 / 播放 / 回中
 *     模式 2 = 示教（任务五(3)(4) 共用）：按键1 = 暂停/继续（一键切换），按键2 = 取消，
 *              按键3 = 统一记点（记满 5 点自动清空重记）、按键4 = 开始绘制
 *   → 换模式不用动这块板、不用重新烧录，两根线（D7→主控D10、GND）也不变。
 *   ⚠ 考核原文要求暂停、继续"分别使用按键触发"：模式 2 的一键切换是 4 键硬件的
 *     妥协；正式演示想分开按，直接在主控 USB 敲 J（暂停）/ K（继续）即可，
 *     主控端这两条命令任何模式下都独立可用。
 *   ⚠ 本文件相对 10.15c 版**只改了本注释块**（同步 10.15e：模式 2 由"过程控制"
 *     改为"示教"、独立的模式 3 已删），代码零变化 —— 已烧录的板子无需重烧。
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
 * 【协议：为什么只发一个字符】
 *   单字符不会被"粘包"：每次都是完整的一条，丢了就丢了，下一条不受影响；
 *   两边代码都简单；8 位机 + 9600 波特率，越短越省。
 *   字符含义（暂停/继续/取消）由主控端决定，本板不解释。
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

  Serial.println("按键板上电：4 键哑板（含义由主控端 F1/F2 模式决定）");
  Serial.println("  D2 按键1   D3 按键2   D4 按键3   D5 按键4");
  Serial.println("  （各键当前含义去看主控串口监视器）");

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
