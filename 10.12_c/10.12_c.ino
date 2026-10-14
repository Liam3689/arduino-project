/* 10.13_c  MeArm 机械臂 · 任务一摇杆 + 任务二 A/B/C + 任务三按键板 + 任务五(1) 直线绘制
 *
 * 命令表：  x90,y30,z20  三轴协同(底座/前臂/后臂)      O / S   开爪 / 合爪
 *          H / L        提速 / 降速(臂和夹爪一起调)    A/B/C   自动夹取-放置
 *          M90,45,60,30 四轴直接给角度(标定)           P       打印当前角度
 *          G            回中                          T       急停
 *          1 / 2 / 3 / 4  任务三按键功能(USB 口直接敲数字同效)   5  打印录制内容(联调)
 *          6 / 7 / 8      任务五：记起点 / 记终点 / 自动画直线(只能电脑串口敲)
 *          I120,0,30      笔尖坐标单点测试(mm，IK 标定用)        ?  打印命令表
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// 轴序：0=底座base(9)  1=前臂fArm(8)  2=后臂rArm(7)  3=夹爪claw(6)
Servo gServo[4];
const int PIN[4] = {9, 8, 7, 6};
const char *AXIS_NAME[4] = {"base", "fArm", "rArm", "claw"};   // 打印轴名用

// 安全限位（通电前务必手盘复核全行程，实测不同就改这里）
const int LIM_MIN[4] = {  0, 0, 70,  30};
const int LIM_MAX[4] = {180, 140, 160, 110};

// 速度表：走 1 度花多少毫秒（越大越慢）；H/L 同时切换下面两张表的档位
int DSDdata[4]  = {28, 18, 13, 8};    // 臂三轴
int CLAW_DSD[4] = {25, 21, 18, 15};   // 夹爪专用，比臂慢，防甩飞物体
int DSDlevel = 2;

// 夹爪开合角（若 O/S 动作反了，先改这两行、别去动 gObj 表）
const int CLAW_OPEN    = 50;    // 开爪（小角度）
const int CLAW_CLOSE   = 105;   // 合爪（大角度）

// 软件角度镜像：记住"命令到哪了"，作为插值起点
int gCur[4] = {90, 90, 90, 90};

// ---- 任务五 几何常量（mm，本组装配体实测）----
// 平行四连杆 ⇒ 腕→笔杆恒水平，故夹爪前伸是常量、不随姿态变；笔尖在笔杆轴线正下方 GEO_PEN 处
//   tipR = GEO_SH_R + GEO_L1*sin(aU) + GEO_L2*sin(aF) + GEO_CLAW
//   tipZ = GEO_L0   + GEO_L1*cos(aU) + GEO_L2*cos(aF) - GEO_PEN
// aU / aF = 大臂 / 小臂 相对"竖直向上"的关节角，向前倾为正（弧度）
float GEO_SH_R = 10.3;  // 肩关节轴 相对 底座轴 的水平前移
float GEO_L0   = 55.6;  // 桌面 → 肩关节轴 的垂直高度
float GEO_L1   = 82.8;  // 大臂：肩 → 肘
float GEO_L2   = 79.9;  // 小臂：肘 → 腕
float GEO_CLAW = 42.0;  // 腕 → 笔杆轴线 的水平前伸（夹爪恒水平，常量）
float GEO_PEN  = 45.0;  // 腕 → 笔尖 的垂直落差（装一次笔测一次）

// ---- 任务五 标定量（舵机角 ↔ 关节角，装一次标一次）----
// 关节角(弧度) = (舵机角 − CAL_*_ZERO) × CAL_*_DIR × π/180
// ⚠ 靠 I 命令的"回算残差" + 尺量微调，不要乱翻
float CAL_BASE_DIR   =  1.0;  // base：舵机角增大 = 从上方看逆时针
float CAL_BASE_ZERO  = 90.0;  // base 90 = 笔尖指向正前方
float CAL_UPPER_DIR  =  1.0;  // 大臂(rArm 下标2)：舵机角增大 = 向下压
float CAL_UPPER_ZERO = 90.0;  // 大臂 90 = 竖直（aU = 0）
float CAL_FORE_DIR   = -1.0;  // 小臂(fArm 下标1)：舵机角增大 = 向上抬
float CAL_FORE_ZERO  =180.0;  // 小臂 180 = 水平前伸（aF = 90°）
float CAL_ELBOW_DIR  =  1.0;  // 肘部解选型（装配体姿态反算）

// ---- 任务五 画线参数 ----
float DRAW_STEP_MM    = 2.0;   // 直线插值步长（直线度差就改 1.0）
float DRAW_LIFT_MM    = 15.0;  // 抬笔高度
float DRAW_MS_PER_DEG = 60.0;  // 每 1 度花多少 ms（别小于 20）

// 画线状态（声明放前面，emergencyStop 里要用）
bool  drawActive = false;      // 正在自动画线
bool  drawHas[2] = {false, false};
float drawTip[2][3];           // [0]=起点 [1]=终点（笔尖 mm）
int   drawIdx   = 0;           // 已画到第几个插值点
int   drawTotal = 0;           // 插值点总数
int   drawPhase = 0;           // 1=移到起点上方 2=落笔 3=画线中 4=收尾抬笔

// 一个关键点 = 四轴角度 + 停留时间
struct KeyPoint {
  int a[4];     // base, fArm, rArm, claw   ← 顺序按轴序，别写反
  int dwell;    // 到位后停留，单位毫秒
};

// A/B/C 关键点表（0=A 1=B 2=C）；行尾注释写的是"这个点该干什么"，当填空模板用
KeyPoint gObj[3][12] = {
  { // 物体 A：取 base=90，放 base=135
    {{ 90, 90, 90, 90}, 300},   // 0 待机
    {{ 123,90,90,45}, 300},   // 1 物体正上方
    {{ 123,35,140,45}, 400},   // 2 下探到夹取位
    {{ 123,35,140,110}, 500},   // 3 合爪夹住
    {{ 123,90,90,110}, 300},   // 4 抬起
    {{ 65, 90, 90, 110}, 300},   // 5 转到放置点上方
    {{ 65, 40, 150, 110}, 400},   // 6 下探放置
    {{ 65, 40, 150, 45}, 500},   // 7 开爪
    {{ 65, 90, 90, 45}, 300},   // 8 抬起
    {{ 90, 90, 90, 90}, 300}    // 9 回中
  },
  { // 物体 B：取 base=45，放 base=90
    {{ 90, 90, 90, 90}, 300},   // 0 待机
    {{ 80, 90, 70, 45}, 300},   // 1 物体正上方
    {{ 80,70,120,45}, 400},   // 2 到夹取位
    {{ 80,70,120,108}, 500},   // 3 合爪夹住
    {{ 80, 90, 90, 108}, 300},   // 4 抬起
    {{ 130, 90, 90, 108}, 300},   // 5 转到放置点上方
    {{ 130, 70, 120, 108}, 400},   // 6 放置
    {{ 130, 70, 120, 45}, 500},   // 7 开爪
    {{ 130, 90, 90, 45}, 300},   // 8 抬起
    {{ 90, 90, 90, 90}, 300}    // 9 回中
  },
  { // 物体 C：取 base=135，放 base=45
    {{ 90, 90, 90, 90}, 300},   // 0 待机
    {{ 105, 90, 90, 45}, 300},   // 1 物体正上方
    {{ 105, 40, 140, 45}, 400},   // 2 下探到夹取位
    {{ 105, 40, 140, 108}, 500},   // 3 合爪夹住
    {{ 105, 90, 90, 108}, 300},   // 4 抬起
    {{ 60, 90, 90, 108}, 300},   // 5 转到放置点上方
    {{ 60, 30, 135, 108}, 400},   // 6 下探放置
    {{ 60, 30, 135, 45}, 500},   // 7 开爪
    {{ 60, 90, 90, 45}, 300},   // 8 抬起
    {{ 90, 90, 90, 90}, 300}    // 9 回中
  }
};
int gObjLen[3] = {10, 10, 10};   // 各物体实际关键点数（必须 ≤ 12）

String inputBuffer = "";

// 第二路串口，接按键板：RX=D10，TX=D11
SoftwareSerial espSerial(10, 11);

// 摇杆 4 通道 ADC（A0~A3 分控 4 根轴，与舵机 D6~D9、软串口 D10/D11 不冲突）
const int JOY_PIN[4] = {A0, A1, A2, A3};   // 下标和轴序对齐：0=base 1=fArm 2=rArm 3=claw

const int JOY_DEADZONE = 60;               // 死区：偏离中点这么多就当成没动
const int JOY_PERIOD_MS = 20;              // 每 20ms 推 1 度
const int JOY_SAMPLES = 30;                // 上电采多少次求中点

int  joyCenter[4] = {512, 512, 512, 512};  // 每路的中点（上电实测覆盖）
unsigned long joyLastMs = 0;

// 录制缓冲：200ms 一帧，最多 100 帧 = 20 秒
const int REC_MAX       = 100;
const int REC_PERIOD_MS = 200;
byte recData[REC_MAX][4];
int  recLen = 0;                    // 当前已录帧数（也是"上一次录制"的成果）
bool recording = false;
unsigned long recLastMs = 0;
unsigned long recStartMs = 0;       // 本段录制的起始时刻（用来核对时长）

const int REC_MIN_MS = 10000;       // 合格线：时长大于 10 秒
const int REC_MIN_TRAVEL = 15;      // 合格线：能体现明显位移

// 播放状态
bool playing = false;
int  playIdx = 0;                   // 已经播到第几帧
unsigned long playStartMs = 0;      // 起播时刻（按时间轴走的基准）

// 按键 1 循环计数器：0=A 1=B 2=C，每按一次往后走一格
int cycleObj = 0;

// 限位检查：4 轴逐个查，越限就打印并返回 false
bool withinLimits(int a[4]) {
  for (int i = 0; i < 4; i++) {
    if (a[i] < LIM_MIN[i] || a[i] > LIM_MAX[i]) {
      Serial.print(F("越限："));
      Serial.print(AXIS_NAME[i]);
      Serial.print(F("["));
      Serial.print(i);
      Serial.print(F("] 值 "));
      Serial.print(a[i]);
      Serial.print(F(" 不在 ["));
      Serial.print(LIM_MIN[i]);
      Serial.print(F(","));
      Serial.print(LIM_MAX[i]);
      Serial.println(F("] 内"));
      return false;
    }
  }
  return true;
}

// 非阻塞运动引擎：把"从当前角度走到目标角度"打包成一段(Seg)塞进队列，
// 每 stepMs 走 1 度，由 loop() 反复调 motionTick() 推进，函数立刻返回。
// mask 位图决定这一段动哪几个轴（0b0111 三轴 / 0b1111 四轴 / 单个位 = 单轴）。
const int SEG_MAX = 12;          // 队列容量

struct Seg {
  int mask;                      // 这一段动哪几个轴（位图）
  int to[4];                     // 各轴目标角度
  int stepMs;                    // 每走 1 度花多少毫秒
};

Seg segQ[SEG_MAX];
int segHead = 0, segTail = 0, segCount = 0;

bool segOn = false;              // 当前是否有一段正在执行
int  runMask = 0;
int  runFrom[4], runTo[4];
int  runTotal = 0, runStep = 0, runStepMs = 1;
unsigned long runLastMs = 0;

// 入队：按 mask 一次动多个轴
void addSegMask(int mask, int to[4], int stepMs) {
  if (segCount >= SEG_MAX) {
    Serial.println(F("警告：动作队列满，这一段被丢弃"));
    return;
  }
  segQ[segTail].mask = mask;
  for (int i = 0; i < 4; i++) segQ[segTail].to[i] = to[i];
  segQ[segTail].stepMs = stepMs;
  segTail = (segTail + 1) % SEG_MAX;
  segCount++;
}

// 入队：只动一个轴（其余轴取当前值，等于不动）
void addSeg(int axis, int to, int stepMs) {
  int t[4] = {gCur[0], gCur[1], gCur[2], gCur[3]};
  t[axis] = to;
  addSegMask(1 << axis, t, stepMs);
}

// 还有没有没走完的动作（段在执行，或队列里还压着）
bool motionBusy() {
  return segOn || segCount > 0;
}

// 从队列取一段开始执行
bool runStartNext() {
  while (segCount > 0) {
    Seg s = segQ[segHead];
    segHead = (segHead + 1) % SEG_MAX;
    segCount--;

    runMask   = s.mask;
    runStepMs = s.stepMs;
    runTotal  = 0;
    for (int i = 0; i < 4; i++) {
      runFrom[i] = gCur[i];
      runTo[i]   = s.to[i];
      if (runMask & (1 << i)) {
        int d = abs(runTo[i] - runFrom[i]);
        if (d > runTotal) runTotal = d;    // 步数 = 各轴跨度的最大值
      }
    }
    if (runTotal == 0) continue;           // 已经在该位姿，跳过这一段

    runStep   = 0;
    runLastMs = millis();
    segOn     = true;
    return true;
  }
  return false;
}

// 走一步
void motionTick() {
  if (!segOn) {
    runStartNext();
    return;                                // 刚起步，等 stepMs 后再走第一度
  }
  if (millis() - runLastMs < (unsigned long)runStepMs) return;
  runLastMs = millis();

  runStep++;
  for (int i = 0; i < 4; i++) {
    if (!(runMask & (1 << i))) continue;
    int v = runFrom[i] + (int)((long)(runTo[i] - runFrom[i]) * runStep / runTotal);
    gServo[i].write(v);
    gCur[i] = v;
  }
  if (runStep >= runTotal) segOn = false;
}

// 三轴比例插值：底座/前臂/后臂 同时出发、同时到位
// 防"夹爪横扫桌面"靠 gObj 表里自己安排高位过渡点，不靠"先动哪根轴"
void moveArm(int to[4], int stepMs) {
  addSegMask(0b0111, to, stepMs);
}

// 夹爪：单独慢速移动
void moveClaw(int target) {
  if (target < LIM_MIN[3]) target = LIM_MIN[3];   // 兜底，防越限
  if (target > LIM_MAX[3]) target = LIM_MAX[3];
  addSeg(3, target, CLAW_DSD[DSDlevel]);
}

// 耗时预估：按和运动引擎完全相同的规则算一遍，不实际运动
int estMs(int from, int to, int stepMs) {
  return abs(to - from) * stepMs;
}

// 三轴同时（x/y/z、M、A/B/C、G）：步数取三轴跨度最大值
int estSyncMs(int toB, int toF, int toR) {
  int m = abs(toB - gCur[0]);
  if (abs(toF - gCur[1]) > m) m = abs(toF - gCur[1]);
  if (abs(toR - gCur[2]) > m) m = abs(toR - gCur[2]);
  return m * DSDdata[DSDlevel];
}

// 整条序列：用临时角度镜像模拟走一遍，把每段 + 停留累加
int estSeqMs(int obj) {
  int c[4];
  for (int i = 0; i < 4; i++) c[i] = gCur[i];
  int dsd  = DSDdata[DSDlevel];
  int cdsd = CLAW_DSD[DSDlevel];
  int ms = 0;
  for (int k = 0; k < gObjLen[obj]; k++) {
    int *a = gObj[obj][k].a;
    int m = abs(a[0] - c[0]);
    if (abs(a[1] - c[1]) > m) m = abs(a[1] - c[1]);
    if (abs(a[2] - c[2]) > m) m = abs(a[2] - c[2]);
    ms += m * dsd;
    for (int i = 0; i < 3; i++) c[i] = a[i];
    if (a[3] != c[3]) { ms += estMs(c[3], a[3], cdsd); c[3] = a[3]; }
    ms += gObj[obj][k].dwell;
  }
  return ms;
}

// 只打印到 0.1 秒，避开浮点
void printEst(int ms) {
  Serial.print(F("预计耗时 "));
  Serial.print(ms / 1000);
  Serial.print(F("."));
  Serial.print((ms % 1000) / 100);
  Serial.println(F(" 秒"));
}

// busy = true 表示"有动作或序列正在执行"，此时拒绝新的运动指令
bool busy = false;

bool rejectWhileBusy(const char *what) {
  if (!busy) return false;
  Serial.print(what);
  Serial.println(F("：机械臂正忙，指令已忽略（等 done 再发，或发 T 急停）"));
  return true;
}

// 录制 / 播放 / 画线中，拒绝"再来一段自动动作"（手动命令不受这条拦）
bool rejectWhileAuto(const char *what) {
  if (playing) {
    Serial.print(what);
    Serial.println(F("：正在播放录制内容，先等播完或发 T 急停"));
    return true;
  }
  if (recording) {
    Serial.print(what);
    Serial.println(F("：正在录制中，先按 2 结束录制或发 T 急停"));
    return true;
  }
  if (drawActive) {
    Serial.print(what);
    Serial.println(F("：正在画线，先等画完或发 T 急停"));
    return true;
  }
  return rejectWhileBusy(what);
}

// 摇杆上电校准：实测每一路的中点（每路采 30 次求平均）
// ⚠ 上电那一刻不要碰摇杆
void joyCalibrate() {
  for (int i = 0; i < 4; i++) joyCenter[i] = 0;
  for (int k = 0; k < JOY_SAMPLES; k++) {
    for (int i = 0; i < 4; i++) joyCenter[i] += analogRead(JOY_PIN[i]);
    delay(2);
  }
  Serial.print(F("摇杆中点实测："));
  for (int i = 0; i < 4; i++) {
    joyCenter[i] /= JOY_SAMPLES;
    Serial.print(joyCenter[i]);
    if (i < 3) Serial.print(F(","));
  }
  Serial.println(F("（上电时手要离开摇杆）"));
}

// 每 20ms 读一次，偏离中点就推 1 度
// （增量式：推多少走多少、松手就停；只排一段不直接写舵机；自动动作在跑时整个让路）
void joyTick() {
  if (busy || playing) return;
  if (millis() - joyLastMs < (unsigned long)JOY_PERIOD_MS) return;
  joyLastMs = millis();

  int to[4];
  int mask = 0;
  for (int i = 0; i < 4; i++) {
    to[i] = gCur[i];
    int d = analogRead(JOY_PIN[i]) - joyCenter[i];
    if (abs(d) <= JOY_DEADZONE) continue;               // 在死区内，当没动

    int t = gCur[i] + (d > 0 ? -1 : 1);                 // 推 1 度
    if (t < LIM_MIN[i] || t > LIM_MAX[i]) continue;     // 顶到限位就停住，不硬怼

    to[i] = t;
    mask |= (1 << i);
  }
  if (mask == 0) return;
  addSegMask(mask, to, 1);                              // 排一段，1ms 就走完
}

// 序列状态机：非阻塞执行 A/B/C
//   seqLoadPoint() 把本点的动作丢进队列立刻返回；seqTick() 等走完 → 计时停留 → 下一个点
bool seqActive   = false;
int  seqObj      = -1;
int  seqIdx      = 0;
bool seqDwelling = false;
unsigned long seqDwellUntil = 0;

void seqFinish(bool ok) {
  if (ok) Serial.println(F("done"));
  seqActive   = false;
  seqObj      = -1;
  seqIdx      = 0;
  seqDwelling = false;
  busy        = false;                 // 必须解锁
}

void seqLoadPoint() {
  int *a = gObj[seqObj][seqIdx].a;

  if (!withinLimits(a)) {
    Serial.print(F("关键点 #"));
    Serial.print(seqIdx);
    Serial.println(F(" 越限，动作中止"));
    seqFinish(false);
    return;
  }

  Serial.print(F("  ["));
  Serial.print(seqIdx + 1);
  Serial.print(F("/"));
  Serial.print(gObjLen[seqObj]);
  Serial.print(F("] b=")); Serial.print(a[0]);
  Serial.print(F("  f=")); Serial.print(a[1]);
  Serial.print(F("  r=")); Serial.println(a[2]);

  moveArm(a, DSDdata[DSDlevel]);                   // 臂三轴：同时插值到位
  if (a[3] != gCur[3]) moveClaw(a[3]);             // 爪：按需单独慢速
  seqDwelling = false;                             // 交给 seqTick 等运动走完
}

void seqAdvance() {
  seqIdx++;
  if (seqIdx >= gObjLen[seqObj]) { seqFinish(true); return; }
  seqLoadPoint();
}

void seqTick() {
  if (!seqActive) return;

  if (!seqDwelling) {
    if (motionBusy()) return;                      // 本点的段还没走完，继续等
    int d = gObj[seqObj][seqIdx].dwell;
    if (d <= 0) { seqAdvance(); return; }
    seqDwellUntil = millis() + (unsigned long)d;
    seqDwelling   = true;
    return;
  }

  // 无符号相减再转 signed 比较，是 millis() 溢出安全的惯用写法
  if ((long)(millis() - seqDwellUntil) >= 0) seqAdvance();
}

// 回放某个物体的动作序列（非阻塞：只投递，交给 seqTick 推进）
void runSequence(int obj) {
  if (obj < 0 || obj > 2) return;
  if (gObjLen[obj] <= 0 || gObjLen[obj] > 12) {
    Serial.print(F("物体 "));
    Serial.print((char)('A' + obj));
    Serial.println(F(" 没有关键点"));
    return;
  }

  seqActive   = true;
  seqObj      = obj;
  seqIdx      = 0;
  seqDwelling = false;
  busy        = true;

  Serial.print(F("执行物体 "));
  Serial.println((char)('A' + obj));
  printEst(estSeqMs(obj));
  seqLoadPoint();
}

// 上电自检：把 gObj 里所有关键点都过一遍限位
void selfCheck() {
  int bad = 0;
  for (int o = 0; o < 3; o++) {
    for (int k = 0; k < gObjLen[o]; k++) {
      int *a = gObj[o][k].a;
      for (int i = 0; i < 4; i++) {
        if (a[i] < LIM_MIN[i] || a[i] > LIM_MAX[i]) {
          Serial.print(F("自检：物体 "));
          Serial.print((char)('A' + o));
          Serial.print(F(" 关键点 #"));
          Serial.print(k);
          Serial.print(F(" 轴"));
          Serial.print(AXIS_NAME[i]);
          Serial.print(F("["));
          Serial.print(i);
          Serial.print(F("]="));
          Serial.print(a[i]);
          Serial.print(F(" 越限 ["));
          Serial.print(LIM_MIN[i]);
          Serial.print(F(","));
          Serial.print(LIM_MAX[i]);
          Serial.println(F("]"));
          bad++;
        }
      }
    }
  }
  if (bad == 0) {
    Serial.println(F("自检：A/B/C 全部关键点都在限位内，OK"));
  } else {
    Serial.print(F("自检：共 "));
    Serial.print(bad);
    Serial.println(F(" 处越限，请先改上面的 gObj 表"));
  }
}

// 急停：立刻停住并清空所有待执行动作（同时中止序列 / 播放 / 画线 / 录制）
void emergencyStop() {
  segHead = segTail = segCount = 0;    // 清空动作队列
  segOn = false;                       // 停住当前这一段
  seqActive = false;                   // 中止正在跑的序列
  seqDwelling = false;
  playing = false;                     // 中止播放
  if (drawActive) {                    // 画线中急停 → 连画线状态一起清
    drawActive = false;
    drawPhase  = 0;
    Serial.println(F("画线已中止（起终点示教记录保留）"));
  }
  if (recording) {                     // 录制中急停 → 顺手停掉录制，保住已录的帧
    recording = false;
    Serial.print(F("录制已中止，保住 "));
    Serial.print(recLen);
    Serial.println(F(" 帧"));
  }
  busy = false;                        // 解锁，之后可以正常发指令
  Serial.println(F("急停：已停住并清空待执行动作"));
}

// 打印当前四轴角度（标定采参数用）
void printAngles() {
  Serial.print(F("P "));
  for (int i = 0; i < 4; i++) {
    Serial.print(gCur[i]);
    if (i < 3) Serial.print(F(","));
    else       Serial.println();
  }
}

// 回中：四轴回 90（臂三轴同时插值，最后关爪到位）
void goHome() {
  printEst(estSyncMs(90, 90, 90) + estMs(gCur[3], 90, CLAW_DSD[DSDlevel]));
  int to[4] = {90, 90, 90, 90};
  moveArm(to, DSDdata[DSDlevel]);
  moveClaw(90);
  Serial.println(F("回中"));
}

// 按键 1：循环夹取（cycleObj 记到哪了，A→B→C→A；被拦掉则不推进，不会白跳一格）
void keyCyclePick() {
  if (rejectWhileAuto("按键1")) return;
  runSequence(cycleObj);
  cycleObj = (cycleObj + 1) % 3;
}

// 按键 2：录制（每 200ms 把四轴当前角度抄一帧；按一次开、再按一次停）
// ⚠ 录制期间不设 busy —— 录制正是要靠手动命令操控机械臂；防的是"录制中再来自动动作"
void recStart() {
  if (recLen > 0) {
    Serial.print(F("⚠ 开始录制会覆盖上一次那 "));
    Serial.print(recLen);
    Serial.println(F(" 帧内容"));
  }
  recLen      = 0;
  recording   = true;
  recLastMs   = millis();
  recStartMs  = millis();
  Serial.println(F("开始录制：现在用 x/y/z 或 M 命令操控机械臂"));
  Serial.print(F("         考核要求时长大于 10 秒，再按一次 2 结束（最长 "));
  Serial.print(REC_MAX * REC_PERIOD_MS / 1000);
  Serial.println(F(" 秒）"));
}

// 四轴里动得最多的那根，一共转了多少度
int recTravelMax() {
  int m = 0;
  for (int i = 0; i < 4; i++) {
    int lo = 255, hi = 0;
    for (int k = 0; k < recLen; k++) {
      if (recData[k][i] < lo) lo = recData[k][i];
      if (recData[k][i] > hi) hi = recData[k][i];
    }
    if (hi - lo > m) m = hi - lo;
  }
  return m;
}

void recStop() {
  recording  = false;
  unsigned long durMs = millis() - recStartMs;
  int travel = recTravelMax();

  Serial.print(F("录制结束：共录到 "));
  Serial.print(recLen);
  Serial.print(F(" 帧，时长 "));
  Serial.print(durMs / 1000);
  Serial.print(F("."));
  Serial.print((durMs % 1000) / 100);
  Serial.println(F(" 秒"));

  if (recLen < 2) {
    Serial.println(F("⚠ 一帧都没录到，这段是空的，按 2 重新录"));
    return;
  }
  if (durMs <= (unsigned long)REC_MIN_MS) {
    Serial.print(F("⚠ 时长不足：考核要求大于 10 秒，这段只有 "));
    Serial.print(durMs / 1000);
    Serial.println(F(" 秒，按 2 重录一段更长的"));
    return;
  }
  if (travel < REC_MIN_TRAVEL) {
    Serial.print(F("⚠ 位移太小：四轴里动得最多的一根只转了 "));
    Serial.print(travel);
    Serial.println(F(" 度，考核要求'能体现明显位移'，重录一段幅度大点的"));
    return;
  }
  Serial.print(F("这段合格（四轴最大位移 "));
  Serial.print(travel);
  Serial.println(F(" 度），可以按 3 播放"));
}

void recToggle() {
  if (recording) recStop();
  else           recStart();
}

// loop() 里反复调用：到点就悄悄采一帧
void recTick() {
  if (!recording) return;

  if (recLen >= REC_MAX) {                 // 缓冲满了，自动收尾
    Serial.println(F("录制缓冲已满，自动结束"));
    recStop();
    return;
  }
  if (millis() - recLastMs < (unsigned long)REC_PERIOD_MS) return;

  recLastMs = millis();
  for (int i = 0; i < 4; i++) recData[recLen][i] = (byte)gCur[i];
  recLen++;
}

// 按键 3：播放
void playStart() {
  if (recLen < 2) {
    Serial.println(F("还没有录制内容，先按一次 2 录一段"));
    return;
  }
  playing     = true;
  playIdx     = 0;
  playStartMs = millis();
  busy        = true;
  Serial.print(F("播放开始：共 "));
  Serial.print(recLen);
  Serial.println(F(" 帧"));
  Serial.println(F("建议：先按 4 回中再播放，起播姿态和录制起点一致才最忠实"));
}

// 播放按时间轴走，不是按循环次数走：帧号 = 已过时间 / 采样周期
// 段内 stepMs 反算，让每段正好花掉 span 个采样周期，回放总时长 = 录制总时长
void playTick() {
  if (!playing) return;
  if (motionBusy()) return;                          // 上一段还没走完，等它

  unsigned long elapsed = millis() - playStartMs;
  int want = (int)(elapsed / REC_PERIOD_MS);         // 现在该播到第几帧

  if (want >= recLen) {                              // 全部播完
    playing = false;
    busy    = false;
    Serial.println(F("播放完成"));
    return;
  }
  if (want <= playIdx) return;                       // 还没到下一帧的时间点

  int span = want - playIdx;                         // 这一段跨了几个采样周期
  int to[4];
  int maxd = 0;
  for (int i = 0; i < 4; i++) {
    to[i] = recData[want][i];
    int d = abs(to[i] - gCur[i]);
    if (d > maxd) maxd = d;
  }
  playIdx = want;
  if (maxd == 0) return;                             // 这段四轴都没动，等下一个周期

  // stepMs 反算（+maxd/2 做四舍五入）
  int stepMs = (REC_PERIOD_MS * span + maxd / 2) / maxd;
  if (stepMs < 1) stepMs = 1;
  addSegMask(0b1111, to, stepMs);                    // 四轴同时动
}

// 按键 5：把录制内容打成表（联调工具，演示时别按）
void printRec() {
  if (recLen == 0) {
    Serial.println(F("还没有录制内容"));
    return;
  }
  Serial.print(F("录制帧数 "));
  Serial.print(recLen);
  Serial.print(F("，每帧 "));
  Serial.print(REC_PERIOD_MS);
  Serial.println(F(" ms："));
  for (int k = 0; k < recLen; k++) {
    Serial.print(F("  #"));
    Serial.print(k);
    Serial.print(F("  b="));  Serial.print(recData[k][0]);
    Serial.print(F("  f="));  Serial.print(recData[k][1]);
    Serial.print(F("  r="));  Serial.print(recData[k][2]);
    Serial.print(F("  c="));  Serial.println(recData[k][3]);
  }
}

// ---- 任务五 part1：直线绘制（FK 正解 + IK 逆解 + 笛卡尔直线插值）----
// 不能直接插舵机角：三轴等比例插值时笔尖走的是曲线，评分要看直线度，所以必须逐点做 IK。
// 6 记起点 → 7 记终点 → 8 自动画线；I x,y,z 单点测试（标定用）。全程非阻塞，T 随时可中断。

// FK 正解：三轴舵机角 → 笔尖坐标（示教记点用）
// servo[0]=base  servo[1]=小臂(fArm)  servo[2]=大臂(rArm)；tip[0..2] = 笔尖 x,y,z（mm）
void fkTip(const int servo[3], float tip[3]) {
  // 1) 舵机角 → 关节角（弧度）。大臂看下标 2，小臂看下标 1，别搞反。
  float aU  = (servo[2] - CAL_UPPER_ZERO) * CAL_UPPER_DIR * DEG_TO_RAD;
  float aF  = (servo[1] - CAL_FORE_ZERO)  * CAL_FORE_DIR  * DEG_TO_RAD;
  float yaw = (servo[0] - CAL_BASE_ZERO)  * CAL_BASE_DIR  * DEG_TO_RAD;

  // 2) 肩→肘→腕：整条臂只在一个竖直平面里动，所以只有"径向"和"高度"两个分量
  float wristR = GEO_SH_R + GEO_L1 * sin(aU) + GEO_L2 * sin(aF);   // 腕关节径向
  float wristZ = GEO_L0   + GEO_L1 * cos(aU) + GEO_L2 * cos(aF);   // 腕关节高度

  // 3) 腕→笔尖：平行四连杆保证笔杆永远水平，所以径向加常量、高度减常量
  float tipR = wristR + GEO_CLAW;

  // 4) 绕底座轴转到 x-y
  tip[0] = tipR * cos(yaw);
  tip[1] = tipR * sin(yaw);
  tip[2] = wristZ - GEO_PEN;
}

// IK 逆解：笔尖坐标 → 三轴舵机角；解不出返回 false
// 平面二连杆的 2 组肘部解仍要枚举，用"离当前姿态最近"挑一组，避免示教到一半突然换肘。
bool ikTip(const float tip[3], int servoOut[3]) {
  const float tipRange = sqrt(tip[0] * tip[0] + tip[1] * tip[1]);  // 笔尖到底座轴的水平距离 ≥ 0
  const float wristZ   = tip[2] + GEO_PEN - GEO_L0;                // 腕关节相对肩关节的高度

  // ---- 第 1 步：方位（只剩一支）----
  float yaw;
  float radTip;
  if (tipRange < 0.5) {
    // 笔尖正好落在底座轴正上方/正下方，方位角无定义 → 保持当前朝向不转
    yaw    = (gCur[0] - CAL_BASE_ZERO) * CAL_BASE_DIR * DEG_TO_RAD;
    radTip = 0.0;
  } else {
    yaw    = atan2(tip[1], tip[0]);
    radTip = tipRange;
  }
  int sB = (int)round(CAL_BASE_ZERO + CAL_BASE_DIR * yaw / DEG_TO_RAD);

  if (sB < LIM_MIN[0] || sB > LIM_MAX[0]) return false;

  // ---- 第 2 步：把"腕相对肩"的位置喂给平面二连杆余弦定理 ----
  float wristR = radTip - GEO_CLAW - GEO_SH_R;      // 腕相对肩的径向
  float r2     = wristR * wristR + wristZ * wristZ;
  float r      = sqrt(r2);                          // 肩 → 腕 的直线距离
  // 真够不到才拒（留 3mm 余量）；"刚好伸直"的夹一下继续解，否则伸直姿态会被误判不可达
  if (r > GEO_L1 + GEO_L2 + 3.0 || r < fabs(GEO_L1 - GEO_L2) - 3.0) return false;
  float rc = constrain(r, fabs(GEO_L1 - GEO_L2) + 0.01, GEO_L1 + GEO_L2 - 0.01);

  // 余弦定理：alpha = 大臂相对"肩→腕连线"要偏的角度；psi = 肘部弯角的补角
  float calpha = (r2 + GEO_L1 * GEO_L1 - GEO_L2 * GEO_L2) / (2.0 * rc * GEO_L1);
  calpha = constrain(calpha, -1.0, 1.0);       // 防 acos 出 NaN
  float alpha = acos(calpha);
  float cpsi  = (r2 - GEO_L1 * GEO_L1 - GEO_L2 * GEO_L2) / (2.0 * GEO_L1 * GEO_L2);
  cpsi = constrain(cpsi, -1.0, 1.0);           // 同上
  float psi   = acos(cpsi);
  float u     = atan2(wristR, wristZ);              // "肩→腕连线"相对竖直方向的角度

  // ---- 第 3 步：枚举 2 组肘部解，取离当前姿态最近且不越限的那组 ----
  bool found    = false;
  long bestCost = 0;
  int  best[3]  = {0, 0, 0};
  for (int e = 0; e < 2; e++) {
    float eDir = (e == 0) ? CAL_ELBOW_DIR : -CAL_ELBOW_DIR;
    float aU   = u - eDir * alpha;
    float aF   = aU + eDir * psi;
    int   sU   = (int)round(CAL_UPPER_ZERO + CAL_UPPER_DIR * aU / DEG_TO_RAD);
    int   sF   = (int)round(CAL_FORE_ZERO  + CAL_FORE_DIR  * aF / DEG_TO_RAD);
    if (sU < LIM_MIN[2] || sU > LIM_MAX[2]) continue;
    if (sF < LIM_MIN[1] || sF > LIM_MAX[1]) continue;
    // 底座已经定了不参与比较，只看大臂和小臂
    long cost = (long)abs(sF - gCur[1]) + abs(sU - gCur[2]);
    if (!found || cost < bestCost) {
      found = true; bestCost = cost;
      best[0] = sB; best[1] = sF; best[2] = sU;
    }
  }
  if (!found) return false;                            // 2 组肘解全越限 → 解不出
  servoOut[0] = best[0]; servoOut[1] = best[1]; servoOut[2] = best[2];
  return true;
}

// 把笔尖挪到 (x,y,z)：IK → 三轴同时插值；解不出返回 false
bool moveTipTo(const float tip[3]) {
  int servo[3];
  if (!ikTip(tip, servo)) return false;
  int v[4] = {servo[0], servo[1], servo[2], gCur[3]};   // 爪不动（笔是夹死的不许松）
  moveArm(v, (int)DRAW_MS_PER_DEG);   // 引擎按"每度花多少 ms"走，天然匀速
  return true;
}

void drawAbort(const __FlashStringHelper *why) {
  drawActive = false;
  drawPhase  = 0;
  busy       = false;
  Serial.print(F("画线中止："));
  Serial.println(why);
}

// 计算直线上第 k 个插值点
void drawPointAt(int k, float out[3]) {
  float s = (drawTotal <= 1) ? 0.0 : (float)k / (float)(drawTotal - 1);
  out[0] = drawTip[0][0] + (drawTip[1][0] - drawTip[0][0]) * s;
  out[1] = drawTip[0][1] + (drawTip[1][1] - drawTip[0][1]) * s;
  out[2] = drawTip[0][2] + (drawTip[1][2] - drawTip[0][2]) * s;
}

// 示教记点的时机保护：6/7 记的是"当前笔尖位置"（靠 fkTip(gCur) 算），
// 而 gCur 在运动中是一度一度变的 —— 臂没停稳就按会把半路位姿记成起点/终点。
// 两道闸：① rejectWhileAuto（播放 / 录制 / 画线 / busy 中的自动动作）
//         ② motionBusy（摇杆排进队列的段：摇杆那一路**不置 busy**，光靠①拦不住）
bool canTeachNow(const char *what) {
  if (rejectWhileAuto(what)) return false;
  if (motionBusy()) {
    Serial.print(what);
    Serial.println(F("：机械臂还在动，等停稳再记"));
    return false;
  }
  return true;
}

// 6 / 7：示教记点（用 FK 把当前位姿换算成笔尖坐标）
void drawTeach(int which) {
  float tip[3];
  fkTip(gCur, tip);
  drawTip[which][0] = tip[0];
  drawTip[which][1] = tip[1];
  drawTip[which][2] = tip[2];
  drawHas[which]    = true;
  if (which == 0) Serial.print(F("已记起点  "));
  else            Serial.print(F("已记终点  "));
  Serial.print(F("x=")); Serial.print(tip[0], 1);
  Serial.print(F(" y=")); Serial.print(tip[1], 1);
  Serial.print(F(" z=")); Serial.println(tip[2], 1);
}

// 8：启动画线
void drawStart() {
  if (rejectWhileAuto("8/画线")) return;
  if (!drawHas[0] || !drawHas[1]) {
    Serial.println(F("先发 6 记起点、7 记终点（笔尖轻触纸面时记）"));
    return;
  }
  float dx = drawTip[1][0] - drawTip[0][0];
  float dy = drawTip[1][1] - drawTip[0][1];
  float dz = drawTip[1][2] - drawTip[0][2];
  float len = sqrt(dx * dx + dy * dy + dz * dz);
  if (len < 5.0) {
    Serial.println(F("两点太近（<5mm），重新示教"));
    return;
  }
  drawTotal = (int)(len / DRAW_STEP_MM) + 1;
  if (drawTotal > 400) drawTotal = 400;          // 上限防耗时失控

  // 先把整条线逐点做一遍 IK，确认全程可解再动臂，避免画到一半才失败
  // （ikTip 成功的解一定三轴都在限位内，所以这里不用再查一次限位）
  float p[3];
  for (int k = 0; k < drawTotal; k++) {
    drawPointAt(k, p);
    int servo[3];
    if (!ikTip(p, servo)) {
      Serial.print(F("第 ")); Serial.print(k);
      Serial.print(F(" 个插值点不可达 (x=")); Serial.print(p[0], 1);
      Serial.print(F(" y=")); Serial.print(p[1], 1);
      Serial.print(F(" z=")); Serial.print(p[2], 1);
      Serial.println(F(")——太远/太近，或该点落在臂背后"));
      return;
    }
  }
  Serial.print(F("画线：长 "));   Serial.print(len, 1);
  Serial.print(F(" mm，"));      Serial.print(drawTotal);
  Serial.println(F(" 个插值点，开始（T 急停可中断）"));

  // 第一步：抬笔并平移到起点上方（z 取两端较高者 + 抬笔量，防中途刮纸）
  float above[3] = {drawTip[0][0], drawTip[0][1], 0.0};
  above[2] = (drawTip[0][2] > drawTip[1][2] ? drawTip[0][2] : drawTip[1][2]) + DRAW_LIFT_MM;
  if (!moveTipTo(above)) { drawAbort(F("起点上方不可达")); return; }
  drawPhase  = 1;
  drawIdx    = 0;
  drawActive = true;
  busy       = true;
}

// loop() 每圈调一次：上一步走完就投下一步
void drawTick() {
  if (!drawActive) return;
  if (motionBusy()) return;                      // 当前段还没走完

  if (drawPhase == 1) {                          // 已到起点上方 → 落笔
    float p[3] = {drawTip[0][0], drawTip[0][1], drawTip[0][2]};
    if (!moveTipTo(p)) { drawAbort(F("落笔点不可达")); return; }
    drawPhase = 2;
    return;
  }
  if (drawPhase == 2) {                          // 已落笔 → 开始逐点画
    drawPhase = 3;
    drawIdx   = 0;
    return;
  }
  if (drawPhase == 3) {                          // 画线中
    drawIdx++;
    if (drawIdx >= drawTotal) {                  // 画到终点 → 抬笔收尾
      float p[3] = {drawTip[1][0], drawTip[1][1], drawTip[1][2] + DRAW_LIFT_MM};
      if (!moveTipTo(p)) { drawAbort(F("收尾抬笔不可达")); return; }
      drawPhase = 4;
      return;
    }
    float p[3];
    drawPointAt(drawIdx, p);
    if (!moveTipTo(p)) { drawAbort(F("中途 IK 失败")); return; }
    return;
  }
  if (drawPhase == 4) {                          // 收尾完成
    drawActive = false;
    drawPhase  = 0;
    busy       = false;
    Serial.println(F("直线绘制完成 done"));
    return;
  }
}

// 按键分发 / 电脑串口
void onKeyChar(char c) {
  // 白名单过滤：只认 '1'~'5'，其余静默丢弃
  if (c < '1' || c > '5') return;

  if (c == '1') {
    keyCyclePick();
  }
  else if (c == '2') {
    recToggle();
  }
  else if (c == '3') {
    if (rejectWhileAuto("按键3")) return;
    playStart();
  }
  else if (c == '4') {
    if (rejectWhileAuto("按键4")) return;
    goHome();
    busy = true;
  }
  else if (c == '5') {
    printRec();
  }
}

// 命令表
void printHelp() {
  Serial.println(F("x90,y30,z20   三轴协同(底座/前臂/后臂，同时到位)"));
  Serial.println(F("O / S         开爪 / 合爪"));
  Serial.println(F("H / L         提速 / 降速(臂和夹爪一起调)"));
  Serial.println(F("A / B / C     执行对应物体的自动夹取(三轴同时插值)"));
  Serial.println(F("M90,45,60,30  四轴直接给角度(标定用)"));
  Serial.println(F("P             打印当前四轴角度"));
  Serial.println(F("G             回中"));
  Serial.println(F("T             急停"));
  Serial.println(F("?             打印命令表"));
  Serial.println(F("--- 任务三：按键板功能，USB 口直接敲数字也一样 ---"));
  Serial.println(F("1             循环夹取 A → B → C → A ..."));
  Serial.println(F("2             录制 开 / 关(录制中用 x/y/z 或 M 操控)"));
  Serial.println(F("3             播放上次录制"));
  Serial.println(F("4             回中"));
  Serial.println(F("5             打印录制内容(联调用)"));
  Serial.println(F("--- 任务五 part1：直线绘制 ---"));
  Serial.println(F("6             记起点(笔尖轻触纸面时发)"));
  Serial.println(F("7             记终点"));
  Serial.println(F("8             自动画直线(抬笔-平移-落笔-画线-抬笔)"));
  Serial.println(F("I120,0,30     笔尖坐标单点测试(IK标定用,单位mm)"));
  Serial.println(F("提示：动作执行中会拒绝新指令，等 done 再发。"));
}

// 抠整数：字母 / 逗号 / 空格一律跳过
int parseInts(String s, int *out, int want) {
  int n = 0;
  int i = 0;
  while (n < want && i < (int)s.length()) {
    char c = s.charAt(i);
    int neg = 0;
    if (c == '-' && i + 1 < (int)s.length() && isDigit(s.charAt(i + 1))) {
      neg = 1;
      i++;
    }
    if (isDigit(s.charAt(i))) {
      int v = 0;
      while (i < (int)s.length() && isDigit(s.charAt(i))) {
        v = v * 10 + (s.charAt(i) - '0');
        i++;
      }
      out[n] = neg ? -v : v;
      n++;
    } else {
      i++;
    }
  }
  return n;
}

// 命令分派：看首字符
void armDataCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char first = cmd.charAt(0);

  // 任务三的按键字符（1~5）：和按键板发过来的是同一条路
  if (first >= '1' && first <= '5') { onKeyChar(first); return; }
  // 6/7 示教记点（必须等机械臂停稳，见 canTeachNow），8 画直线。三者只能电脑敲（按键板只有 4 个键）
  else if (first == '6') { if (canTeachNow("6")) drawTeach(0); return; }
  else if (first == '7') { if (canTeachNow("7")) drawTeach(1); return; }
  else if (first == '8') { drawStart(); return; }
  // x,y,z：任务一的三轴协同（格式不可改）；x=底座, y=前臂 fArm(D8), z=后臂 rArm(D7)
  else if (first == 'x' || first == 'X') {
    if (rejectWhileBusy("x/y/z")) return;
    int got[3];
    if (parseInts(cmd, got, 3) < 3) {
      Serial.println(F("格式应为 x90,y30,z20"));
      return;
    }
    int v[4] = {got[0], got[1], got[2], gCur[3]};   // 爪保持不动
    if (!withinLimits(v)) return;
    printEst(estSyncMs(v[0], v[1], v[2]));
    moveArm(v, DSDdata[DSDlevel]);
    busy = true;
    Serial.print(F("x=")); Serial.print(v[0]);
    Serial.print(F(" y=")); Serial.print(v[1]);
    Serial.print(F(" z=")); Serial.println(v[2]);
  }
  // M：四轴直接给角度（标定用；臂三轴仍同时走）
  else if (first == 'M' || first == 'm') {
    if (rejectWhileBusy("M")) return;
    int v[4];
    if (parseInts(cmd, v, 4) < 4) {
      Serial.println(F("格式应为 M90,45,60,30"));
      return;
    }
    if (!withinLimits(v)) return;
    int est = estSyncMs(v[0], v[1], v[2]);
    if (v[3] != gCur[3]) est += estMs(gCur[3], v[3], CLAW_DSD[DSDlevel]);
    printEst(est);
    moveArm(v, DSDdata[DSDlevel]);
    if (v[3] != gCur[3]) moveClaw(v[3]);
    busy = true;
    // 非阻塞：这里回显的是"目标值"，要看实时角度请发 P
    Serial.print(F("目标 b=")); Serial.print(v[0]);
    Serial.print(F("  f="));    Serial.print(v[1]);
    Serial.print(F("  r="));    Serial.print(v[2]);
    Serial.print(F("  c="));    Serial.println(v[3]);
  }
  // I：笔尖坐标单点测试（任务五 IK 标定用）
  else if (first == 'I' || first == 'i') {
    if (rejectWhileAuto("I")) return;
    int got[3];
    if (parseInts(cmd, got, 3) < 3) {
      Serial.println(F("格式应为 I120,0,30（笔尖 x,y,z，单位 mm）"));
      return;
    }
    float tip[3] = {(float)got[0], (float)got[1], (float)got[2]};
    int servo[3];
    if (!ikTip(tip, servo)) {
      Serial.println(F("IK：该点不可达（够不到 / 落在臂背后 / 肘解越限），检查几何常量或坐标"));
      return;
    }
    Serial.print(F("IK x=")); Serial.print(tip[0], 1);
    Serial.print(F(" y="));   Serial.print(tip[1], 1);
    Serial.print(F(" z="));   Serial.print(tip[2], 1);
    Serial.print(F("  →  b=")); Serial.print(servo[0]);
    Serial.print(F(" f="));     Serial.print(servo[1]);
    Serial.print(F(" r="));     Serial.println(servo[2]);
    // 回代校验：解出的角度再正解一次，看能否回到目标点；残差大 = 几何常量或标定量不对
    float back[3];
    fkTip(servo, back);
    Serial.print(F("    回算 x=")); Serial.print(back[0], 1);
    Serial.print(F(" y="));        Serial.print(back[1], 1);
    Serial.print(F(" z="));        Serial.print(back[2], 1);
    Serial.print(F(" 残差 "));
    Serial.println(sqrt(sq(back[0] - tip[0]) + sq(back[1] - tip[1])
                      + sq(back[2] - tip[2])), 1);
    int v[4] = {servo[0], servo[1], servo[2], gCur[3]};
    moveArm(v, DSDdata[DSDlevel]);
    busy = true;
  }
  // A/B/C：任务二，执行夹取-放置（非阻塞序列）
  else if (first == 'A' || first == 'a') { if (!rejectWhileAuto("A")) runSequence(0); }
  else if (first == 'B' || first == 'b') { if (!rejectWhileAuto("B")) runSequence(1); }
  else if (first == 'C' || first == 'c') { if (!rejectWhileAuto("C")) runSequence(2); }
  // 开爪 / 合爪
  else if (first == 'O' || first == 'o' || first == '0') {
    if (rejectWhileBusy("O")) return;
    printEst(estMs(gCur[3], CLAW_OPEN, CLAW_DSD[DSDlevel]));
    moveClaw(CLAW_OPEN);
    busy = true;
    Serial.println(F("开爪"));
  }
  else if (first == 'S' || first == 's') {
    if (rejectWhileBusy("S")) return;
    printEst(estMs(gCur[3], CLAW_CLOSE, CLAW_DSD[DSDlevel]));
    moveClaw(CLAW_CLOSE);
    busy = true;
    Serial.println(F("合爪"));
  }
  // 速度档（改档不影响正在执行的那一段）
  else if (first == 'H' || first == 'h') {
    if (DSDlevel < 3) DSDlevel++;
    Serial.print(F("速度档 ")); Serial.println(DSDlevel);
  }
  else if (first == 'L' || first == 'l') {
    if (DSDlevel > 0) DSDlevel--;
    Serial.print(F("速度档 ")); Serial.println(DSDlevel);
  }
  // 工具
  else if (first == 'P' || first == 'p') { printAngles(); }
  else if (first == 'G' || first == 'g') {
    if (rejectWhileAuto("G")) return;
    goHome();
    busy = true;
  }
  else if (first == '?') { printHelp(); }
  // T：急停（放在最后，且不受 busy 限制）
  else if (first == 'T' || first == 't') { emergencyStop(); }
  else {
    Serial.println(F("未知命令，发 ? 查看命令表"));
  }
}

// 串口组帧：USB 口攒到换行符算一条命令；按键板来一个字符处理一个
void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        armDataCmd(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 32) inputBuffer = "";   // 防溢出
    }
  }

  while (espSerial.available() > 0) {
    char c = espSerial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;    // 按键板可能带尾巴，跳过
    if (c == '?') {                                      // 链路自检：只回一句，不动臂
      Serial.println(F("+LINK OK"));
      continue;
    }
    onKeyChar(c);
  }
}

void setup() {
  Serial.begin(9600);              // USB 口：看调试信息 + 直接敲命令
  espSerial.begin(9600);           // 按键板口：收单字符

  for (int i = 0; i < 4; i++) {
    gServo[i].attach(PIN[i]);
    delay(200);                    // 逐个上电，避免电流冲击
  }
  for (int i = 0; i < 4; i++) {
    gCur[i] = 90;                  // 插值起点必须是 90，否则首步会突跳
    gServo[i].write(90);
  }

  Serial.println(F("ready"));
  joyCalibrate();                  // 摇杆中点实测（上电时手要离开摇杆）
  printHelp();
  selfCheck();
}

void loop() {
  readSerial();     // USB + 按键板，两路口都收

  joyTick();        // 摇杆：偏离中点就排一段（自动动作在跑时自动让路）
  motionTick();     // 推进运动引擎一步
  seqTick();        // 推进序列状态机一步
  recTick();        // 录制中：到点采一帧
  playTick();       // 播放中：投递下一帧
  drawTick();       // 画线中：上一步走完投下一步

  // 单条指令跑完自动解锁；序列 / 画线结束各自解锁；播放中的间隙锁不解，所以要把 playing 算进来
  if (busy && !seqActive && !playing && !drawActive && !motionBusy()) busy = false;
}

/* 标定与联调（详见 10.13a_测量与标定.md）
 *
 * A/B/C：手盘复核限位 → 发 G 回中 → 用 M 摆位（等 done）→ 发 P 记四个数 → 抄进 gObj 表重烧 → 发 A/B/C 验证
 *   ⚠ M 的四个数按轴序 = 底座, 前臂, 后臂, 夹爪（b, f, r, c）；回显的是目标值，要确认真走到了就发 P
 *   ⚠ Servo::read() 只返回"最后一次 write 的值"，不是物理位置 —— 只能靠 M 摆位 + P 读数
 *
 * 任务五：GEO_PEN 要自己量（腕轴心 → 笔尖 的垂直落差）；纸放臂前方（不解底座背后那半圈）
 *   发 I130,0,30 看回算残差 + 尺量偏差：残差 > 2mm → 先查 GEO_PEN；底座转向反 → CAL_BASE_DIR；
 *   肘部姿态反 → CAL_ELBOW_DIR；大臂方向反 → CAL_UPPER_DIR；小臂越调越偏 → CAL_FORE_DIR；
 *   高度整体差一截 → GEO_PEN / GEO_L0；左右不对称 → CAL_BASE_ZERO 不是 90
 *   摇杆把笔尖压到起点 → 发 6 → 移到终点（保持触纸）→ 发 7 → 发 8；线不直 → DRAW_STEP_MM 改 1.0
 *
 * 上电与联调：上电前手离开摇杆 → 敲 1 三次依次夹 A/B/C → 敲 2 录 10 秒以上再敲 2 → 敲 3 播放 → 敲 4 回中
 *   接按键板（TX→D10，GND 共地），按键 1~4 = 敲 1~4；按键板发 ? → 回 "+LINK OK"（不动臂）
 *
 * 已知限制：
 * ⚠ 臂部一律三轴同时插值，防"夹爪横扫桌面"要在 gObj 表里自己安排高位过渡点
 * ⚠ H/L 同时调臂和夹爪；轴序 0=base 1=fArm(前臂/D8) 2=rArm(后臂/D7) 3=claw
 * ⚠ 摇杆在自动动作（A/B/C、回中、播放、画线）跑的时候不响应 —— 这是故意的
 * ⚠ 摇杆顶到限位会停住不硬怼（防堵转扫齿），但要退回来才能往反方向走
 * ⚠ 录制期间按 1/3/4 会被拒绝（录制中只允许手动操控）；按键 5 会吐一大段，演示时别按
 * ⚠ 软串口靠中断模拟，会和 Servo 的 PWM 定时器轻微打架，9600 下看不出来
 * ⚠ I / 6 / 7 / 8 只能电脑串口敲（按键板只有 1~4）；画线全程非阻塞，T 急停随时可断
 */
