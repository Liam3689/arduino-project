/* 10.11a  MeArm 机械臂 · 任务一摇杆 + 任务二 A/B/C + 任务三按键板
 *
 * 命令表：
 *   x90,y30,z20   三轴协同：底座 / 前臂 / 后臂
 *   O / S         开爪 / 合爪
 *   H / L         提速 / 降速（4 档，臂和夹爪一起调）
 *   A / B / C     执行对应物体的自动夹取-放置（任务二）
 *   M90,45,60,30  四轴直接给角度（标定用）
 *   P             打印当前四轴角度
 *   J             摇杆开 / 关
 *   G             回中（四轴回 90）
 *   T             急停（清空待执行动作）
 *   1 / 2 / 3 / 4 任务三的四个按键功能（USB 口直接敲 = 按按键板）
 *   5             打印录制内容（联调工具）
 *   ?             打印命令表
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// 轴序：0=底座base(9)  1=前臂fArm(8)  2=后臂rArm(7)  3=夹爪claw(6)
Servo gServo[4];
const int PIN[4] = {9, 8, 7, 6};

// 打印轴名用（下标对齐轴序）
const char *AXIS_NAME[4] = {"base", "fArm", "rArm", "claw"};

// 安全限位（通电前务必手盘复核全行程，实测不同就改这里）
const int LIM_MIN[4] = {  0, 30, 70,  60};
const int LIM_MAX[4] = {180, 140, 150, 118};

// 速度表：走 1 度花多少毫秒（越大越慢），H/L 同时切换下面两张表的档位
int DSDdata[4]  = {19, 15, 11, 8};    // 臂三轴
int CLAW_DSD[4] = {25, 21, 17, 13};   // 夹爪专用，比臂慢，防甩飞物体
int DSDlevel = 1;

// 夹爪开合角（若 O/S 反了先改这两行，别动 gObj 表）
const int CLAW_OPEN    = 70;    // 开爪（小角度）
const int CLAW_CLOSE   = 116;   // 合爪（大角度）

// 软件角度镜像：记住"命令到哪了"，作为插值起点
int gCur[4] = {90, 90, 90, 90};

// 一个关键点 = 四轴角度 + 停留时间
struct KeyPoint {
  int a[4];     // base, fArm, rArm, claw
  int dwell;    // 到位后停留（毫秒）
};

// A/B/C 关键点表（0=A 1=B 2=C）
// ⚠ 全表都是占位角度，考核前必须用 M 实测填回（爪 70=开 / 116=合）
KeyPoint gObj[3][12] = {
  { // 物体 A：取 base=90，放 base=135
    {{ 90, 90, 90, 90}, 300},   // 0 待机
    {{ 90, 90, 90, 90}, 300},   // 1 物体正上方
    {{ 90, 90, 90, 90}, 400},   // 2 下探到夹取位
    {{ 90, 90, 90, 90}, 500},   // 3 合爪夹住
    {{ 90, 90, 90, 90}, 300},   // 4 抬起
    {{ 90, 90, 90, 90}, 300},   // 5 转到放置点上方
    {{ 90, 90, 90, 90}, 400},   // 6 下探放置
    {{ 90, 90, 90, 90}, 500},   // 7 开爪
    {{ 90, 90, 90, 90}, 300},   // 8 抬起
    {{ 90, 90, 90, 90}, 300}    // 9 回中
  },
  { // 物体 B：取 base=45，放 base=90
    {{ 90, 90, 90, 90}, 300},   // 0 待机
    {{ 90, 90, 90, 90}, 300},   // 1 物体正上方
    {{ 90, 90, 90, 90}, 400},   // 2 下探到夹取位
    {{ 90, 90, 90, 90}, 500},   // 3 合爪夹住
    {{ 90, 90, 90, 90}, 300},   // 4 抬起
    {{ 90, 90, 90, 90}, 300},   // 5 转到放置点上方
    {{ 90, 90, 90, 90}, 400},   // 6 下探放置
    {{ 90, 90, 90, 90}, 500},   // 7 开爪
    {{ 90, 90, 90, 90}, 300},   // 8 抬起
    {{ 90, 90, 90, 90}, 300}    // 9 回中
  },
  { // 物体 C：取 base=135，放 base=45
    {{ 90, 90, 90, 90}, 300},   // 0 待机
    {{ 90, 90, 90, 90}, 300},   // 1 物体正上方
    {{ 90, 90, 90, 90}, 400},   // 2 下探到夹取位
    {{ 90, 90, 90, 90}, 500},   // 3 合爪夹住
    {{ 90, 90, 90, 90}, 300},   // 4 抬起
    {{ 90, 90, 90, 90}, 300},   // 5 转到放置点上方
    {{ 90, 90, 90, 90}, 400},   // 6 下探放置
    {{ 90, 90, 90, 90}, 500},   // 7 开爪
    {{ 90, 90, 90, 90}, 300},   // 8 抬起
    {{ 90, 90, 90, 90}, 300}    // 9 回中
  }
};
int gObjLen[3] = {10, 10, 10};   // 各物体实际关键点数（≤ 12）

String inputBuffer = "";

// 第二路串口，接按键板：RX=D10，TX=D11
SoftwareSerial espSerial(10, 11);

// 悬空噪声字节计数（诊断用）
unsigned int gJunkCnt = 0;

// 摇杆 4 通道 ADC（A0~A3 分控 4 根轴）
const int JOY_PIN[4] = {A0, A1, A2, A3};   // 下标对齐轴序

const int JOY_DEADZONE = 60;               // 死区
const int JOY_PERIOD_MS = 20;              // 每 20ms 推 1 度
const int JOY_SAMPLES = 30;                // 上电采样次数

int  joyCenter[4] = {512, 512, 512, 512};  // 中点（上电实测覆盖）
bool joyOn = true;                         // 摇杆总开关（J）
unsigned long joyLastMs = 0;

// 录制缓冲：200ms 一帧，最多 100 帧 = 20 秒
const int REC_MAX       = 100;
const int REC_PERIOD_MS = 200;
byte recData[REC_MAX][4];
int  recLen = 0;                    // 已录帧数
bool recording = false;
unsigned long recLastMs = 0;
unsigned long recStartMs = 0;       // 本段起始时刻

const int REC_MIN_MS = 10000;       // 合格线：时长 > 10 秒
const int REC_MIN_TRAVEL = 15;      // 合格线：明显位移 ≥ 15 度

// 播放状态
bool playing = false;
int  playIdx = 0;                   // 已播到第几帧
unsigned long playStartMs = 0;      // 起播时刻

// 按键 1 循环计数器：0=A 1=B 2=C
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

// 非阻塞运动引擎：把"从当前角度走到目标角度"打包成一段塞进队列，
// 每次 motionTick() 走一步，函数立刻返回（mask 位图决定这一段动哪几个轴）
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

// 入队：只动一个轴（其余轴取当前值）
void addSeg(int axis, int to, int stepMs) {
  int t[4] = {gCur[0], gCur[1], gCur[2], gCur[3]};
  t[axis] = to;
  addSegMask(1 << axis, t, stepMs);
}

// 还有没有没走完的动作
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

// 主循环每次调用一次：走一步
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

// 三轴比例插值（非阻塞版）：底座/前臂/后臂 同时出发、同时到位
void moveArm(int to[4], int stepMs) {
  addSegMask(0b0111, to, stepMs);
}

// 夹爪：单独慢速移动（非阻塞版，用夹爪专用速度表）
void moveClaw(int target) {
  if (target < LIM_MIN[3]) target = LIM_MIN[3];   // 兜底，防越限
  if (target > LIM_MAX[3]) target = LIM_MAX[3];
  addSeg(3, target, CLAW_DSD[DSDlevel]);
}

// 耗时预估：按和运动引擎完全相同的规则算一遍，不实际运动
int estMs(int from, int to, int stepMs) {
  return abs(to - from) * stepMs;
}

// 三轴同时（x/y/z、M、A/B/C、G 都用它）：步数取三轴跨度最大值
int estSyncMs(int toB, int toF, int toR) {
  int m = abs(toB - gCur[0]);
  if (abs(toF - gCur[1]) > m) m = abs(toF - gCur[1]);
  if (abs(toR - gCur[2]) > m) m = abs(toR - gCur[2]);
  return m * DSDdata[DSDlevel];
}

// 整条序列：用一份临时角度镜像模拟走一遍，把每段 + 停留累加
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

// 录制 / 播放中，拒绝"再来一段自动动作"（手动命令不受拦）
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
  return rejectWhileBusy(what);
}

// 摇杆上电校准：实测每一路的中点（每路采 30 次求平均）
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

// 每 20ms 读一次，偏离中点就推 1 度（增量式；只排一段；自动动作在跑时让路）
void joyTick() {
  if (!joyOn) return;
  if (busy || playing) return;
  if (millis() - joyLastMs < (unsigned long)JOY_PERIOD_MS) return;
  joyLastMs = millis();

  int to[4];
  int mask = 0;
  for (int i = 0; i < 4; i++) {
    to[i] = gCur[i];
    int d = analogRead(JOY_PIN[i]) - joyCenter[i];
    if (abs(d) <= JOY_DEADZONE) continue;               // 在死区内，当没动

    int t = gCur[i] + (d > 0 ? 1 : -1);                 // 推 1 度
    if (t < LIM_MIN[i] || t > LIM_MAX[i]) continue;     // 顶到限位就停住

    to[i] = t;
    mask |= (1 << i);
  }
  if (mask == 0) return;
  addSegMask(mask, to, 1);                              // 排一段，1ms 就走完
}

// 序列状态机：非阻塞执行 A/B/C
//   每个关键点：seqLoadPoint() 丢动作进队列 → seqTick() 等走完 → 停留 → 下一个点
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

// 急停：立刻停住并清空所有待执行动作
void emergencyStop() {
  segHead = segTail = segCount = 0;    // 清空动作队列
  segOn = false;                       // 停住当前这一段
  seqActive = false;                   // 中止正在跑的序列
  seqDwelling = false;
  playing = false;                     // 中止播放
  if (recording) {                     // 录制中急停 → 停掉录制，保住已录的帧
    recording = false;
    Serial.print(F("录制已中止，保住 "));
    Serial.print(recLen);
    Serial.println(F(" 帧"));
  }
  busy = false;                        // 解锁
  Serial.println(F("急停：已停住并清空待执行动作"));
}

// 打印当前四轴角度（标定用）
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

// 按键 1：循环夹取（cycleObj 记到哪了，A→B→C→A；被拦掉则不推进）
void keyCyclePick() {
  if (rejectWhileAuto("按键1")) return;
  runSequence(cycleObj);
  cycleObj = (cycleObj + 1) % 3;
}

// 按键 2：录制（每 200ms 把四轴当前角度抄一帧；按一次开，再按一次停）
// ⚠ 录制期间不设 busy —— 正是要靠手动命令操控机械臂
void recStart() {
  if (recLen > 0) {                        // 重录会覆盖上一次
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

// 四轴里动得最多的那根一共转了多少度（判断"有没有明显位移"）
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

  // 当场核两条硬指标，不达标直接说
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
// 把录下的帧两两相连一段段走；段内 stepMs 反算，保证每段正好花 200ms，
// 所以回放总时长 = 录制总时长。一次只投一段，等走完再投下一段。
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

// 播放按时间轴走：帧号 = 已过时间 / 采样周期
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

  // stepMs 反算：让这一段正好花掉 span 个采样周期（+maxd/2 做四舍五入）
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

// 按键分发：按键板发来的 / 电脑串口敲的，都走这里
void onKeyChar(char c) {
  // 白名单过滤：按键板断电后 D10 悬空会收到随机字节，只认 '1'~'5'，其余静默丢弃
  if (c < '1' || c > '5') {
    if (gJunkCnt < 65535) gJunkCnt++;        // 饱和累加，不溢出
    return;
  }

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
  Serial.println(F("J             摇杆开 / 关(演示怕误碰就关掉)"));
  Serial.println(F("G             回中"));
  Serial.println(F("T             急停"));
  Serial.println(F("?             打印命令表"));
  Serial.println(F("--- 任务三：按键板功能，USB 口直接敲数字也一样 ---"));
  Serial.println(F("1             循环夹取 A → B → C → A ..."));
  Serial.println(F("2             录制 开 / 关(录制中用 x/y/z 或 M 操控)"));
  Serial.println(F("3             播放上次录制"));
  Serial.println(F("4             回中"));
  Serial.println(F("5             打印录制内容(联调用)"));
  Serial.println(F("K             看悬空噪声计数(按键板联调诊断)"));
  Serial.println(F("R             噪声计数清零"));
  Serial.println(F("提示：动作执行中会拒绝新指令，等 done 再发。"));
}

// 抠整数：字母 / 逗号 / 空格一律跳过
// 不能用 String::toInt()——它底层是 atol()，首字符不是数字就直接返回 0
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
  // x,y,z：任务一的三轴协同（格式不可改，保持同时到位）
  //   x = 底座, y = 前臂 fArm(D8), z = 后臂 rArm(D7)
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
    // 非阻塞：这里回显目标值，要看实时角度请发 P
    Serial.print(F("目标 b=")); Serial.print(v[0]);
    Serial.print(F("  f="));    Serial.print(v[1]);
    Serial.print(F("  r="));    Serial.print(v[2]);
    Serial.print(F("  c="));    Serial.println(v[3]);
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
  else if (first == 'J' || first == 'j') {          // 摇杆开关
    joyOn = !joyOn;
    Serial.print(F("摇杆 "));
    Serial.println(joyOn ? "开" : "关");
  }
  else if (first == 'G' || first == 'g') {
    if (rejectWhileAuto("G")) return;
    goHome();
    busy = true;
  }
  else if (first == '?') { printHelp(); }
  // K：看一眼悬空噪声计数（按键板联调诊断）
  else if (first == 'K' || first == 'k') {
    Serial.print(F("悬空噪声累计 = "));
    Serial.print(gJunkCnt);
    Serial.println(F(" 字节（正常应为 0；断电后持续增长 = 下拉没接好）"));
  }
  // R：清零噪声计数
  else if (first == 'R' || first == 'r') {
    gJunkCnt = 0;
    Serial.println(F("噪声计数已清零"));
  }
  // T：急停（放在最后，不受 busy 限制）
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
  Serial.begin(9600);              // USB 口：调试信息 + 直接敲命令
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
  Serial.println(F("注意：A/B/C 关键点是占位示例，请实测后修改"));
}

void loop() {
  readSerial();     // USB + 按键板，两路口都收

  joyTick();        // 摇杆：偏离中点就排一段（自动动作在跑时自动让路）
  motionTick();     // 推进运动引擎一步
  seqTick();        // 推进序列状态机一步
  recTick();        // 录制中：到点采一帧
  playTick();       // 播放中：投递下一帧

  // 单条指令跑完自动解锁；序列结束由 seqFinish 解锁；播放间隙锁不解，所以要把 playing 算进来
  if (busy && !seqActive && !playing && !motionBusy()) busy = false;
}

/* 标定流程：
 * 1) 先手盘复核四轴全行程，确认 LIM_MIN / LIM_MAX 与实机一致。
 * 2) 上电，发 P 看当前角度；发 G 回中（等 done）。
 * 3) 用 M 一条命令摆一个姿态；等 done 后发 P 记下四个数。
 *      M 的四个数按轴序 = 底座, 前臂, 后臂, 夹爪（b, f, r, c）
 *      M90,140,70,100    ← 例：物体正上方
 *      M90,120,85,100    ← 例：下探到夹取位
 *      M90,120,85,35     ← 例：合爪
 * 4) 把这 10 组数字抄进上面的 gObj 表，重新编译烧录。
 * 5) 发 A / B / C 验证。
 *
 * 注意：Servo::read() 只返回"最后一次 write 的值"，不是物理位置，
 *       所以"断电手掰到位再读回"拿不到真实角度 —— 只能靠 M 摆位 + P 读数。
 *
 * 上电与联调：
 * 0) ⚠ 上电前把摇杆放手、别碰 —— 程序要采 30 次实测定中点
 * 1) 上电 → 应打印"摇杆中点实测：xxx,xxx,xxx,xxx"
 * 2) 推摇杆 → 对应那根轴跟着走，松手就停
 * 3) 只插 USB，敲 1（连敲三次）→ 应依次夹 A、B、C，每次结束打印 done
 * 4) 敲 2 开始录制 → 用摇杆推出一段有位移的动作，持续 10 秒以上
 * 5) 再敲 2 → 打印帧数、时长，并自动核对两条硬指标
 * 6) 敲 3 → 机械臂按刚录的轨迹走一遍
 * 7) 敲 4 → 回中
 * 8) 接上按键板（TX→D10，GND 共地），按键 1~4 应和敲 1~4 完全一样
 * 9) 按键板发 ? → 本板回 "+LINK OK"（不动臂），用来验证那根线和共地
 *
 * 已知限制：
 * ⚠ 臂部一律三轴同时插值：想避免夹爪横着扫过桌面，要在 gObj 表里自己安排高位过渡点
 * ⚠ H/L 同时调臂和夹爪：DSDdata 管臂三轴，CLAW_DSD 管夹爪（数字更大=更慢）
 * ⚠ 轴序为 0=base 1=fArm(前臂/D8) 2=rArm(后臂/D7) 3=claw
 * ⚠ 摇杆在自动动作（A/B/C、回中、播放）跑的时候不响应 —— 故意如此
 * ⚠ 摇杆顶到限位会停住不硬怼（防堵转扫齿），但要退回来才能往反方向走
 * ⚠ 录制期间不要按 1 / 3 / 4，会被拒绝（录制中只允许手动操控）
 * ⚠ 按键 5 会往串口吐一大段，几秒内主循环被发送拖住，演示时别按
 * ⚠ 软件串口靠中断模拟，会和 Servo 的 PWM 定时器轻微打架，9600 下看不出来
 */
