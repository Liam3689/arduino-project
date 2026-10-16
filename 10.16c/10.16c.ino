/* 任务一 摇杆 | 任务二 A/B/C | 任务三 按键板 | 任务五(1)(2)(3)(4)
 * 轴序：0=base D9 | 1=fArm 小臂 D8 | 2=rArm 大臂 D7 | 3=claw D6
 * tipR = GEO_SH_R + L1*sin(aU) + L2*sin(aF) + GEO_CLAW
 * tipZ = GEO_L0   + L1*cos(aU) + L2*cos(aF) - GEO_PEN
 * 命令表：发 ? 查看
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// ---------- 轴序与命名----------
Servo gServo[4];
const int PIN[4] = {9, 8, 7, 6};

// 打印轴名用
const char *AXIS_NAME[4] = {"base", "fArm", "rArm", "claw"};

// 安全限位
const int LIM_MIN[4] = {  0, 0, 70,  30};
const int LIM_MAX[4] = {180, 140, 160, 110};

// 速度表：走 1 度花多少毫秒（越大越慢）；H/L 同时切下面两张表的档位
int DSDdata[4]  = {28, 18, 13, 8};      // 臂三轴（底座/前臂/后臂）
int CLAW_DSD[4] = {25, 21, 18, 15};     // 夹爪专用，数字大一截 → 永远比臂慢，防甩飞物体
int DSDlevel = 2;

// 夹爪开合角
const int CLAW_OPEN    = 50;      // 开爪（小角度）
const int CLAW_CLOSE   = 105;     // 合爪（大角度）

// 软件角度镜像：记住"命令到哪了"，作为插值起点
int gCur[4] = {90, 90, 90, 90};

// ---------- 任务五：几何常量（尺寸，单位 mm）----------
// 尺寸取自本组 MeArm 装配体实测（不是标准 MeArm 的 80/80/80 猜值）
float GEO_SH_R = 10.3;    // 肩关节轴 相对 底座轴 的水平前移（原模型漏了这一项）
float GEO_L0   = 55.6;    // 底座轴原点(桌面) → 肩关节轴 的垂直高度
float GEO_L1   = 82.8;    // 大臂：肩关节轴 → 肘关节轴 轴心距
float GEO_L2   = 79.9;    // 小臂：肘关节轴 → 腕关节轴 轴心距
float GEO_CLAW = 42.0;    // 腕关节轴 → 笔杆轴线 的水平前伸量（夹爪恒水平，常量）
float GEO_PEN  = 45.0;    // 腕关节轴 → 笔尖 的垂直落差（装一次笔测一次）

// ---------- 任务五：标定量----------
float CAL_BASE_DIR   =  1.0;    // base：舵机角增大 = 从上方看**逆时针**转
float CAL_BASE_ZERO  = 90.0;    // base 舵机 90 = 笔尖指向"正前方"（基准方位，可改）
float CAL_UPPER_DIR  =  1.0;    // 大臂(rArm 下标2)：舵机角增大 = 大臂**向下压**
float CAL_UPPER_ZERO = 90.0;    // 大臂舵机 90 = 大臂竖直（aU = 0）
float CAL_FORE_DIR   = -1.0;    // 小臂(fArm 下标1)：舵机角增大 = 小臂**向上抬**
float CAL_FORE_ZERO  =180.0;  // 小臂舵机 180 = 小臂水平前伸（aF = 90°）
float CAL_ELBOW_DIR  =  1.0;    // 肘部解选型：+1 = 肘内那支（唯一机械可行）；10.14b 起不再枚举另一支
// ---------- 任务五：绘制参数 ----------
float DRAW_STEP_MM    = 2.0;     // 插值步长（越小越直越慢；直线度差就改 1.0）
float DRAW_LIFT_MM    = 15.0;    // 抬笔高度
float DRAW_MS_PER_DEG = 60.0;    // 绘制时每 1 度花多少 ms（越大越稳越慢，别小于 20）

// ---------- 任务五：绘制路径（顶点表）----------
// 本版画的字母 Z 恰好是"一笔画"折线，所以段与段之间不需要抬笔。
const int DRAW_MAXPTS = 5;              // 顶点上限
float pathPt[DRAW_MAXPTS][3];           // 顶点笔尖坐标（mm）；纸面水平，z 取记点时的高度

// ----------  一键命令的预存表----------
int LINE_ANG[2][3] = { {  90,  42,  72 },      // 直线：起点
                       {  90,  42, 107 } };    //       终点
int Z_ANG[4][3]    = { { 109,  54,  140 },      // Z 左上
                       { 70,  55, 140 },      //   右上
                       { 120,  30,  110 },      //   左下
                       {  59,  29, 111 } };    //   右下
int   pathN = 0;                     // 已定义顶点数

bool  drawActive = false;            // 正在自动绘制
int   drawSegs  = 0;                 // 总段数 = pathN − 1
int   drawSeg   = 0;                 // 当前画到第几段（第 0 段 = pathPt[0] → pathPt[1]）
int   drawSteps = 0;                 // 当前段的插值步数（这一段的点数 = 步数 + 1）
int   drawIdx   = 0;                 // 当前段已走到第几个插值点
int   drawPhase = 0;                 // 0=回待机 1=首点上方 2=落笔 3=进段 4=逐段画 5=画完回中 6=取消回中
bool  drawPaused = false;            // 任务五(3)：暂停标志（J 暂停 / K 继续）
int   gKeyMode   = 1;                // 按键板模式：1=任务三 2=示教（记点/画线 + 暂停/继续/取消）（主控串口 F1/F2 切）

struct KeyPoint {
  int a[4];       // base, fArm, rArm, claw   ← 顺序按轴序，别写反
  int dwell;      // 到位后停留，单位毫秒
};


KeyPoint gObj[3][12] = {
  {   // ---- 物体 A：取 base=90，放 base=135 ----
    {{ 90, 90, 90, 90}, 300},     // 0 待机
    {{ 123,90,90,45}, 300},     // 1 物体正上方
    {{ 123,35,140,45}, 400},     // 2 下探到夹取位
    {{ 123,35,140,110}, 500},     // 3 合爪夹住
    {{ 123,90,90,110}, 300},     // 4 抬起
    {{ 65, 90, 90, 110}, 300},     // 5 转到放置点上方
    {{ 65, 40, 150, 110}, 400},     // 6 下探放置
    {{ 65, 40, 150, 45}, 500},     // 7 开爪
    {{ 65, 90, 90, 45}, 300},     // 8 抬起
    {{ 90, 90, 90, 90}, 300}      // 9 回中
  },
  {   // ---- 物体 B：取 base=45，放 base=90 ----
    {{ 90, 90, 90, 90}, 300},     // 0 待机
    {{ 80, 90, 70, 45}, 300},     // 1 物体正上方
    {{ 80,70,120,45}, 400},     // 2 下探到夹取位
    {{ 80,70,120,108}, 500},     // 3 合爪夹住
    {{ 80, 90, 90, 108}, 300},     // 4 抬起
    {{ 130, 90, 90, 108}, 300},     // 5 转到放置点上方
    {{ 130, 70, 120, 108}, 400},     // 6 下探放置
    {{ 130, 70, 120, 45}, 500},     // 7 开爪
    {{ 130, 90, 90, 45}, 300},     // 8 抬起
    {{ 90, 90, 90, 90}, 300}      // 9 回中
  },
  {   // ---- 物体 C：取 base=135，放 base=45 ----
    {{ 90, 90, 90, 90}, 300},     // 0 待机
    {{ 105, 90, 90, 45}, 300},     // 1 物体正上方
    {{ 105, 40, 140, 45}, 400},     // 2 下探到夹取位
    {{ 105, 40, 140, 108}, 500},     // 3 合爪夹住
    {{ 105, 90, 90, 108}, 300},     // 4 抬起
    {{ 60, 90, 90, 108}, 300},     // 5 转到放置点上方
    {{ 60, 30, 135, 108}, 400},     // 6 下探放置
    {{ 60, 30, 135, 45}, 500},     // 7 开爪
    {{ 60, 90, 90, 45}, 300},     // 8 抬起
    {{ 90, 90, 90, 90}, 300}      // 9 回中
  }
};
int gObjLen[3] = {10, 10, 10};     // 各物体实际关键点数（必须 ≤ 12）

String inputBuffer = "";

// ---------- 任务三：第二路串口----------
SoftwareSerial espSerial(10, 11);

// ---------- 任务一 · 摇杆实时操控 ----------
const int JOY_PIN[4] = {A0, A1, A2, A3};     // 下标和轴序对齐：0=base 1=fArm 2=rArm 3=claw

const int JOY_DEADZONE = 60;

const int JOY_PERIOD_MS = 20;

const int JOY_SAMPLES = 30;                  // 上电采多少次求中点

int  joyCenter[4] = {512, 512, 512, 512};    // 每路的中点（上电实测覆盖）
unsigned long joyLastMs = 0;

// ---------- 任务三：录制缓冲 ----------
const int REC_MAX       = 100;        // 最多存多少帧
const int REC_PERIOD_MS = 200;        // 每 200ms 采一帧 → 最长 100×200ms = 20 秒
byte recData[REC_MAX][4];
int  recLen = 0;                      // 当前已录帧数（也是"上一次录制"的成果）
bool recording = false;
unsigned long recLastMs = 0;
unsigned long recStartMs = 0;         // 本段录制的起始时刻（用来核对时长）

const int REC_MIN_MS = 10000;
const int REC_MIN_TRAVEL = 15;

// ---------- 任务三：播放状态 ----------
bool playing = false;
bool playAligning = false;            // 起播对齐中：这段不按时间轴推帧
int  playIdx = 0;                     // 已经播到第几帧
unsigned long playStartMs = 0;        // 起播时刻（按时间轴走的基准）

// ---------- 任务三：按键 1 的循环计数器 ----------
int cycleObj = 0;

// ---------- 限位检查：4 轴逐个查，越限就打印并返回 false ----------
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

// ---------- 非阻塞运动引擎（段队列 + motionTick：每 stepMs 走 1 度）----------
const int SEG_MAX = 12;            // 队列容量（一个关键点最多用 5 段）

struct Seg {
  int mask;                        // 这一段动哪几个轴（位图）
  int to[4];                       // 各轴目标角度
  int stepMs;                      // 每走 1 度花多少毫秒
};

Seg segQ[SEG_MAX];
int segHead = 0, segTail = 0, segCount = 0;

bool segOn = false;
int  runMask = 0;
int  runFrom[4], runTo[4];
int  runTotal = 0, runStep = 0, runStepMs = 1;
unsigned long runLastMs = 0;

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

void addSeg(int axis, int to, int stepMs) {
  int t[4] = {gCur[0], gCur[1], gCur[2], gCur[3]};
  t[axis] = to;
  addSegMask(1 << axis, t, stepMs);
}

bool motionBusy() {
  return segOn || segCount > 0;
}

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
        if (d > runTotal) runTotal = d;
      }
    }
    if (runTotal == 0) continue;

    runStep   = 0;
    runLastMs = millis();
    segOn     = true;
    return true;
  }
  return false;
}

void motionTick() {
  if (!segOn) {
    runStartNext();
    return;
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

// ---------- 三轴比例插值（非阻塞版）：底座/前臂/后臂 同时出发、同时到位 ----------
void moveArm(int to[4], int stepMs) {
  addSegMask(0b0111, to, stepMs);
}

// ---------- 夹爪：单独慢速移动（非阻塞版） ----------
void moveClaw(int target) {
  if (target < LIM_MIN[3]) target = LIM_MIN[3];
  if (target > LIM_MAX[3]) target = LIM_MAX[3];
  addSeg(3, target, CLAW_DSD[DSDlevel]);
}

// ---------- 耗时预估：按和运动引擎完全相同的规则算一遍，不实际运动 ----------
int estMs(int from, int to, int stepMs) {
  return abs(to - from) * stepMs;
}

int estSyncMs(int toB, int toF, int toR) {
  int m = abs(toB - gCur[0]);
  if (abs(toF - gCur[1]) > m) m = abs(toF - gCur[1]);
  if (abs(toR - gCur[2]) > m) m = abs(toR - gCur[2]);
  return m * DSDdata[DSDlevel];
}

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

void printEst(int ms) {
  Serial.print(F("预计耗时 "));
  Serial.print(ms / 1000);
  Serial.print(F("."));
  Serial.print((ms % 1000) / 100);
  Serial.println(F(" 秒"));
}

// ---------- busy 保护 ----------
bool busy = false;

bool rejectWhileBusy(const char *what) {
  if (!busy) return false;
  Serial.print(what);
  Serial.println(F("：机械臂正忙，指令已忽略（等 done 再发，或发 T 急停）"));
  return true;
}

// ---------- 任务三：录制 / 播放中，拒绝"再来一段自动动作" ----------
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
    Serial.println(F("：正在绘制路径（J 暂停 / Q 取消 / T 急停），或等画完"));
    return true;
  }
  return rejectWhileBusy(what);
}

// ---------- 摇杆：校准 + 实时操控 ----------

// ---------- 上电校准：实测每一路的中点 ----------
// ⚠ 上电那一刻手不要碰摇杆，否则会把"你按住的位置"当成中点
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

// ---------- 每 20ms 读一次，偏离中点就推 1 度 ----------
void joyTick() {
  if (busy || playing) return;
  if (millis() - joyLastMs < (unsigned long)JOY_PERIOD_MS) return;
  joyLastMs = millis();

  int to[4];
  int mask = 0;
  for (int i = 0; i < 4; i++) {
    to[i] = gCur[i];
    int d = analogRead(JOY_PIN[i]) - joyCenter[i];
    if (abs(d) <= JOY_DEADZONE) continue;

    int t = gCur[i] + (d > 0 ? -1 : 1);
    if (t < LIM_MIN[i] || t > LIM_MAX[i]) continue;

    to[i] = t;
    mask |= (1 << i);
  }
  if (mask == 0) return;
  addSegMask(mask, to, 1);
}

// ---------- 序列状态机：非阻塞执行 A/B/C ----------
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
  busy        = false;
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

  moveArm(a, DSDdata[DSDlevel]);
  if (a[3] != gCur[3]) moveClaw(a[3]);
  seqDwelling = false;
}

void seqAdvance() {
  seqIdx++;
  if (seqIdx >= gObjLen[seqObj]) { seqFinish(true); return; }
  seqLoadPoint();
}

void seqTick() {
  if (!seqActive) return;

  if (!seqDwelling) {
    if (motionBusy()) return;
    int d = gObj[seqObj][seqIdx].dwell;
    if (d <= 0) { seqAdvance(); return; }
    seqDwellUntil = millis() + (unsigned long)d;
    seqDwelling   = true;
    return;
  }

  if ((long)(millis() - seqDwellUntil) >= 0) seqAdvance();
}

// ---------- 回放某个物体的动作序列（非阻塞：只投递，交给 seqTick 推进） ----------
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

// ---------- 上电自检：把 gObj 里所有关键点都过一遍限位 ----------
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

// ---------- 急停：立刻停住并清空所有待执行动作（任何时候都能用） ----------
void emergencyStop() {
  segHead = segTail = segCount = 0;
  segOn = false;
  seqActive = false;
  seqDwelling = false;
  playing = false;
  playAligning = false;
  if (drawActive) {
    drawActive = false;
    drawPhase  = 0;
    drawPaused = false;
    Serial.println(F("绘制已中止（路径顶点保留，重发 8 可重画）"));
  }
  if (recording) {
    recording = false;
    Serial.print(F("录制已中止，保住 "));
    Serial.print(recLen);
    Serial.println(F(" 帧"));
  }
  busy = false;
  Serial.println(F("急停：已停住并清空待执行动作"));
}

// ---------- 打印当前四轴角度（标定采参数用） ----------
void printAngles() {
  Serial.print(F("P "));
  for (int i = 0; i < 4; i++) {
    Serial.print(gCur[i]);
    if (i < 3) Serial.print(F(","));
    else       Serial.println();
  }
}

// ---------- 回中：四轴回 90（臂三轴同时插值，最后关爪到位） ----------
void goHome() {
  printEst(estSyncMs(90, 90, 90) + estMs(gCur[3], 90, CLAW_DSD[DSDlevel]));
  int to[4] = {90, 90, 90, 90};
  moveArm(to, DSDdata[DSDlevel]);
  moveClaw(90);
  Serial.println(F("回中"));
}

// ---------- 任务三 ① 按键 1：循环夹取（0=A 1=B 2=C）----------
void keyCyclePick() {
  if (rejectWhileAuto("按键1")) return;
  runSequence(cycleObj);
  cycleObj = (cycleObj + 1) % 3;
}

// ---------- 任务三 ② 按键 2：录制 ----------
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

void recTick() {
  if (!recording) return;

  if (recLen >= REC_MAX) {
    Serial.println(F("录制缓冲已满，自动结束"));
    recStop();
    return;
  }
  if (millis() - recLastMs < (unsigned long)REC_PERIOD_MS) return;

  recLastMs = millis();
  for (int i = 0; i < 4; i++) recData[recLen][i] = (byte)gCur[i];
  recLen++;
}

// ---------- 任务三 ③ 按键 3：播放 ----------
void playStart() {
  if (recLen < 2) {
    Serial.println(F("还没有录制内容，先按一次 2 录一段"));
    return;
  }
  // 摇杆刚排的段还没走完 → 直接拒。否则对齐段可能挤不进队列（被静默丢弃），
  // playAligning 会立刻被清掉，等于没对齐，还是猛冲。
  if (motionBusy()) {
    Serial.println(F("机械臂还在动，等停稳再按 3 播放"));
    return;
  }
  busy        = true;
  playIdx     = 0;
  playStartMs = millis();

  // ⚠ 起播对齐：当前姿态离录制第 0 帧太远时，先用正常速度平滑走过去再开始回放。
  // 不做这一步，这段"对齐误差"会被当成"一帧的正常位移"挤进一个 200ms 采样周期 —— 就是突然抽一下。
  int far = 0;
  for (int i = 0; i < 4; i++) {
    int d = abs((int)recData[0][i] - gCur[i]);
    if (d > far) far = d;
  }
  playing = true;
  if (far > 8) {
    int to[4] = {recData[0][0], recData[0][1], recData[0][2], recData[0][3]};
    playAligning = true;
    addSegMask(0b1111, to, DSDdata[DSDlevel]);
    Serial.print(F("播放开始：起播姿态差 "));
    Serial.print(far);
    Serial.println(F(" 度，先平滑对齐到录制起点"));
    return;
  }
  playAligning = false;
  Serial.print(F("播放开始：共 "));
  Serial.print(recLen);
  Serial.println(F(" 帧"));
}

// 播放按时间轴走（帧号 = 经过时间 / 采样周期）
void playTick() {
  if (!playing) return;
  if (motionBusy()) return;

  if (playAligning) {                  // 对齐段刚走完 → 从这一刻起才开始算时间轴
    playAligning = false;
    playStartMs  = millis();
    playIdx      = 0;
    Serial.println(F("已对齐，开始回放"));
    return;
  }

  unsigned long elapsed = millis() - playStartMs;
  int want = (int)(elapsed / REC_PERIOD_MS);

  if (want >= recLen) {
    playing = false;
    busy    = false;
    Serial.println(F("播放完成"));
    return;
  }
  if (want <= playIdx) return;

  int span = want - playIdx;
  int to[4];
  int maxd = 0;
  for (int i = 0; i < 4; i++) {
    to[i] = recData[want][i];
    int d = abs(to[i] - gCur[i]);
    if (d > maxd) maxd = d;
  }
  playIdx = want;
  if (maxd == 0) return;

  int stepMs = (REC_PERIOD_MS * span + maxd / 2) / maxd;
  if (stepMs < 1) stepMs = 1;
  addSegMask(0b1111, to, stepMs);
}

// ---------- 任务三 ④  把录制内容打成表（USB 敲 5，联调工具） ----------
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

// ---------- 任务五(1) 直线绘制：FK 正解 + IK 逆解 + 笛卡尔插值 ----------

// ---------- FK 正解：三轴舵机角 → 笔尖坐标（示教记点用） ----------
void fkTip(const int servo[3], float tip[3]) {
  float aU  = (servo[2] - CAL_UPPER_ZERO) * CAL_UPPER_DIR * DEG_TO_RAD;
  float aF  = (servo[1] - CAL_FORE_ZERO)  * CAL_FORE_DIR  * DEG_TO_RAD;
  float yaw = (servo[0] - CAL_BASE_ZERO)  * CAL_BASE_DIR  * DEG_TO_RAD;

  float wristR = GEO_SH_R + GEO_L1 * sin(aU) + GEO_L2 * sin(aF);     // 腕关节径向
  float wristZ = GEO_L0   + GEO_L1 * cos(aU) + GEO_L2 * cos(aF);     // 腕关节高度

  float tipR = wristR + GEO_CLAW;

  tip[0] = tipR * cos(yaw);
  tip[1] = tipR * sin(yaw);
  tip[2] = wristZ - GEO_PEN;
}

// ---------- IK 逆解：笔尖坐标 → 三轴舵机角；解不出返回 false ----------
bool ikTip(const float tip[3], int servoOut[3]) {
  const float tipRange = sqrt(tip[0] * tip[0] + tip[1] * tip[1]);    // 笔尖到底座轴的水平距离 ≥ 0
  const float wristZ   = tip[2] + GEO_PEN - GEO_L0;                  // 腕关节相对肩关节的高度

  float yaw = atan2(tip[1], tip[0]);
  int sB = (int)round(CAL_BASE_ZERO + CAL_BASE_DIR * yaw / DEG_TO_RAD);
  // 底座只有 0~180°，不做 360° 归一化：越界就是"目标在臂背后"，直接拒解
  if (sB < LIM_MIN[0] || sB > LIM_MAX[0]) return false;

  float wristR = tipRange - GEO_CLAW - GEO_SH_R;      // 腕相对肩的径向
  float r2     = wristR * wristR + wristZ * wristZ;
  float r      = sqrt(r2);                            // 肩 → 腕 的直线距离
  // 余弦定理直接用 r（分子 r2、分母 r，天然自洽）。太远解不到、太近小臂要上折，都拒。
  // ⚠ 下限这道判断必须有：r=0 会除零，NaN 又过不了 clamp（比较恒假），必须在 acos 之前挡住。
  if (r > GEO_L1 + GEO_L2 + 3.0) return false;
  if (!(r >= fabs(GEO_L1 - GEO_L2))) return false;

  float calpha = (r2 + GEO_L1 * GEO_L1 - GEO_L2 * GEO_L2) / (2.0 * r * GEO_L1);
  calpha = constrain(calpha, -1.0, 1.0);
  float alpha = acos(calpha);
  float cpsi  = (r2 - GEO_L1 * GEO_L1 - GEO_L2 * GEO_L2) / (2.0 * GEO_L1 * GEO_L2);
  cpsi = constrain(cpsi, -1.0, 1.0);
  float psi   = acos(cpsi);
  float u     = atan2(wristR, wristZ);                // "肩→腕连线"相对竖直方向的角度

  float aU = u - CAL_ELBOW_DIR * alpha;
  float aF = aU + CAL_ELBOW_DIR * psi;
  int   sU = (int)round(CAL_UPPER_ZERO + CAL_UPPER_DIR * aU / DEG_TO_RAD);
  int   sF = (int)round(CAL_FORE_ZERO  + CAL_FORE_DIR  * aF / DEG_TO_RAD);
  if (sU < LIM_MIN[2] || sU > LIM_MAX[2]) return false;      // 大臂越限 → 不可达
  if (sF < LIM_MIN[1] || sF > LIM_MAX[1]) return false;      // 小臂越限 → 不可达
  servoOut[0] = sB; servoOut[1] = sF; servoOut[2] = sU;
  return true;
}

// ---------- 把笔尖挪到 (x,y,z)：IK → 三轴同时插值；解不出返回 false ----------
bool moveTipTo(const float tip[3]) {
  int servo[3];
  if (!ikTip(tip, servo)) return false;
  int v[4] = {servo[0], servo[1], servo[2], gCur[3]};
  moveArm(v, (int)DRAW_MS_PER_DEG);
  return true;
}

// ---------- 绘制状态机 ----------

void drawAbort(const __FlashStringHelper *why) {
  drawActive = false;
  drawPhase  = 0;
  drawPaused = false;
  busy       = false;
  Serial.print(F("绘制中止："));
  Serial.println(why);
}

// ---------- 任务五(3) · J：暂停 ----------
void drawPause() {
  if (!drawActive) { Serial.println(F("J：当前没有绘制任务")); return; }
  if (drawPaused)  { Serial.println(F("J：本来就暂停着，发 K 继续")); return; }
  drawPaused = true;
  Serial.println(F("已暂停（保持当前状态）—— 发 K 继续 / Q 取消"));
}

// ---------- 任务五(3) · K：继续 ----------
void drawResume() {
  if (!drawActive)     { Serial.println(F("K：当前没有绘制任务")); return; }
  if (!drawPaused)     { Serial.println(F("K：没在暂停中，正在画")); return; }
  drawPaused = false;
  Serial.println(F("继续：从暂停位置接续绘制"));
}

// ---------- 任务五(3) · Q：取消 ----------
// Q 回待机不动爪：画图时笔是夹死的，goHome() 里的 moveClaw(90) 会松笔
void drawCancel() {
  if (!drawActive) { Serial.println(F("Q：当前没有绘制任务")); return; }
  drawPaused = false;
  float tip[3];
  fkTip(gCur, tip);
  float up[3] = {tip[0], tip[1], tip[2] + DRAW_LIFT_MM};
  drawPhase = 6;
  if (!moveTipTo(up)) {
    drawAbort(F("取消时抬笔失败"));
    return;
  }
  Serial.println(F("取消：抬笔中，随后返回待机"));
}

void printTip(const float t[3]) {
  Serial.print(F("x=")); Serial.print(t[0], 1);
  Serial.print(F(" y=")); Serial.print(t[1], 1);
  Serial.print(F(" z=")); Serial.println(t[2], 1);
}

void drawPointAt(int seg, int k, int steps, float out[3]) {
  float s = (steps <= 0) ? 0.0 : (float)k / (float)steps;
  const float *a = pathPt[seg];
  const float *b = pathPt[seg + 1];
  out[0] = a[0] + (b[0] - a[0]) * s;
  out[1] = a[1] + (b[1] - a[1]) * s;
  out[2] = a[2] + (b[2] - a[2]) * s;
}

int drawSegSteps(int seg) {
  const float *a = pathPt[seg];
  const float *b = pathPt[seg + 1];
  float dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
  float len = sqrt(dx * dx + dy * dy + dz * dz);
  int n = (int)(len / DRAW_STEP_MM);
  if (n < 1)   n = 1;
  if (n > 200) n = 200;
  return n;
}

// ---------- 示教记点的时机保护----------
bool canTeachNow(const char *what) {
  if (rejectWhileAuto(what)) return false;
  if (motionBusy()) {
    Serial.print(what);
    Serial.println(F("：机械臂还在动（摇杆那一路没走完），等停稳再记"));
    return false;
  }
  return true;
}

// ---------- 6：清空路径，记第 1 个点（直线起点 / 折线起点）----------
void drawBegin() {
  float tip[3];
  fkTip(gCur, tip);
  pathPt[0][0] = tip[0];
  pathPt[0][1] = tip[1];
  pathPt[0][2] = tip[2];
  pathN = 1;
  Serial.print(F("路径第 1 点  "));
  printTip(tip);
}

// ---------- 追加一个顶点（只由统一记点入口调用，串口没有 7 了）----------
void drawAppend() {
  if (pathN == 0) {
    Serial.println(F("先发 6 记路径第 1 个点"));
    return;
  }
  if (pathN >= DRAW_MAXPTS) {
    Serial.print(F("已记满 "));
    Serial.print(DRAW_MAXPTS);
    Serial.println(F(" 个点，自动清空，从第 1 点重新记："));
    drawBegin();
    return;
  }
  float tip[3];
  fkTip(gCur, tip);
  pathPt[pathN][0] = tip[0];
  pathPt[pathN][1] = tip[1];
  pathPt[pathN][2] = tip[2];
  pathN++;
  Serial.print(F("路径第 "));
  Serial.print(pathN);
  Serial.print(F(" / "));
  Serial.print(DRAW_MAXPTS);
  Serial.print(F(" 点  "));
  printTip(tip);
}

// ---------- 统一记点入口：串口 6 与按键板 F2 按键3 共用 ----------
void teachRecordOne() {
  if (pathN == 0) {
    Serial.println(F("[示教] 记第 1 点"));
    drawBegin();
    return;
  }
  if (pathN >= DRAW_MAXPTS) {
    Serial.print(F("[示教] 已记满 "));
    Serial.print(DRAW_MAXPTS);
    Serial.println(F(" 个点，自动清空，从第 1 点重新记："));
    drawBegin();
    return;
  }
  drawAppend();
  Serial.print(F("[示教] 已记 "));
  Serial.print(pathN);
  Serial.print(F(" / "));
  Serial.print(DRAW_MAXPTS);
  if (pathN >= DRAW_MAXPTS) {
    Serial.println(F(" 点（已满）—— 按4开始绘制，或按3清空重记"));
  } else {
    Serial.println(F(" 点 —— 继续按3记点，或按4开始绘制"));
  }
}

// ---------- 10.16b · N：一键画直线（预存两个姿态，收到即画）----------
void drawStart();
void cmdLine() {
  if (rejectWhileAuto("N/直线")) return;
  for (int k = 0; k < 2; k++) fkTip(LINE_ANG[k], pathPt[k]);     // 角 → 坐标（执行时正解）
  pathN = 2;
  Serial.println(F("一键直线（用预存角度 LINE_ANG，执行时正解→逆解）："));
  for (int k = 0; k < 2; k++) printTip(pathPt[k]);
  drawStart();
}

// ---------- 10.16b · Z 命令：字母 Z（4 组预存角度，收到即画）----------
void figZ() {
  if (rejectWhileAuto("Z")) return;
  for (int k = 0; k < 4; k++) fkTip(Z_ANG[k], pathPt[k]);     // 角 → 坐标（执行时正解）
  pathN = 4;

  Serial.println(F("已生成字母 Z：4 个顶点 / 3 段（用预存角度 Z_ANG，执行时正解→逆解）"));
  for (int k = 0; k < 4; k++) {
    Serial.print(F("   顶点 "));
    Serial.print(k + 1);
    Serial.print(F("  "));
    printTip(pathPt[k]);
  }
  drawStart();
}

// ---------- 8：绘制整条路径----------
void drawStart() {
  if (rejectWhileAuto("绘制/开始")) return;
  if (pathN < 2) {
    Serial.println(F("先记路径点：USB 敲 6，或按键板【F2】按键3"));
    Serial.println(F("每记一次 = 一个顶点；满 5 点自动清空重记"));
    Serial.println(F("记满 2 点以上再敲 8（F2 模式按键4）开始画；画字母 Z 直接发 Z"));
    return;
  }
  drawSegs = pathN - 1;

  // 先整条路径逐点 IK 预检，全程可解再动臂（ikTip 成功即三轴都在限位内，无需再查）
  for (int s = 0; s < drawSegs; s++) {
    int steps = drawSegSteps(s);
    for (int k = 0; k <= steps; k++) {
      float p[3];
      drawPointAt(s, k, steps, p);
      int servo[3];
      if (!ikTip(p, servo)) {
        Serial.print(F("第 ")); Serial.print(s + 1);
        Serial.print(F(" 段第 ")); Serial.print(k);
        Serial.print(F(" 点不可达  ")); printTip(p);
        Serial.println(F("——太远/太近，或该点落在臂背后"));
        return;
      }
    }
  }

  float total = 0.0;
  for (int s = 0; s < drawSegs; s++) {
    const float *a = pathPt[s];
    const float *b = pathPt[s + 1];
    total += sqrt(sq(b[0] - a[0]) + sq(b[1] - a[1]) + sq(b[2] - a[2]));
  }
  Serial.print(F("绘制："));
  Serial.print(pathN);
  Serial.print(F(" 个顶点 / "));
  Serial.print(drawSegs);
  Serial.print(F(" 段，总长 "));
  Serial.print(total, 1);
  Serial.println(F(" mm，开始（F2 模式按键1=暂停/继续、按键2=取消；USB 可敲 J/K/Q；T 急停）"));
  Serial.println(F("→ 先返回待机位置，再移动到第 1 个目标点"));

  //必须先把 drawActive 立起来：phase 0 期间 busy 由回中那段自己置，drawActive 没置的话 rejectWhileAuto 拦不住
  drawSeg    = 0;
  drawIdx    = 0;
  drawActive = true;
  drawPaused = false;
  drawPhase  = 0;
}

// ---------- loop() 每圈调一次：上一步走完就投下一步 ----------
void drawTick() {
  if (!drawActive) return;
  if (drawPaused) return;
  if (motionBusy()) return;

  if (drawPhase == 0) {
    int to[4] = {90, 90, 90, gCur[3]};
    printEst(estSyncMs(90, 90, 90));
    moveArm(to, DSDdata[DSDlevel]);
    drawPhase = 1;
    return;
  }
  if (drawPhase == 1) {
    float zmax = pathPt[0][2];
    for (int k = 1; k < pathN; k++) if (pathPt[k][2] > zmax) zmax = pathPt[k][2];
    float above[3] = {pathPt[0][0], pathPt[0][1], zmax + DRAW_LIFT_MM};
    if (!moveTipTo(above)) { drawAbort(F("首点上方不可达")); return; }
    drawPhase = 2;
    return;
  }
  if (drawPhase == 2) {
    float p[3] = {pathPt[0][0], pathPt[0][1], pathPt[0][2]};
    if (!moveTipTo(p)) { drawAbort(F("落笔点不可达")); return; }
    drawPhase = 3;
    return;
  }
  if (drawPhase == 3) {
    drawPhase = 4;
    drawSeg   = 0;
    drawIdx   = 0;
    drawSteps = drawSegSteps(0);
    return;
  }
  if (drawPhase == 4) {
    drawIdx++;
    if (drawIdx > drawSteps) {
      if (drawSeg + 1 >= drawSegs) {
        float p[3] = {pathPt[pathN - 1][0], pathPt[pathN - 1][1],
                      pathPt[pathN - 1][2] + DRAW_LIFT_MM};
        if (!moveTipTo(p)) { drawAbort(F("收尾抬笔不可达")); return; }
        drawPhase = 5;
        return;
      }
      drawSeg++;
      drawIdx   = 0;
      drawSteps = drawSegSteps(drawSeg);
      return;
    }
    float p[3];
    drawPointAt(drawSeg, drawIdx, drawSteps, p);
    if (!moveTipTo(p)) { drawAbort(F("中途 IK 失败")); return; }
    return;
  }
  if (drawPhase == 5) {
    int to[4] = {90, 90, 90, gCur[3]};
    printEst(estSyncMs(90, 90, 90));
    moveArm(to, DSDdata[DSDlevel]);
    drawActive = false;
    drawPhase  = 0;
    busy       = true;
    Serial.println(F("绘制完成 done，已返回待机"));
    return;
  }
  if (drawPhase == 6) {
    int to[4] = {90, 90, 90, gCur[3]};
    printEst(estSyncMs(90, 90, 90));
    moveArm(to, DSDdata[DSDlevel]);
    drawActive = false;
    drawPhase  = 0;
    busy       = true;
    Serial.println(F("绘制已取消，返回待机"));
    return;
  }
}

// ---------- 任务三 ⑤ 按键分发入口 ----------
void onKeyChar(char c) {
  if (c < '1' || c > '5') return;

  // ⚠ 回显当前模式：一眼就能看出主控在哪个模式（断电/复位会退回模式 1）
  Serial.print(F("[按键板 "));
  Serial.print(gKeyMode == 2 ? F("示教") : F("任务三"));
  Serial.print(F("] 键"));
  Serial.println(c);

  // ---- 模式 2（示教）：所有键都在这里处理完，一个都不许落到下面的任务三分支 ----
  if (gKeyMode == 2) {
    if (c == '1') {
      if (!drawActive) { Serial.println(F("当前没有绘制任务")); return; }
      drawPaused = !drawPaused;
      if (drawPaused) Serial.println(F("已暂停（保持当前状态）—— 再按接续 / 按键2 取消"));
      else            Serial.println(F("继续：从暂停位置接续绘制"));
    }
    else if (c == '2') { drawCancel(); }
    else if (c == '3') { if (canTeachNow("按键3记点")) teachRecordOne(); }
    else if (c == '4') { drawStart(); }
    else               { Serial.println(F("示教模式下按键5 没有功能")); }
    return;
  }

  // ---- 模式 1（任务三）----
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
  else {
    printRec();
  }
}

// ---------- 命令表 ----------
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
  Serial.println(F("F1 / F2        切按键板模式(1=任务三 2=示教：记点/画线/暂停/继续/取消)"));
  Serial.println(F("--- 任务五 part1：绘制路径(直线 / 折线) ---"));
  Serial.println(F("N             一键画直线(用预存角度 LINE_ANG，改表可调)"));
  Serial.println(F("6             记一个路径点(第 1 次开新路径，之后追加；满 5 点自动清空重记)"));
  Serial.println(F("8             自动绘制整条路径(需先记满 2 点；回中-抬笔-平移-落笔-逐段画-抬笔)"));
  Serial.println(F("--- 任务五 part2：基础图形绘制(字母 Z) ---"));
  Serial.println(F("Z             一键画字母 Z(用预存角度 Z_ANG 的 4 个顶点)"));
  Serial.println(F("--- 任务五(3)：绘图过程控制(USB 敲 J/K/Q；按键板 F2 后按键1/2) ---"));
  Serial.println(F("J             暂停绘制(保持当前状态)"));
  Serial.println(F("K             从暂停位置继续绘制"));
  Serial.println(F("Q             取消绘制(抬笔后返回待机，爪不动)"));
  Serial.println(F("--- 任务五(4)：五点示教折线(USB 敲 6 / 8；按键板 F2 后按键3记点 / 按键4画) ---"));
  Serial.println(F("6 8           记点 → 绘制；画前自动回待机，画完自动回待机"));
  Serial.println(F("I120,0,30     笔尖坐标单点测试(IK标定用,单位mm)"));
  Serial.println(F("提示：动作执行中会拒绝新指令，等 done 再发。"));
}

// ---------- 抠整数：字母 / 逗号 / 空格一律跳过 ----------
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

// ---------- 命令分派：看首字符 ----------
void armDataCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char first = cmd.charAt(0);

  if (first >= '1' && first <= '5') { onKeyChar(first); return; }
  else if (first == 'F' || first == 'f') {
    char m = (cmd.length() >= 2) ? cmd.charAt(1) : '0';
    if (m >= '1' && m <= '2') {
      if (rejectWhileAuto("F 模式切换")) return;
      gKeyMode = m - '0';
      Serial.print(F("按键板模式 = "));
      Serial.println(gKeyMode);
      if (gKeyMode == 1) Serial.println(F("  按键1/2/3/4 = 夹取循环/录制/播放/回中"));
      else               Serial.println(F("  示教（任务五(3)(4)）：1=暂停/继续 2=取消 3=记点 4=开始画"));
    } else {
      Serial.println(F("格式：F1（任务三）/ F2（示教：记点/画线 + 暂停/继续/取消）"));
    }
    return;
  }
//这三条不走 rejectWhileAuto
  else if (first == 'J' || first == 'j') { drawPause();  return; }
  else if (first == 'K' || first == 'k') { drawResume(); return; }
  else if (first == 'Q' || first == 'q') { drawCancel(); return; }
  else if (first == '6') { if (canTeachNow("6")) teachRecordOne(); return; }
  else if (first == '8') { drawStart(); return; }
  else if (first == 'N' || first == 'n') { cmdLine(); return; }
  else if (first == 'Z' || first == 'z') { figZ(); return; }
  else if (first == 'x' || first == 'X') {
    if (rejectWhileBusy("x/y/z")) return;
    int got[3];
    if (parseInts(cmd, got, 3) < 3) {
      Serial.println(F("格式应为 x90,y30,z20"));
      return;
    }
    int v[4] = {got[0], got[1], got[2], gCur[3]};
    if (!withinLimits(v)) return;
    printEst(estSyncMs(v[0], v[1], v[2]));
    moveArm(v, DSDdata[DSDlevel]);
    busy = true;
    Serial.print(F("x=")); Serial.print(v[0]);
    Serial.print(F(" y=")); Serial.print(v[1]);
    Serial.print(F(" z=")); Serial.println(v[2]);
  }
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
    Serial.print(F("目标 b=")); Serial.print(v[0]);
    Serial.print(F("  f="));    Serial.print(v[1]);
    Serial.print(F("  r="));    Serial.print(v[2]);
    Serial.print(F("  c="));    Serial.println(v[3]);
  }
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
  else if (first == 'A' || first == 'a') { if (!rejectWhileAuto("A")) runSequence(0); }
  else if (first == 'B' || first == 'b') { if (!rejectWhileAuto("B")) runSequence(1); }
  else if (first == 'C' || first == 'c') { if (!rejectWhileAuto("C")) runSequence(2); }
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
  else if (first == 'H' || first == 'h') {
    if (DSDlevel < 3) DSDlevel++;
    Serial.print(F("速度档 ")); Serial.println(DSDlevel);
  }
  else if (first == 'L' || first == 'l') {
    if (DSDlevel > 0) DSDlevel--;
    Serial.print(F("速度档 ")); Serial.println(DSDlevel);
  }
  else if (first == 'P' || first == 'p') { printAngles(); }
  else if (first == 'G' || first == 'g') {
    if (rejectWhileAuto("G")) return;
    goHome();
    busy = true;
  }
  else if (first == '?') { printHelp(); }
  else if (first == 'T' || first == 't') { emergencyStop(); }
  else {
    Serial.println(F("未知命令，发 ? 查看命令表"));
  }
}

// ---------- 串口组帧 ----------
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
      if (inputBuffer.length() > 32) inputBuffer = "";
    }
  }

  while (espSerial.available() > 0) {
    char c = espSerial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    if (c == '?') {                                        // 链路自检：只回一句，不动臂
      Serial.println(F("+LINK OK"));
      continue;
    }
    onKeyChar(c);
  }
}

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  for (int i = 0; i < 4; i++) {
    gServo[i].attach(PIN[i]);
    delay(200);
  }
  for (int i = 0; i < 4; i++) {
    gCur[i] = 90;
    gServo[i].write(90);
  }

  Serial.println(F("ready"));
  Serial.println(F("按键板模式 1（任务三键位）—— 主控串口发 F2 = 示教（记点/画线 + 暂停/继续/取消）"));
  joyCalibrate();
  printHelp();
  selfCheck();
  Serial.println(F("注意：A/B/C 关键点已按本机回填；换物体/摆放后需重新标定"));
}

void loop() {
  readSerial();

  joyTick();
  motionTick();
  seqTick();
  recTick();
  playTick();
  drawTick();

  if (busy && !seqActive && !playing && !drawActive && !motionBusy()) busy = false;
}

// ---------- 标定 / 操作流程见 10.16b_程序逻辑说明_评审版.md；命令表发 ? 看 ----------
