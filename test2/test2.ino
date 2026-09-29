// C++ code
//
#include <Servo.h>
Servo base, fArm, rArm, claw;
  int dataIn = 0;
  int DSDdata[4] = {10,15,23,40};
  int DSDlevel = 1;

const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 45;
const int rArmMax = 180;
const int fArmMin = 35;
const int fArmMax = 120;
const int clawMin = 25;
const int clawMax = 100;

  String inputBuffer = "";

void servoCmd(char servoName, int toPos, int servoDelay);

void setup()
{

  Serial.begin(9600);
  base.attach(9);     // base 伺服舵机连接引脚11 舵机代号'b'
  delay(200);          // 稳定性等待
  rArm.attach(8);     // rArm 伺服舵机连接引脚10 舵机代号'r'
  delay(200);          // 稳定性等待
  fArm.attach(7);      // fArm 伺服舵机连接引脚9  舵机代号'f'
  delay(200);          // 稳定性等待
  claw.attach(6);      // claw 伺服舵机连接引脚6  舵机代号'c'
  delay(200);          // 稳定性等待
 

  base.write(90); 
  delay(10);
  rArm.write(90); 
  delay(10);
  fArm.write(90); 
  delay(10);
  claw.write(90);  
  delay(10); 
} 


void loop()
{
    readSerial(); 

}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        armDataCmd(inputBuffer);
        inputBuffer = "";
      }
    }
    else {
      inputBuffer += c;
      if (inputBuffer.length() > 32) inputBuffer = "";  // 防溢出
    }
  }
}

void armDataCmd(String cmd) {
  cmd.trim();                       // 去掉空格、换行
  if (cmd.length() == 0) return;

  char first = cmd.charAt(0);       // 取首字符

  // ===== 1. x,y,z 指令 =====
  // x为底座，y为后臂，z为前臂
  if (first == 'x' || first == 'X') {
    int servoData[3];
    int p1 = cmd.indexOf(',');
    int p2 = cmd.indexOf(',', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      servoData[0] = cmd.substring(1, p1).toInt();
      servoData[1] = cmd.substring(p1 + 1, p2).toInt();
      servoData[2] = cmd.substring(p2 + 1).toInt();
      
      servoCmd('b', servoData[0], DSDdata[DSDlevel]);//执行运动
      servoCmd('r', servoData[1], DSDdata[DSDlevel]);
      servoCmd('f', servoData[2], DSDdata[DSDlevel]);
      
      //targetPos[0] = x;  targetPos[1] = y;  targetPos[2] = z;
      Serial.print("x="); Serial.print(servoData[0]);
      Serial.print(" y="); Serial.print(servoData[1]);
      Serial.print(" z="); Serial.println(servoData[2]);
    }

    
  else if (first == '0' || first == 'O' || first == 'o') {
    // 开爪
    Serial.println("Open claw");
  }
  else if (first == 'S' || first == 's') {
    // 关爪
    Serial.println("Close claw");
  }
  else if (first == 'H' || first == 'h') {
    // 提速
     if (DSDlevel<3){
      DSDlevel++;
    Serial.println("Speed up");
    }else{
      Serial.println("Highest Speed");
    }
  }
  else if (first == 'L' || first == 'l') {
    // 降速
    if (DSDlevel>0){
      DSDlevel--;
       Serial.println("Speed down");
    }else{
      Serial.println("Lowest Speed");
    }
  }
  else {
    Serial.println("Unknown command");
    }
  }
}

void servoCmd(char servoName, int toPos, int servoDelay){  //执行运动
  Servo servo2go;  //创建servo对象
 
  //串口监视器输出接收指令信息
  Serial.println("");
  Serial.print("+Command: Servo ");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");
  Serial.println("");  
  
  int fromPos; //建立变量，存储电机起始运动角度值
  
  switch(servoName){
    case 'b':
      if(toPos >= baseMin && toPos <= baseMax){
        servo2go = base;
        fromPos = base.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: Base Servo Value Out Of Limit!");
        return;
      }
 
    case 'c':
      if(toPos >= clawMin && toPos <= clawMax){    
        servo2go = claw;
        fromPos = claw.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: Claw Servo Value Out Of Limit!");
        return;        
      }
 
    case 'f':
      if(toPos >= fArmMin && toPos <= fArmMax){
        servo2go = fArm;
        fromPos = fArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: fArm Servo Value Out Of Limit!");
        return;
      }
         
    case 'r':
      if(toPos >= rArmMin && toPos <= rArmMax){
        servo2go = rArm;
        fromPos = rArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: rArm Servo Value Out Of Limit!");
        return;
      }      
  }
 
  //指挥电机运行
  if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
    for (int i=fromPos; i<=toPos; i++){
      servo2go.write(i);
      delay (servoDelay);
    }
  }  else { //否则“起始角度值”大于“目标角度值”
    for (int i=fromPos; i>=toPos; i--){
      servo2go.write(i);
      delay (servoDelay);
    }
  }
}

//      case'b':
//      b.write(data);
//      break;
//      
//      case'f':
//      f.write(data);
//      break;
