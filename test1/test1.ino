// C++ code
//
#include <Servo.h>
Servo c;
  int dataIn = 0;

void setup()
{
  c.attach(6);
 
  Serial.begin(9600);
  Serial.println("please:");
}

void loop()
{
  if (Serial.available()){
  char name = Serial.read();
    Serial.println(name);
    
    int data = Serial.parseInt();
    
    switch(name){

      case'c':
      c.write(data);
      break;
  }
  }

}
