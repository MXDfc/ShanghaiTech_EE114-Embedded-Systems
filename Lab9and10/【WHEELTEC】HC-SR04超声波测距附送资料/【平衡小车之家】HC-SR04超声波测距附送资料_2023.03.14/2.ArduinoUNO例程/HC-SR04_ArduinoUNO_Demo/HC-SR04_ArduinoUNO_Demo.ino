
/////////////HC-SR04引脚接线//////////////
//HC-SR04丝印标识----------ArduinoUNO主板引脚
//           GND----------GND
//           VCC----------5V
//          Trig----------12
//          Echo----------13
//HC-SR04丝印标识----------ArduinoUNO主板引脚

//设定HC-SR04连接的Arduino引脚
#define TrigPin 12 
#define EchoPin 13 

double distance; //定义距离变量

void setup()
{  
  //初始化串口通信及连接SR04的引脚
  Serial.begin(9600);
  pinMode(TrigPin, OUTPUT); //超声波发射端
  pinMode(EchoPin, INPUT); //超声波接收端
}
void loop()
{
  //产生一个10us的高脉冲去触发TrigPin，发送8个40KHz的超声波脉冲
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  
  //pulseIn(A, H)函数,A为引脚名称，H为电平HIGH/LOW，用于检测引脚A电平H的持续时间t，单位为us
  //t为超声波模块Trig发射超声波到Echo接收超声波之间的时间
  //则距离distance=v*（t/1000）/2=0.01715*t=t/58.3，v=34300cm/s（音速），1s=100000us，2是因为t是来回时间
  distance = pulseIn(EchoPin, HIGH) / 58.3;                                                
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
  delay(500);
}
