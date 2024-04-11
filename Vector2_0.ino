/*
  made by Nebula Technology
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050.h>
#include <Math.h>



Adafruit_MPU6050 mpu;  //SCL--5
                       //SDA--4
                       //3.3V供电
MPU6050 accelgyro;
int x = 0;
int y = 0;
int z = 0;

unsigned long now, lastTime = 0;
float dt;                                   //微分时间

int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    //角度变量
long axo = 0, ayo = 0, azo = 0;             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //加速度计比例系数
float GyroRatio = 131.0;                    //陀螺仪比例系数

uint8_t n_sample = 8;                       //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,y轴采样队列
long aax_sum, aay_sum,aaz_sum;                      //x,y轴采样和
 
float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //加速度计协方差计算队列
float Px=1, Rx, Kx, Sx, Vx, Qx;             //x轴卡尔曼变量
float Py=1, Ry, Ky, Sy, Vy, Qy;             //y轴卡尔曼变量
float Pz=1, Rz, Kz, Sz, Vz, Qz;             //z轴卡尔曼变量

Servo servo1; 
Servo servo2;
Servo servo_pin_2;
//Servo servo3;
//Servo servo4;

int value  = 0;

void setup() {
  Serial.begin(115200);
  //Wire.begin();

  // 初始化
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  accelgyro.initialize(); 
  Serial.println("MPU6050 Found!");

  // 将加速度计范围设置为 +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // 将陀螺仪范围设置为 +- 500 度/秒
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // 将滤波器带宽设置为 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  unsigned short times = 200;             //采样次数
    for(int i=0;i<times;i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
        axo += ax; ayo += ay; azo += az;      //采样和
        gxo += gx; gyo += gy; gzo += gz;
    
    }
    
    axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
    gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移

  //delay(100);
  servo1.attach(8);
  servo2.attach(6); 
  //servo3.attach(6); 
  //servo4.attach(5); 
  servo_pin_2.attach(9);
  
  servo1.write(90);
  servo2.write(90);
  //servo3.write(0);
  //servo4.write(0);
  servo_pin_2.write( 180 );
  Serial.println("Everything is OK");
}

void loop() {
  /* 使用读数获取新的传感器事件 */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
  
//Serial.print(x);Serial.print(" ");Serial.println(y);Serial.print(" ");
  
if (x < 10 && x > 0 && y < 4 && y > -4  && y != 0){
  Serial.println("up");
   value = map(x,  0, 10, 0, 180);
   servo1.write(90+value);
   Serial.print(90+value);
   delay(0);
  }
else if (x > -10 && x < 0 && y < 4 && y > -4  && y != 0){   //2
  Serial.println("down");
  value = map(x,  -10, 0, 180, 0);
  servo1.write(90-value);
  Serial.print(90-value);
  delay(0);
  }

if (y < 10 && y > 0 && x < 4 && x > -4 && x != 0){      //3
  Serial.println("Right");
  value = map(y,  0, 10, 0, 180);
  servo2.write(90-value);
  Serial.print(90-value);
  delay(0);
  }
else if (y > -10 && y < 0  && x < 4 && x > -4  && x != 0){
  Serial.println("left");
  value = map(y,  -10, 0, 180, 0);
  servo2.write(90+value);
  Serial.print(90+value);
  delay(0);
  }
  
if (x == 0 && y == 0){
  servo1.write(90);
  servo2.write(90);
  }

  unsigned long now = millis();             //当前时间(ms)
    dt = (now - lastTime) / 1000.0;           //微分时间(s)
    lastTime = now;                           //上一次采样时间(ms)
 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
 
    float accx = ax / AcceRatio;              //x轴加速度
    float accy = ay / AcceRatio;              //y轴加速度
    float accz = az / AcceRatio;              //z轴加速度
 
    aax = atan(accy / accz) * (-180) / pi;    //y轴对于z轴的夹角
    aay = atan(accx / accz) * 180 / pi;       //x轴对于z轴的夹角
    aaz = atan(accz / accy) * 180 / pi;       //z轴对于y轴的夹角
 
    aax_sum = 0;                              // 对于加速度计原始数据的滑动加权滤波算法
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //角度调幅至0-90°
    aays[n_sample-1] = aay;                        //此处应用实验法取得合适的系数
    aay_sum += aay * n_sample;                     //本例系数为9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;
 
    float gyrox = - (gx-gxo) / GyroRatio * dt; //x轴角速度
    float gyroy = - (gy-gyo) / GyroRatio * dt; //y轴角速度
    float gyroz = - (gz-gzo) / GyroRatio * dt; //z轴角速度
    agx += gyrox;                             //x轴角速度积分
    agy += gyroy;                             //x轴角速度积分
    agz += gyroz;
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //算测量值的平均值
        a_x[i-1] = a_x[i];                      //加速度平均值
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 //x轴加速度平均值
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 //y轴加速度平均值
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;
 
    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              //得到方差
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         
    Kx = Px / (Px + Rx);                      //计算卡尔曼增益
    agx = agx + Kx * (aax - agx);             //陀螺仪角度与加速度计速度叠加
    Px = (1 - Kx) * Px;                       //更新p值
 
    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;


    
    if(agx>70)//开伞角度
  {
    digitalWrite(8,1);
    servo_pin_2.write( 0 );
    delay(2000);
    
  
Serial.print("开伞");
while(1)
 {
  }
    }
  else
  {
     digitalWrite(8,1);
  }
  if(agy>70)//开伞角度
  {
    digitalWrite(8,1);
    servo_pin_2.write( 0 );
    delay(2000);
    
  
Serial.print("开伞");
while(1)
{
  }
    }
  else
  {
     digitalWrite(8,1);
  }

  if(agy<-70)//开伞角度
  {
    digitalWrite(8,1);
    servo_pin_2.write( 0 );
    delay(2000);
    
  
Serial.print("开伞");
while(1)
{
  }
    }
  else
  {
     digitalWrite(8,1);
  }

  if(agx<-70)//开伞角度
  {
    digitalWrite(8,1);
    servo_pin_2.write( 0 );
    delay(2000);
    
  
Serial.print("开伞");
while(1)
{
  }
    }
  else
  {
     digitalWrite(8,1);
  }
 
 
    //Serial.print(agx);Serial.print(",");
    //Serial.print(agy);Serial.print(",");
    //Serial.print(agz);Serial.println();
}
