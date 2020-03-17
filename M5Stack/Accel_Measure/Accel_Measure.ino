#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "BluetoothSerial.h"
#include <MadgwickAHRS.h>


MPU9250 IMU;

Madgwick filter;  // 姿勢を計算するオブジェクトを作る
unsigned long microsPerReading, microsPrevious;
int Tr=5;
const int PressPin=36;
int PressValue=0;
float start;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32test1"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
    M5.begin();
    M5.Lcd.setCursor(0, 0, 2);
    M5.Lcd.fillScreen(BLACK);
    Serial.begin(115200);
    Wire.begin();
    if (IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 0x71) {
        M5.Lcd.print("cannnot find MPU9250");
        while (true) ;
    }
    //リレー使用
    //スピーカーノイズ対策
    dacWrite(25, 0);
    pinMode(Tr, OUTPUT);
    
    IMU.initMPU9250();  // MPU9250を初期化する

// ##################################################################################
    //IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);  // キャリブレートする
// ##################################################################################

    filter.begin(1000);  // 1000Hz  filterを初期化する

    microsPerReading = 20000000 / 1000;//20msecごと
    microsPrevious = micros();
    start = millis();
    
    M5.Lcd.clear();
    M5.Lcd.setBrightness(200);
    
    M5.Lcd.print("Working");
}

#define INTERVAL 5
float roll, pitch, yaw;
bool once = true;
int Btn = 0;
int Count = 0;
int i = 0;
int Signal = 0;
int Contact = 0;
int CountTime = 0;
int Flag ;//= false;
float timing;
byte Signalbyte = 0x00;
byte Finish = 0x00;
int Set = 0;

void loop() {

  timing =( millis() - start )/1000;
  M5.update();

    if (micros() - microsPrevious > microsPerReading) {
        
        while (!(IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)) ;
        IMU.readAccelData(IMU.accelCount);  // 加速度の生データーを取得する
        IMU.getAres();  // スケール値を取得する
        // x/y/z軸の加速度値を計算する
        IMU.ax = (float)IMU.accelCount[0] * IMU.aRes - IMU.accelBias[0];
        IMU.ay = (float)IMU.accelCount[1] * IMU.aRes - IMU.accelBias[1];
        IMU.az = (float)IMU.accelCount[2] * IMU.aRes - IMU.accelBias[2];

        IMU.readGyroData(IMU.gyroCount);  // ジャイロの生データーを取得する
        IMU.getGres();  // スケール値を取得する
        // x/y/z軸のジャイロ値を計算する
        IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes - IMU.gyroBias[0];
        IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes - IMU.gyroBias[0];
        IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes - IMU.gyroBias[0];

        // Display to M5Stack
        //M5.Lcd.setCursor(0, 5, 2);
        //M5.Lcd.printf("%6.2f %6.2f %6.2f", IMU.ax, IMU.ay, IMU.az);
        //M5.Lcd.setCursor(0, 40);
        //M5.Lcd.printf("%6.2f %6.2f %6.2f", IMU.gx, IMU.gy, IMU.gz);

        filter.updateIMU(IMU.gx, IMU.gy, IMU.gz, IMU.ax, IMU.ay, IMU.az);
        
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();

        // Display to M5Stack
        //M5.Lcd.setCursor(0, 75);
        //M5.Lcd.printf("\t%f\t%f\t%f, \t%f\t%f\t%f",IMU.ax,IMU.ay,IMU.az, roll, pitch, yaw);
        
        SerialBT.printf("\t%.4f\t%.4f\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\r\n",timing,IMU.ax,IMU.ay,IMU.az, roll, pitch, yaw);


        SerialBT.flush();
        microsPrevious = microsPrevious + microsPerReading;
    }
}
