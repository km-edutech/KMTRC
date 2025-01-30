#include "KMTRC.h"          // Подключение библиотеки робоконтроллера
#include <Adafruit_MPU6050.h>  // Подключение библиотеки MPU6050

KMTRC trbot;                // Инициализация объекта Робоконтроллера "trbot" для дальнейшего использования его функций
Adafruit_MPU6050 mpu;              // Инициализация объекта гироскопа-акселерометра "mpu" для дальнейшего использования его функций

void setup()
{
  trbot.begin();                          // Инициализируем Робоконтроллер
  Wire.begin(32, 33);                    // Инициализация шины I2C для работы с MPU6050

  if (!mpu.begin()) {                       // Прооверка успешности инициализации MPU6050
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);     // Установка предела измерений акселерометра MPU6050
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);             // Установка предела измерений гироскопа MPU6050
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);              // Установка фильтра измерений MPU6050
}

void loop()
{
    sensors_event_t a, g, temp;             // Объявление переменных для значений акселерометра и гироскопа
    mpu.getEvent(&a, &g, &temp);        // Получение значений акселерометра и гироскопа
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);           // Вывод значения ускорения по оси X акселерометра
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);          // Вывод значения ускорения по оси Y акселерометра
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);          // Вывод значения ускорения по оси Z акселерометра
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);                      // Вывод значения гироскопа по оси X 
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);                      // Вывод значения гироскопа по оси Y 
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);                      // Вывод значения гироскопа по оси Z 
    Serial.println("");
}