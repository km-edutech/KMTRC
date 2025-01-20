#include "KMOneCore.h" // Подключение библиотеки робоконтроллера

KMOneCore OneCore; // Инициализация объекта Робоконтроллера "OneCore" для дальнейшего использования его функций

int valpot = 0;                               // Инициализируем переменную для значения угла поворота потенциометра

void setup()
{
  OneCore.begin();                          // Инициализируем Робоконтроллер
}

void loop()
{
  valpot = OneCore.ADCRead(1);  // Читаем значения потенциометра, подключеного к порту ADC 1

  int angleServo = map(valpot, 0, 4095, 0, 180);    // Приводим значения с порта ADC в соотвествие с значениями углов для сервоприводов

  OneCore.setAnalogServoAngle(1, angleServo);    // Устанавливаем угол сервомотора, подключенного к порту SRV 1
  OneCore.setAnalogServoAngle(2, angleServo);    // Устанавливаем угол сервомотора, подключенного к порту SRV 2
  OneCore.setAnalogServoAngle(3, angleServo);    // Устанавливаем угол сервомотора, подключенного к порту SRV 2
}