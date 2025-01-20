#include "KMOneCore.h" // Подключение библиотеки робоконтроллера

KMOneCore OneCore; // Инициализация объекта Робоконтроллера "OneCore" для дальнейшего использования его функций

void setup()
{
  OneCore.begin(); // Инициализируем Робоконтроллер
}

void loop()
{
  mechanumWheelsTest(60, 3000); // Вызов функции "mecanumWheelsTest" с указанием скорости и задержки в миллисекундах
}

// Функция для проверки движения робота с колёсами типа mecanum
void mechanumWheelsTest(uint8_t speed, uint16_t delay_time_ms)
{
  delay(delay_time_ms);
  OneCore.mecanumMoveForward(speed);  // Задаём движение вперёд
  delay(delay_time_ms);
  OneCore.mecanumMoveBackward(speed); // Задаём движение назад
  delay(delay_time_ms);
  OneCore.mecanumrotateRight(speed); // Задаём вращение на месте вправо
  delay(delay_time_ms);
  OneCore.mecanumrotateLeft(speed); // Задаём вращение на месте влево
  delay(delay_time_ms);
  OneCore.mecanumMoveSidewaysRight(speed); // Задаём движение боком вправо
  delay(delay_time_ms);
  OneCore.mecanumMoveSidewaysLeft(speed); // Задаём движение боком влево
  delay(delay_time_ms);
  OneCore.mecanumMoveRightForward(speed); // Задаём движение вперёд направо под углом 45 градусов
  delay(delay_time_ms);
  OneCore.mecanumMoveLeftForward(speed); // Задаём движение вперёд налево под углом 45 градусов
  delay(delay_time_ms);
  OneCore.mecanumStopMoving(speed); // Останавливаем движение робота
  delay(delay_time_ms);
}
