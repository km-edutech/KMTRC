#include "KMTRC.h" // Подключение библиотеки робоконтроллера

KMTRC trbot; // Инициализация объекта Робоконтроллера "trbot" для дальнейшего использования его функций

void setup()
{
  trbot.begin(); // Инициализируем Робоконтроллер
}

void loop()
{
  mechanumWheelsTest(60, 3000); // Вызов функции "mecanumWheelsTest" с указанием скорости и задержки в миллисекундах
}

// Функция для проверки движения робота с колёсами типа mecanum
void mechanumWheelsTest(uint8_t speed, uint16_t delay_time_ms)
{
  delay(delay_time_ms);
  trbot.mecanumMoveForward(speed);  // Задаём движение вперёд
  delay(delay_time_ms);
  trbot.mecanumMoveBackward(speed); // Задаём движение назад
  delay(delay_time_ms);
  trbot.mecanumrotateRight(speed); // Задаём вращение на месте вправо
  delay(delay_time_ms);
  trbot.mecanumrotateLeft(speed); // Задаём вращение на месте влево
  delay(delay_time_ms);
  trbot.mecanumMoveSidewaysRight(speed); // Задаём движение боком вправо
  delay(delay_time_ms);
  trbot.mecanumMoveSidewaysLeft(speed); // Задаём движение боком влево
  delay(delay_time_ms);
  trbot.mecanumMoveRightForward(speed); // Задаём движение вперёд направо под углом 45 градусов
  delay(delay_time_ms);
  trbot.mecanumMoveLeftForward(speed); // Задаём движение вперёд налево под углом 45 градусов
  delay(delay_time_ms);
  trbot.mecanumStopMoving(speed); // Останавливаем движение робота
  delay(delay_time_ms);
}
