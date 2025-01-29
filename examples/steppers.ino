#include "KMTRC.h" // Подключение библиотеки робоконтроллера

KMTRC trbot; // Инициализация объекта Робоконтроллера "trbot" для дальнейшего использования его функций

void setup()
{
  trbot.begin();                          // Инициализируем Робоконтроллер
  trbot.setStepperSpeed(1, 60);   // Устанавливаем значение скорости для шагового мотора 1
  trbot.setStepperSpeed(2, 60);   // Устанавливаем значение скорости для шагового мотора 2
}
void loop()
{
  Serial.println("clockwise");
  trbot.setStepperStep(1, 1000);  // Запускаем вращение шагового мотора 1 на 1000 шагов в прямом направлении
  delay(500);

  Serial.println("counterclockwise");
  trbot.setStepperStep(2, -1000);  // Запускаем вращение шагового мотора 2 на 1000 шагов в обратном направлении
  delay(500);
}