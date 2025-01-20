#include "KMOneCore.h" // Подключение библиотеки робоконтроллера

KMOneCore OneCore; // Инициализация объекта Робоконтроллера "OneCore" для дальнейшего использования его функций

void setup()
{
  OneCore.begin();                          // Инициализируем Робоконтроллер
  OneCore.setStepperSpeed(1, 60);   // Устанавливаем значение скорости для шагового мотора 1
  OneCore.setStepperSpeed(2, 60);   // Устанавливаем значение скорости для шагового мотора 2
}
void loop()
{
  Serial.println("clockwise");
  OneCore.setStepperStep(1, 1000);  // Запускаем вращение шагового мотора 1 на 1000 шагов в прямом направлении
  delay(500);

  Serial.println("counterclockwise");
  OneCore.setStepperStep(2, -1000);  // Запускаем вращение шагового мотора 2 на 1000 шагов в обратном направлении
  delay(500);
}