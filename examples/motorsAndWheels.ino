#include "KMOneCore.h" // Подключение библиотеки робоконтроллера

KMOneCore OneCore; // Инициализация объекта Робоконтроллера "OneCore" для дальнейшего использования его функций

void setup()
{
  OneCore.begin(); // Инициализируем Робоконтроллер
}

void loop()
{
  OneCore.moveWheel(1, 50);          // Запускаем вращение колеса 1 со скоростью 50 в прямом направлении
  delay(1000);                     // Ждём 1 секунду
  OneCore.moveWheel(1, 0);           // Останавливаем вращение колеса 1
  delay(1000);                     // Ждём 1 секунду
  OneCore.moveWheel(2, -50);         // Запускаем вращение колеса 2 со скоростью 50 в обратном направлении
  delay(1000);                     // Ждём 1 секунду
  OneCore.moveWheel(2, 0);           // Останавливаем вращение колеса 2
  delay(1000);                     // Ждём 1 секунду
  OneCore.DC_Motor(3, 50, BACKWARD); // Запускаем вращение мотора 3 со скоростью 50 в обратном (BACKWARD) направлении
  delay(1000);                     // Ждём 1 секунду
  OneCore.DC_Motor(3, 0, RELEASE);   // Останавливаем вращение мотора 3
  delay(1000);                     // Ждём 1 секунду
  OneCore.DC_Motor(4, 50, FORWARD);  // Запускаем вращение мотора 4 со скоростью 50 в обратном (BACKWARD) направлении
  delay(1000);                     // Ждём 1 секунду
  OneCore.DC_Motor(4, 0, RELEASE);   // Останавливаем вращение мотора 4
  delay(1000);                     // Ждём 1 секунду
}