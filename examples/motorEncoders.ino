#include "KMOneCore.h" // Подключение библиотеки робоконтроллера

KMOneCore OneCore; // Инициализация объекта Робоконтроллера "OneCore" для дальнейшего использования его функций

void setup()
{
  OneCore.begin();         // Инициализируем Робоконтроллер
  OneCore.encoderSetup(1); // Инициализируем энкодер мотора 1
  OneCore.encoderSetup(2); // Инициализируем энкодер мотора 2
  OneCore.encoderSetup(3); // Инициализируем энкодер мотора 3
  OneCore.encoderSetup(4); // Инициализируем энкодер мотора 4
}

void loop()
{
  Serial.print(OneCore.encPosition(1)); // Получаем и выводим в последовательный порт значения энкодера мотора 1
  Serial.print('\t');
  Serial.print(OneCore.encPosition(2)); // Получаем и выводим в последовательный порт значения энкодера мотора 2
  Serial.print('\t');
  Serial.print(OneCore.encPosition(3)); // Получаем и выводим в последовательный порт значения энкодера мотора 3
  Serial.print('\t');
  Serial.println(OneCore.encPosition(4)); // Получаем и выводим в последовательный порт значения энкодера мотора 4

  if (OneCore.ReadButton(1)) // Если нажата кнопка 1 на робоконтроллере
  {
    Serial.println("Clear Encoder Data");
    OneCore.clearEncoderData(1); // Обнуляем значения энкодера мотора 1
    OneCore.clearEncoderData(2); // Обнуляем значения энкодера мотора 2
    OneCore.clearEncoderData(3); // Обнуляем значения энкодера мотора 3
    OneCore.clearEncoderData(4); // Обнуляем значения энкодера мотора 4
  }
}