#include "KMTRC.h" // Подключение библиотеки робоконтроллера

KMTRC trbot; // Инициализация объекта Робоконтроллера "trbot" для дальнейшего использования его функций

void setup()
{
  trbot.begin();         // Инициализируем Робоконтроллер
  trbot.encoderSetup(1); // Инициализируем энкодер мотора 1
  trbot.encoderSetup(2); // Инициализируем энкодер мотора 2
  trbot.encoderSetup(3); // Инициализируем энкодер мотора 3
  trbot.encoderSetup(4); // Инициализируем энкодер мотора 4
}

void loop()
{
  Serial.print(trbot.encPosition(1)); // Получаем и выводим в последовательный порт значения энкодера мотора 1
  Serial.print('\t');
  Serial.print(trbot.encPosition(2)); // Получаем и выводим в последовательный порт значения энкодера мотора 2
  Serial.print('\t');
  Serial.print(trbot.encPosition(3)); // Получаем и выводим в последовательный порт значения энкодера мотора 3
  Serial.print('\t');
  Serial.println(trbot.encPosition(4)); // Получаем и выводим в последовательный порт значения энкодера мотора 4

  if (trbot.ReadButton(1)) // Если нажата кнопка 1 на робоконтроллере
  {
    Serial.println("Clear Encoder Data");
    trbot.clearEncoderData(1); // Обнуляем значения энкодера мотора 1
    trbot.clearEncoderData(2); // Обнуляем значения энкодера мотора 2
    trbot.clearEncoderData(3); // Обнуляем значения энкодера мотора 3
    trbot.clearEncoderData(4); // Обнуляем значения энкодера мотора 4
  }
}