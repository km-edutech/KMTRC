#include "KMTRC.h" // Подключение библиотеки робоконтроллера

KMTRC trbot; // Инициализация объекта Робоконтроллера "trbot" для дальнейшего использования его функций
int ledlevel = 0;     // Инициализируем переменную для значения угла поворота потенциометра

void setup()
{
  trbot.begin(); // Инициализируем Робоконтроллер
}

void loop()
{
  if (!trbot.ReadButton(1)) // Пока не нажата кнопка 1 на робоконтроллере
  {
    trbot.setBuzzer(0, 0); // Отключаем звуковой излучатель если он работает
    ledlevel = map(trbot.ADCRead(1), 0, 4095, 0, 180); // Читаем значения потенциометра, подключеного к порту ADC 1
    trbot.setLedDIO4(ledlevel);                        // Задаем яркость светодиода DIO4 в зависимости от значения угла поворота ручки потенциометра
  }
  else // Если кнопка 1 нажата
  {
    trbot.setLedDIO4(0);         // Выключаем светодиод
    trbot.setBuzzer(10000, 200); // Включаем звуковой излучатель на 200мс с частотой 10000 Гц
  }
}