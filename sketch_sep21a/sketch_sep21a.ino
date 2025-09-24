#include <SPI.h>
#include <USBHIDMouse.h>

#define PIN_CS     5   // NCS
#define PIN_MISO   4   // MISO
#define PIN_MOSI   3   // MOSI
#define PIN_SCK    2   // SCLK
#define PIN_RESET  6   // NRESET
#define PIN_MOT    1   // MOT

// Регистры датчика
#define REG_PRODUCT_ID      0x00
#define REG_MOTION          0x02
#define REG_DELTA_X         0x03
#define REG_DELTA_Y         0x05
#define REG_CONFIG          0x10
#define REG_POWER_UP_RESET  0x3A
#define REG_INVERSE_PRODUCT_ID 0x3F

USBHIDMouse Mouse;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Инициализация PAW3395DM...");
  
  // Инициализация пинов
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_MOT, INPUT);

  digitalWrite(PIN_CS, HIGH);

  // Сброс сенсора
  Serial.println("Сброс датчика...");
  digitalWrite(PIN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_RESET, HIGH);
  delay(100);

  // Настройка SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  // Проверка подключения
  byte product_id = readRegister(REG_PRODUCT_ID);
  byte inverse_id = readRegister(REG_INVERSE_PRODUCT_ID);
  
  Serial.print("Product ID: 0x");
  Serial.println(product_id, HEX);
  Serial.print("Inverse Product ID: 0x");
  Serial.println(inverse_id, HEX);

  // Программный сброс
  writeRegister(REG_POWER_UP_RESET, 0x5A);
  delay(50);

  // Настройка датчика для работы
  // Здесь можно добавить настройку разрешения, частоты опроса и т.д.
  
  Serial.println("Датчик готов к работе!");
}

void loop() {
  // Проверяем наличие данных о движении
  if(digitalRead(PIN_MOT) == LOW) {
    // Читаем данные движения
    int8_t dx = (int8_t)readRegister(REG_DELTA_X);
    int8_t dy = (int8_t)readRegister(REG_DELTA_Y);
    
    // Отправляем движение мыши
    if(dx != 0 || dy != 0) {
      Mouse.move(dx, dy, 0);
      
      // Вывод для отладки
      Serial.print("X: ");
      Serial.print(dx);
      Serial.print(" Y: ");
      Serial.println(dy);
    }
  }
  
  delay(1); // Небольшая задержка
}

byte readRegister(byte reg) {
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(reg & 0x7F); // Чтение - бит 7 = 0
  delayMicroseconds(10);
  byte val = SPI.transfer(0);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
  return val;
}

void writeRegister(byte reg, byte val) {
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(reg | 0x80); // Запись - бит 7 = 1
  SPI.transfer(val);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);
}