/*
 Master — очередь из main -> espnowTask, отправка по broadcast.
*/

#include <Arduino.h>
#include <SPI.h>
#include "esp_now.h"
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ================= USER CONFIG =================
#define CPI_DEFAULT       400
#define ENABLE_CGM        1
#define BURST_MODE        0
#define ESPNOW_CHANNEL    1
// ===============================================

// Пины сенсора
#define PIN_CS    5
#define PIN_MISO  4
#define PIN_MOSI  3
#define PIN_SCK   2
#define PIN_RESET 6
#define PIN_MOT   1

SPIClass SPI_Adns(HSPI);

// PAW3395 регистры — ПЕРЕМЕСТИЛ В НАЧАЛО
#define Motion            0x02
#define Delta_X_L         0x03
#define Delta_Y_L         0x05
#define Power_Up_Reset    0x3A
#define Performance       0x40
#define Set_Resolution    0x47
#define Resolution_X_Low  0x48
#define Resolution_X_High 0x49
#define Resolution_Y_Low  0x4A
#define Resolution_Y_High 0x4B
#define Motion_Burst      0x16

// ================= ESP-NOW =================
static uint8_t receiverAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct __attribute__((packed)) {
    int8_t dx;
    int8_t dy;
    uint8_t buttons;
    uint32_t timestamp;
} mouse_data_t;

// ============ globals ============
volatile bool motionFlag = false;
static QueueHandle_t mouseQueue = NULL;
static const size_t QUEUE_DEPTH =2;  // Уменьшил для скорости

static uint32_t packet_count = 0;
static uint32_t drop_count = 0;

// ============ ISR ============
void IRAM_ATTR motionISR() {
    motionFlag = true;
}

// ============ utils ============
int8_t convTwosComp(uint8_t b) {
    return (b & 0x80) ? -((~b + 1) & 0xFF) : b;
}

// ================= ESP-NOW ================
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
    // Минимальный лог для скорости
    if(status != ESP_NOW_SEND_SUCCESS) {
        Serial.printf("[SEND_ERR] %d\n", (int)status);
    }
}

// ============ ESP-NOW TASK ================
void espnowTask(void *parameter){
    // Инициализация WiFi и ESP-NOW ПЕРЕНЕСЕНА сюда из setup()
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(50);
    
    if(esp_now_init() != ESP_OK){
        Serial.println("ESP-NOW init failed");
        vTaskDelete(NULL);
        return;
    }

    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, receiverAddress, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
 
    if(esp_now_add_peer(&peer) != ESP_OK){
        Serial.println("Failed to add peer");
        vTaskDelete(NULL);
        return;
    }

    Serial.println("ESP-NOW task ready");

    // Основной цикл отправки
    mouse_data_t ev;
    for(;;){
        if(xQueueReceive(mouseQueue, &ev, portMAX_DELAY) == pdTRUE){
            ev.timestamp = micros();
            
            esp_err_t res = esp_now_send(receiverAddress, (uint8_t*)&ev, sizeof(ev));
            if(res == ESP_OK){
                packet_count++;
            } else {
                Serial.printf("[SEND] %s\n", esp_err_to_name(res));
            }
        }
    }
}

// ============ setup / loop ============
void setup(){
    Serial.begin(115200);
    
    // Быстрая инициализация сенсора
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);
    pinMode(PIN_MOT, INPUT_PULLUP);
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_RESET, HIGH);

    SPI_Adns.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    SPI_Adns.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));

    // Быстрый сброс сенсора
    digitalWrite(PIN_RESET, LOW); 
    delayMicroseconds(100);
    digitalWrite(PIN_RESET, HIGH);
    delay(10);

    adns_powerup_init();
    adns_set_cpi(CPI_DEFAULT);
    if (ENABLE_CGM) adns_enable_cgm();

    attachInterrupt(digitalPinToInterrupt(PIN_MOT), motionISR, FALLING);
    
    // Создаем очередь ДО запуска задачи
    mouseQueue = xQueueCreate(QUEUE_DEPTH, sizeof(mouse_data_t));
    if (!mouseQueue) {
        Serial.println("Failed to create queue");
        while(1) delay(1000);
    }

    // Запускаем задачу ESP-NOW с высоким приоритетом
    xTaskCreatePinnedToCore(espnowTask, "espnow", 4096, NULL, 3, NULL, 1);

    Serial.println("Wireless Mouse Ready");
}

void loop(){
    if (motionFlag){
        motionFlag = false;
        int8_t dx = 0, dy = 0;
        
        #if BURST_MODE
            read_motion_burst(dx, dy);
        #else
            read_motion_data(dx, dy);
        #endif

        if (dx != 0 || dy != 0) {
            mouse_data_t ev = {dx, dy, 0, micros()};  // Быстрая инициализация
            
            if (xQueueSend(mouseQueue, &ev, 0) != pdTRUE) {
                drop_count++;
            }
        }
    }

    // Статистика раз в 10 секунд вместо 1000
    static uint32_t lastTs = 0;
    if (millis() - lastTs > 10000) {
        lastTs = millis();
        Serial.printf("Sent: %lu, Drops: %lu, Free: %d\n", 
                     packet_count, drop_count, uxQueueSpacesAvailable(mouseQueue));
    }

    delayMicroseconds(500);  // Быстрый опрос
}
// ============ SPI functions (как у тебя) ============
void adns_com_begin() { digitalWrite(PIN_CS, LOW); delayMicroseconds(1); }
void adns_com_end()   { delayMicroseconds(1); digitalWrite(PIN_CS, HIGH); }
void adns_write_reg(uint8_t reg, uint8_t val) {
    adns_com_begin();
    SPI_Adns.transfer(reg | 0x80);
    SPI_Adns.transfer(val);
    adns_com_end();
    delayMicroseconds(20);
}
uint8_t adns_read_reg(uint8_t reg) {
    adns_com_begin();
    SPI_Adns.transfer(reg & 0x7F);
    delayMicroseconds(35);
    uint8_t val = SPI_Adns.transfer(0);
    adns_com_end();
    delayMicroseconds(1);
    return val;
}

void set_high_performance_mode() {
    // Оставил именно те записи, что были у тебя
    adns_write_reg(0x7F, 0x05);
    adns_write_reg(0x51, 0x40);
    adns_write_reg(0x53, 0x40);
    adns_write_reg(0x61, 0x31);
    adns_write_reg(0x6E, 0x0F);
    adns_write_reg(0x7F, 0x07);
    adns_write_reg(0x42, 0x32);
    adns_write_reg(0x43, 0x00);
    adns_write_reg(0x7F, 0x0D);
    adns_write_reg(0x51, 0x00);
    adns_write_reg(0x52, 0x49);
    adns_write_reg(0x53, 0x00);
    adns_write_reg(0x54, 0x58);
    adns_write_reg(0x55, 0x00);
    adns_write_reg(0x56, 0x64);
    adns_write_reg(0x57, 0x02);
    adns_write_reg(0x58, 0xA5);
    adns_write_reg(0x7F, 0x00);
    adns_write_reg(0x54, 0x54);
    adns_write_reg(0x78, 0x01);
    adns_write_reg(0x79, 0x9C);

    uint8_t perf = adns_read_reg(Performance);
    perf = (perf & 0xFC) | 0x00;
    adns_write_reg(Performance, perf);
    Serial.println("High Performance Mode: ENABLED");
}

void adns_enable_cgm() {
    set_high_performance_mode();
    uint8_t perf = adns_read_reg(Performance);
    perf = (perf & 0xFC) | 0x03;
    adns_write_reg(Performance, perf);
    Serial.println("Corded Gaming Mode: ENABLED");
}

void adns_set_cpi(uint16_t cpi) {
    if(cpi<50) cpi=50;
    if(cpi>26000) cpi=26000;
    uint16_t val = cpi / 50;
    adns_write_reg(Resolution_X_Low, val & 0xFF);
    adns_write_reg(Resolution_X_High, (val >> 8) & 0xFF);
    adns_write_reg(Resolution_Y_Low, val & 0xFF);
    adns_write_reg(Resolution_Y_High, (val >> 8) & 0xFF);
    adns_write_reg(Set_Resolution, 0x01);
    Serial.print("CPI set to: "); Serial.println(cpi);
}

void adns_powerup_init() {
    delay(50);
    adns_com_end(); delayMicroseconds(40);
    adns_com_begin(); delayMicroseconds(40);
    adns_com_end(); delayMicroseconds(40);
    adns_write_reg(Power_Up_Reset, 0x5A);
    delay(5);
    adns_read_reg(Motion);
    adns_read_reg(Delta_X_L);
    adns_read_reg(Delta_Y_L);
    set_high_performance_mode();
}

// Чтение motion (не менял)
void read_motion_data(int8_t &dx,int8_t &dy){
    uint8_t motion = adns_read_reg(Motion);
    if(motion & 0x80){
        dx = convTwosComp(adns_read_reg(Delta_X_L));
        dy = convTwosComp(adns_read_reg(Delta_Y_L));
    } else dx=dy=0;
}

void read_motion_burst(int8_t &dx,int8_t &dy){
    adns_com_begin();
    SPI_Adns.transfer(Motion_Burst & 0x7F);
    delayMicroseconds(35);
    uint8_t dx_raw = SPI_Adns.transfer(0);
    SPI_Adns.transfer(0);
    uint8_t dy_raw = SPI_Adns.transfer(0);
    SPI_Adns.transfer(0);
    adns_com_end();
    dx = convTwosComp(dx_raw);
    dy = convTwosComp(dy_raw);
}