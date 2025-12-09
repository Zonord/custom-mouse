#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <USB.h>
#include <USBHIDMouse.h>

#define ESPNOW_CHANNEL 6

typedef struct __attribute__((packed)) {
    int8_t dx;
    int8_t dy;
    uint8_t buttons;
    uint32_t timestamp;
} mouse_data_t;

USBHIDMouse Mouse;
uint32_t received_count = 0;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
    if(len != sizeof(mouse_data_t)) return;

    mouse_data_t *mouse_data = (mouse_data_t*)data;
    received_count++;

    // Двигаем мышь
    Mouse.move(-mouse_data->dx, mouse_data->dy, 0);

    if(received_count % 50 == 0){
        Serial.printf("Packets: %lu | dx=%d dy=%d\n", received_count, mouse_data->dx, mouse_data->dy);
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);

    USB.begin();
    Mouse.begin();

    WiFi.mode(WIFI_STA);
    Serial.println("MAC: " + WiFi.macAddress());

    if(esp_now_init() != ESP_OK){
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("Receiver ready!");
}

void loop(){
    delay(10);
}
