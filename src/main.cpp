#include <espcamfinal_inferencing.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/yolo.h>
#include <WiFi.h>
#ifndef RX
#define RX 44
#endif

#ifndef TX
#define TX 43
#endif

using eloq::camera;
using eloq::ei::yolo;

/*
uint8_t startCommand[5] = {0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
uint8_t stopCommand[5] = {0x51, 0x51, 0x51, 0x51, 0x51};
uint8_t receivedCommand[5];
bool commandReceive = false;
bool stopReceive = false;
*/
//size_t heapSize = 1048576;
char pos;
HardwareSerial uSerial(0);

void setup() {
    Serial.begin(115200);
    uSerial.begin(115200, SERIAL_8N1, RX, TX);
    camera.brownout.disable();
    camera.resolution.yolo();
    WiFi.mode( WIFI_MODE_NULL);
    btStop();
    

    while (!camera.begin().isOk());
        Serial.println(camera.exception.toString());
    
    log_d("Total heap: %d", ESP.getHeapSize());
    log_d("Free heap: %d", ESP.getFreeHeap());
    log_d("Total PSRAM: %d", ESP.getPsramSize());
    log_d("Free PSRAM: %d", ESP.getFreePsram());
    /*
        while (!wifi.connect(ssid, password).isOk())
            Serial.println(wifi.exception.toString());
        Serial.println("WiFi OK");

        while (!yoloStream.begin().isOk())
            Serial.println(yoloStream.exception.toString());
        Serial.println(yoloStream.address());
    */
}

void loop() {
/*
    uint8_t* yoloHeap = (uint8_t*)heap_caps_malloc(heapSize, MALLOC_CAP_SPIRAM);
    if (yoloHeap == nullptr) {
        Serial.println("Failed to allocate memory from PSRAM!");
        return;
    } else {
        Serial.println("Memory allocated from PSRAM successfully!");
    }
*/
    if (!camera.capture().isOk()) {
            Serial.println(camera.exception.toString());
            return;
    }
/*
    while (!commandReceive) {
        if (uSerial.available() >= 5) {
            for (int i = 0; i < 5; i++) {
                receivedCommand[i] = uSerial.read();
            }
            if (memcmp(receivedCommand, startCommand, sizeof(startCommand)) == 0) {
                commandReceive = true;
                stopReceive = false;
            }
            else {
                stopReceive = true;
                commandReceive = false;
            }
        }
    }
    if (commandReceive && !stopReceive) {
*/
        if (!yolo.run().isOk()) {
            Serial.println(yolo.exception.toString());
            return;
        }
        if (!yolo.foundAnyObject()) return;
        if (yolo.first.cx <= EI_CLASSIFIER_INPUT_WIDTH / 3) pos = 'l';
        else if (yolo.first.cx <= EI_CLASSIFIER_INPUT_WIDTH * 2 / 3) pos = 'c';
        else pos = 'r';
        byte esp[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, pos, 0x00};
        Serial.write(esp, sizeof(esp));

        if (yolo.count() > 1) {
            yolo.forEach([](int i, bbox_t bbox) {
                if (bbox.cx <= EI_CLASSIFIER_INPUT_WIDTH / 3) pos = 'l';
                else if (bbox.cx <= EI_CLASSIFIER_INPUT_WIDTH * 2 / 3) pos = 'c';
                else pos = 'r';
                byte esp[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, pos, 0x00};
                Serial.write(esp, sizeof(esp));
            });
        }
//    }
//heap_caps_free(yoloHeap); 

}