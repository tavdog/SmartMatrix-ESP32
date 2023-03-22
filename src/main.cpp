#include <Arduino.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <ArduinoJson.h>
#include <AsyncMqtt_Generic.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <FS.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <webp/demux.h>

#include <esp32fota.hpp>

#include "LittleFS.h"
#include "secrets.h"

const char *manifestURL =
    "https://pub-34eaf0d2dcbb40c396065db28dcc4418.r2.dev/manifest.json";

MatrixPanel_I2S_DMA matrix = MatrixPanel_I2S_DMA();

Adafruit_TSL2561_Unified tsl =
    Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

char hostName[18];
char appletTopic[26];
char statusTopic[26];
char commandTopic[27];
char scheduleTopic[28];
char appletPath[30];
char statusApplet[30];

WebPData webPData;

int desiredBrightness = 100;
int currentBrightness = 100;

bool newApplet;
bool hasApplet;
bool hasSentBootMessage;
bool newStatusApplet;
bool tslEnabled;
bool skipStates[100];
bool pinStates[100];

AsyncMqttClient client;
WiFiManager wifiManager;
esp32FOTA esp32FOTA("SmartMatrix32", APP_VERSION, false, true);

uint8_t tempPixelBuffer[MATRIX_HEIGHT * MATRIX_WIDTH * 4];

WebPDemuxer *webPDemux;
WebPIterator iter;
uint32_t webPFlags;
uint32_t currentFrameIndex = 1;
uint32_t appletFrameCount;
float luxLevel;

TaskHandle_t matrixTask;
TaskHandle_t scheduleTask;
TimerHandle_t mqttReconnectTimer;

File pushingAppletFile;

unsigned long lastFrameDuration = 0;
unsigned long lastFrameTime = 0;
unsigned long lastTSLCheckTime = 0;
unsigned long lastAdjustBrightnessTime = 0;
unsigned long currentAppletExecutionDuration = 0;
unsigned long currentAppletExecutionStartTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastOTACheckTime = 0;

unsigned long currentAppletID = 0;
unsigned long pushingAppletID = 0;

unsigned long durations[100];
unsigned long scheduledItemsCount;

int showApplet(const char *applet) {
    sprintf(appletPath, "/%s.webp", applet);

    if (LittleFS.exists(appletPath)) {
        File file = LittleFS.open(appletPath, FILE_READ);

        // Set buffers
        hasApplet = false;
        WebPDataClear(&webPData);
        if (webPFlags & ANIMATION_FLAG) {
            WebPDemuxReleaseIterator(&iter);
            WebPDemuxDelete(webPDemux);
        }

        // setup webp buffer and populate from temporary buffer
        webPData.size = file.size();
        webPData.bytes = (uint8_t *)WebPMalloc(file.size());

        // Fill buffer!
        file.readBytes((char *)webPData.bytes, file.size());
        file.close();

        // Show!
        if (strncmp((const char *)webPData.bytes, "RIFF", 4) == 0) {
            newApplet = true;
            hasApplet = true;
            return 1;
        } else {
            return 2;
        }
    } else {
        return 0;
    }
}

void updateLux() {
    sensors_event_t event;
    tsl.getEvent(&event);
    luxLevel = event.light;
    if (luxLevel > 10) {
        desiredBrightness = 200;
    } else if (luxLevel > 0) {
        desiredBrightness = 30;
    } else {
        desiredBrightness = 0;
    }
}

void markAppletToShow(const char *applet) {
    strcpy(statusApplet, applet);
    newStatusApplet = true;
}

void configModeCallback(WiFiManager *wm) {
    if (!hasSentBootMessage) {
        markAppletToShow("wifi_config");
    }
}
void saveConfigCallback() {
    if (!hasSentBootMessage) {
        markAppletToShow("wifi_connecting");
    }
}

void connectToMqtt() {
    if (!hasSentBootMessage) {
        markAppletToShow("mqtt_connecting");
    }
    client.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            if (hasSentBootMessage) {
                markAppletToShow("wifi_connected");
            }
            connectToMqtt();
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            xTimerStop(mqttReconnectTimer, 0);
            break;
        default:
            break;
    }
}

void onMqttConnect(bool sessionPresent) {
    client.subscribe(commandTopic, 2);
    client.subscribe(appletTopic, 2);
    client.subscribe(scheduleTopic, 2);
    if (!hasSentBootMessage) {
        markAppletToShow("mqtt_connected");
        hasSentBootMessage = true;
        char jsonMessageBuf[20];
        StaticJsonDocument<20> doc;
        doc["type"] = "boot";
        serializeJson(doc, jsonMessageBuf);
        client.publish(statusTopic, 1, false, jsonMessageBuf);
    }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    if (!hasSentBootMessage) {
        markAppletToShow("mqtt_disconnected");
    }
    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttMessage(char *topic, char *payload,
                   const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index,
                   const size_t &total) {
    if (strcmp(topic, commandTopic) == 0) {
        // Receiving new applet manifest!
        StaticJsonDocument<200> doc;
        deserializeJson(doc, payload);
        const char *command = doc["command"];
        if (strcmp(command, "send_app_graphic") == 0) {
            const char *appid = doc["params"]["appid"];
            pushingAppletID = atoi(appid);
            char tmpFileName[14];
            sprintf(tmpFileName, "/%lu.tmp", pushingAppletID);
            pushingAppletFile = LittleFS.open(tmpFileName, FILE_WRITE);
            char jsonMessageBuf[50];
            StaticJsonDocument<50> doc;
            doc["type"] = "success";
            doc["next"] = "send_chunk";
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
        } else if (strcmp(command, "app_graphic_stop") == 0) {
            pushingAppletFile.close();
            char tmpFileName[14];
            sprintf(tmpFileName, "/%lu.tmp", pushingAppletID);
            LittleFS.remove(tmpFileName);
            char jsonMessageBuf[50];
            StaticJsonDocument<50> doc;
            doc["type"] = "success";
            doc["next"] = "send_next";
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
        } else if (strcmp(command, "app_graphic_sent") == 0) {
            pushingAppletFile.close();
            // Move temp file to real applet
            char tmpFileName[14];
            char realFileName[15];
            sprintf(tmpFileName, "/%lu.tmp", pushingAppletID);
            sprintf(realFileName, "/%lu.webp", pushingAppletID);
            LittleFS.rename(tmpFileName, realFileName);

            char jsonMessageBuf[50];
            StaticJsonDocument<50> doc;
            doc["type"] = "success";
            doc["next"] = "none";
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
        } else if (strcmp(command, "device_reboot") == 0) {
            ESP.restart();
        } else if (strcmp(command, "device_reset") == 0) {
            WiFiManager wm;
            wm.resetSettings();
            ESP.restart();
        } else if (strcmp(command, "ping") == 0) {
            char jsonMessageBuf[20];
            StaticJsonDocument<20> doc;
            doc["type"] = "pong";
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
        }
    } else if (strcmp(topic, appletTopic) == 0) {
        pushingAppletFile.write((uint8_t *)payload, len);
        if (index + len >= total) {
            char jsonMessageBuf[50];
            StaticJsonDocument<50> doc;
            doc["type"] = "success";
            doc["next"] = "send_chunk";
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
        }
    } else if (strcmp(topic, scheduleTopic) == 0) {
        StaticJsonDocument<2048> schedule;
        DeserializationError error = deserializeJson(schedule, payload, len);

        if (error) {
            char jsonMessageBuf[50];
            StaticJsonDocument<50> doc;
            doc["type"] = "error";
            doc["info"] = "schedule_decode_error";
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
            return;
        } else {
            JsonArray array = schedule["items"].as<JsonArray>();
            int i = 0;
            scheduledItemsCount = array.size();
            for (JsonObject item : array) {
                durations[i] = item["d"];
                skipStates[i] = item["s"];
                pinStates[i] = item["p"];
                i++;
            }

            char jsonMessageBuf[200];
            StaticJsonDocument<200> doc;
            doc["type"] = "success";
            doc["info"] = "schedule_received";
            doc["hash"] = schedule["hash"];
            serializeJson(doc, jsonMessageBuf);
            client.publish(statusTopic, 1, false, jsonMessageBuf);
        }
    }
}

void scheduleLoop(void *parameter) {
    int schedCheckID = 0;
    for (;;) {
        vTaskDelay(100);
        if (millis() - currentAppletExecutionStartTime >
            currentAppletExecutionDuration) {
            if (scheduledItemsCount == 0) {
                continue;
            }
            int tcID = schedCheckID;
            schedCheckID++;

            if (schedCheckID >= scheduledItemsCount) {
                schedCheckID = 0;
                tcID = scheduledItemsCount - 1;
            }

            bool currentAppletPinned = pinStates[tcID];
            bool skipApplet = skipStates[schedCheckID];
            unsigned long duration = durations[schedCheckID];

            if (currentAppletPinned || skipApplet) {
                continue;
            }

            currentAppletID = schedCheckID;
            char newAppletName[5];
            sprintf(newAppletName, "%lu", currentAppletID);
            int result = showApplet(newAppletName);
            if (result == 1) {
                currentAppletExecutionStartTime = millis();
                currentAppletExecutionDuration = duration * 1000;
            }
        }
    }
}

void matrixLoop(void *parameter) {
    for (;;) {
        if (hasApplet) {
            if (newApplet) {
                webPDemux = WebPDemux(&webPData);
                appletFrameCount =
                    WebPDemuxGetI(webPDemux, WEBP_FF_FRAME_COUNT);
                webPFlags = WebPDemuxGetI(webPDemux, WEBP_FF_FORMAT_FLAGS);
                lastFrameDuration = 0;
                lastFrameTime = 0;
                newApplet = false;
                currentFrameIndex = 1;
            } else {
                if (webPFlags & ANIMATION_FLAG) {
                    if (millis() - lastFrameTime > lastFrameDuration) {
                        if (WebPDemuxGetFrame(webPDemux, currentFrameIndex,
                                              &iter)) {
                            if (WebPDecodeRGBAInto(iter.fragment.bytes,
                                                   iter.fragment.size,
                                                   tempPixelBuffer,
                                                   iter.width * iter.height * 4,
                                                   iter.width * 4) != NULL) {
                                int px = 0;
                                for (int y = iter.y_offset;
                                     y < (iter.y_offset + iter.height); y++) {
                                    for (int x = iter.x_offset;
                                         x < (iter.x_offset + iter.width);
                                         x++) {
                                        // go pixel by pixel.
                                        int pixelOffsetCF =
                                            ((y * MATRIX_WIDTH) + x) * 3;
                                        int pixelOffsetFT = px * 4;

                                        int alphaValue =
                                            tempPixelBuffer[pixelOffsetFT + 3];

                                        if (alphaValue == 255 && true) {
                                            matrix.writePixel(
                                                x, y,
                                                matrix.color565(
                                                    tempPixelBuffer
                                                        [pixelOffsetFT],
                                                    tempPixelBuffer
                                                        [pixelOffsetFT + 1],
                                                    tempPixelBuffer
                                                        [pixelOffsetFT + 2]));
                                        }

                                        px++;
                                    }
                                }

                                lastFrameTime = millis();
                                lastFrameDuration = iter.duration;
                                currentFrameIndex++;
                                if (currentFrameIndex > appletFrameCount) {
                                    currentFrameIndex = 1;
                                }
                            }
                        } else {
                            hasApplet = false;
                        }
                    }
                } else {
                    // Static WebP
                    if (WebPDecodeRGBInto(webPData.bytes, webPData.size,
                                          tempPixelBuffer,
                                          MATRIX_HEIGHT * MATRIX_WIDTH * 3,
                                          MATRIX_WIDTH * 3) != NULL) {
                        for (int y = 0; y < MATRIX_HEIGHT; y++) {
                            for (int x = 0; x < MATRIX_WIDTH; x++) {
                                int pixBitStart = ((y * MATRIX_WIDTH) + x) * 3;
                                matrix.writePixel(
                                    x, y,
                                    matrix.color565(
                                        tempPixelBuffer[pixBitStart],
                                        tempPixelBuffer[pixBitStart + 1],
                                        tempPixelBuffer[pixBitStart + 2]));
                            }
                        }

                        hasApplet = false;
                    }
                }
            }
        }

        if (newStatusApplet) {
            newStatusApplet = false;
            showApplet(statusApplet);
        }

        vTaskDelay(10);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    matrix.begin();
    LittleFS.begin(true);
    esp32FOTA.setManifestURL(manifestURL);

    tslEnabled = tsl.begin();
    if (tslEnabled) {
        updateLux();
        tsl.enableAutoRange(true);
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    }

    matrix.setBrightness8(desiredBrightness);  // 0-255
    matrix.setLatBlanking(0);
    matrix.clearScreen();

    xTaskCreatePinnedToCore(matrixLoop,   /* Function to implement the task */
                            "MatrixTask", /* Name of the task */
                            3600,         /* Stack size in words */
                            NULL,         /* Task input parameter */
                            1,            /* Priority of the task */
                            &matrixTask,  /* Task handle. */
                            0);           /* Core where the task should run */

    xTaskCreatePinnedToCore(scheduleLoop,   /* Function to implement the task */
                            "ScheduleTask", /* Name of the task */
                            10000,          /* Stack size in words */
                            NULL,           /* Task input parameter */
                            1,              /* Priority of the task */
                            &scheduleTask,  /* Task handle. */
                            0);             /* Core where the task should run */

    showApplet("startup");

    delay(500);

    uint8_t baseMac[6];
    char macFull[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf(hostName, 18, PSTR("SmartMatrix%s"), macFull);

    WiFi.setHostname(hostName);
    WiFi.setAutoReconnect(true);

    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf_P(appletTopic, 26, PSTR("smartmatrix/%s/applet"), macFull);
    snprintf_P(statusTopic, 26, PSTR("smartmatrix/%s/status"), macFull);
    snprintf_P(commandTopic, 27, PSTR("smartmatrix/%s/command"), macFull);
    snprintf_P(scheduleTopic, 28, PSTR("smartmatrix/%s/schedule"), macFull);

    mqttReconnectTimer =
        xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0,
                     reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

    WiFi.onEvent(WiFiEvent);

    esp32FOTA.setUpdateFinishedCb([](int partition, bool restart_after) {
        if (client.connected()) {
            StaticJsonDocument<30> doc;
            char hbMessage[30];
            doc["type"] = "ota";
            doc["info"] = "success";
            serializeJson(doc, hbMessage);
            client.publish(statusTopic, 1, false, hbMessage);
        }

        if (restart_after) {
            ESP.restart();
        }
    });

    client.onConnect(onMqttConnect);
    client.onDisconnect(onMqttDisconnect);
    client.onMessage(onMqttMessage);
    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
}

void loop() {
    if ((millis() - lastOTACheckTime > 60000 || lastOTACheckTime == 0) && WiFi.isConnected()) {
        lastOTACheckTime = millis();
        bool updateNeeded = esp32FOTA.execHTTPcheck();
        if (updateNeeded) {
            esp32FOTA.execOTA();
        }
    }

    if (millis() - lastHeartbeatTime > 10000 && client.connected()) {
        lastHeartbeatTime = millis();
        StaticJsonDocument<150> doc;
        char hbMessage[150];
        doc["type"] = "heartbeat";
        doc["appid"] = currentAppletID;
        doc["heap"] = esp_get_free_heap_size();
        doc["uptime"] = floor(esp_timer_get_time() / 1000000);
        doc["appver"] = APP_VERSION;
        if (tslEnabled) {
            doc["lux"] = luxLevel;
        }
        serializeJson(doc, hbMessage);
        client.publish(statusTopic, 1, false, hbMessage);
    }

    // Update desired brightness
    if (millis() - lastTSLCheckTime > 1500 && tslEnabled) {
        updateLux();
        lastTSLCheckTime = millis();
    }

    if (millis() - lastAdjustBrightnessTime > 30 &&
        currentBrightness != desiredBrightness) {
        if (currentBrightness > desiredBrightness) {
            currentBrightness--;
        } else {
            currentBrightness++;
        }
        matrix.setBrightness8(currentBrightness);
        lastAdjustBrightnessTime = millis();
    }

    if (!WiFi.isConnected()) {
        if (!hasSentBootMessage) {
            markAppletToShow("wifi_connecting");
        }
        wifiManager.setConnectTimeout(30);
        wifiManager.setConfigPortalTimeout(120);
        wifiManager.setAPCallback(configModeCallback);
        wifiManager.setSaveConfigCallback(saveConfigCallback);
        wifiManager.autoConnect(hostName);
    }
}