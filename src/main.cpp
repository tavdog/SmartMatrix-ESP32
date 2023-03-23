#include <Arduino.h>
#include <ArduinoLog.h>

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
#include <webp/decode.h>
#include <webp/demux.h>

#include <esp32fota.hpp>

#include "LittleFS.h"
#include "secrets.h"

const char *manifestURL =
    "https://pub-34eaf0d2dcbb40c396065db28dcc4418.r2.dev/manifest.json";

MatrixPanel_I2S_DMA *matrix = nullptr;

Adafruit_TSL2561_Unified tsl =
    Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

char hostName[18];
char appletTopic[26];
char statusTopic[26];
char commandTopic[27];
char scheduleTopic[28];
char asyncAppletName[30];

WebPData *webPData = nullptr;

int desiredBrightness = 100;
int currentBrightness = 100;

bool newApplet;
bool hasApplet;
bool hasSentBootMessage;
bool newStatusApplet;
bool needToDecodeSchedule = true;
bool tslEnabled;
bool updateStarted;

AsyncMqttClient client;
WiFiManager wifiManager;
esp32FOTA esp32FOTA("SmartMatrix32", APP_VERSION, false, true);

float luxLevel;

TaskHandle_t matrixTask;
TaskHandle_t scheduleTask;
TimerHandle_t mqttReconnectTimer;

File pushingAppletFile;

int currentAppletExecutionDuration = 0;

unsigned long lastTSLCheckTime = 0;
unsigned long lastAdjustBrightnessTime = 0;
unsigned long currentAppletExecutionStartTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastOTACheckTime = 0;
unsigned long updateStartTime;

int currentAppletID = -1;
int pushingAppletID = 0;

int scheduledItemsCount;
int durations[50];
bool skipStates[50];
bool pinStates[50];

void publish(const char *topic, const char *message) {
    client.publish(topic, 1, false, message);
    Log.traceln("[smx/mqtt] publishing %s to topic: %s", message, topic);
}

int showApplet(const char *applet) {
    char appletPath[30];
    sprintf(appletPath, "/%s.webp", applet);
    Log.traceln("[smx/dec] attepting to decode %s", applet);
    if (LittleFS.exists(appletPath)) {
        File file = LittleFS.open(appletPath, FILE_READ);
        // Set buffers
        hasApplet = false;
        newApplet = false;

        if (webPData != nullptr) {
            Log.traceln("[smx/dec] freeing WebPData");
            WebPDataClear(webPData);
            webPData = nullptr;
        }

        Log.traceln("[smx/dec] file %s: %d bytes", applet, (int)file.size());

        // setup webp buffer and populate from temporary buffer
        webPData = new WebPData();
        webPData->size = file.size();
        webPData->bytes = (uint8_t *)WebPMalloc(file.size());

        if (webPData->bytes == NULL) {
            Log.traceln("[smx/dec] error mallocing");
            WebPDataClear(webPData);
            webPData = nullptr;
            return 2;
        }

        // Fill buffer!
        file.readBytes((char *)webPData->bytes, file.size());
        file.close();

        // Attempt to decode applet
        int w = 0;
        int h = 0;
        int validity = WebPGetInfo(webPData->bytes, webPData->size, &w, &h);
        if (validity) {
            Log.traceln("[smx/dec] preemptive decode succeeded %s", applet);
            newApplet = true;
            hasApplet = true;
            return 1;
        } else {
            Log.warningln("[smx/dec] %s invalid", applet);
            return 2;
        }
    } else {
        Log.warningln("[smx/dec] %s not found", applet);
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

void showAppletAsync(const char *applet) {
    strcpy(asyncAppletName, applet);
    newStatusApplet = true;
}

void configModeCallback(WiFiManager *wm) {
    Log.traceln("[smx/wifi] entering setup mode");
    if (!hasSentBootMessage) {
        showAppletAsync("setup");
    }
}
void saveConfigCallback() {
    Log.traceln("[smx/wifi] exiting setup mode");
    if (!hasSentBootMessage) {
        showAppletAsync("connect_wifi");
    }
}

void connectToMqtt() {
    Log.traceln("[smx/mqtt] connecting to %s:%d", MQTT_HOST, MQTT_PORT);
    if (!hasSentBootMessage) {
        showAppletAsync("connect_cloud");
    }
    client.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Log.traceln("[smx/wifi] got connection event");
            connectToMqtt();
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Log.traceln("[smx/wifi] got disconnection event");
            xTimerStop(mqttReconnectTimer, 0);
            break;
        default:
            break;
    }
}

void onMqttConnect(bool sessionPresent) {
    Log.traceln("[smx/mqtt] connected to %s:%d", MQTT_HOST, MQTT_PORT);
    client.subscribe(commandTopic, 2);
    client.subscribe(appletTopic, 2);
    client.subscribe(scheduleTopic, 2);
    if (!hasSentBootMessage) {
        hasSentBootMessage = true;
        char jsonMessageBuf[20];
        StaticJsonDocument<20> doc;
        doc["type"] = "boot";
        serializeJson(doc, jsonMessageBuf);
        publish(statusTopic, jsonMessageBuf);
        showAppletAsync("ready");
    }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Log.traceln("[smx/mqtt] disconnected");
    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttMessage(char *topic, char *payload,
                   const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index,
                   const size_t &total) {
    if (strcmp(topic, commandTopic) == 0) {
        Log.traceln("[smx/mqtt] received command: %s", payload);
        // Receiving new applet manifest!
        StaticJsonDocument<200> doc;
        deserializeJson(doc, payload);
        const char *command = doc["command"];
        if (strcmp(command, "send_app_graphic") == 0) {
            const char *appid = doc["params"]["appid"];
            pushingAppletID = atoi(appid);

            char tmpFileName[20];
            sprintf(tmpFileName, "/app/%d.webp.tmp", pushingAppletID);

            if (LittleFS.exists(tmpFileName)) {
                LittleFS.remove(tmpFileName);
            }

            pushingAppletFile = LittleFS.open(tmpFileName, FILE_WRITE);
            char jsonMessageBuf[100];
            StaticJsonDocument<100> doc;
            doc["type"] = "success";
            doc["info"] = "applet_update";
            doc["next"] = "send_chunk";
            serializeJson(doc, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        } else if (strcmp(command, "app_graphic_stop") == 0) {
            pushingAppletFile.close();
            char tmpFileName[20];
            sprintf(tmpFileName, "/app/%d.webp.tmp", pushingAppletID);
            LittleFS.remove(tmpFileName);
            char jsonMessageBuf[100];
            StaticJsonDocument<100> doc;
            doc["type"] = "success";
            doc["info"] = "applet_update";
            doc["next"] = "send_next";
            serializeJson(doc, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        } else if (strcmp(command, "app_graphic_sent") == 0) {
            pushingAppletFile.close();

            // Move temp file to real applet
            char tmpFileName[20];
            char realFileName[20];
            sprintf(tmpFileName, "/app/%d.webp.tmp", pushingAppletID);
            sprintf(realFileName, "/app/%d.webp", pushingAppletID);

            if (LittleFS.rename(tmpFileName, realFileName)) {
                char jsonMessageBuf[100];
                StaticJsonDocument<100> doc;
                doc["type"] = "success";
                doc["info"] = "applet_update";
                doc["next"] = "none";
                serializeJson(doc, jsonMessageBuf);
                publish(statusTopic, jsonMessageBuf);
            }
        } else if (strcmp(command, "device_reboot") == 0) {
            ESP.restart();
        } else if (strcmp(command, "device_reset") == 0) {
            wifiManager.resetSettings();
            ESP.restart();
        } else if (strcmp(command, "ping") == 0) {
            char jsonMessageBuf[20];
            StaticJsonDocument<20> doc;
            doc["type"] = "pong";
            serializeJson(doc, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        }
    } else if (strcmp(topic, appletTopic) == 0) {
        if (index == 0) {
            Log.traceln("[smx/applet] receiving %d bytes for %d", (int)total,
                        pushingAppletID);
        }
        pushingAppletFile.seek(index);
        pushingAppletFile.write((uint8_t *)payload, len);
        if (index + len >= total) {
            Log.traceln("[smx/applet] stored %d bytes for %d", (int)total,
                        pushingAppletID);
            char jsonMessageBuf[100];
            StaticJsonDocument<100> doc;
            doc["type"] = "success";
            doc["info"] = "applet_update";
            doc["next"] = "send_chunk";
            serializeJson(doc, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        }
    } else if (strcmp(topic, scheduleTopic) == 0) {
        File scheduleFile =
            LittleFS.open("/app/schedule.json.tmp", FILE_APPEND);
        scheduleFile.write((uint8_t *)payload, len);
        scheduleFile.close();
        if (index + len >= total) {
            Log.traceln(
                "[smx/mqtt] ready to decode received schedule: %d bytes",
                (int)total);
            needToDecodeSchedule = true;
        }
    }
}

bool needForceUpdateApplet;
int forceUpdateAppletID;

void forceUpdateApplet(int id) {
    Log.traceln("[smx/applet] forcing update of %d", id);
    forceUpdateAppletID = id;
    needForceUpdateApplet = true;
}

void scheduleLoop(void *parameter) {
    int schedCheckID = 0;
    for (;;) {
        vTaskDelay(500);
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
            int duration = durations[schedCheckID];

            if (currentAppletPinned) {
                Log.traceln("[smx/schedule] %d is pinned", tcID);
                vTaskDelay(1000);
                continue;
            }

            if (skipApplet) {
                Log.traceln("[smx/schedule] skipping %d", schedCheckID);
                continue;
            }

            Log.traceln("[smx/schedule] time to show %d", schedCheckID);

            char newAppletName[10];
            sprintf(newAppletName, "app/%d", schedCheckID);
            int result = showApplet(newAppletName);
            if (result == 0) {
                forceUpdateApplet(schedCheckID);
            } else if (result == 1) {
                currentAppletID = schedCheckID;
                currentAppletExecutionStartTime = millis();
                currentAppletExecutionDuration = duration * 1000;

                StaticJsonDocument<70> doc;
                char hbMessage[70];
                doc["type"] = "success";
                doc["info"] = "applet_displayed";
                doc["appid"] = currentAppletID;
                serializeJson(doc, hbMessage);
                publish(statusTopic, hbMessage);
            } else if (result == 2) {
                forceUpdateApplet(schedCheckID);
            }
        }
    }
}

void matrixLoop(void *parameter) {
    unsigned long animStartTS = 0;
    int lastFrameTimestamp = 0;
    int currentFrame = 0;
    int errorCount = 0;
    WebPAnimDecoderOptions dec_options;
    WebPAnimDecoderOptionsInit(&dec_options);
    WebPAnimDecoder *dec = NULL;
    for (;;) {
        if (hasApplet) {
            if (newApplet && webPData != nullptr) {
                Log.traceln("[smx/display] new applet");
                newApplet = false;
                WebPAnimDecoderDelete(dec);
                dec = WebPAnimDecoderNew(webPData, &dec_options);
                animStartTS = millis();
                lastFrameTimestamp = 0;
                currentFrame = 0;
                errorCount = 0;
            } else {
                if (millis() - animStartTS > lastFrameTimestamp) {
                    if (currentFrame == 0) {
                        animStartTS = millis();
                        lastFrameTimestamp = 0;
                    }

                    bool hasMoreFrames = WebPAnimDecoderHasMoreFrames(dec);

                    if (hasMoreFrames) {
                        uint8_t *buf;
                        if (WebPAnimDecoderGetNext(dec, &buf,
                                                   &lastFrameTimestamp)) {
                            int px = 0;
                            for (int y = 0; y < MATRIX_HEIGHT; y++) {
                                for (int x = 0; x < MATRIX_WIDTH; x++) {
                                    matrix->writePixel(
                                        x, y,
                                        matrix->color565(buf[px * 4],
                                                         buf[px * 4 + 1],
                                                         buf[px * 4 + 2]));

                                    px++;
                                }
                            }
                            currentFrame++;
                            if (!WebPAnimDecoderHasMoreFrames(dec)) {
                                currentFrame = 0;
                                WebPAnimDecoderReset(dec);
                            }
                        } else {
                            Log.errorln("[smx/display] decode error for %d",
                                        currentAppletID);
                            errorCount++;
                            if (errorCount > 3) {
                                forceUpdateApplet(currentAppletID);
                                currentAppletExecutionStartTime = 0;
                                hasApplet = false;
                            }
                        }
                    }
                }
            }
        }

        if (newStatusApplet) {
            newStatusApplet = false;
            showApplet(asyncAppletName);
        }

        vTaskDelay(10);
    }
}

void setup() {
    Serial.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.noticeln("[smx/setup] starting up");
    HUB75_I2S_CFG::i2s_pins _pins = {25, 26, 27, 14, 12, 13, 23,
                                     19, 5,  17, -1, 4,  15, 16};
    HUB75_I2S_CFG mxconfig(64, 32, 1, _pins);

    matrix = new MatrixPanel_I2S_DMA(mxconfig);
    matrix->begin();

    Log.traceln("[smx/setup] matrix begin");

    Wire.begin();
    if (!LittleFS.begin(true)) {
        Log.errorln("[smx/setup] couldn't init littlefs");
        ESP.restart();
    }

    LittleFS.mkdir("/app");
    esp32FOTA.setManifestURL(manifestURL);

    Log.traceln("[smx/setup] ota manifest: %s", manifestURL);

    tslEnabled = tsl.begin();
    if (tslEnabled) {
        Log.traceln("[smx/setup] tsl enabled");
        updateLux();
        tsl.enableAutoRange(true);
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    }

    wifiManager.setDebugOutput(false);

    matrix->setBrightness8(desiredBrightness);  // 0-255
    matrix->clearScreen();

    xTaskCreatePinnedToCore(matrixLoop,   /* Function to implement the task */
                            "MatrixTask", /* Name of the task */
                            5000,         /* Stack size in words */
                            NULL,         /* Task input parameter */
                            30,           /* Priority of the task */
                            &matrixTask,  /* Task handle. */
                            0);           /* Core where the task should run */

    xTaskCreatePinnedToCore(scheduleLoop,   /* Function to implement the task */
                            "ScheduleTask", /* Name of the task */
                            3500,           /* Stack size in words */
                            NULL,           /* Task input parameter */
                            1,              /* Priority of the task */
                            &scheduleTask,  /* Task handle. */
                            1);             /* Core where the task should run */

    showAppletAsync("boot");

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
        Log.noticeln("[smx/ota] update finished, took %lms",
                     millis() - updateStartTime);
        if (client.connected()) {
            StaticJsonDocument<50> doc;
            char hbMessage[50];
            doc["type"] = "ota";
            doc["info"] = "success";
            serializeJson(doc, hbMessage);
            publish(statusTopic, hbMessage);
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

unsigned long lastReportHeapTime = 0;

void loop() {
    // OTA
    if ((millis() - lastOTACheckTime > 60000 || lastOTACheckTime == 0) &&
        WiFi.isConnected() && !updateStarted) {
        Log.traceln("[smx/ota] checking for update");
        lastOTACheckTime = millis();
        bool updateNeeded = esp32FOTA.execHTTPcheck();
        if (updateNeeded) {
            Log.noticeln("[smx/ota] starting update at %l", millis());
            updateStarted = true;
            updateStartTime = millis();
            esp32FOTA.execOTA();
        } else {
            Log.traceln("[smx/ota] up to date, running ver %d", APP_VERSION);
        }
    }

    if (millis() - lastReportHeapTime > 10000) {
        lastReportHeapTime = millis();
        Log.traceln("[smx/loop] free heap: %l", (long)esp_get_free_heap_size());
    }

    // In case update fucks
    if (millis() - updateStartTime > 300000 && updateStarted) {
        Log.errorln("[smx/ota] update timeout");
        ESP.restart();
    }

    // Heartbeat
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
        publish(statusTopic, hbMessage);
    }

    // Update lux level
    if (millis() - lastTSLCheckTime > 1000 && tslEnabled) {
        updateLux();
        lastTSLCheckTime = millis();
    }

    // Update display brightness
    if (millis() - lastAdjustBrightnessTime > 10 &&
        currentBrightness != desiredBrightness) {
        if (currentBrightness > desiredBrightness) {
            currentBrightness--;
        } else {
            currentBrightness++;
        }
        matrix->setBrightness8(currentBrightness);
        lastAdjustBrightnessTime = millis();
    }

    // decode schedule
    if (needToDecodeSchedule && client.connected()) {
        Log.traceln("[smx/schedule] decoding");
        needToDecodeSchedule = false;
        File scheduleFile;
        if (LittleFS.exists("/app/schedule.json.tmp")) {
            Log.traceln("[smx/schedule] using tmp schedule");
            scheduleFile = LittleFS.open("/app/schedule.json.tmp", FILE_READ);
        } else {
            if (LittleFS.exists("/app/schedule.json")) {
                Log.traceln("[smx/schedule] using cached schedule");
                scheduleFile = LittleFS.open("/app/schedule.json", FILE_READ);
            } else {
                return;
            }
        }

        StaticJsonDocument<2048> schedule;
        DeserializationError error = deserializeJson(schedule, scheduleFile);

        if (error) {
            Log.traceln("[smx/schedule] decode error");
            char jsonMessageBuf[50];
            StaticJsonDocument<50> doc;
            doc["type"] = "error";
            doc["info"] = "schedule_decode_error";
            serializeJson(doc, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
            return;
        } else {
            JsonArray array = schedule["items"].as<JsonArray>();
            int i = 0;
            scheduledItemsCount = array.size();
            Log.traceln("[smx/schedule] decoded schedule, %d items",
                        scheduledItemsCount);
            for (JsonObject item : array) {
                durations[i] = item["d"];
                skipStates[i] = item["s"];
                pinStates[i] = item["p"];
                i++;
            }

            scheduleFile.close();
            LittleFS.rename("/app/schedule.json.tmp", "/app/schedule.json");

            if (LittleFS.exists("/app/schedule.hash")) {
                LittleFS.remove("/app/schedule.hash");
            }

            Log.traceln("[smx/schedule] writing hash: %s", schedule["hash"]);

            File hashFile = LittleFS.open("/app/schedule.hash", FILE_WRITE);
            hashFile.write(schedule["hash"]);
            hashFile.close();

            char jsonMessageBuf[200];
            StaticJsonDocument<200> doc;
            doc["type"] = "success";
            doc["info"] = "schedule_received";
            doc["hash"] = schedule["hash"];
            serializeJson(doc, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        }
    }

    // show full boot anim
    if (!WiFi.isConnected() && millis() > 5700) {
        Log.traceln("[smx/wifi] wifi task");
        if (!hasSentBootMessage) {
            showAppletAsync("connect_wifi");
        }
        wifiManager.setConnectTimeout(30);
        wifiManager.setConfigPortalTimeout(120);
        wifiManager.setAPCallback(configModeCallback);
        wifiManager.setSaveConfigCallback(saveConfigCallback);
        wifiManager.autoConnect(hostName);
    }

    // if applet had an error
    if (needForceUpdateApplet) {
        needForceUpdateApplet = false;
        StaticJsonDocument<70> doc;
        char hbMessage[70];
        doc["type"] = "error";
        doc["info"] = "malformed_applet";
        doc["appid"] = forceUpdateAppletID;
        serializeJson(doc, hbMessage);
        publish(statusTopic, hbMessage);
    }
}