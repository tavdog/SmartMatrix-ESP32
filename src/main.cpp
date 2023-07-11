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
    "http://pub-34eaf0d2dcbb40c396065db28dcc4418.r2.dev/manifest.json";
#ifdef TIDBYT
// Change these to whatever suits
#define R1_PIN 21
#define G1_PIN 2
#define B1_PIN 22
#define R2_PIN 23
#define G2_PIN 4
#define B2_PIN 27
#define A_PIN 26
#define B_PIN 5
#define C_PIN 25
#define D_PIN 18
#define E_PIN -1 // required for 1/32 scan panels, like 64x64px. Any available pin would do, i.e. IO32
#define LAT_PIN 19
#define OE_PIN 32
#define CLK_PIN 33

//HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
HUB75_I2S_CFG::i2s_pins _pins = {2, 22, 21, 4, 27, 23, 26, 5, 25, 18, -1, 19, 32, 33}; // what actually works for me
#else
HUB75_I2S_CFG::i2s_pins _pins = {25, 26, 27, 14, 12, 13, 23,
                                 19, 5,  17, -1, 4,  15, 16};
#endif
HUB75_I2S_CFG mxconfig(64, 32, 1, _pins);
MatrixPanel_I2S_DMA matrix = MatrixPanel_I2S_DMA(mxconfig);

Adafruit_TSL2561_Unified tsl =
    Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

char hostName[18];
char appletTopic[26];
char statusTopic[26];
char commandTopic[27];
char scheduleTopic[28];
char asyncAppletName[30];
char tmpFileName[20];
char realFileName[20];

static uint8_t spriteBuffer[60000];
static uint8_t decBuffer[4 * MATRIX_WIDTH * MATRIX_HEIGHT];
static WebPData webPData;
WebPAnimDecoder *dec = NULL;

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

int currentAppletID = 0;
int pushingAppletID = 0;

int scheduledItemsCount;
int durations[50];
bool skipStates[50];
bool pinStates[50];
StaticJsonDocument<2048> docIn;
StaticJsonDocument<200> docOut;
char jsonMessageBuf[200];

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

        Log.traceln("[smx/dec] file %s: %d bytes", applet, (int)file.size());

        // Fill buffer!
        file.readBytes((char *)spriteBuffer, file.size());

        webPData.size = file.size();
        webPData.bytes = spriteBuffer;

        // Attempt to decode applet
        int w = 0;
        int h = 0;
        int validity = WebPGetInfo(webPData.bytes, webPData.size, &w, &h);
        if (validity) {
            Log.traceln("[smx/dec] preemptive decoding succeeded for %s",
                        applet);
            newApplet = true;
            hasApplet = true;
            return 1;
        } else {
            Log.errorln("[smx/dec] preemptive decoding failed for %s", applet);
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
        desiredBrightness = 100;
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
        docOut.clear();
        docOut["type"] = "boot";
        serializeJson(docOut, jsonMessageBuf);
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

        deserializeJson(docIn, payload);
        const char *command = docIn["command"];
        if (strcmp(command, "send_app_graphic") == 0) {
            pushingAppletFile.close();
            pushingAppletID = atoi(docIn["params"]["appid"]);

            sprintf(tmpFileName, "/app/%d.webp.tmp", pushingAppletID);

            if (LittleFS.exists(tmpFileName)) {
                LittleFS.remove(tmpFileName);
            }

            pushingAppletFile = LittleFS.open(tmpFileName, FILE_WRITE);
            docOut.clear();
            docOut["type"] = "success";
            docOut["info"] = "applet_update";
            docOut["next"] = "send_chunk";
            serializeJson(docOut, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        } else if (strcmp(command, "app_graphic_stop") == 0) {
            pushingAppletFile.close();

            sprintf(tmpFileName, "/app/%d.webp.tmp", pushingAppletID);
            LittleFS.remove(tmpFileName);
            docOut.clear();
            docOut["type"] = "success";
            docOut["info"] = "applet_update";
            docOut["next"] = "send_next";
            serializeJson(docOut, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        } else if (strcmp(command, "app_graphic_sent") == 0) {
            pushingAppletFile.close();

            // Move temp file to real applet
            sprintf(realFileName, "/app/%d.webp", pushingAppletID);

            if (LittleFS.rename(tmpFileName, realFileName)) {
                docOut.clear();
                docOut["type"] = "success";
                docOut["info"] = "applet_update";
                docOut["next"] = "none";
                serializeJson(docOut, jsonMessageBuf);
                publish(statusTopic, jsonMessageBuf);
            }
        } else if (strcmp(command, "device_reboot") == 0) {
            ESP.restart();
        } else if (strcmp(command, "device_reset") == 0) {
            wifiManager.resetSettings();
            ESP.restart();
        } else if (strcmp(command, "ping") == 0) {
            docOut.clear();
            docOut["type"] = "pong";
            serializeJson(docOut, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
        }
    } else if (strcmp(topic, appletTopic) == 0) {
        if (index == 0) {
            Log.traceln("[smx/applet] receiving %d bytes for %d", (int)total,
                        pushingAppletID);
        }
        pushingAppletFile.write((uint8_t *)payload, len);
        if (index + len >= total) {
            Log.traceln("[smx/applet] stored %d bytes for %d", (int)total,
                        pushingAppletID);
            docOut.clear();
            docOut["type"] = "success";
            docOut["info"] = "applet_update";
            docOut["next"] = "send_chunk";
            serializeJson(docOut, jsonMessageBuf);
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
    docIn.clear();
    docIn.garbageCollect();
}

bool needForceUpdateApplet;
int forceUpdateAppletID;

void forceUpdateApplet(int id) {
    Log.traceln("[smx/applet] forcing update of %d", id);
    forceUpdateAppletID = id;
    needForceUpdateApplet = true;
}

void scheduleLoop(void *parameter) {
    int schedCheckID = -1;
    for (;;) {
        vTaskDelay(1000);
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

                docOut.clear();
                docOut["type"] = "success";
                docOut["info"] = "applet_displayed";
                docOut["appid"] = currentAppletID;
                serializeJson(docOut, jsonMessageBuf);
                publish(statusTopic, jsonMessageBuf);
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

    for (;;) {
        if (hasApplet) {
            if (newApplet && webPData.bytes != 0) {
                Log.traceln("[smx/display] new applet");
                newApplet = false;
                WebPAnimDecoderDelete(dec);
                dec = WebPAnimDecoderNew(&webPData, NULL);
                if (dec == NULL) {
                    hasApplet = false;
                    continue;
                }
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
                                    matrix.writePixel(
                                        x, y,
                                        matrix.color565(buf[px * 4],
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
                            char appletName[20];
                            sprintf(appletName, "app/%d", currentAppletID);
                            showAppletAsync(appletName);
                            errorCount++;
                            if (errorCount > 3) {
                                currentAppletExecutionStartTime = 0;
                            }
                            hasApplet = false;
                        }
                    }
                }
            }
        }

        if (newStatusApplet) {
            newStatusApplet = false;
            showApplet(asyncAppletName);
        }

        vTaskDelay(30);
    }
}

void setup() {
    Serial.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.noticeln("[smx/setup] starting up");
    matrix.begin();

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

    matrix.setBrightness8(desiredBrightness);  // 0-255
    matrix.clearScreen();

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

        if (restart_after) {
            ESP.restart();
        }
    });

    client.onConnect(onMqttConnect);
    client.onDisconnect(onMqttDisconnect);
    client.onMessage(onMqttMessage);
    client.setServer(MQTT_HOST, MQTT_PORT);
    // only set password if they are not blank
    if (String(MQTT_USERNAME).length() > 0) {
        client.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
    }
}

unsigned long lastReportHeapTime = 0;

void loop() {
    // OTA
    if ((millis() - lastOTACheckTime > 60000 || lastOTACheckTime == 0) &&
        WiFi.isConnected() && !updateStarted) {
        Log.traceln("[smx/ota] checking for update");
        lastOTACheckTime = millis();
        if (esp32FOTA.execHTTPcheck()) {
            Log.noticeln("[smx/ota] starting update at %l", millis());
            updateStarted = true;
            updateStartTime = millis();
            esp32FOTA.execOTA();
        } else {
            Log.traceln("[smx/ota] up to date, running ver %d", APP_VERSION);
        }
    }

    if (millis() - lastReportHeapTime > 5000) {
        lastReportHeapTime = millis();
        docOut.garbageCollect();
        docIn.garbageCollect();
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
        docOut.clear();
        docOut["type"] = "heartbeat";
        docOut["appid"] = currentAppletID;
        docOut["heap"] = esp_get_free_heap_size();
        docOut["uptime"] = floor(esp_timer_get_time() / 1000000);
        docOut["appver"] = APP_VERSION;
        if (tslEnabled) {
            docOut["lux"] = luxLevel;
        }
        serializeJson(docOut, jsonMessageBuf);
        publish(statusTopic, jsonMessageBuf);
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
        matrix.setBrightness8(currentBrightness);
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

        DeserializationError error = deserializeJson(docIn, scheduleFile);

        if (error) {
            Log.traceln("[smx/schedule] decode error");
            docOut.clear();
            docOut["type"] = "error";
            docOut["info"] = "schedule_decode_error";
            serializeJson(docOut, jsonMessageBuf);
            publish(statusTopic, jsonMessageBuf);
            return;
        } else {
            JsonArray array = docIn["items"].as<JsonArray>();
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

            Log.traceln("[smx/schedule] writing hash: %s", docIn["hash"]);

            File hashFile = LittleFS.open("/app/schedule.hash", FILE_WRITE);
            hashFile.write(docIn["hash"]);
            hashFile.close();

            docOut.clear();
            docOut["type"] = "success";
            docOut["info"] = "schedule_received";
            docOut["hash"] = docIn["hash"];
            serializeJson(docOut, jsonMessageBuf);

            docIn.clear();
            docIn.garbageCollect();

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
        docOut.clear();
        docOut["type"] = "error";
        docOut["info"] = "malformed_applet";
        docOut["appid"] = forceUpdateAppletID;
        serializeJson(docOut, jsonMessageBuf);
        publish(statusTopic, jsonMessageBuf);
    }
}