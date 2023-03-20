//#define TIDBYT
//#define MQTT_SSL

#include <FS.h>
#include "LittleFS.h"

#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifdef MQTT_SSL
#include <WiFiClientSecure.h>
#else
#include <WiFiClient.h>
#endif

#include <webp/demux.h>

#include <constants.h>
#include "secrets.h"

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#ifndef TIDBYT
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#endif

#define NO_OTA_PORT
#include <ArduinoOTA.h>

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

HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};

HUB75_I2S_CFG mxconfig(
	64, // Module width
	32, // Module height
	1, // chain length
	_pins // pin mapping
);

MatrixPanel_I2S_DMA dma_display = MatrixPanel_I2S_DMA(mxconfig);
#else
MatrixPanel_I2S_DMA dma_display = MatrixPanel_I2S_DMA();
#endif

#ifndef TIDBYT
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#endif

boolean newapplet = false;

char hostName[18];
char applet_topic[26];
char status_topic[26];
char command_topic[27];
char appletPath[30];
char messageToPublish[100];

WebPData webp_data;

int currentMode = WELCOME;
int desiredBrightness = 20;
int currentBrightness = 20;
bool otaInProgress = false;
bool need_publish = false;
bool tslEnabled;

#ifdef MQTT_SSL
WiFiClientSecure wifiClient;
#else
WiFiClient wifiClient;
#endif

PubSubClient client(wifiClient);

uint8_t tempPixelBuffer[MATRIX_HEIGHT * MATRIX_WIDTH * 4];

WebPDemuxer* demux;
WebPIterator iter;
uint32_t webp_flags;
uint32_t current_frame = 1;
uint32_t frame_count;

TaskHandle_t matrixTask;
TaskHandle_t mqttTask;
TaskHandle_t connectionTask;

//variables for applet being currently pushed
File pushingAppletFile;
char pushingAppletUUID[37];

char currentAppletUUID[37];

unsigned long last_frame_duration = 0;
unsigned long last_frame_time = 0;
unsigned long last_check_tsl_time = 0;
unsigned long last_adjust_brightness_time = 0;

int showApplet(const char * applet) {
    sprintf(appletPath, "/%s.webp", applet);

    if(LittleFS.exists(appletPath)) {
        File file = LittleFS.open(appletPath, FILE_READ);

        //Set buffers
        currentMode = NONE;
        WebPDataClear(&webp_data);
        if(webp_flags & ANIMATION_FLAG) {
            WebPDemuxReleaseIterator(&iter);
            WebPDemuxDelete(demux);
        }

        //setup webp buffer and populate from temporary buffer
        webp_data.size = file.size();
        webp_data.bytes = (uint8_t *) WebPMalloc(file.size());

        //Fill buffer!
        file.readBytes((char *)webp_data.bytes, file.size());
        file.close();

        //Show!
        if (strncmp((const char*)webp_data.bytes, "RIFF", 4) == 0) {
            newapplet = true;
            currentMode = APPLET;
            return 1;
        } else {
            return 2;
        }
    } else {
        return 0;
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, command_topic) == 0) {
        //Receiving new applet manifest!
        StaticJsonDocument<200> doc;
        deserializeJson(doc, payload);
        const char* command = doc["command"];
        Serial.print("Recieved command: ");
        Serial.println(command);
        if(strcmp(command, "send_app_graphic") == 0) {
            const char* appid = doc["params"]["appid"];
            Serial.print("Appid: ");
            Serial.println(appid);
            strcpy(pushingAppletUUID, appid);
            char tmpFileName[14];
            sprintf(tmpFileName, "/%s.tmp", pushingAppletUUID);
            Serial.println(tmpFileName);

            pushingAppletFile = LittleFS.open(tmpFileName, FILE_WRITE);
            StaticJsonDocument<50> doc;
            doc["type"] = "success";
            doc["next"] = "send_chunk";
            serializeJson(doc, messageToPublish);
            need_publish = true;
        } else if(strcmp(command, "app_graphic_sent") == 0) {
            Serial.println("Moving");
            pushingAppletFile.close();
            //Move temp file to real applet
            char tmpFileName[14];
            char realFileName[15];
            sprintf(tmpFileName, "/%s.tmp", pushingAppletUUID);
            sprintf(realFileName, "/%s.webp", pushingAppletUUID);
            LittleFS.rename(tmpFileName, realFileName);

            StaticJsonDocument<50> doc;
            doc["type"] = "success";
            doc["next"] = "none";
            serializeJson(doc, messageToPublish);
            need_publish = true;
            Serial.println("Moved");
        } else if(strcmp(command, "display_app_graphic") == 0) {
            const char* appid = doc["params"]["appid"];
            Serial.println("Displaying");
            Serial.println(appid);
            int result = showApplet(appid);
            Serial.println(result);
            if(result == 0) {
                //Applet does not exist!
                StaticJsonDocument<50> doc;
                doc["type"] = "error";
                doc["info"] = "not_found";
                serializeJson(doc, messageToPublish);
                need_publish = true;
            } else if(result == 1) {
                StaticJsonDocument<50> doc;
                doc["type"] = "success";
                doc["next"] = "none";
                doc["info"] = "applet_displayed";
                serializeJson(doc, messageToPublish);
                need_publish = true;
            } else if(result == 2) {
                StaticJsonDocument<50> doc;
                doc["type"] = "error";
                doc["info"] = "malformed_header";
                serializeJson(doc, messageToPublish);
                need_publish = true;
            }
        } else if(strcmp(command, "device_reboot") == 0) {
            ESP.restart();
        } else if(strcmp(command, "device_reset") == 0) {
            WiFiManager wm;
            wm.resetSettings();
            ESP.restart();
        } else if(strcmp(command, "ping") == 0) {
            StaticJsonDocument<20> doc;
            doc["type"] = "pong";
            serializeJson(doc, messageToPublish);
            need_publish = true;
        }
    } else if (strcmp(topic, applet_topic) == 0) {
        Serial.print("Recieved chunk: ");
        Serial.println(length);
        pushingAppletFile.write(payload, length);
        StaticJsonDocument<30> doc;
        doc["type"] = "success";
        doc["next"] = "send_chunk";
        serializeJson(doc, messageToPublish);
        need_publish = true;
    }
}

void configModeCallback (WiFiManager *wm) {
    showApplet("wifi_config");
}

void matrixLoop(void * parameter) {
    for(;;) {
        if (currentMode == APPLET) {
            if(newapplet) {
                demux = WebPDemux(&webp_data);
                frame_count = WebPDemuxGetI(demux, WEBP_FF_FRAME_COUNT);
                webp_flags = WebPDemuxGetI(demux, WEBP_FF_FORMAT_FLAGS);
                last_frame_duration = 0;
                last_frame_time = millis() - 10;
                newapplet = false;
                current_frame = 1;
            } else {
                if(webp_flags & ANIMATION_FLAG) {
                    if(millis() - last_frame_time > last_frame_duration) {
                        if(WebPDemuxGetFrame(demux, current_frame, &iter)) {
                            if(WebPDecodeRGBAInto(iter.fragment.bytes, iter.fragment.size, tempPixelBuffer, iter.width * iter.height * 4, iter.width * 4) != NULL) {
                                int px = 0;
                                for(int y = iter.y_offset; y < (iter.y_offset + iter.height); y++) {
                                    for(int x = iter.x_offset; x < (iter.x_offset + iter.width); x++) {
                                        //go pixel by pixel.
                                        int pixelOffsetCF = ((y*MATRIX_WIDTH)+x)*3;
                                        int pixelOffsetFT = px*4;

                                        int alphaValue = tempPixelBuffer[pixelOffsetFT+3];
                                        
                                        if(alphaValue == 255 && true) {
                                            dma_display.writePixel(x,y, dma_display.color565(tempPixelBuffer[pixelOffsetFT],tempPixelBuffer[pixelOffsetFT+1],tempPixelBuffer[pixelOffsetFT+2]));
                                        }
                                        
                                        px++;
                                    }
                                }

                                last_frame_time = millis();
                                last_frame_duration = iter.duration;
                                current_frame++;
                                if(current_frame > frame_count) {
                                    current_frame = 1;
                                }
                            }
                        } else {
                            currentMode = NONE;
                        }
                    }
                } else {
                    //Static WebP
                    if(WebPDecodeRGBInto(webp_data.bytes, webp_data.size, tempPixelBuffer, MATRIX_HEIGHT * MATRIX_WIDTH * 3, MATRIX_WIDTH * 3) != NULL) {
                        
                        for(int y = 0; y < MATRIX_HEIGHT; y++) {
                            for(int x = 0; x < MATRIX_WIDTH; x++) {
                                int pixBitStart = ((y*MATRIX_WIDTH)+x)*3;
                                dma_display.writePixel(x,y, dma_display.color565(tempPixelBuffer[pixBitStart],tempPixelBuffer[pixBitStart+1],tempPixelBuffer[pixBitStart+2]));
                            }
                        }

                        currentMode = NONE;
                    }
                }
            }
        }

        #ifndef TIDBYT
        //Update desired brightness
        if (millis() - last_check_tsl_time > 500 && tslEnabled) {
            sensors_event_t event;
            tsl.getEvent(&event);

            if(event.light > 50) {
                //low brightness
                desiredBrightness = 80;
            } else if(event.light > 0) {
                //low brightness
                desiredBrightness = 20;
            } else {
                desiredBrightness = 0;
            }
            
            last_check_tsl_time = millis();
        }

        if (millis() - last_adjust_brightness_time > 10 && currentBrightness != desiredBrightness) {
            if(currentBrightness > desiredBrightness) {
                currentBrightness--;
            } else {
                currentBrightness++;
            }
            dma_display.setBrightness8(currentBrightness);
            last_adjust_brightness_time = millis();
        }
        #endif

        vTaskDelay(10);
    }
}

void mqttLoop(void * parameter) {
    for(;;) {
        if(need_publish && client.connected()) {
            client.publish(status_topic, messageToPublish);
            need_publish = false;
        }

        client.loop();
        vTaskDelay(10);
    }
}

void connectionLoop(void * parameter) {
    for(;;) {
        if(otaInProgress) {
            continue;
        }

        if(WiFi.isConnected() && WiFi.getMode() == WIFI_STA) {
            if(!client.connected()) {
                showApplet("mqtt_connecting");
                client.connect(hostName, MQTT_USERNAME, MQTT_PASSWORD);
                if (client.connected()) {
                    showApplet("mqtt_connected");
                    client.subscribe(applet_topic);
                    client.subscribe(command_topic);
                    showApplet("ready");
                }
            }
        } else {
            showApplet("wifi_connecting");
            WiFi.reconnect();
        }
        vTaskDelay(10000);

        if(WiFi.isConnected() && client.connected()) {
            StaticJsonDocument<30> doc;
            doc["type"] = "heartbeat";
            serializeJson(doc, messageToPublish);
            need_publish = true;
        }
    }
}

void setup() {
    Serial.begin(115200);

    dma_display.begin();
    dma_display.setBrightness8(currentBrightness); //0-255
    dma_display.setLatBlanking(3);
    dma_display.clearScreen();

    if(!LittleFS.begin(true)){
        return;
    }

    xTaskCreatePinnedToCore(
      matrixLoop, /* Function to implement the task */
      "MatrixTask", /* Name of the task */
      3600,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &matrixTask,  /* Task handle. */
    1); /* Core where the task should run */

    xTaskCreatePinnedToCore(
      mqttLoop, /* Function to implement the task */
      "MqttTask", /* Name of the task */
      15000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &mqttTask,  /* Task handle. */
    0); /* Core where the task should run */

    showApplet("startup");

    vTaskDelay(500);
    
    #ifndef TIDBYT
    Wire.begin();
    tslEnabled = true;
    if(!tsl.begin())
    {
        tslEnabled = false;
    } else {
        tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    }      /* fast but low resolution */
    #endif

    WiFiManager wifiManager;

    uint8_t baseMac[6];
    char macFull[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf(hostName, 18, PSTR("SmartMatrix%s"),macFull);

    showApplet("wifi_connecting");

    WiFi.setHostname(hostName);
    WiFi.setAutoReconnect(true);

    wifiManager.setConnectTimeout(30);
    wifiManager.setConfigPortalTimeout(120);
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setCleanConnect(true);
    wifiManager.autoConnect(hostName);

    ArduinoOTA.setHostname(hostName);

    ArduinoOTA.onStart([]() {
        currentMode = NONE;
        otaInProgress = true;
        showApplet("startup");
    });

    ArduinoOTA.onError([](ota_error_t error) {
        ESP.restart();
    });

    ArduinoOTA.begin();

    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf_P(applet_topic, 26, PSTR("%s/%s/applet"), TOPIC_PREFIX, macFull);
    snprintf_P(status_topic, 26, PSTR("%s/%s/status"), TOPIC_PREFIX, macFull);
    snprintf_P(command_topic, 27, PSTR("%s/%s/command"), TOPIC_PREFIX, macFull);

    #ifdef MQTT_SSL
    wifiClient.setInsecure();
    #endif

    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(mqttCallback);

    xTaskCreatePinnedToCore(
      connectionLoop, /* Function to implement the task */
      "ConnectionTask", /* Name of the task */
      3000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &connectionTask,  /* Task handle. */
    0); /* Core where the task should run */
}

void loop() {
    ArduinoOTA.handle();
}