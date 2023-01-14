#include <FS.h>
#include "SPIFFS.h"

#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <webp/demux.h>

#include <constants.h>

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#define NO_OTA_PORT
#include <ArduinoOTA.h>

MatrixPanel_I2S_DMA dma_display = MatrixPanel_I2S_DMA();
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

char mqtt_server[80] = "**REDACTED**";
char mqtt_user[80] = "**REDACTED**";
char mqtt_password[80] = "**REDACTED**";

boolean newapplet = false;

char hostName[80];
char applet_topic[22];
char applet_rts_topic[26];
char brightness_topic[22];
char lastProgText[12];

WebPData webp_data;

int currentMode = WELCOME;
int desiredBrightness = 20;
int currentBrightness = 20;
unsigned long bufferPos;
bool recv_length = false;

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);

uint8_t *tmpbuf;
unsigned long bufsize;

WebPDemuxer* demux;
WebPIterator iter;
uint32_t webp_flags;
uint32_t current_frame = 1;
uint32_t frame_count;

//Frame buffers for animated WebP
uint8_t * currentFrame;

unsigned long mqtt_timeout_lastTime = 0;
unsigned long last_frame_duration = 0;
unsigned long last_frame_time = 0;
unsigned long last_check_tsl_time = 0;
unsigned long last_adjust_brightness_time = 0;

void marqueeText(const String &buf, uint16_t color) {
    dma_display.clearScreen();
    dma_display.drawRect(0,0,MATRIX_WIDTH,MATRIX_HEIGHT,color);
    int16_t x1, y1;
    uint16_t w, h;
    dma_display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h); //calc width of new string
    dma_display.setCursor(32 - w / 2, 16 - h / 2);
    dma_display.print(buf);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, applet_topic) == 0) {
        if(strncmp((char *)payload,"START",3) == 0) {
            mqtt_timeout_lastTime = millis();
            recv_length = false;
            bufferPos = 0;

            client.publish(applet_rts_topic, "OK");
        }else if(strncmp((char *)payload,"PING",4) == 0) {
            client.publish(applet_rts_topic, "PONG");
        } else if(!recv_length) {
            mqtt_timeout_lastTime = millis();
            bufsize = atoi((char *)payload);
            tmpbuf = (uint8_t *) malloc(bufsize);
            recv_length = true;
            client.publish(applet_rts_topic, "OK");
        } else {
            if(strncmp((char *)payload,"FINISH",6) == 0) {
                mqtt_timeout_lastTime = 0;
                if (strncmp((const char*)tmpbuf, "RIFF", 4) == 0) {
                    //Clear and reset all libwebp buffers.
                    WebPDataClear(&webp_data);
                    WebPDemuxReleaseIterator(&iter);
                    WebPDemuxDelete(demux);

                    //Free the frame buffers if necessary.
                    if(currentFrame != nullptr) {
                        free(currentFrame);
                    }

                    //setup webp buffer and populate from temporary buffer
                    webp_data.size = bufsize;
                    webp_data.bytes = (uint8_t *) WebPMalloc(bufsize);

                    memcpy((void *)webp_data.bytes, tmpbuf, bufsize);

                    //free temporary buffer
                    free(tmpbuf);

                    //set display flags!
                    newapplet = true;
                    currentMode = APPLET;
                    client.publish(applet_rts_topic, "PUSHED");
                } else {
                    client.publish(applet_rts_topic, "DECODE_ERROR");
                }
                bufferPos = 0;
                recv_length = false;
            } else {
                mqtt_timeout_lastTime = millis();
                memcpy((void *)(tmpbuf+bufferPos), payload, length);
                bufferPos += length;
                client.publish(applet_rts_topic, "OK");
            }
        }
    }
}


void mqttReconnect() {
    client.disconnect();
    wifiClient.setInsecure();
    client.setServer(mqtt_server, 8883);
    client.connect(hostName, mqtt_user, mqtt_password);
    client.setCallback(mqttCallback);
    if (client.connected()) {
        client.subscribe(brightness_topic);
        client.subscribe(applet_topic);
    }
}

void configModeCallback (WiFiManager *wm) {
    marqueeText("setup", dma_display.color565(0,255,255));
}

void setup() {
    Serial.begin(115200);
    dma_display.begin();
    dma_display.setBrightness8(currentBrightness); //0-255
    dma_display.setLatBlanking(3);
    dma_display.clearScreen();

    Wire.begin();
    if(!tsl.begin())
    {
        /* There was a problem detecting the TSL2561 ... check your connections */
        Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */

    WiFiManager wifiManager;

    uint8_t baseMac[6];
    char macFull[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf(hostName, 11, PSTR("PLM-%s"),macFull);

    WiFi.setHostname(hostName);
    WiFi.setAutoReconnect(true);

    wifiManager.setTimeout(180);
    wifiManager.setCleanConnect(true);

    if (!wifiManager.autoConnect(hostName)) {
        delay(3000);
    }

    ArduinoOTA.setHostname(hostName);
    ArduinoOTA.setMdnsEnabled(true);
    //ArduinoOTA.setPassword("PLM");

    ArduinoOTA.onStart([]() {
        currentMode = NONE;

        //Clear and reset all libwebp buffers.
        WebPDataClear(&webp_data);
        WebPDemuxReleaseIterator(&iter);
        WebPDemuxDelete(demux);

        //Free the frame buffers if necessary.
        if(currentFrame != nullptr) {
            free(currentFrame);
        }

        marqueeText("OTA: Start", dma_display.color565(0,255,0));
        delay(2500);
    });
    ArduinoOTA.onEnd([]() {
        marqueeText("OTA: Done", dma_display.color565(0,255,0));
        delay(2500);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        char progText[12];
        sprintf(progText, "OTA: %u%%", (progress / (total / 100)));

        if(strcmp(progText, lastProgText) != 0) {
            marqueeText(progText, dma_display.color565(255,255,0));
            strcpy(lastProgText, progText);
        }
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    snprintf_P(applet_topic, 22, PSTR("%s/%s/rx"), TOPIC_PREFIX, macFull);
    snprintf_P(applet_rts_topic, 26, PSTR("%s/%s/tx"), TOPIC_PREFIX, macFull);

    wifiClient.setInsecure();
    client.setServer(mqtt_server, 8883);
    client.connect(hostName, mqtt_user, mqtt_password);
    client.setCallback(mqttCallback);

    if (client.connected()) {
        client.subscribe(applet_topic);
        client.publish(applet_rts_topic, "DEVICE_BOOT");
    }
}

void loop() {
    // Try to reconnect to wifi if connection lost
    while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0,0,0,0)) {
        WiFi.reconnect();
        marqueeText("wi-fi", dma_display.color565(255,0,0));
        delay(10000);
    }

    client.loop();

    ArduinoOTA.handle();

    if (!client.connected()) {
        marqueeText("mqtt", dma_display.color565(255,0,0));
        mqttReconnect();
    }

    //Update desired brightness
    if (millis() - last_check_tsl_time > 500) {
        sensors_event_t event;
        tsl.getEvent(&event);

        if(event.light > 50) {
            //low brightness
            desiredBrightness = 120;
        } else if(event.light > 5) {
            //low brightness
            desiredBrightness = 30;
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

    if (currentMode == WELCOME) {
        currentMode = NONE;
    }

    if (currentMode == APPLET) {
        if(newapplet) {
            demux = WebPDemux(&webp_data);
            frame_count = WebPDemuxGetI(demux, WEBP_FF_FRAME_COUNT);
            webp_flags = WebPDemuxGetI(demux, WEBP_FF_FORMAT_FLAGS);

            currentFrame = (uint8_t *) malloc(MATRIX_HEIGHT * MATRIX_WIDTH * 3);

            newapplet = false;
            current_frame = 1;
        } else {
            if(webp_flags & ANIMATION_FLAG) {
                if(millis() - last_frame_time > last_frame_duration) {
                    if(WebPDemuxGetFrame(demux, current_frame, &iter)) {
                        uint8_t * fragmentTmp = (uint8_t *) malloc(iter.width*iter.height*4);
                        if(WebPDecodeRGBAInto(iter.fragment.bytes, iter.fragment.size, fragmentTmp, iter.width * iter.height * 4, iter.width * 4) != NULL) {
                            int px = 0;
                            for(int y = iter.y_offset; y < (iter.y_offset + iter.height); y++) {
                                for(int x = iter.x_offset; x < (iter.x_offset + iter.width); x++) {
                                    //go pixel by pixel.
                                    int pixelOffsetCF = ((y*MATRIX_WIDTH)+x)*3;
                                    int pixelOffsetFT = px*4;

                                    int alphaValue = fragmentTmp[pixelOffsetFT+3];
                                    
                                    if(alphaValue == 255) {
                                        memcpy(currentFrame+pixelOffsetCF, fragmentTmp+pixelOffsetFT, 3);
                                    }
                                    
                                    px++;
                                }
                            }

                            //currentframe is good to send to screen now
                            for(int y = 0; y < MATRIX_HEIGHT; y++) {
                                for(int x = 0; x < MATRIX_WIDTH; x++) {
                                    int pixBitStart = ((y*MATRIX_WIDTH)+x)*3;
                                    dma_display.writePixel(x,y, dma_display.color565(currentFrame[pixBitStart],currentFrame[pixBitStart+1],currentFrame[pixBitStart+2]));
                                }
                            }

                            last_frame_time = millis();
                            last_frame_duration = iter.duration;
                            current_frame++;
                            if(current_frame > frame_count) {
                                current_frame = 1;
                            }
                        }
                        free(fragmentTmp);
                    } else {
                        currentMode = NONE;
                    }
                }
            } else {
                //Static WebP
                uint8_t * tempPixelBuffer = (uint8_t *) malloc(MATRIX_HEIGHT * MATRIX_WIDTH * 3);
                WebPDecodeRGBInto(webp_data.bytes, webp_data.size, tempPixelBuffer, MATRIX_HEIGHT * MATRIX_WIDTH * 3, MATRIX_WIDTH * 3);
                
                for(int y = 0; y < MATRIX_HEIGHT; y++) {
                    for(int x = 0; x < MATRIX_WIDTH; x++) {
                        int pixBitStart = ((y*MATRIX_WIDTH)+x)*3;
                        dma_display.writePixel(x,y, dma_display.color565(tempPixelBuffer[pixBitStart],tempPixelBuffer[pixBitStart+1],tempPixelBuffer[pixBitStart+2]));
                    }
                }

                free(tempPixelBuffer);

                currentMode = NONE;
            }
        }
    }

    if(millis() - mqtt_timeout_lastTime > 3000 && mqtt_timeout_lastTime != 0) {
        mqtt_timeout_lastTime = 0;
        recv_length = false;
        bufferPos = 0;
        client.publish(applet_rts_topic, "TIMEOUT");
    }
}