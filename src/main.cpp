
#include <FS.h>
#include "SPIFFS.h"

#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#ifdef MQTT_SSL
#include <WiFiClientSecure.h>
#else
#include <WiFiClient.h>
#endif

#include <time.h>
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
HUB75_I2S_CFG::i2s_pins _pins = {2, 22, 21, 4, 27, 23, 26, 5, 25, 18, -1, 19, 32, 33}; // what actually works for me
//HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};

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

char device_name[25] = "skidbyt_1";
char hostName[80];
char applet_topic[22];
char applet_rts_topic[26];
char heap_topic[26];
char brightness_topic[22];
char lastProgText[12];

WebPData webp_data;

int currentMode = WELCOME;
int desiredBrightness = 20;
int currentBrightness = 100;
unsigned long bufferPos;
bool recv_length = false;

#ifdef MQTT_SSL
WiFiClientSecure wifiClient;
#else
WiFiClient wifiClient;
#endif

PubSubClient client(wifiClient);

uint8_t *tmpbuf;
unsigned long bufsize;

WebPDemuxer* demux;
WebPIterator iter;
uint32_t webp_flags;
uint32_t current_frame = 1;
uint32_t frame_count;

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
            Serial.print("got start.");

            client.publish(applet_rts_topic, "OK");
        } else if(strncmp((char *)payload,"PING",4) == 0) {
            client.publish(applet_rts_topic, "PONG");
        } else if(!recv_length) {
            mqtt_timeout_lastTime = millis();
            
            bufsize = atoi((char *)payload);
            Serial.print("Buff Size: "); Serial.print(bufsize);Serial.print(".");
            tmpbuf = (uint8_t *) malloc(bufsize);
            recv_length = true;
            client.publish(applet_rts_topic, "OK");
            // Serial.println("got length");

        } else {
            if(strncmp((char *)payload,"FINISH",6) == 0) {
                mqtt_timeout_lastTime = 0;
                Serial.println("got finish");
                if (strncmp((const char*)tmpbuf, "RIFF", 4) == 0) {
                    Serial.print("got riff.");
                    //Clear and reset all libwebp buffers.
                    WebPDataClear(&webp_data);
                    WebPDemuxReleaseIterator(&iter);
                    WebPDemuxDelete(demux);

                    //setup webp buffer and populate from temporary buffer
                    webp_data.size = bufsize;
                    webp_data.bytes = (uint8_t *) WebPMalloc(bufsize);

                    memcpy((void *)webp_data.bytes, tmpbuf, bufsize);

                    //free temporary buffer
                    // Serial.println("freeing buffer");
                    free(tmpbuf);

                    //set display flags!
                    newapplet = true;
                    currentMode = APPLET;
                    // client.publish(applet_rts_topic, "PUSHED");
                    // Serial.println("published PUSHED");
                } else {
                    client.publish(applet_rts_topic, "DECODE_ERROR");
                }
                bufferPos = 0;
                recv_length = false;
            } else {
                Serial.print("got data.");
                mqtt_timeout_lastTime = millis();
                memcpy((void *)(tmpbuf+bufferPos), payload, length);
                bufferPos += length;
                // client.publish(applet_rts_topic, "OK");
            }
        }
    } else if(strcmp(topic, brightness_topic) == 0 ) { // 1 - 9 brightness
        char str[2]; // 1 characters + null terminator
        str[0] = payload[0];
        str[1] = '\0'; // Null-terminate the string

        int single = atoi(str); // Convert the string to an integer
        desiredBrightness = map(single,0,9,0,100);
        // String str = String((char *)payload).substring(0,2);
        // Serial.println(str);
        // desiredBrightness = str.toInt();
        Serial.print("got brightness : ");
        Serial.println(desiredBrightness);
    }
}


void mqttReconnect() {
    if (!client.connected()) {
        Serial.println("attempting reconnnect");
        client.connect(hostName, MQTT_USERNAME, MQTT_PASSWORD);
        delay(1000);
    } else {
        Serial.println("mqtt connection recovered");
    }

    if (client.connected())
    {
        Serial.println("Subscribing to : " + String(applet_topic));
        client.subscribe(applet_topic);
        client.subscribe(brightness_topic);
        Serial.println("Publishing MQTT_RECONNECT to : " + String(applet_rts_topic));
        client.publish(applet_rts_topic, "MQTT_RECONNECT");
    }
}

void configModeCallback (WiFiManager *wm) {
    marqueeText("Setup", dma_display.color565(0,255,255));
}

void setup() {
    Serial.begin(115200);

    // starting wifi here increases reliability of connecting to wifi and not erroneously going into wifimanager
    uint8_t baseMac[6];
    char macFull[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf(hostName, 11, PSTR("Skidbyt-%s"),macFull);
    WiFi.setHostname(hostName);
    WiFi.setAutoReconnect(true);
    WiFi.begin(); 

    dma_display.begin();
    dma_display.setBrightness8(currentBrightness); //0-255
    dma_display.setLatBlanking(1);
    dma_display.clearScreen();
    marqueeText("SkidByt", dma_display.color565(0,255,255));

    #ifndef TIDBYT
    Wire.begin();
    if(!tsl.begin())
    {
        marqueeText("NO ALS", dma_display.color565(255,0,0));
        while(1);
    }

    tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
    #endif

    WiFiManager wifiManager;

    delay(5000); // wait 5 secs for connection before calling autoconnect.
    wifiManager.setTimeout(180);
    wifiManager.setCleanConnect(true);

    if (!wifiManager.autoConnect(hostName)) {
        delay(3000);
    }

    ArduinoOTA.setHostname(hostName);
    //ArduinoOTA.setPassword("SmX");

    ArduinoOTA.onStart([]() {
        currentMode = NONE;

        //Clear and reset all libwebp buffers.
        WebPDataClear(&webp_data);
        WebPDemuxReleaseIterator(&iter);
        WebPDemuxDelete(demux);

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
        marqueeText("OTA: error", dma_display.color565(0,255,0));
        delay(2500);
    });

    ArduinoOTA.begin();
    snprintf_P(applet_topic, 22, PSTR("%s/img"), device_name);
    snprintf_P(brightness_topic, 22, PSTR("%s/brightness"), device_name);
    snprintf_P(applet_rts_topic, 26, PSTR("%s/tx"), device_name);
    snprintf_P(heap_topic, 26, PSTR("%s/heap"), device_name);


    #ifdef MQTT_SSL
    wifiClient.setInsecure();
    #endif

    client.setServer(MQTT_HOST, MQTT_PORT);
    
    client.connect(hostName, MQTT_USERNAME, MQTT_PASSWORD);
    client.setCallback(mqttCallback);
    client.setBufferSize(500000); // 500kb

        if (client.connected())
    {
        Serial.println("Subscribing to : " + String(applet_topic));
        client.subscribe(applet_topic);
        client.subscribe(brightness_topic);
        Serial.println("Publishing BOOT to : " + String(applet_rts_topic));
        client.publish(applet_rts_topic, "DEVICE_BOOT");
    }

    marqueeText("ready..", dma_display.color565(255,255,255));
}

void loop() {
    // Try to reconnect to wifi if connection lost
    while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0,0,0,0)) {
        WiFi.reconnect();
        marqueeText("Wi-Fi", dma_display.color565(255,0,0));
        delay(5000);
    }


    ArduinoOTA.handle();

    if (!client.connected()) {
        // marqueeText("MQTT", dma_display.color565(255,0,0));
        // delay(5000);
        mqttReconnect();
    }

    client.loop();
    //Update desired brightness

    if (millis() - last_adjust_brightness_time > 10 && currentBrightness != desiredBrightness) {
        if(currentBrightness > desiredBrightness) {
            currentBrightness--;
        } else {
            currentBrightness++;
        }
        dma_display.setBrightness8(currentBrightness);
        last_adjust_brightness_time = millis();
    }

    if (currentMode == APPLET) {
        if(newapplet) {
            demux = WebPDemux(&webp_data);
            frame_count = WebPDemuxGetI(demux, WEBP_FF_FRAME_COUNT);
            webp_flags = WebPDemuxGetI(demux, WEBP_FF_FORMAT_FLAGS);

            newapplet = false;
            current_frame = 1;
            char heap[50]; // Ensure this is large enough
            int h = ESP.getFreeHeap();
            snprintf(heap, sizeof(heap), "%d", heap);
            Serial.print(heap);
            client.publish(heap_topic, heap);
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

                                    int pixelOffsetFT = px*4;
                                    int alphaValue = fragmentTmp[pixelOffsetFT+3];
                                    
                                    if(alphaValue == 255) {
                                        dma_display.writePixel(x,y, dma_display.color565(fragmentTmp[pixelOffsetFT],fragmentTmp[pixelOffsetFT+1],fragmentTmp[pixelOffsetFT+2]));
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