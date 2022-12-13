#include <FS.h>
#include "SPIFFS.h"

#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <webp/demux.h>

#include <constants.h>
//#include <mqtt.h>

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

MatrixPanel_I2S_DMA *dma_display = nullptr;

//Another way of creating config structure
//Custom pin mapping for all pins

const uint16_t kMatrixWidth = 64;       // Set to the width of your display, must be a multiple of 8
const uint16_t kMatrixHeight = 32;      // Set to the height of your display

HUB75_I2S_CFG::i2s_pins _pins={2, 15, 4, 16, 27, 17, 5, 18, 19, 21, -1, 26, 25, 22};
HUB75_I2S_CFG mxconfig(
						64,   // width
						32,   // height
						 1,   // chain length
					 _pins,   // pin mapping
  HUB75_I2S_CFG::FM6124     // driver chip
);


void setupTopics();

// range 0-255
int defaultBrightness = 20;

byte mac[6];
char macFull[6];

char mqtt_server[80] = "**REDACTED**";
char mqtt_user[80] = "**REDACTED**";
char mqtt_password[80] = "**REDACTED**";

boolean newapplet = false;

char hostName[80];
char applet_topic[22];
char applet_rts_topic[26];
char brightness_topic[22];

WebPData webp_data;

int currentMode = WELCOME;
int brightness = -1;
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
uint8_t * lastFrame;

unsigned long mqtt_timeout_lastTime = 0;
unsigned long last_frame_duration = 0;
unsigned long last_frame_time = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, applet_topic) == 0) {
        if(strncmp((char *)payload,"START",3) == 0) {
            mqtt_timeout_lastTime = millis();
            currentMode = NONE;
            recv_length = false;
            bufferPos = 0;
            WebPDataClear(&webp_data);
            client.publish(applet_rts_topic, "OK");
        }else if(strncmp((char *)payload,"PING",4) == 0) {
            client.publish(applet_rts_topic, "PONG");
        } else if(!recv_length) {
            mqtt_timeout_lastTime = millis();
            webp_data.size = atoi((char *)payload);
            webp_data.bytes = (uint8_t *) WebPMalloc(webp_data.size);
            recv_length = true;
            client.publish(applet_rts_topic, "OK");
        } else {
            if(strncmp((char *)payload,"FINISH",6) == 0) {
                mqtt_timeout_lastTime = 0;
                if (strncmp((const char*)webp_data.bytes, "RIFF", 4) == 0) {
                    
                    if(currentFrame != nullptr) {
                        free(currentFrame);
                    }

                    if(lastFrame != nullptr) {
                        free(lastFrame);
                    }

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
                memcpy((void*) (webp_data.bytes+bufferPos), payload, length);
                bufferPos += length;
                client.publish(applet_rts_topic, "OK");
            }
        }
    }
    if (strcmp(topic, brightness_topic) == 0) {
        payload[length] = '\0';
        brightness = atoi((char*)payload);
    }
}


void mqttReconnect(char* mqtt_user, char* mqtt_password) {
    while (!client.connected()) {
        if (client.connect(hostName, mqtt_user, mqtt_password)) {
            client.subscribe(brightness_topic);
            client.subscribe(applet_topic);
        }
    }
}

bool saveConfig = false;

void saveConfigCallback () {
    saveConfig = true;
}

void setup() {
    Serial.begin(115200);

    WiFiManager wifiManager;
    WiFi.macAddress(mac);
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(macFull, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
    snprintf(hostName, 11, PSTR("PLM-%s"),macFull);

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin();
    dma_display->setBrightness8(defaultBrightness); //0-255
    dma_display->clearScreen();

    WiFi.setHostname(hostName);
    WiFi.setAutoReconnect(true);

    wifiManager.setTimeout(180);
    wifiManager.setCleanConnect(true);

    if (!wifiManager.autoConnect(hostName)) {
        delay(3000);
    }

    setupTopics();

    wifiClient.setInsecure();
    client.setServer(mqtt_server, 8883);
    client.connect(hostName, mqtt_user, mqtt_password);
    client.setCallback(mqttCallback);

    if (client.connected()) {
        client.subscribe(applet_topic);
        client.subscribe(brightness_topic);
        client.publish(applet_rts_topic, "DEVICE_BOOT");
    }
}

void loop() {
    // Try to reconnect to wifi if connection lost
    while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0,0,0,0)) {
        WiFi.reconnect();
        delay(10000);
    }

    client.loop();

    if (!client.connected()) {
        mqttReconnect(mqtt_user, mqtt_password);
    }

    if (brightness > 0) {
        dma_display->setBrightness8((brightness*255) / 100);
        brightness = -1;
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
            lastFrame = (uint8_t *) malloc(MATRIX_HEIGHT * MATRIX_WIDTH * 3);

            newapplet = false;
        } else {
            if(webp_flags & ANIMATION_FLAG) {
                if(millis() - last_frame_time > last_frame_duration) {
                    WebPDemuxGetFrame(demux, current_frame, &iter);

                    if(iter.width == MATRIX_WIDTH && iter.height == MATRIX_HEIGHT) {
                        //replace entire buffer on screen.
                        WebPDecodeRGBInto(iter.fragment.bytes, iter.fragment.size, currentFrame, MATRIX_HEIGHT * MATRIX_WIDTH * 3, MATRIX_WIDTH * 3);
                    } else {
                        //else, start with last frame and add fragment!
                        memcpy(currentFrame, lastFrame, MATRIX_HEIGHT * MATRIX_WIDTH * 3);

                        uint8_t * fragmentTmp = (uint8_t *) malloc(iter.width*iter.height*4);
                        WebPDecodeRGBAInto(iter.fragment.bytes, iter.fragment.size, fragmentTmp, iter.width * iter.height * 4, iter.width * 4);

                        int px = 0;
                        for(int y = iter.y_offset; y < (iter.y_offset + iter.height); y++) {
                            for(int x = iter.x_offset; x < (iter.x_offset + iter.width); x++) {
                                //go pixel by pixel.
                                int pixelOffsetCF = ((y*MATRIX_WIDTH)+x)*3;
                                int pixelOffsetFT = px*4;
                                if(fragmentTmp[pixelOffsetFT+3] == 255) {
                                    memcpy(currentFrame+pixelOffsetCF, fragmentTmp+pixelOffsetFT, 3);
                                }
                                px++;
                            }
                        }
                        free(fragmentTmp);
                    }

                    //currentframe is good to send to screen now
                    for(int y = 0; y < MATRIX_HEIGHT; y++) {
                        for(int x = 0; x < MATRIX_WIDTH; x++) {
                            int pixBitStart = ((y*MATRIX_WIDTH)+x)*3;
                            dma_display->writePixel(x,y, dma_display->color565(currentFrame[pixBitStart],currentFrame[pixBitStart+1],currentFrame[pixBitStart+2]));
                        }
                    }

                    last_frame_time = millis();
                    last_frame_duration = iter.duration;
                    memcpy(lastFrame, currentFrame, MATRIX_HEIGHT * MATRIX_WIDTH * 3);
                    current_frame++;
                    if(current_frame > frame_count) {
                        current_frame = 1;
                    }
                }
            } else {
                //Static WebP
                uint8_t * tempPixelBuffer = (uint8_t *) malloc(MATRIX_HEIGHT * MATRIX_WIDTH * 3);
                WebPDecodeRGBInto(webp_data.bytes, webp_data.size, tempPixelBuffer, MATRIX_HEIGHT * MATRIX_WIDTH * 3, MATRIX_WIDTH * 3);
                
                for(int y = 0; y < MATRIX_HEIGHT; y++) {
                    for(int x = 0; x < MATRIX_WIDTH; x++) {
                        int pixBitStart = ((y*MATRIX_WIDTH)+x)*3;
                        dma_display->writePixel(x,y, dma_display->color565(tempPixelBuffer[pixBitStart],tempPixelBuffer[pixBitStart+1],tempPixelBuffer[pixBitStart+2]));
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

void setupTopics() {
    snprintf_P(applet_topic, 22, PSTR("%s/%s/applet"), TOPIC_PREFIX, macFull);
    snprintf_P(applet_rts_topic, 26, PSTR("%s/%s/applet/rts"), TOPIC_PREFIX, macFull);
    snprintf_P(brightness_topic, 22, PSTR("%s/%s/brightness"), TOPIC_PREFIX, macFull);
}