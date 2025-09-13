#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"

// --- New Libraries for DS18B20 ---
#include <OneWire.h>
#include <DallasTemperature.h>
#include <base64.h>  // for encoding JPEG into JSON-safe string

// Replace with your network credentials
const char* ssid = "PLDTHOMEFIBRB74t8";
const char* password = "PLDTWIFIcnRg9";

// DS18B20 on GPIO 13
#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Soil moisture digital pin
#define SOIL_MOISTURE_PIN 14

// Flash LED (fixed on GPIO 4 for ESP32-CAM)
#define FLASH_LED_PIN 4

// Output pin for soil dry signal
#define SOIL_DRY_SIGNAL_PIN 12

// Camera model (AI Thinker)
#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

httpd_handle_t stream_httpd = NULL;

// --- Interval control for snapshots ---
unsigned long lastCaptureTime = 0;
const unsigned long captureInterval = 1UL * 30UL * 1000UL; // 10 minutes

// --- Capture handler (returns base64 jpeg inside JSON) ---
static esp_err_t capture_handler(httpd_req_t *req){
  unsigned long now = millis();
  httpd_resp_set_type(req, "application/json");

  if (now - lastCaptureTime < captureInterval) {
    String json = "{\"status\":\"wait\",\"remaining\":" + 
                  String((captureInterval - (now - lastCaptureTime)) / 1000) + "}";
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
  }

  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(200);

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    digitalWrite(FLASH_LED_PIN, LOW);
    String json = "{\"error\":\"Failed to capture image\"}";
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_FAIL;
  }

  // Encode JPEG buffer to Base64 string
  String imageBase64 = base64::encode((uint8_t*)fb->buf, fb->len);

  String json = "{\"status\":\"ok\",\"image\":\"" + imageBase64 + "\"}";
  httpd_resp_send(req, json.c_str(), json.length());

  esp_camera_fb_return(fb);
  digitalWrite(FLASH_LED_PIN, LOW);
  lastCaptureTime = now;
  return ESP_OK;
}

// --- Temperature handler ---
static esp_err_t temperature_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "application/json");
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  String json;
  if (tempC == DEVICE_DISCONNECTED_C) {
    json = "{\"temperature\":\"error\"}";
  } else {
    json = "{\"temperature\":" + String(tempC, 2) + "}";
  }
  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

// --- Moisture handler ---
static esp_err_t moisture_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "application/json");
  int moistureState = digitalRead(SOIL_MOISTURE_PIN);
  String json;

  if (moistureState == HIGH) {
    digitalWrite(SOIL_DRY_SIGNAL_PIN, HIGH);
    json = "{\"moisture\":\"dry\"}";
  } else {
    digitalWrite(SOIL_DRY_SIGNAL_PIN, LOW);
    json = "{\"moisture\":\"wet\"}";
  }

  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

// --- Index summary handler ---
static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "application/json");

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  int moistureState = digitalRead(SOIL_MOISTURE_PIN);

  String json = "{";
  json += "\"temperature\":";
  json += (tempC == DEVICE_DISCONNECTED_C) ? "\"error\"" : String(tempC, 2);
  json += ",";
  json += "\"moisture\":\"";
  json += (moistureState == HIGH) ? "dry" : "wet";
  json += "\"}";
  
  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_page = { .uri="/", .method=HTTP_GET, .handler=index_handler };
  httpd_uri_t capture_uri = { .uri="/capture", .method=HTTP_GET, .handler=capture_handler };
  httpd_uri_t temp_uri = { .uri="/temperature", .method=HTTP_GET, .handler=temperature_handler };
  httpd_uri_t moist_uri = { .uri="/moisture", .method=HTTP_GET, .handler=moisture_handler };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_page);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
    httpd_register_uri_handler(stream_httpd, &temp_uri);
    httpd_register_uri_handler(stream_httpd, &moist_uri);
  } else {
    Serial.println("HTTP Server start failed");
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);     
  pinMode(SOIL_MOISTURE_PIN, INPUT_PULLUP);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  pinMode(SOIL_DRY_SIGNAL_PIN, OUTPUT);
  digitalWrite(SOIL_DRY_SIGNAL_PIN, LOW);

  sensors.begin();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 35;
    config.fb_count = 1;
  }
  
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiStart > 20000) {
      Serial.println("\nWiFi connect timeout");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("Server Ready! Go to: http://");
    Serial.println(WiFi.localIP());
    startCameraServer();
  } else {
    Serial.println("WiFi not connected - server not started");
  }
}

void loop() {
  int moistureState = digitalRead(SOIL_MOISTURE_PIN);
  if (moistureState == HIGH) {
    digitalWrite(SOIL_DRY_SIGNAL_PIN, LOW);
  } else {
    digitalWrite(SOIL_DRY_SIGNAL_PIN, HIGH);
  }
  delay(1000);
}
