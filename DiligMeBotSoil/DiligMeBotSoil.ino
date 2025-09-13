#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"

// --- New Libraries for DS18B20 ---
#include <OneWire.h>
#include <DallasTemperature.h>

//Replace with your network credentials
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
// NOTE: set to 10 minutes (change to 30UL*1000UL for 30 seconds)
const unsigned long captureInterval = 10UL * 60UL * 1000UL; // 10 minutes

// --- Snapshot handler ---
static esp_err_t capture_handler(httpd_req_t *req){
  unsigned long now = millis();

  if (now - lastCaptureTime < captureInterval) {
    String msg = "Please wait before next capture. Remaining: " +
                 String((captureInterval - (now - lastCaptureTime)) / 1000) + " seconds";
    httpd_resp_send(req, msg.c_str(), msg.length());
    return ESP_OK;
  }

  // Turn flash ON
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(200); // allow camera to adjust

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get FB");
    httpd_resp_send_500(req);
    // Turn flash OFF even on failure
    digitalWrite(FLASH_LED_PIN, LOW);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_send(req, (const char *)fb->buf, fb->len);

  esp_camera_fb_return(fb);

  // Turn flash OFF
  digitalWrite(FLASH_LED_PIN, LOW);

  lastCaptureTime = now;
  return ESP_OK;
}

// --- DS18B20 temperature handler ---
static esp_err_t temperature_handler(httpd_req_t *req){
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  String message;
  if (tempC == DEVICE_DISCONNECTED_C) {
    message = "Error: DS18B20 not detected";
  } else {
    message = "Soil Temperature: " + String(tempC) + " °C";
  }
  httpd_resp_send(req, message.c_str(), message.length());
  return ESP_OK;
}

// --- Soil Moisture handler ---
static esp_err_t moisture_handler(httpd_req_t *req){
  int moistureState = digitalRead(SOIL_MOISTURE_PIN);
  String message;

  if (moistureState == HIGH) {   // dry
    message = "Soil Moisture: DRY";
    digitalWrite(SOIL_DRY_SIGNAL_PIN, HIGH);  // signal ON
  } else {                       // wet
    message = "Soil Moisture: WET";
    digitalWrite(SOIL_DRY_SIGNAL_PIN, LOW);   // signal OFF
  }

  httpd_resp_send(req, message.c_str(), message.length());
  return ESP_OK;
}

// --- Index page handler ---
static esp_err_t index_handler(httpd_req_t *req){
  const char* resp_str =
    "<html>"
    "<head><title>ESP32-CAM Sensor Hub</title></head>"
    "<body>"
    "<h2>ESP32-CAM Sensor Hub</h2>"
    "<p><a href=\"/capture\">Take Photo (every 10 mins)</a></p>"
    "<p><a href=\"/temperature\">Check Soil Temperature</a></p>"
    "<p><a href=\"/moisture\">Check Soil Moisture</a></p>"
    "</body>"
    "</html>";
  httpd_resp_send(req, resp_str, strlen(resp_str));
  return ESP_OK;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_page = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t capture_uri = {
    .uri       = "/capture",
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t temp_uri = {
    .uri       = "/temperature",
    .method    = HTTP_GET,
    .handler   = temperature_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t moist_uri = {
    .uri       = "/moisture",
    .method    = HTTP_GET,
    .handler   = moisture_handler,
    .user_ctx  = NULL
  };

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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // Setup sensor pins
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);     
  pinMode(SOIL_MOISTURE_PIN, INPUT_PULLUP);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  pinMode(SOIL_DRY_SIGNAL_PIN, OUTPUT);
  digitalWrite(SOIL_DRY_SIGNAL_PIN, LOW);

  // Sensors
  sensors.begin();

  // Camera config
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
    Serial.println("PSRAM found: using fb_count=1");
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 35;
    config.fb_count = 1;
    Serial.println("No PSRAM: using fb_count=1");
  }
  
  // Init camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Wi-Fi connection
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
  // Keep soil dry signal updated continuously
  int moistureState = digitalRead(SOIL_MOISTURE_PIN);
  if (moistureState == HIGH) {
    digitalWrite(SOIL_DRY_SIGNAL_PIN, HIGH); // dry
    Serial.println("WATERED");
  } else {
    digitalWrite(SOIL_DRY_SIGNAL_PIN, LOW);  // wet
  }
  delay(1000);
}
