/* ============================================================
 * ESP32-CAM + Edge Impulse + OLED + Smooth Web Camera + Detection
 * ============================================================
 * Features:
 *  - Live browser camera using MJPEG stream
 *  - When detection starts, browser stream closes automatically
 *  - ESP32 focuses only on detection while detection is ON
 *  - When detection stops, camera stream returns
 *  - Serial + OLED detection output
 *  - Lower lag by using one camera task at a time
 * ============================================================ */

#include <nakiz0-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── Camera model ────────────────────────────────────────────
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM  32
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM   0
  #define SIOD_GPIO_NUM  26
  #define SIOC_GPIO_NUM  27
  #define Y9_GPIO_NUM    35
  #define Y8_GPIO_NUM    34
  #define Y7_GPIO_NUM    39
  #define Y6_GPIO_NUM    36
  #define Y5_GPIO_NUM    21
  #define Y4_GPIO_NUM    19
  #define Y3_GPIO_NUM    18
  #define Y2_GPIO_NUM     5
  #define VSYNC_GPIO_NUM 25
  #define HREF_GPIO_NUM  23
  #define PCLK_GPIO_NUM  22
#else
  #error "Camera model not selected"
#endif

// ─── EI frame constants ──────────────────────────────────────
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_FRAME_BYTE_SIZE        3

// ─── OLED ────────────────────────────────────────────────────
#define I2C_SDA        15
#define I2C_SCL        14
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

// ─── WiFi ────────────────────────────────────────────────────
const char* WIFI_SSID     = "Nothing Phone (3a)_1813";
const char* WIFI_PASSWORD = "12345677";

// ─── Web server ──────────────────────────────────────────────
WebServer server(80);

// ─── State ───────────────────────────────────────────────────
static bool detection_enabled = false;
static bool is_initialised = false;
static bool debug_nn = false;

struct DetResult {
  char  label[32];
  float confidence;
  bool  found;
} lastResult = {"", 0.0f, false};

uint8_t *snapshot_buf = nullptr;

static camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sscb_sda   = SIOD_GPIO_NUM,
    .pin_sscb_scl   = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,

    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_QVGA,
    .jpeg_quality   = 10,
    .fb_count       = 2,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    .grab_mode      = CAMERA_GRAB_LATEST,
};

// ─── Forward declarations ────────────────────────────────────
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t w, uint32_t h, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void runDetection(void);
void updateOLED(void);
void handleStream(void);
void handleToggle(void);
void handleResult(void);
void handleRoot(void);

// ─────────────────────────────────────────────────────────────
//  HTML
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-CAM Edge Impulse</title>
<style>
  body {
    font-family: Arial, sans-serif;
    background: #0f1116;
    color: #e9eef7;
    text-align: center;
    margin: 0;
    padding: 18px;
  }
  h2 { margin: 10px 0 16px; font-weight: 600; }
  #wrap {
    width: 340px;
    margin: 0 auto;
  }
  #stream {
    width: 320px;
    height: 240px;
    border: 1px solid #2b3140;
    border-radius: 10px;
    object-fit: cover;
    background: #000;
  }
  #status {
    margin-top: 10px;
    font-size: 14px;
    color: #b9c3d4;
    min-height: 20px;
  }
  #result {
    margin-top: 8px;
    font-size: 16px;
    color: #7ee787;
    min-height: 24px;
  }
  #btnDetect {
    margin-top: 16px;
    padding: 12px 24px;
    font-size: 16px;
    border: none;
    border-radius: 8px;
    background: #2e7d32;
    color: white;
    cursor: pointer;
  }
  #btnDetect.stop {
    background: #c62828;
  }
  .small {
    margin-top: 8px;
    font-size: 12px;
    color: #8a93a6;
  }
</style>
</head>
<body>
  <div id="wrap">
    <h2>ESP32-CAM Object Detection</h2>
    <img id="stream" src="/stream" alt="camera">
    <div id="status">Live camera ready</div>
    <div id="result">No result yet</div>
    <button id="btnDetect">Start Detection</button>
    <div class="small">Detection pauses camera preview to reduce lag</div>
  </div>

<script>
  const btn = document.getElementById('btnDetect');
  const img = document.getElementById('stream');
  const statusBox = document.getElementById('status');
  const resultBox = document.getElementById('result');

  let detecting = false;
  let pollTimer = null;

  async function setDetectionState(on) {
    const res = await fetch('/toggle?state=' + (on ? '1' : '0'), { cache: 'no-store' });
    return res.ok;
  }

  async function pollResult() {
    if (!detecting) return;

    try {
      const res = await fetch('/result', { cache: 'no-store' });
      if (!res.ok) return;
      const data = await res.json();

      if (data.found) {
        resultBox.textContent = data.label + ' - ' + data.confidence + '%';
      } else {
        resultBox.textContent = 'No object detected';
      }
    } catch (e) {
      // ignore
    }
  }

  btn.addEventListener('click', async () => {
    detecting = !detecting;

    if (detecting) {
      img.src = '';
      btn.textContent = 'Stop Detection';
      btn.classList.add('stop');
      statusBox.textContent = 'Detection running. Camera preview paused.';
      resultBox.textContent = 'Waiting for result...';

      const ok = await setDetectionState(true);
      if (!ok) {
        statusBox.textContent = 'Failed to start detection';
        detecting = false;
        btn.textContent = 'Start Detection';
        btn.classList.remove('stop');
        return;
      }

      if (pollTimer) clearInterval(pollTimer);
      pollTimer = setInterval(pollResult, 700);
    } else {
      const ok = await setDetectionState(false);
      if (!ok) {
        statusBox.textContent = 'Failed to stop detection';
        detecting = true;
        return;
      }

      if (pollTimer) {
        clearInterval(pollTimer);
        pollTimer = null;
      }

      btn.textContent = 'Start Detection';
      btn.classList.remove('stop');
      statusBox.textContent = 'Live camera ready';
      resultBox.textContent = 'No result yet';
      img.src = '/stream?t=' + Date.now();
    }
  });
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────────
//  Web handlers
// ─────────────────────────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleToggle() {
  if (server.hasArg("state")) {
    detection_enabled = (server.arg("state") == "1");
    Serial.print("Detection: ");
    Serial.println(detection_enabled ? "ON" : "OFF");
  }
  server.send(200, "text/plain", detection_enabled ? "on" : "off");
}

void handleResult() {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"found\":%s,\"label\":\"%s\",\"confidence\":%d}",
           lastResult.found ? "true" : "false",
           lastResult.label,
           (int)(lastResult.confidence * 100.0f));
  server.send(200, "application/json", buf);
}

void handleStream() {
  if (detection_enabled) {
    server.send(503, "text/plain", "Stream disabled during detection");
    return;
  }

  WiFiClient client = server.client();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Cache-Control: no-cache, no-store, must-revalidate");
  client.println("Pragma: no-cache");
  client.println("Connection: close");
  client.println();

  while (client.connected() && !detection_enabled) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

    client.printf("--frame\r\n");
    client.printf("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", (unsigned)fb->len);
    client.write(fb->buf, fb->len);
    client.printf("\r\n");

    esp_camera_fb_return(fb);
    delay(35);
  }

  client.stop();
}

// ─────────────────────────────────────────────────────────────
//  OLED helper
// ─────────────────────────────────────────────────────────────
void updateOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (!detection_enabled) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Live preview");
    display.println("Detection OFF");
    display.setCursor(0, 24);
    display.println(WiFi.localIP().toString());
    display.setCursor(0, 42);
    display.println("Open browser");
  } else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Detection ON");

    if (lastResult.found) {
      display.setCursor(0, 18);
      display.setTextSize(2);
      display.println(lastResult.label);
      display.setTextSize(1);
      display.setCursor(0, 48);
      display.print("Conf: ");
      display.print((int)(lastResult.confidence * 100.0f));
      display.println("%");
    } else {
      display.setCursor(0, 24);
      display.setTextSize(2);
      display.println("No obj");
    }
  }

  display.display();
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 OLED init failed");
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Edge Impulse");
  display.println("Booting...");
  display.display();

  if (!ei_camera_init()) {
    Serial.println("Camera init failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Camera fail");
    display.display();
    while (true) delay(1000);
  }

  snapshot_buf = (uint8_t*)malloc(
    EI_CAMERA_RAW_FRAME_BUFFER_COLS *
    EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
    EI_CAMERA_FRAME_BYTE_SIZE
  );

  if (!snapshot_buf) {
    Serial.println("ERR: snapshot buffer alloc failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Buffer fail");
    display.display();
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi");
  display.display();

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected: ");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi connected");
  display.setCursor(0, 12);
  display.println(WiFi.localIP().toString());
  display.setCursor(0, 28);
  display.println("Open browser");
  display.display();

  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.on("/toggle", handleToggle);
  server.on("/result", handleResult);
  server.begin();

  Serial.println("HTTP server started");
  updateOLED();
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────
unsigned long lastDetectionMs = 0;
#define DETECTION_INTERVAL_MS 300

void loop() {
  server.handleClient();

  if (!detection_enabled) {
    updateOLED();
    delay(10);
    return;
  }

  if (millis() - lastDetectionMs < DETECTION_INTERVAL_MS) {
    delay(2);
    return;
  }

  lastDetectionMs = millis();
  runDetection();
  updateOLED();
}

// ─────────────────────────────────────────────────────────────
//  Detection
// ─────────────────────────────────────────────────────────────
void runDetection() {
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (!ei_camera_capture((uint32_t)EI_CLASSIFIER_INPUT_WIDTH,
                         (uint32_t)EI_CLASSIFIER_INPUT_HEIGHT,
                         snapshot_buf)) {
    Serial.println("ERR: capture failed");
    return;
  }

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

  if (err != EI_IMPULSE_OK) {
    Serial.printf("ERR: classifier failed (%d)\n", err);
    return;
  }

  Serial.printf("Predictions (DSP: %d ms, Class: %d ms, Anomaly: %d ms)\n",
                result.timing.dsp,
                result.timing.classification,
                result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  lastResult.found = false;
  lastResult.confidence = 0.0f;
  strncpy(lastResult.label, "none", sizeof(lastResult.label) - 1);
  lastResult.label[sizeof(lastResult.label) - 1] = '\0';

  float best_val = 0.0f;

  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    if (bb.value <= 0) continue;

    Serial.printf("  %s (%.2f) [x:%u y:%u w:%u h:%u]\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

    if (bb.value > best_val) {
      best_val = bb.value;
      lastResult.found = true;
      lastResult.confidence = bb.value;
      strncpy(lastResult.label, bb.label, sizeof(lastResult.label) - 1);
      lastResult.label[sizeof(lastResult.label) - 1] = '\0';
    }
  }

  if (!lastResult.found) {
    Serial.println("  No objects found");
  }
#else
  float best_val = 0.0f;
  const char *best_label = "none";

  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float v = result.classification[i].value;
    const char *label = ei_classifier_inferencing_categories[i];
    Serial.printf("  %s: %.5f\n", label, v);

    if (v > best_val) {
      best_val = v;
      best_label = label;
    }
  }

  lastResult.found = true;
  lastResult.confidence = best_val;
  strncpy(lastResult.label, best_label, sizeof(lastResult.label) - 1);
  lastResult.label[sizeof(lastResult.label) - 1] = '\0';
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
  Serial.printf("Anomaly: %.3f\n", result.anomaly);
#endif
}

// ─────────────────────────────────────────────────────────────
//  Camera helpers
// ─────────────────────────────────────────────────────────────
bool ei_camera_init(void) {
  if (is_initialised) return true;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
  }

  is_initialised = true;
  return true;
}

void ei_camera_deinit(void) {
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    Serial.println("Camera deinit failed");
  }
  is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) {
    Serial.println("ERR: Camera not initialized");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);
  esp_camera_fb_return(fb);

  if (!converted) {
    Serial.println("Conversion failed");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) ||
      (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height
    );
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t out_ptr_ix = 0;

  while (length != 0) {
    out_ptr[out_ptr_ix++] =
      (snapshot_buf[pixel_ix + 2] << 16) +
      (snapshot_buf[pixel_ix + 1] << 8) +
      snapshot_buf[pixel_ix];

    pixel_ix += 3;
    length--;
  }

  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif