#include <Garbage_classification_inferencing.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
// #include "HX711.h"  // Sensor de peso deshabilitado por ahora
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <Preferences.h>

/* =========================
   WIFI
   ========================= */
const char* ssid = "Hotspot3";
const char* password = "12345678";

/* =========================
   UBIDOTS
   ========================= */

const char* UBIDOTS_TOKEN = "BBUS-pFREBdzamEXnQvOOpsILJKFsQqeKlE";
const char* DEVICE_LABEL = "caneca";
const char* MQTT_BROKER  = "industrial.api.ubidots.com";
const int   MQTT_PORT    = 1883;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

int count_organico = 0;
int count_aprovechable = 0;
int count_no_aprovechable = 0;

Preferences prefs;

// Evitar mensajes corruptos cuando imprimen varias tareas
SemaphoreHandle_t xSerialMutex = NULL;

#define SERIAL_PRINT(fmt, ...) \
    if (xSerialMutex && xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(100))) { \
        Serial.printf(fmt, ##__VA_ARGS__); \
        xSemaphoreGive(xSerialMutex); \
    }


/* =========================
    HX711 (deshabilitado)
    ========================= */

// #define HX_DT 4
// #define HX_SCK 5
// HX711 scale;

/* =========================
    Servo + LEDs + Buzzer
   ========================= */

#define SERVO_PIN 6
#define LED_VERDE 7
#define LED_ROJO 8
#define BUZZER 9

// Para evitar conflictos de timers/canales LEDC con cámara+servo, deshabilitar tono por defecto.
// Si tu buzzer es pasivo y necesitas tono, ponlo en 1 (pero puede requerir ajustar canales).
#define USE_TONE 0

Servo compuerta;

/* =========================
   Pines Cámara XIAO ESP32S3
   ========================= */

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define IMG_WIDTH   EI_CLASSIFIER_INPUT_WIDTH
#define IMG_HEIGHT  EI_CLASSIFIER_INPUT_HEIGHT

static uint8_t *resized_buf = nullptr;   // 64x64 RGB888
static uint8_t *rgb888_buf  = nullptr;   // QQVGA RGB888 (decodificado de JPEG)
static volatile bool frame_ready = false;

static volatile int accion_pendiente = 0;

static volatile bool verbose_scores = true;
static volatile bool use_center_crop = true;

// Requiere N detecciones consecutivas de la misma clase para actuar
#define CONFIRM_FRAMES 3
static int    consecutive_count = 0;
static String consecutive_label = "";

// Umbral propio (más alto que el del modelo para evitar falsos positivos)
#define CONFIDENCE_THRESHOLD 0.80f

static sensor_t *g_sensor = NULL;

/* =========================
   TAREA DE INFERENCIA (Core 0)
   ========================= */

void tareaInferencia(void *param) {

    for (;;) {

        if (!frame_ready) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

        // NO limpiar frame_ready aquí: resized_buf sigue en uso por el clasificador
        signal_t signal;
        signal.total_length = IMG_WIDTH * IMG_HEIGHT;
        signal.get_data = &ei_camera_get_data;

        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, false);

        // Ahora sí permitir que loop() escriba un nuevo frame
        frame_ready = false;

        if (ei_error != EI_IMPULSE_OK) {
            SERIAL_PRINT("Error classifier: %d\n", ei_error);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        float max_val = 0;
        int best_ix = 0;

        if (verbose_scores) {
            SERIAL_PRINT("--- Resultados ---\n");
        }
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (verbose_scores) {
                SERIAL_PRINT("  [%d] %s: %.3f\n", (int)ix, result.classification[ix].label, result.classification[ix].value);
            }
            if (result.classification[ix].value > max_val) {
                max_val = result.classification[ix].value;
                best_ix = (int)ix;
            }
        }

        String best_label = result.classification[best_ix].label;

        if (max_val < CONFIDENCE_THRESHOLD) {
            SERIAL_PRINT("=> uncertain (%.3f < umbral)\n", max_val);
            consecutive_count = 0;
            consecutive_label = "";
            accion_pendiente = 3;  // LED rojo (nada detectado)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        SERIAL_PRINT("=> Clase: %s | Conf: %.3f\n", best_label.c_str(), max_val);

        // Si tu modelo incluye 'idle', ignóralo explícitamente (fondo/ningún residuo)
        if (best_label.equalsIgnoreCase("idle")) {
            consecutive_count = 0;
            consecutive_label = "";
            accion_pendiente = 3;  // LED rojo (nada detectado)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        // Verificar detecciones consecutivas
        if (best_label.equalsIgnoreCase(consecutive_label.c_str())) {
            consecutive_count++;
        } else {
            consecutive_count = 1;
            consecutive_label = best_label;
        }

        SERIAL_PRINT("   Consecutivas: %d/%d\n", consecutive_count, CONFIRM_FRAMES);

        if (consecutive_count < CONFIRM_FRAMES) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        // Confirmado: N frames seguidos con la misma clase
        consecutive_count = 0;
        consecutive_label = "";

        // Ejecutar actuadores desde loop() (Core 1)
        if (best_label.equalsIgnoreCase("organico")) {
            count_organico++;
            prefs.putInt("organico", count_organico);
            accion_pendiente = 1;
            sendToUbidots("{\"organico\":" + String(count_organico) + ",\"ultima_clase\":1}");
        }
        else if (best_label.equalsIgnoreCase("aprovechable")) {
            count_aprovechable++;
            prefs.putInt("aprovechable", count_aprovechable);
            accion_pendiente = 1;
            sendToUbidots("{\"aprovechable\":" + String(count_aprovechable) + ",\"ultima_clase\":2}");
        }
        else if (best_label.equalsIgnoreCase("no_aprovechable")) {
            count_no_aprovechable++;
            prefs.putInt("no_aprov", count_no_aprovechable);
            accion_pendiente = 1;
            sendToUbidots("{\"no_aprovechable\":" + String(count_no_aprovechable) + ",\"ultima_clase\":3}");
        }
        else {
            SERIAL_PRINT("Clase '%s' no reconocida en el codigo\n", best_label.c_str());
        }

        mqtt.loop();
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

/* =========================
   MQTT FUNCTIONS
   ========================= */

void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  String clientId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (!mqtt.connect(clientId.c_str(), UBIDOTS_TOKEN, "")) {
    Serial.println("MQTT: fallo conexion");
  }
}

void sendToUbidots(String payload) {

  mqttReconnect();
  if (!mqtt.connected()) return;

  String topic = "/v1.6/devices/" + String(DEVICE_LABEL);

  mqtt.publish(topic.c_str(), payload.c_str());
  mqtt.loop();
}

/* =========================
   Resize (RGB888 – 3 bytes/pixel)
   ========================= */

static void resize_rgb888(const uint8_t *src, int src_w, int src_h,
                          uint8_t *dst, int dst_w, int dst_h)
{
    for (int y = 0; y < dst_h; y++) {
        int src_y = y * src_h / dst_h;
        for (int x = 0; x < dst_w; x++) {
            int src_x = x * src_w / dst_w;
            int si = (src_y * src_w + src_x) * 3;
            int di = (y * dst_w + x) * 3;
            dst[di]     = src[si];
            dst[di + 1] = src[si + 1];
            dst[di + 2] = src[si + 2];
        }
    }
}

static void crop_center_and_resize_rgb888(const uint8_t *src, int src_w, int src_h,
                                          uint8_t *dst, int dst_w, int dst_h)
{
    int crop_size = (src_w < src_h) ? src_w : src_h;
    int x0 = (src_w - crop_size) / 2;
    int y0 = (src_h - crop_size) / 2;

    for (int y = 0; y < dst_h; y++) {
        int src_y = y0 + (y * crop_size) / dst_h;
        for (int x = 0; x < dst_w; x++) {
            int src_x = x0 + (x * crop_size) / dst_w;
            int si = (src_y * src_w + src_x) * 3;
            int di = (y * dst_w + x) * 3;
            dst[di]     = src[si];
            dst[di + 1] = src[si + 1];
            dst[di + 2] = src[si + 2];
        }
    }
}

/* =========================
   EI Callback
   ========================= */

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // resized_buf es RGB888: [R][G][B][R][G][B]...
    size_t byte_ix = offset * 3;

    for (size_t i = 0; i < length; i++) {
        uint8_t r = resized_buf[byte_ix];
        uint8_t g = resized_buf[byte_ix + 1];
        uint8_t b = resized_buf[byte_ix + 2];

        out_ptr[i] = (float)((r << 16) | (g << 8) | b);
        byte_ix += 3;
    }

    return 0;
}

/* =========================
   FUNCIONES
   ========================= */

void abrirCompuerta() {

    compuerta.write(90);
    delay(3000);
    compuerta.write(0);
}

void aceptarResiduo() {

    digitalWrite(LED_ROJO, LOW);   // apagar rojo
    digitalWrite(LED_VERDE, HIGH); // prender verde

#if USE_TONE
    tone(BUZZER, 2000, 200);
#endif

    abrirCompuerta();

    delay(500);

    digitalWrite(LED_VERDE, LOW);
}

void rechazarResiduo() {

    digitalWrite(LED_ROJO, HIGH);

#if USE_TONE
    tone(BUZZER, 500, 800);
#endif

    delay(1000);

    digitalWrite(LED_ROJO, LOW);
}

/* =========================
   SETUP
   ========================= */

void setup() {

    Serial.begin(115200);
    xSerialMutex = xSemaphoreCreateMutex();

    delay(2000);

    pinMode(LED_VERDE, OUTPUT);
    pinMode(LED_ROJO, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    compuerta.attach(SERVO_PIN);
    compuerta.write(0);

    // scale.begin(HX_DT, HX_SCK);  // Sensor de peso deshabilitado
    // scale.set_scale(2280.f);
    // scale.tare();

    prefs.begin("contadores", false);
    count_organico = prefs.getInt("organico", 0);
    count_aprovechable = prefs.getInt("aprovechable", 0);
    count_no_aprovechable = prefs.getInt("no_aprov", 0);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nIP:");
    Serial.println(WiFi.localIP());

    Serial.printf("Clases del modelo (%d):\n", EI_CLASSIFIER_LABEL_COUNT);
    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("  [%d] '%s'\n", i, ei_classifier_inferencing_categories[i]);
    }
    Serial.printf("Umbral: %.3f\n", (float)EI_CLASSIFIER_THRESHOLD);
    Serial.printf("Input: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);

    camera_config_t config = {};

    // Evitar colisión con ESP32Servo (y tone si se habilita)
    config.ledc_channel = LEDC_CHANNEL_1;
    config.ledc_timer   = LEDC_TIMER_1;

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

    config.xclk_freq_hz = 20000000;
    config.frame_size   = FRAMESIZE_QQVGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_camera_init(&config);

    g_sensor = esp_camera_sensor_get();
    if (g_sensor) {
        g_sensor->set_vflip(g_sensor, 1);
    }

    // Buffer para QQVGA decodificado a RGB888 (160*120*3)
    rgb888_buf = (uint8_t *)ps_malloc(160 * 120 * 3);
    // Buffer para imagen redimensionada 64x64 RGB888
    resized_buf = (uint8_t *)malloc(IMG_WIDTH * IMG_HEIGHT * 3);

    xTaskCreatePinnedToCore(tareaInferencia, "inferencia", 16384, NULL, 1, NULL, 0);
}

/* =========================
   LOOP
   ========================= */

void loop() {

    // Ejecutar acciones de servo/LED desde Core 1 (donde se inicializaron)
    if (accion_pendiente == 1) {
        accion_pendiente = 0;
        aceptarResiduo();
    }
    else if (accion_pendiente == 3) {
        accion_pendiente = 0;
        // Nada detectado: LED rojo encendido
        digitalWrite(LED_VERDE, LOW);
        digitalWrite(LED_ROJO, HIGH);
    }

    // Reset contadores por Serial: escribe "reset" en el monitor serial
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("reset")) {
            count_organico = 0;
            count_aprovechable = 0;
            count_no_aprovechable = 0;
            prefs.putInt("organico", 0);
            prefs.putInt("aprovechable", 0);
            prefs.putInt("no_aprov", 0);
            sendToUbidots("{\"organico\":0,\"aprovechable\":0,\"no_aprovechable\":0,\"ultima_clase\":0}");
            SERIAL_PRINT("Contadores reiniciados a 0\n");
        }
        else if (cmd.equalsIgnoreCase("test_accept")) {
            SERIAL_PRINT("TEST: aceptarResiduo()\n");
            aceptarResiduo();
        }
        else if (cmd.equalsIgnoreCase("test_reject")) {
            SERIAL_PRINT("TEST: rechazarResiduo()\n");
            rechazarResiduo();
        }
        else if (cmd.equalsIgnoreCase("debug0")) {
            verbose_scores = false;
            SERIAL_PRINT("verbose_scores=0\n");
        }
        else if (cmd.equalsIgnoreCase("debug1")) {
            verbose_scores = true;
            SERIAL_PRINT("verbose_scores=1\n");
        }
        else if (cmd.equalsIgnoreCase("mirror0")) {
            if (g_sensor) g_sensor->set_hmirror(g_sensor, 0);
            SERIAL_PRINT("hmirror=0\n");
        }
        else if (cmd.equalsIgnoreCase("mirror1")) {
            if (g_sensor) g_sensor->set_hmirror(g_sensor, 1);
            SERIAL_PRINT("hmirror=1\n");
        }
        else if (cmd.equalsIgnoreCase("flip0")) {
            if (g_sensor) g_sensor->set_vflip(g_sensor, 0);
            SERIAL_PRINT("vflip=0\n");
        }
        else if (cmd.equalsIgnoreCase("flip1")) {
            if (g_sensor) g_sensor->set_vflip(g_sensor, 1);
            SERIAL_PRINT("vflip=1\n");
        }
        else if (cmd.equalsIgnoreCase("crop0")) {
            use_center_crop = false;
            SERIAL_PRINT("use_center_crop=0 (squash)\n");
        }
        else if (cmd.equalsIgnoreCase("crop1")) {
            use_center_crop = true;
            SERIAL_PRINT("use_center_crop=1 (center crop)\n");
        }
    }

    if (frame_ready) { delay(10); return; }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(10); return; }

    // Decodificar JPEG a RGB888
    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb888_buf);
    int fw = fb->width;
    int fh = fb->height;
    esp_camera_fb_return(fb);

    if (!ok) { delay(10); return; }

    if (use_center_crop) {
        crop_center_and_resize_rgb888(rgb888_buf, fw, fh,
                                     resized_buf, IMG_WIDTH, IMG_HEIGHT);
    } else {
        resize_rgb888(rgb888_buf, fw, fh,
                      resized_buf, IMG_WIDTH, IMG_HEIGHT);
    }

    frame_ready = true;
}