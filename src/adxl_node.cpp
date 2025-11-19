#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <rms_model_int8.h>
#include <math.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#define LED_PIN 2
#define NODE_ID "ADXL_EDGE"
#define WIFI_SSID "Tầng5"
#define WIFI_PASSWORD "ANH12345678"
uint8_t sinkMac[] = {0xC0, 0xCD, 0xD6, 0x8D, 0x43, 0x44};

typedef struct
{
  char node_id[10];
  float computed_rms;
  float predicted_rms;
  float error;
} struct_message;

struct_message myData;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

constexpr int DYN_SAMPLE_HZ = 10;
constexpr int DYN_WINDOW_SIZE = 100;
constexpr int RMS_WINDOW_SIZE = DYN_WINDOW_SIZE / DYN_SAMPLE_HZ;
constexpr int BATCH_PRINT = 10;

double sumDynSq = 0.0;
float gx = 0, gy = 0, gz = 0;
bool gravityInit = false;
float dynSqBuf[DYN_WINDOW_SIZE];
float rmsBuf[RMS_WINDOW_SIZE];
int dynIdx = 0, dynCnt = 0, rmsIdx = 0, rmsCnt = 0, batchCount = 0;

static float ALPHA_GRAV = 0.96f;
constexpr float MEAN_X[RMS_WINDOW_SIZE] = {0.048191f, 0.048178f, 0.048164f, 0.048153f, 0.048146f, 0.048139f, 0.048133f, 0.048127f, 0.048115f, 0.048102f};
constexpr float SCALE_X[RMS_WINDOW_SIZE] = {0.003210f, 0.003203f, 0.003193f, 0.003188f, 0.003185f, 0.003181f, 0.003178f, 0.003177f, 0.003174f, 0.003175f};
constexpr float MEAN_Y = 0.048090f;
constexpr float SCALE_Y = 0.003176f;

uint8_t tensor_arena[8 * 1024];
tflite::MicroInterpreter *interpreter;
TfLiteTensor *input;
TfLiteTensor *output;
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("onDataSent -> status: ");
  Serial.println(status);
}

void addPeer(int channel)
{
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, sinkMac, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;

  while (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Add peer failed, retry...");
    delay(1000);
  }
  Serial.println("Peer added");
}

void feedModel(float currentRms)
{
  for (int i = 0; i < RMS_WINDOW_SIZE; i++)
  {
    int idx = (rmsIdx + i) % RMS_WINDOW_SIZE;
    float norm = (rmsBuf[idx] - MEAN_X[i]) / SCALE_X[i];
    float scaled = norm / input->params.scale + input->params.zero_point;
    int q = round(scaled);
    q = constrain(q, -128, 127);
    input->data.int8[i] = (int8_t)q;
  }

  if (interpreter->Invoke() != kTfLiteOk)
  {
    Serial.println("Invoke error");
    return;
  }

  int8_t out_q = output->data.int8[0];
  float out_deq = (out_q - output->params.zero_point) * output->params.scale;
  float out_real = out_deq * SCALE_Y + MEAN_Y;
  float error = fabs(currentRms - out_real);

  myData.computed_rms = currentRms;
  myData.predicted_rms = out_real;
  myData.error = error;

  Serial.printf("RMS=%.6f | Pred=%.6f | Err=%.6f\n", currentRms, out_real, error);

  esp_err_t result = esp_now_send(sinkMac, (uint8_t *)&myData, sizeof(myData));
  Serial.print("esp_now_send result: ");
  Serial.println(result);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  accel.begin();
  accel.setRange(ADXL345_RANGE_2_G);

  static tflite::MicroErrorReporter micro_error_reporter;
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      tflite::GetModel(rms_model_int8_tflite), resolver,
      tensor_arena, sizeof(tensor_arena), &micro_error_reporter);

  interpreter = &static_interpreter;
  interpreter->AllocateTensors();
  input = interpreter->input(0);
  output = interpreter->output(0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    while (1)
      delay(1000);
  }
  int channel = WiFi.channel();
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_now_register_send_cb(onDataSent);
  addPeer(channel);

  strncpy(myData.node_id, NODE_ID, sizeof(myData.node_id) - 1);

  Serial.println("=== ADXL Sensor Node starting ===");
  Serial.println("Sensor node ready");
}

void loop()
{
  unsigned long start = millis();

  sensors_event_t e;
  accel.getEvent(&e);
  float ax = e.acceleration.x;
  float ay = e.acceleration.y;
  float az = e.acceleration.z;

  if (!gravityInit)
  {
    gx = ax;
    gy = ay;
    gz = az;
    gravityInit = true;
  }
  else
  {
    gx = ALPHA_GRAV * gx + (1 - ALPHA_GRAV) * ax;
    gy = ALPHA_GRAV * gy + (1 - ALPHA_GRAV) * ay;
    gz = ALPHA_GRAV * gz + (1 - ALPHA_GRAV) * az;
  }

  float dx = ax - gx, dy = ay - gy, dz = az - gz;
  float dynSq = dx * dx + dy * dy + dz * dz;

  if (dynCnt < DYN_WINDOW_SIZE)
  {
    dynSqBuf[dynIdx] = dynSq;
    sumDynSq += dynSq;
    dynCnt++;
  }
  else
  {
    sumDynSq -= dynSqBuf[dynIdx];
    dynSqBuf[dynIdx] = dynSq;
    sumDynSq += dynSq;
  }

  dynIdx = (dynIdx + 1) % DYN_WINDOW_SIZE;

  batchCount++;
  if (batchCount >= BATCH_PRINT)
  {
    batchCount = 0;
    float rms = sqrt(sumDynSq / dynCnt);
    rmsBuf[rmsIdx] = rms;

    if (rmsCnt < RMS_WINDOW_SIZE)
      rmsCnt++;
    rmsIdx = (rmsIdx + 1) % RMS_WINDOW_SIZE;

    if (rmsCnt >= RMS_WINDOW_SIZE)
      feedModel(rms);
  }

  long wait = (1000 / DYN_SAMPLE_HZ) - (millis() - start);
  if (wait > 0)
    delay(wait);
}