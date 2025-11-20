
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <rms_mpu_model_int8.h>
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#define LED_PIN 2
#define NODE_ID "MPU_EDGE"
#define WIFI_SSID "Tầng5"
#define WIFI_PASSWORD "ANH12345678"
uint8_t sinkMac[] = {0xC0, 0xCD, 0xD6, 0x8D, 0x43, 0x44};
MPU6050 mpu;

typedef struct
{
  char node_id[10];
  float computed_rms;
  float predicted_rms;
  float error;
  char vibration_level[20];
} struct_message;

struct_message myData;

constexpr int DYN_SAMPLE_HZ = 10;
constexpr int DYN_WINDOW_SIZE = 100;
constexpr int RMS_WINDOW_SIZE = DYN_WINDOW_SIZE / DYN_SAMPLE_HZ;
constexpr int BATCH_PRINT = 10;
int cnt = 0;
double sumDynSq = 0.0;
float gx = 0, gy = 0, gz = 0;
bool gravityInit = false;
float dynSqBuf[DYN_WINDOW_SIZE];
float rmsBuf[RMS_WINDOW_SIZE];
int dynIdx = 0, dynCnt = 0, rmsIdx = 0, rmsCnt = 0, batchCount = 0;
static float ALPHA_GRAV = 0.96f;
const float ACCEL_SCALE = 16384.0f;

constexpr float MEAN_X[RMS_WINDOW_SIZE] = {0.006153f, 0.006153f, 0.006152f, 0.006152f, 0.006152f, 0.006151f, 0.006151f, 0.006151f, 0.006151f, 0.006152f};
constexpr float SCALE_X[RMS_WINDOW_SIZE] = {0.000283f, 0.000283f, 0.000283f, 0.000283f, 0.000284f, 0.000284f, 0.000284f, 0.000284f, 0.000284f, 0.000284f};
constexpr float MEAN_Y = 0.006152f;
constexpr float SCALE_Y = 0.000283f;

uint8_t tensor_arena[20 * 1024];
tflite::MicroInterpreter *interpreter;
TfLiteTensor *input;
TfLiteTensor *output;

String classifyVibrationLevel(float error)
{
  if (error < 0.00087304f)
    return "NONE_SHAKING";
  else if (error < 0.020f) // 2
    return "MICRO_SHAKING";
  else if (error < 0.052f) // 3
    return "MINOR_SHAKING";
  else if (error < 0.130f) // 4
    return "LIGHT_SHAKING";
  else if (error < 0.21f) // 5
    return "MODERATE_SHAKING";
  else
    return "SEVERE_SHAKING"; //>6
}

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
  strcpy(myData.vibration_level, classifyVibrationLevel(error).c_str());
  // Serial.println(error, 6);
  Serial.printf("RMS=%.6f | Pred=%.6f | Err=%.6f\n", currentRms, out_real, error);
  Serial.print("Vibration Level: ");
  Serial.println(myData.vibration_level);

  esp_err_t result = esp_now_send(sinkMac, (uint8_t *)&myData, sizeof(myData));
  Serial.print("esp_now_send result: ");
  Serial.println(result);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Error");
  }

  static tflite::MicroErrorReporter micro_error_reporter;
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      tflite::GetModel(rms_mpu_model_int8_tflite), resolver,
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

  Serial.println("=== MPU Sensor Node starting ===");
  Serial.println("MPU node ready");
}

void loop()
{
  unsigned long start = millis();

  int16_t axr, ayr, azr;
  mpu.getAcceleration(&axr, &ayr, &azr);
  float ax = axr / ACCEL_SCALE;
  float ay = ayr / ACCEL_SCALE;
  float az = azr / ACCEL_SCALE;

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

  float dx = ax - gx;
  float dy = ay - gy;
  float dz = az - gz;
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
    // Serial.println(rms, 6);
    // Serial.println(rms, 6);
    // cnt++;

    // ghi vào file:
    // File f = SPIFFS.open("/log.txt", FILE_APPEND);
    // if (f)
    // {
    //   f.print(rms, 6);
    //   f.print("\n");
    //   f.close();
    // }

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