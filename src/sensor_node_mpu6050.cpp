#include "FS.h"
#include "SPIFFS.h"

#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>
#include <rms_model_int8.h>
#include <math.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#define LED_PIN 2

MPU6050 mpu;

// ==== Giữ nguyên AI config ====
constexpr int DYN_SAMPLE_HZ = 10;
constexpr int DYN_WINDOW_SIZE = 100;
constexpr int RMS_WINDOW_SIZE = DYN_WINDOW_SIZE / DYN_SAMPLE_HZ;
constexpr int BATCH_PRINT = 10;
double sumDynSq = 0.0;

static float ALPHA_GRAV = 0.96f;
constexpr float ERROR_THRESHOLD = 0.0011856213922079436f;

static int collectedRMS = 0;

constexpr float MEAN_X[RMS_WINDOW_SIZE] = {
    0.048191f, 0.048178f, 0.048164f, 0.048153f, 0.048146f,
    0.048139f, 0.048133f, 0.048127f, 0.048115f, 0.048102f};

constexpr float SCALE_X[RMS_WINDOW_SIZE] = {
    0.003210f, 0.003203f, 0.003193f, 0.003188f, 0.003185f,
    0.003181f, 0.003178f, 0.003177f, 0.003174f, 0.003175f};

constexpr float MEAN_Y = 0.048090f;
constexpr float SCALE_Y = 0.003176f;

float gx = 0, gy = 0, gz = 0;
bool gravityInit = false;

constexpr int kArenaSize = 8 * 1024;
uint8_t tensor_arena[kArenaSize];

const tflite::Model *model = tflite::GetModel(rms_model_int8_tflite);
tflite::MicroInterpreter *interpreter;
TfLiteTensor *input;
TfLiteTensor *output;

float dynSq_buf_window[DYN_WINDOW_SIZE];
int dynBufIndex = 0, dynBufCount = 0;

float rms_buf_window[RMS_WINDOW_SIZE];
int rmsBufIndex = 0, rmsBufCount = 0;

int batchCount = 0;
bool ledState = false;

const float ACCEL_SCALE = 16384.0f; // ±2g

// =============================================================
//                    FEED MODEL
// =============================================================
void feedModelAndPrint(float currentRms)
{
  for (int i = 0; i < RMS_WINDOW_SIZE; i++)
  {
    int idx = (rmsBufIndex + i) % RMS_WINDOW_SIZE;
    float norm = (rms_buf_window[idx] - MEAN_X[i]) / SCALE_X[i];

    float scaled = norm / input->params.scale + input->params.zero_point;
    int q = round(scaled);
    if (q < -128)
      q = -128;
    if (q > 127)
      q = 127;
    input->data.int8[i] = (int8_t)q;
  }

  if (interpreter->Invoke() != kTfLiteOk)
  {
    Serial.println("Invoke error");
    return;
  }

  int8_t outQ = output->data.int8[0];
  float outDeQ = (outQ - output->params.zero_point) * output->params.scale;
  float outReal = outDeQ * SCALE_Y + MEAN_Y;

  float error = fabs(currentRms - outReal);

  Serial.print("RMS: ");
  Serial.print(currentRms, 6);
  Serial.print(" | Pred: ");
  Serial.print(outReal, 6);
  Serial.print(" | Err: ");
  Serial.println(error, 6);

  if (!ledState && error > ERROR_THRESHOLD)
  {
    ledState = true;
    digitalWrite(LED_PIN, HIGH);
  }
  else if (ledState && error < ERROR_THRESHOLD)
  {
    ledState = false;
    digitalWrite(LED_PIN, LOW);
  }
}

// =============================================================
//                           SETUP
// =============================================================
void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();

  Serial.println("Init MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 fail!");
    while (1)
      ;
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // giống ADXL345 ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // KHÔNG dùng nhưng cần set

  // ==== AI INIT ====
  static tflite::MicroErrorReporter micro_error_reporter;
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kArenaSize, &micro_error_reporter);

  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk)
  {
    Serial.println("Alloc tensor fail");
    while (1)
      ;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("Model OK");

  if (!SPIFFS.begin(true))
    Serial.println("SPIFFS error");
}

// =============================================================
//                            LOOP
// =============================================================
void loop()
{
  unsigned long start = millis();

  int16_t ax_raw, ay_raw, az_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);

  // Quy đổi giống ADXL: đơn vị g
  float ax = ax_raw / ACCEL_SCALE;
  float ay = ay_raw / ACCEL_SCALE;
  float az = az_raw / ACCEL_SCALE;

  // ====== Gravity LPF ======
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

  if (dynBufCount < DYN_WINDOW_SIZE)
  {
    dynSq_buf_window[dynBufIndex] = dynSq;
    sumDynSq += dynSq;
    dynBufCount++;
  }
  else
  {
    sumDynSq -= dynSq_buf_window[dynBufIndex];
    dynSq_buf_window[dynBufIndex] = dynSq;
    sumDynSq += dynSq;
  }

  dynBufIndex = (dynBufIndex + 1) % DYN_WINDOW_SIZE;

  batchCount++;
  if (batchCount >= BATCH_PRINT)
  {
    batchCount = 0;

    float rms = sqrt(sumDynSq / dynBufCount);
    rms_buf_window[rmsBufIndex] = rms;

    if (rmsBufCount < RMS_WINDOW_SIZE)
      rmsBufCount++;

    rmsBufIndex = (rmsBufIndex + 1) % RMS_WINDOW_SIZE;

    if (rmsBufCount >= RMS_WINDOW_SIZE)
    {
      feedModelAndPrint(rms);
    }
  }

  long wait = (1000 / DYN_SAMPLE_HZ) - (millis() - start);
  if (wait > 0)
    delay(wait);
}
