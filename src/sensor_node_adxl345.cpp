#include "FS.h"
#include "SPIFFS.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Arduino.h>
#include <rms_model_int8.h>
#include <math.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#define LED_PIN 2

Adafruit_ADXL345_Unified accel(12345);

constexpr int DYN_SAMPLE_HZ = 10;
constexpr int DYN_WINDOW_SIZE = 100;
constexpr int RMS_WINDOW_SIZE = DYN_WINDOW_SIZE / DYN_SAMPLE_HZ;
constexpr int BATCH_PRINT = 10;
double sumDynSq = 0.0;

static float ALPHA_GRAV = 0.96f;
constexpr float ERROR_THRESHOLD = 0.0011856213922079436f;

// static int loopCount = 0;
static int collectedRMS = 0;

// float hpf_x_prev = 0, hpf_y_prev = 0, hpf_z_prev = 0;
// float ax_prev = 0, ay_prev = 0, az_prev = 0;

// const float HPF_ALPHA = 0.95;

constexpr float MEAN_X[RMS_WINDOW_SIZE] = {
    0.048191f, 0.048178f, 0.048164f, 0.048153f, 0.048146f, 0.048139f, 0.048133f, 0.048127f, 0.048115f, 0.048102f};

constexpr float SCALE_X[RMS_WINDOW_SIZE] = {
    0.003210f, 0.003203f, 0.003193f, 0.003188f, 0.003185f, 0.003181f, 0.003178f, 0.003177f, 0.003174f, 0.003175f};

constexpr float MEAN_Y = 0.048090f;
constexpr float SCALE_Y = 0.003176f;

float gx = 0.0f, gy = 0.0f, gz = 0.0f;
bool gravityInit = false;

constexpr int kArenaSize = 8 * 1024;
uint8_t tensor_arena[kArenaSize];

const tflite::Model *model = tflite::GetModel(rms_model_int8_tflite);
tflite::MicroInterpreter *interpreter;
TfLiteTensor *input;
TfLiteTensor *output;

float dynSq_buf_window[DYN_WINDOW_SIZE];
int dynBufIndex = 0;
int dynBufCount = 0;

float rms_buf_window[RMS_WINDOW_SIZE];
int rmsBufIndex = 0;
int rmsBufCount = 0;

int batchCount = 0;
bool ledState = false;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin();

  if (!accel.begin())
  {
    Serial.println("ADXL345 not found!");
    while (1)
      ;
  }
  accel.setRange(ADXL345_RANGE_2_G);

  if (model->version() != TFLITE_SCHEMA_VERSION)
  {
    Serial.println("Model schema not supported");
    while (1)
      ;
  }

  static tflite::MicroErrorReporter micro_error_reporter;
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kArenaSize, &micro_error_reporter);

  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk)
  {
    Serial.println("Allocate tensor error");
    while (1)
      ;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("Model OK");
  Serial.printf("DYN_SAMPLE_HZ=%d, DYN_WINDOW_SIZE=%d, RMS_WINDOW_SIZE=%d\n",
                DYN_SAMPLE_HZ, DYN_WINDOW_SIZE, RMS_WINDOW_SIZE);
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Error");
  }
}

void feedModelAndPrint(float currentRms)
{
  for (int i = 0; i < RMS_WINDOW_SIZE; i++)
  {
    int idx = (rmsBufIndex + i) % RMS_WINDOW_SIZE;
    float norm = (rms_buf_window[idx] - MEAN_X[i]) / SCALE_X[i];

    float scaled = norm / input->params.scale + input->params.zero_point;
    int q = (int)round(scaled);
    if (q < -128)
      q = -128;
    if (q > 127)
      q = 127;
    input->data.int8[i] = static_cast<int8_t>(q);
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
  Serial.print("RMS: ");
  Serial.print(currentRms, 6);
  Serial.print(" | Pred: ");
  Serial.print(out_real, 6);
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

void loop()
{
  unsigned long start = millis();

  sensors_event_t e;
  accel.getEvent(&e);

  float ax = e.acceleration.x;
  float ay = e.acceleration.y;
  float az = e.acceleration.z;
  // Serial.print("Raw Accel: ");
  // Serial.print(ax);
  // Serial.print(", ");
  // Serial.print(ay);
  // Serial.print(", ");
  // Serial.println(az);

  //   float hpf_x = HPF_ALPHA * (hpf_x_prev + (ax - ax_prev));
  //   float hpf_y = HPF_ALPHA * (hpf_y_prev + (ay - ay_prev));
  //   float hpf_z = HPF_ALPHA * (hpf_z_prev + (az - az_prev));
  // hpf_x_prev = hpf_x;
  // hpf_y_prev = hpf_y;
  // hpf_z_prev = hpf_z;

  // ax_prev = ax;
  // ay_prev = ay;
  // az_prev = az;
  //   ax = hpf_x;
  //   ay = hpf_y;
  //   az = hpf_z;

  if (!gravityInit)
  {
    gx = ax;
    gy = ay;
    gz = az;
    gravityInit = true;
  }
  else
  {
    gx = ALPHA_GRAV * gx + (1.0f - ALPHA_GRAV) * ax;
    gy = ALPHA_GRAV * gy + (1.0f - ALPHA_GRAV) * ay;
    gz = ALPHA_GRAV * gz + (1.0f - ALPHA_GRAV) * az;
  }

  float dx = ax - gx;
  float dy = ay - gy;
  float dz = az - gz;
  float dynSq = dx * dx + dy * dy + dz * dz;
  // float dyn = sqrt(dx * dx + dy * dy + dz * dz);
  // static float dyn_lp = 0.0f;
  // dyn_lp = 0.9f * dyn_lp + 0.1f * dyn; // lọc nhiễu 90%

  // float dynSq = dyn_lp * dyn_lp;
  // Serial.print("DynSq: ");
  // Serial.println(dynSq, 6);
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
    float rms = sqrt(sumDynSq / (float)dynBufCount);
    rms_buf_window[rmsBufIndex] = rms;
    if (rmsBufCount < RMS_WINDOW_SIZE)
      rmsBufCount++;
    rmsBufIndex = (rmsBufIndex + 1) % RMS_WINDOW_SIZE;

    if (rmsBufCount >= RMS_WINDOW_SIZE)
    {
      feedModelAndPrint(rms);
      collectedRMS++;
      // Serial.print(collectedRMS);
      // Serial.print(" : ");
      // Serial.println(rms, 6);
      // File f = SPIFFS.open("/log.txt", FILE_APPEND);
      // if (f)
      // {
      //   f.print(rms, 6);
      //   f.print("\n");
      //   f.close();
      // }
      //   if (loopCount == 30)
      //   {
      //     ALPHA_GRAV += 0.01f;
      //     Serial.print("ALPHA_GRAV: ");
      //     Serial.println(ALPHA_GRAV, 6);

      //     // Ghi vào file
      //     File f = SPIFFS.open("/rms_log.txt", FILE_APPEND);
      //     if (f)
      //     {
      //       f.println("ALPHA_GRAV: " + String(ALPHA_GRAV, 6));
      //       f.close();
      //     }

      //     gravityInit = false;

      //     sumDynSq = 0;
      //     dynBufCount = 0;
      //     dynBufIndex = 0;

      //     rmsBufCount = 0;
      //     rmsBufIndex = 0;
      //     loopCount = 0;
      //   }

      //   if (floor(ALPHA_GRAV) > 1)
      //     return;

      //   File f = SPIFFS.open("/rms_log.txt", FILE_APPEND);
      //   if (f)
      //   {
      //     f.print(rms, 6);
      //     f.print("\n");
      //     f.close();
      //   }
      //   loopCount++;
    }
  }
  long wait = (1000 / DYN_SAMPLE_HZ) - (millis() - start);
  if (wait > 0)
    delay(wait);
}
