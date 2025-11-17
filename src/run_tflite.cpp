
void setup()
{
  Serial.begin(115200);
  Serial.println("Khởi động model...");

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kArenaSize);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk)
  {
    Serial.println("❌ Lỗi cấp phát tensor");
    while (1)
      ;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("✅ Model loaded OK");
}

void loop()
{
  // ví dụ: truyền 10 giá trị dynSq đã scale về int8
  int8_t dynSq_window[10] = {-5, 2, 3, -1, 4, 0, -2, 1, 3, -4};

  for (int i = 0; i < 10; i++)
  {
    input->data.int8[i] = dynSq_window[i];
  }

  if (interpreter->Invoke() != kTfLiteOk)
  {
    Serial.println("❌ Lỗi chạy model");
    return;
  }

  int8_t y_pred_int8 = output->data.int8[0];
  Serial.print("Dự đoán (int8): ");
  Serial.println(y_pred_int8);

  delay(1000);
}
