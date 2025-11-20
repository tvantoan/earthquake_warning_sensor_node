#include "SPIFFS.h"

void setup()
{
  Serial.begin(115200);
  delay(500);

  if (!SPIFFS.begin(true))
  {
    Serial.println("Mount SPIFFS fail!");
    return;
  }

  Serial.println("=== SPIFFS Files List ===");

  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  while (file)
  {
    String name = file.name();

    if (name.length() == 0)
    {
      file = root.openNextFile();
      continue;
    }

    Serial.printf("File: %s | Size: %d\n", name.c_str(), file.size());
    file = root.openNextFile();
  }

  Serial.println("=== Xóa tất cả file ===");

  root = SPIFFS.open("/");
  file = root.openNextFile();

  while (file)
  {
    String name = file.name();

    if (name.length() == 0)
    {
      file = root.openNextFile();
      continue;
    }

    if (!name.startsWith("/"))
      name = "/" + name;

    Serial.println("Xóa: " + name);
    SPIFFS.remove(name);

    file = root.openNextFile();
  }

  Serial.println("=== Kiểm tra lại file còn tồn tại ===");

  root = SPIFFS.open("/");
  file = root.openNextFile();

  bool hasFile = false;

  while (file)
  {
    String name = file.name();

    if (name.length() != 0)
    {
      hasFile = true;
      Serial.printf("Còn sót file: %s | Size: %d\n", name.c_str(), file.size());
    }

    file = root.openNextFile();
  }

  if (!hasFile)
  {
    Serial.println("Không còn file nào trên SPIFFS.");
  }

  Serial.println("=== SPIFFS sạch rồi ===");
}

void loop() {}
