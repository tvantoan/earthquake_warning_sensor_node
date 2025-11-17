#include "FS.h"
#include "SPIFFS.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  if (!SPIFFS.begin(true))
  { // true để format nếu mount thất bại
    Serial.println("SPIFFS Mount Failed!");
    return;
  }

  if (!SPIFFS.exists("/log.txt"))
  {
    Serial.println("File log.txt không tồn tại!");
    return;
  }

  File file = SPIFFS.open("/log.txt", FILE_READ);
  if (!file)
  {
    Serial.println("Không mở được log.txt!");
    return;
  }

  // Đọc từng dòng và gửi ra Serial
  while (file.available())
  {
    String line = file.readStringUntil('\n'); // đọc đến dấu newline
    line.trim();                              // bỏ khoảng trắng đầu/cuối
    if (line.length() > 0)
    {
      Serial.println(line);
    }
  }

  file.close();

  // Gửi tín hiệu kết thúc để Python biết xong
  Serial.println("END");
}

void loop()
{
  // Không làm gì trong loop
}
