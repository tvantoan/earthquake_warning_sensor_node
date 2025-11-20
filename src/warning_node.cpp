#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ArduinoJson.h>

// ===== WIFI & MQTT =====
const char *ssid = "Tầng5";
const char *password = "ANH12345678";
const char *mqttServer = "mqtt.eu.thingsboard.cloud";
const int mqttPort = 1883;
const char *accessToken = "RJJMZau9ejd5FwzC9qhO";

WiFiClient espClient;
PubSubClient client(espClient);

// ===== PIN SETUP =====
#define BUZZER 25
#define LED 2

// ===== ST7735S SPI PINS =====
#define TFT_CS 5
#define TFT_RST 4
#define TFT_DC 16
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ===== STATE =====
int lastLevel = -1;

// ===== LEVEL LABELS =====
const char *labels[] = {
    "NONE_SHAKING",
    "MICRO_SHAKING",
    "MINOR_SHAKING",
    "LIGHT_SHAKING",
    "MODERATE_SHAKING",
    "SEVERE_SHAKING"};

// ===== LEVEL COLORS =====
uint16_t colors[] = {
    ST77XX_GREEN,  // 0
    ST77XX_CYAN,   // 1
    ST77XX_YELLOW, // 2
    0xFE40,        // 3 - vàng đậm
    0xFD20,        // 4 - cam
    ST77XX_RED     // 5 - đỏ
};

// ===== BUZZER RULE =====
bool shouldBuzz(int level)
{
  return (level >= 3); // từ LIGHT trở lên sẽ bật
}

// ===== MQTT CALLBACK =====
void callback(char *topic, byte *payload, unsigned int length)
{
  String msg = "";
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];

  Serial.println("RPC Received: " + msg);

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err)
  {
    Serial.println("JSON Parse Error");
    return;
  }

  if (!doc.containsKey("params"))
  {
    Serial.println("No params key found!");
    return;
  }

  int level = doc["params"].as<int>(); // 0..5

  if (level < 0 || level > 5)
  {
    Serial.println("Invalid level");
    return;
  }

  if (level == lastLevel)
    return;

  lastLevel = level;

  // ===== UPDATE TFT =====
  tft.fillScreen(colors[level]);
  tft.setCursor(10, 40);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print(labels[level]);

  // ===== BUZZER AND LED =====
  if (shouldBuzz(level))
  {
    digitalWrite(BUZZER, HIGH);
    digitalWrite(LED, HIGH);
  }
  else
  {
    digitalWrite(BUZZER, LOW);
    digitalWrite(LED, LOW);
  }
}

// ===== MQTT RECONNECT =====
void reconnect()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("Warning-Node", accessToken, NULL))
    {
      client.subscribe("v1/devices/me/rpc/request/+");
      Serial.println("RPC Subscribed");
    }
    else
    {
      delay(2000);
    }
  }
}

// ===== SETUP =====
void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  // TFT init
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(10, 40);
  tft.print("Waiting...");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

// ===== LOOP =====
void loop()
{
  if (!client.connected())
    reconnect();
  client.loop();
}

// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ST7735.h>

// // ===== WIFI & MQTT =====
// const char *ssid = "Tầng5";
// const char *password = "ANH12345678";
// const char *mqttServer = "mqtt.eu.thingsboard.cloud";
// const int mqttPort = 1883;
// const char *accessToken = "RJJMZau9ejd5FwzC9qhO";

// WiFiClient espClient;
// PubSubClient client(espClient);

// // ===== PIN SETUP =====
// #define BUZZER 25 // Active buzzer → dùng digitalWrite
// #define LED 2     // LED tích hợp trên ESP32

// // ===== ST7735S SPI PINS =====
// #define TFT_CS 5
// #define TFT_RST 4
// #define TFT_DC 16
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// String lastMsg = ""; // tránh nhấp nháy liên tục
// bool alarmEnabled = true;

// void drawStatus(const char *text, uint16_t color)
// {
//   tft.fillScreen(ST77XX_BLACK);
//   tft.setCursor(0, 30);
//   tft.setTextSize(2);
//   tft.setTextColor(color);
//   tft.print(text);
// }

// void handleAlarmToggle(bool enable, const String &msgLabel)
// {
//   alarmEnabled = enable;
//   if (!enable)
//   {
//     digitalWrite(LED, LOW);
//     digitalWrite(BUZZER, LOW);
//     drawStatus("Alarm OFF", ST77XX_GREEN);
//   }
//   lastMsg = msgLabel;
// }

// String extractRequestId(const char *topic)
// {
//   String topicStr(topic);
//   int lastSlash = topicStr.lastIndexOf('/');
//   if (lastSlash < 0)
//     return "";
//   return topicStr.substring(lastSlash + 1);
// }

// String extractJsonString(const String &json, const String &key)
// {
//   String pattern = "\"" + key + "\"";
//   int keyPos = json.indexOf(pattern);
//   if (keyPos < 0)
//     return "";
//   int colonPos = json.indexOf(':', keyPos);
//   if (colonPos < 0)
//     return "";
//   int firstQuote = json.indexOf('"', colonPos + 1);
//   if (firstQuote < 0)
//     return "";
//   int secondQuote = json.indexOf('"', firstQuote + 1);
//   if (secondQuote < 0)
//     return "";
//   return json.substring(firstQuote + 1, secondQuote);
// }

// bool extractJsonBool(const String &json, const String &key, bool &value)
// {
//   String pattern = "\"" + key + "\"";
//   int keyPos = json.indexOf(pattern);
//   if (keyPos < 0)
//     return false;
//   int colonPos = json.indexOf(':', keyPos);
//   if (colonPos < 0)
//     return false;
//   int pos = colonPos + 1;
//   while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' || json[pos] == '"'))
//     pos++;
//   if (json.startsWith("true", pos))
//   {
//     value = true;
//     return true;
//   }
//   if (json.startsWith("false", pos))
//   {
//     value = false;
//     return true;
//   }
//   return false;
// }

// void publishStateResponse(const String &requestId)
// {
//   if (requestId.length() == 0)
//     return;
//   String respTopic = "v1/devices/me/rpc/response/" + requestId;
//   String payload = String("{\"state\":") + (alarmEnabled ? "true" : "false") + "}";
//   client.publish(respTopic.c_str(), payload.c_str());
// }

// // ===== CALLBACK MQTT =====
// void callback(char *topic, byte *payload, unsigned int length)
// {
//   String msg = "";
//   for (int i = 0; i < length; i++)
//     msg += (char)payload[i];
//   Serial.println("RPC Received: " + msg);

//   if (msg.indexOf("\"method\"") >= 0)
//   {
//     String method = extractJsonString(msg, "method");
//     method.toUpperCase();
//     String requestId = extractRequestId(topic);

//     if (method == "GETSTATE")
//     {
//       publishStateResponse(requestId);
//       return;
//     }
//     if (method == "SETSTATE")
//     {
//       bool desiredState;
//       if (extractJsonBool(msg, "params", desiredState))
//       {
//         handleAlarmToggle(desiredState, desiredState ? "ON" : "OFF");
//         publishStateResponse(requestId);
//       }
//       else
//       {
//         Serial.println("Khong doc duoc truong params trong setState");
//       }
//       return;
//     }
//   }

//   String msgUpper = msg;
//   msgUpper.toUpperCase();

//   if (msgUpper == "ON" || msgUpper == "ENABLE")
//   {
//     handleAlarmToggle(true, msg);
//     // khi bật lại, giữ nguyên nội dung màn hình và đợi mức cảnh báo mới
//     return;
//   }
//   if (msgUpper == "OFF" || msgUpper == "DISABLE")
//   {
//     handleAlarmToggle(false, msg);
//     return;
//   }

//   if (!alarmEnabled)
//   {
//     Serial.println("Alarm đang tắt, bỏ qua cảnh báo");
//     return;
//   }

//   if (msg == lastMsg)
//     return; // tránh clear lặp
//   lastMsg = msg;

//   tft.fillScreen(ST77XX_BLACK); // xóa màn

//   if (msg.indexOf("SEVERE") >= 0)
//   {
//     digitalWrite(LED, HIGH);
//     digitalWrite(BUZZER, HIGH);
//     tft.setCursor(0, 30);
//     tft.setTextColor(ST77XX_RED);
//     tft.setTextSize(2);
//     tft.print("SEVERE ALERT!");
//   }
//   else if (msg.indexOf("MODERATE") >= 0)
//   {
//     digitalWrite(LED, HIGH);
//     digitalWrite(BUZZER, HIGH);
//     tft.setCursor(0, 30);
//     tft.setTextColor(ST77XX_YELLOW);
//     tft.setTextSize(2);
//     tft.print("Moderate!");
//   }
//   else
//   {
//     digitalWrite(LED, LOW);
//     digitalWrite(BUZZER, LOW);
//     tft.setCursor(0, 30);
//     tft.setTextColor(ST77XX_GREEN);
//     tft.setTextSize(2);
//     tft.print("Safe");
//   }
// }

// // ===== RECONNECT MQTT =====
// void reconnect()
// {
//   while (!client.connected())
//   {
//     Serial.println("Connecting to MQTT...");
//     if (client.connect("Warning-Node", accessToken, NULL))
//     {
//       client.subscribe("v1/devices/me/rpc/request/+");
//       Serial.println("RPC Subscribed");
//     }
//     else
//     {
//       delay(2000);
//     }
//   }
// }

// void setup()
// {
//   Serial.begin(115200);
//   WiFi.begin(ssid, password);

//   pinMode(BUZZER, OUTPUT);
//   pinMode(LED, OUTPUT);

//   // ===== TFT INIT =====
//   tft.initR(INITR_BLACKTAB); // hoặc INITR_GREENTAB tuỳ module
//   tft.setRotation(1);
//   tft.fillScreen(ST77XX_BLACK);
//   tft.setTextSize(2);
//   tft.setTextColor(ST77XX_WHITE);
//   tft.setCursor(0, 30);
//   tft.print("Warning Node");

//   client.setServer(mqttServer, mqttPort);
//   client.setCallback(callback);
// }

// void loop()
// {
//   if (!client.connected())
//     reconnect();
//   client.loop();
// }
