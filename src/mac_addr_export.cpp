// export esp32 mac address to serial
#include <WiFi.h>
void setup()
{
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("MAC (STA): %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
void loop()
{
}