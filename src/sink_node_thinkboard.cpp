/*
 * ESP-NOW SINK NODE - Kết nối ThingsBoard qua MQTT
 *
 * Chức năng:
 * 1. Nhận dữ liệu từ Edge Node qua ESP-NOW
 * 2. Gửi trực tiếp lên ThingsBoard qua MQTT
 *
 * Flow: Edge Node → ESP-NOW → Sink Node → ThingsBoard MQTT
 */

#include <WiFi.h>
#include <esp_now.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- Cấu hình Wi-Fi ---
const char *ssid = "Tiendeptry";
const char *password = "123456789";

// --- Cấu hình MQTT ThingsBoard ---
const char *mqtt_server = "mqtt.eu.thingsboard.cloud"; // Tên server ThingsBoard (có thể là IP hoặc domain)
const int mqtt_port = 1883;                            // Port MQTT (thường là 1883, hoặc 8883 cho SSL)
const char *access_token = "RJJMZau9ejd5FwzC9qhO";     // Access Token từ ThingsBoard Device
const char *mqtt_topic = "v1/devices/me/telemetry";    // Topic để publish telemetry lên ThingsBoard

WiFiClient espClient;
PubSubClient client(espClient);

// ĐỊNH NGHĨA STRUCT (Phải giống hệt bên Node Thu Thập)
typedef struct struct_message
{
    char node_id[10]; // ID của Edge Node (ví dụ: "EDGE_01", "EDGE_02")
    float rms_value;
    float out_real;
    float error;
} struct_message;

struct_message myData;

// --- Hàm kết nối Wi-Fi ---
void setup_wifi()
{
    delay(10);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA); // Cần cho ESP-NOW
    WiFi.begin(ssid, password);

    int attempts = 0;
    const int maxAttempts = 20; // Timeout sau 10 giây
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("\nFailed to connect to WiFi!");
        return;
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    Serial.printf("WiFi Channel: %d\n", WiFi.channel());
}

// --- Hàm kết nối lại MQTT ---
void reconnect_mqtt()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32-SinkNode-";
        clientId += String(random(0xffff), HEX);

        // Kết nối với ThingsBoard: dùng Access Token làm username, password để trống
        if (client.connect(clientId.c_str(), access_token, NULL))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// --- Hàm gửi dữ liệu lên ThingsBoard qua MQTT ---
void send_to_thinkboard()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected. Cannot send to ThingsBoard.");
        return;
    }

    // Kiểm tra kết nối MQTT trước khi publish
    if (!client.connected())
    {
        Serial.println("MQTT not connected. Attempting reconnect...");
        reconnect_mqtt();
    }

    // Tạo JSON payload cho ThingsBoard
    JsonDocument doc;

    // Thêm dữ liệu từ sensor
    doc["node_id"] = String(myData.node_id);
    doc["rms"] = myData.rms_value;
    doc["out_real"] = myData.out_real;
    doc["error"] = myData.error;

    // Thêm timestamp
    doc["timestamp"] = millis();

    // Serialize JSON
    String jsonPayload;
    serializeJson(doc, jsonPayload);

    // In payload để debug
    Serial.println("=== Sending to ThingsBoard ===");
    Serial.println("JSON Payload:");
    Serial.println(jsonPayload);

    // Publish (gửi) chuỗi JSON lên ThingsBoard qua MQTT
    if (client.connected())
    {
        bool published = client.publish(mqtt_topic, jsonPayload.c_str());
        if (published)
        {
            Serial.print("Published to ThingsBoard MQTT: ");
            Serial.println(jsonPayload);
            Serial.println("Data sent successfully to ThingsBoard!");
        }
        else
        {
            Serial.println("Failed to publish to ThingsBoard MQTT");
        }
    }
    else
    {
        Serial.println("Cannot publish: MQTT not connected");
    }

    Serial.println("==================================\n");
}

// --- HÀM QUAN TRỌNG: Được gọi khi nhận dữ liệu ESP-NOW ---
// Call back function : tự động gọi mỗi khi có một gói tin mới từ Node Cảm biến gửi đến.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // 1. Kiểm tra độ dài dữ liệu
    if (len != sizeof(struct_message))
    {
        Serial.printf("Error: Data length mismatch. Expected %d, got %d\n", sizeof(struct_message), len);
        return;
    }

    // 2. Sao chép dữ liệu nhận được vào struct
    memcpy(&myData, incomingData, sizeof(myData));

    // 3. In MAC address của Edge Node gửi đến
    char macStr[18];
    sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("Data received from: ");
    Serial.println(macStr);

    // 4. In ra Serial để kiểm tra
    Serial.print("  Node ID: ");
    Serial.print(myData.node_id);
    Serial.print(", RMS: ");
    Serial.print(myData.rms_value, 6); // In 6 chữ số sau dấu phẩy
    Serial.print(", out_real: ");
    Serial.print(myData.out_real);
    Serial.printf("error", myData.error);
    // 5. Gửi dữ liệu lên ThingsBoard qua MQTT
    send_to_thinkboard();
}

// --- HÀM SETUP ---
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("ESP-NOW Sink Node - ThingsBoard MQTT");
    Serial.println("========================================");

    // 1. Kết nối Wi-Fi (cần cho MQTT)
    setup_wifi();

    // 2. Cấu hình MQTT
    client.setServer(mqtt_server, mqtt_port);

    // 3. Khởi tạo ESP-NOW (ở chế độ STA)
    // WiFi.mode(WIFI_STA) đã được gọi trong setup_wifi()
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 4. Đăng ký hàm callback để "nhận" dữ liệu
    esp_now_register_recv_cb(OnDataRecv);

    // 5. Kết nối MQTT lần đầu
    reconnect_mqtt();

    Serial.println("========================================");
    Serial.println("ESP-NOW Sink Node ready to receive data");
    Serial.print("MQTT Server: ");
    Serial.println(mqtt_server);
    Serial.print("MQTT Topic: ");
    Serial.println(mqtt_topic);
    Serial.println("========================================");
}

// --- HÀM LOOP ---
void loop()
{
    // 1. Kiểm tra kết nối MQTT, nếu mất thì kết nối lại
    if (!client.connected())
    {
        reconnect_mqtt();
    }

    // 2. Duy trì kết nối MQTT
    client.loop();

    // Không cần làm gì khác, hàm OnDataRecv() sẽ tự động chạy
    // khi có dữ liệu mới từ Node Thu Thập và gửi lên ThingsBoard.
    delay(10); // Thêm delay nhỏ để hệ thống ổn định
}
