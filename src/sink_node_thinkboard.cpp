#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===== WiFi & MQTT config =====
const char *ssid = "Tầng5";
const char *password = "ANH12345678";

const char *mqtt_server = "mqtt.eu.thingsboard.cloud";
const int mqtt_port = 1883;
const char *access_token = "RJJMZau9ejd5FwzC9qhO";
const char *mqtt_topic = "v1/devices/me/telemetry";

// ===== Edge nodes =====
#define NUM_EDGE_NODES 2
uint8_t edgeMacs[NUM_EDGE_NODES][6] = {
    {0x48, 0xE7, 0x29, 0xB4, 0x80, 0xCC},
    {0x6C, 0xC8, 0x40, 0x8B, 0x43, 0x08}};

const char *EXPECTED_NODES[NUM_EDGE_NODES] = {
    "ADXL_EDGE",
    "MPU_EDGE"};

// ===== Data struct =====
typedef struct
{
    char node_id[10];
    float computed_rms;
    float predicted_rms;
    float error;
    char vibration_level[20];
} struct_message;

struct_message dataNode[NUM_EDGE_NODES];
bool nodeReceived[NUM_EDGE_NODES] = {false};
struct_message myData;

WiFiClient espClient;
PubSubClient client(espClient);

// ===== Helpers =====
void printMac(const uint8_t *mac)
{
    char s[18];
    sprintf(s, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print(s);
}

int findNodeIndex(const char *id)
{
    for (int i = 0; i < NUM_EDGE_NODES; i++)
        if (strcmp(id, EXPECTED_NODES[i]) == 0)
            return i;
    return -1;
}

int compareVibrationLevel(const String &level1, const String &level2)
{
    const char *order[] = {"NONE_SHAKING", "MICRO_SHAKING", "MINOR_SHAKING", "LIGHT_SHAKING", "MODERATE_SHAKING", "SEVERE_SHAKING"};
    int idx1 = 0, idx2 = 0;
    for (int i = 0; i < 6; i++)
    {
        if (level1 == order[i])
            idx1 = i;
        if (level2 == order[i])
            idx2 = i;
    }
    return (idx1 < idx2) ? -1 : (idx1 > idx2) ? 1
                                              : 0;
}

// Chọn nhãn cao nhất từ tất cả node
String getHighestVibrationLevel()
{
    String highest = dataNode[0].vibration_level;
    for (int i = 1; i < NUM_EDGE_NODES; i++)
    {
        if (compareVibrationLevel(dataNode[i].vibration_level, highest) > 0)
            highest = dataNode[i].vibration_level;
    }
    return highest;
}
// ===== Publish full packet =====
void publishAllNodes(String highestLevel)
{
    if (!client.connected())
        return;

    StaticJsonDocument<1024> doc;

    JsonObject nodes = doc.createNestedObject("nodes");

    float meanError = 0;
    int cnt = 0;

    for (int i = 0; i < NUM_EDGE_NODES; i++)
    {
        Serial.printf("Node %s: err=%.6f, level=%s\n", dataNode[i].node_id, dataNode[i].error, dataNode[i].vibration_level);
        doc[dataNode[i].node_id] = dataNode[i].error;
    }

    doc["vibration_level"] = highestLevel;
    Serial.print("Overall Vibration Level: ");
    Serial.println(highestLevel);
    doc["sink_timestamp"] = millis();

    String payload;
    serializeJson(doc, payload);
    bool ok = client.publish(mqtt_topic, payload.c_str());

    if (ok)
    {
        Serial.println("MQTT publish successful!");
    }
    else
    {
        Serial.println("MQTT publish FAILED!");
    }
}

// ===== ESP-NOW callback =====
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    memcpy(&myData, data, sizeof(myData));

    int idx = findNodeIndex(myData.node_id);
    if (idx < 0)
    {
        Serial.println("Unknown node -> skipped");
        return;
    }

    dataNode[idx] = myData;
    nodeReceived[idx] = true;

    Serial.printf("Node %s received!\n", myData.node_id);

    bool allOK = true;
    for (int i = 0; i < NUM_EDGE_NODES; i++)
        if (!nodeReceived[i])
            allOK = false;

    if (allOK)
    {

        String highestLevel = getHighestVibrationLevel();
        Serial.print("Highest Vibration Level: ");
        Serial.println(highestLevel);
        Serial.println("=== All nodes ready → publishing ===");
        publishAllNodes(highestLevel);
        memset(nodeReceived, 0, sizeof(nodeReceived));
    }
}

// ===== Add ESP-NOW peers =====
void addPeers(int channel)
{
    for (int i = 0; i < NUM_EDGE_NODES; i++)
    {
        esp_now_peer_info_t peer{};
        memcpy(peer.peer_addr, edgeMacs[i], 6);
        peer.channel = channel;
        peer.encrypt = false;

        esp_err_t st = esp_now_add_peer(&peer);

        Serial.print("Peer: ");
        printMac(edgeMacs[i]);
        Serial.printf(" CH=%d ", channel);

        if (st == ESP_OK)
            Serial.println("→ added");
        else if (st == ESP_ERR_ESPNOW_EXIST)
            Serial.println("→ exists");
        else
            Serial.printf("→ FAILED %d\n", st);
    }
}

// ===== WiFi setup =====
void setup_wifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.println("Connecting WiFi...");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nWiFi FAILED!");
    }

    int channel = WiFi.channel();
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW INIT FAIL");
        while (1)
            delay(1000);
    }

    esp_now_register_recv_cb(onDataRecv);
    addPeers(channel);
}

// ===== MQTT reconnect =====
void reconnect_mqtt()
{
    while (!client.connected())
    {
        String clientId = "ESP32Sink-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), access_token, NULL))
        {
            Serial.println("MQTT connected");
        }
        else
        {
            Serial.print("MQTT fail rc=");
            Serial.println(client.state());
            delay(3000);
        }
    }
}

// ===== Setup =====
void setup()
{
    Serial.begin(115200);
    delay(300);

    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    reconnect_mqtt();
}

// ===== Loop =====
void loop()
{
    client.loop();
}

// ===== ESP-NOW callback =====
// void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
// {
//     struct_message msg;
//     memcpy(&msg, data, sizeof(msg));

//     // Đẩy vào queue nếu còn chỗ
//     int nextTail = (queueTail + 1) % QUEUE_SIZE;
//     if (nextTail != queueHead)
//     {
//         publishQueue[queueTail] = msg;
//         queueTail = nextTail;
//         Serial.printf("[RECV] Node: %s | err=%.4f | rms=%.4f\n",
//                       msg.node_id, msg.error, msg.computed_rms);
//     }
//     else
//     {
//         Serial.println("[WARN] Publish queue full, dropping packet");
//     }
// }