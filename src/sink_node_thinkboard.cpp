#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char *ssid = "Tầng5";
const char *password = "ANH12345678";

const char *mqtt_server = "mqtt.eu.thingsboard.cloud";
const int mqtt_port = 1883;
const char *access_token = "RJJMZau9ejd5FwzC9qhO";
const char *mqtt_topic = "v1/devices/me/telemetry";

#define NUM_EDGE_NODES 2

uint8_t edgeMacs[NUM_EDGE_NODES][6] = {
    {0x48, 0xE7, 0x29, 0xB4, 0x80, 0xCC},
    {0x6C, 0xC8, 0x40, 0x8B, 0x43, 0x08}};

const char *EXPECTED_NODES[NUM_EDGE_NODES] = {
    "ADXL_EDGE",
    "MPU_EDGE"};

#define WIFI_CHANNEL_LOCK 1
#define LOW_THRESHOLD 0.001185f
#define MEDIUM_THRESHOLD 0.125888f
#define HIGH_THRESHOLD 0.250591f
#define RUN_NOW_THRESHOLD 0.5f

WiFiClient espClient;
PubSubClient client(espClient);

typedef struct
{
    char node_id[10];
    float computed_rms;
    float predicted_rms;
    float error;
} struct_message;

struct_message myData;
struct_message dataNode[NUM_EDGE_NODES];
bool nodeReceived[NUM_EDGE_NODES] = {false, false};
void send_to_thingsboard(String vibrationLevel, float meanError);
void reconnect_mqtt();
String classifyVibrationLevel(float error);

void printMac(const uint8_t *mac)
{
    char s[18];
    sprintf(s, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print(s);
}

String classifyVibrationLevel(float error)
{
    if (error < 0.00118562f)
        return "NONE_SHAKING";
    else if (error < 0.25f)
        return "MICRO_SHAKING";
    else if (error < 0.45f)
        return "MINOR_SHAKING";
    else if (error < 0.95f)
        return "LIGHT_SHAKING";
    else if (error < 2.3f)
        return "MODERATE_SHAKING";
    else
        return "SEVERE_SHAKING";
}

bool isValidEdgeNode(const uint8_t *mac)
{
    for (int i = 0; i < NUM_EDGE_NODES; i++)
    {
        if (memcmp(mac, edgeMacs[i], 6) == 0)
        {
            return true;
        }
    }
    return false;
}

bool isValidNodeId(const char *node_id)
{
    for (int i = 0; i < NUM_EDGE_NODES; i++)
    {
        if (strcmp(node_id, EXPECTED_NODES[i]) == 0)
        {
            return true;
        }
    }
    return false;
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    memcpy(&myData, data, sizeof(myData));

    int nodeIndex = -1;
    for (int i = 0; i < NUM_EDGE_NODES; i++)
    {
        if (strcmp(myData.node_id, EXPECTED_NODES[i]) == 0)
        {
            nodeIndex = i;
            break;
        }
    }

    if (nodeIndex == -1)
    {
        Serial.println("⚠ Unknown node ID, skipping");
        return;
    }

    dataNode[nodeIndex] = myData;
    nodeReceived[nodeIndex] = true;

    Serial.printf("Node %s received and stored!\n", myData.node_id);

    bool allOK = true;
    for (int i = 0; i < NUM_EDGE_NODES; i++)
    {
        if (!nodeReceived[i])
            allOK = false;
    }

    if (allOK)
    {
        Serial.println("Received ALL NODES → Processing…");
        float meanError = 0.0;
        for (int i = 0; i < NUM_EDGE_NODES; i++)
        {
            if (strcmp(dataNode[i].node_id, "MPU_EDGE") == 0)
            {
                continue;
            }

            meanError += dataNode[i].error;
        }
        // meanError /= NUM_EDGE_NODES;

        String vibrationLevel = classifyVibrationLevel(meanError);
        send_to_thingsboard(vibrationLevel, meanError);
    }
}

void addPeers()
{
    Serial.println("\n[Peer] Adding 2 peers with assigned MAC addresses…");

    esp_now_peer_info_t peer1{};
    memcpy(peer1.peer_addr, edgeMacs[0], 6);
    peer1.channel = WIFI_CHANNEL_LOCK;
    peer1.encrypt = false;

    Serial.print("  Peer 1 MAC: ");
    printMac(edgeMacs[0]);
    Serial.printf(" Channel: %d\n", peer1.channel);

    esp_err_t st1 = esp_now_add_peer(&peer1);
    if (st1 == ESP_OK)
    {
        Serial.println("     Peer 1 added");
    }
    else if (st1 == ESP_ERR_ESPNOW_EXIST)
    {
        Serial.println("     Peer 1 already existed");
    }
    else
    {
        Serial.print("     Peer 1 add failed: ");
        Serial.println(st1);
    }

    esp_now_peer_info_t peer2{};
    memcpy(peer2.peer_addr, edgeMacs[1], 6);
    peer2.channel = WIFI_CHANNEL_LOCK;
    peer2.encrypt = false;

    Serial.print("  Peer 2 MAC: ");
    printMac(edgeMacs[1]);
    Serial.printf(" Channel: %d\n", peer2.channel);

    esp_err_t st2 = esp_now_add_peer(&peer2);
    if (st2 == ESP_OK)
    {
        Serial.println("     Peer 2 added");
    }
    else if (st2 == ESP_ERR_ESPNOW_EXIST)
    {
        Serial.println("    Peer 2 already existed");
    }
    else
    {
        Serial.print("     Peer 2 add failed: ");
        Serial.println(st2);
    }
}

void setup_wifi()
{
    delay(10);
    Serial.println("\n===== SINK NODE START =====");

    WiFi.mode(WIFI_STA);

    Serial.print("[WiFi] This ESP MAC: ");
    Serial.println(WiFi.macAddress());

    Serial.print("[WiFi] Connecting to: ");
    Serial.println(ssid);
    WiFi.disconnect(true);
    delay(100);

    WiFi.mode(WIFI_STA);
    // WiFi.config(
    //     IPAddress(192, 168, 0, 149),
    //     IPAddress(192, 168, 0, 1),
    //     IPAddress(255, 255, 255, 0),
    //     IPAddress(8, 8, 8, 8));
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\n[WiFi] Connected!");
        Serial.print("[WiFi] IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WiFi] Channel: ");
        Serial.println(WiFi.channel());
        Serial.println(WiFi.localIP());
        Serial.println(WiFi.gatewayIP());
        Serial.println(WiFi.dnsIP());
    }
    else
    {
        Serial.println("\n[WiFi] Connection failed!");
        Serial.print("[WiFi] Status: ");
        Serial.println(WiFi.status());
        Serial.println("[WiFi] ESP-NOW will still work, but MQTT won't");
    }

    int channel = WiFi.channel();

    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    Serial.print("[WiFi] Channel locked to: ");
    Serial.println(channel);

    Serial.print("[ESPNOW] Initializing… ");
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("❌ FAIL");
        while (1)
            delay(1000);
    }
    Serial.println("OK");

    esp_now_register_recv_cb(onDataRecv);

    addPeers();
    Serial.println("===== Sink ready. Waiting for packets from 2 Edge Nodes… =====");
}

void reconnect_mqtt()
{
    IPAddress testIP;
    if (!WiFi.hostByName("mqtt.eu.thingsboard.cloud", testIP))
    {
        Serial.println("DNS FAIL - cannot resolve hostname!");
    }
    else
    {
        Serial.print("Resolved: ");
        Serial.println(testIP);
    }

    while (!client.connected())
    {
        Serial.print("Attempt MQTT connect...");
        String clientId = "ESP32-Sink-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), access_token, NULL))
        {
            Serial.println("MQTT connected");
        }
        else
        {
            Serial.print("MQTT fail rc=");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

void send_to_thingsboard(String vibrationLevel, float meanError)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected. Skip send");
        return;
    }
    if (!client.connected())
        reconnect_mqtt();
    StaticJsonDocument<256> doc;
    doc["node_id"] = "SINK_NODE";
    doc["mean_error"] = meanError;
    doc["vibration_level"] = vibrationLevel;
    doc["timestamp"] = millis();
    String payload;
    serializeJson(doc, payload);
    Serial.println("Publish payload:");
    Serial.println(payload);
    if (client.publish(mqtt_topic, payload.c_str()))
    {
        Serial.println("Published OK");
    }
    else
    {
        Serial.println("Publish FAIL");
    }
    for (int i = 0; i < NUM_EDGE_NODES; i++)
        nodeReceived[i] = false;
    for (int i = 0; i < NUM_EDGE_NODES; i++)
        memset(&dataNode[i], 0, sizeof(struct_message));
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("Sink Node starting");

    setup_wifi();

    client.setServer(mqtt_server, mqtt_port);

    reconnect_mqtt();

    Serial.println("Sink ready. Waiting for incoming ESP-NOW packets");
}

void loop()
{
    if (!client.connected())
        reconnect_mqtt();
    client.loop();
    delay(10);
}
