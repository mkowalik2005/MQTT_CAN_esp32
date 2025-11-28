/*
 * KOMPLETNY KOD: MQTT (W5500) + ODrive CAN + RAMP MODE + REBOOT
 * Platforma: ESP32 + W5500
 */

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <CAN.h>
#include <ArduinoJson.h>

// ==========================================
// 1. USTAWIENIA BEZPIECZEŃSTWA
// ==========================================
const unsigned long SAFETY_TIMEOUT = 500;
unsigned long lastMqttCmdTime = 0; 

// ==========================================
// 2. KONFIGURACJA PINÓW
// ==========================================
#define ETH_CS_PIN    21
#define ETH_RST_PIN   22
#define CAN_TX_PIN    5
#define CAN_RX_PIN    4
#define CAN_BAUDRATE  250000

// ==========================================
// 3. KONFIGURACJA SIECI
// ==========================================
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 177);      
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const char* mqtt_server = "192.168.1.1"; 
const int mqtt_port = 1883;

// Tematy MQTT
const char* TOPIC_SET_VEL  = "odrive/set_velocity";
const char* TOPIC_CMD      = "odrive/cmd";
const char* TOPIC_FEEDBACK = "odrive/feedback";

// ==========================================
// 4. KONFIGURACJA ODRIVE (Command IDs)
// ==========================================
#define ODRIVE_NODE_ID 0

#define CMD_ID_SET_AXIS_STATE      0x007
#define CMD_ID_GET_ENCODER         0x009
#define CMD_ID_SET_CONTROLLER_MODE 0x00B
#define CMD_ID_SET_INPUT_VEL       0x00D
#define CMD_ID_REBOOT_ODRIVE       0x016 // <--- Nowe ID: Reboot

// ODrive Enums
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL        8

#define CONTROL_MODE_VELOCITY_CONTROL         2
#define INPUT_MODE_PASSTHROUGH                1
#define INPUT_MODE_VEL_RAMP                   2  

// Globalne obiekty
EthernetClient ethClient;
PubSubClient client(ethClient);

// Zmienne sterujące
float targetVelocity = 0.0f;
float measuredPos = 0.0f;
float measuredVel = 0.0f;

// Deklaracje funkcji
void sendVelocity(float velocity);
void setControlMode(int32_t controlMode, int32_t inputMode);
void setAxisState(int32_t state);
void requestEncoderData();
void rebootODrive(); // <--- Nowa funkcja

// ==========================================
// MQTT CALLBACK (Odbiór danych)
// ==========================================
void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String messageTemp;
  for (int i = 0; i < length; i++) messageTemp += (char)payload[i];

  // A. ODBIÓR PRĘDKOŚCI
  if (topicStr == TOPIC_SET_VEL) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, messageTemp);
    if (!error) {
      targetVelocity = doc["velocity"];
      lastMqttCmdTime = millis(); 
    }
  }
  
  // B. ODBIÓR KOMEND SYSTEMOWYCH
  else if (topicStr == TOPIC_CMD) {
    Serial.print("Komenda MQTT: ");
    Serial.println(messageTemp);
    lastMqttCmdTime = millis(); 

    if (messageTemp == "calibrate") {
      setAxisState(AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    }
    else if (messageTemp == "closed_loop") {
      setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
    else if (messageTemp == "set_vel_mode") {
      setControlMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
      Serial.println(">> Tryb: VELOCITY PASSTHROUGH");
    }
    else if (messageTemp == "set_ramp_mode") {
      setControlMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
      Serial.println(">> Tryb: VELOCITY RAMP");
    }
    else if (messageTemp == "reboot_odrive") { // <--- Obsługa Reboot
      rebootODrive();
      Serial.println(">> REBOOTING ODRIVE...");
    }
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  while(!Serial); 
  
  Serial.println("\n--- START SYSTEMU ODRIVE ---");

  // Reset W5500
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW); delay(100);
  digitalWrite(ETH_RST_PIN, HIGH); delay(100);
  
  // Inicjalizacja Ethernet
  Ethernet.init(ETH_CS_PIN);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  Serial.print("IP Arduino: "); Serial.println(Ethernet.localIP());

  // Inicjalizacja MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Inicjalizacja CAN
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  if (!CAN.begin(CAN_BAUDRATE)) {
    Serial.println("BLAD STARTU CAN! Sprawdz kable.");
    while (1);
  }
  Serial.println("CAN OK.");
}

// Funkcja ponownego łączenia MQTT
void reconnect() {
  while (!client.connected()) {
    targetVelocity = 0.0f; 
    Serial.print("Laczenie z MQTT...");
    if (client.connect("ESP32_ODrive_Controller")) {
      Serial.println(" Polaczono!");
      client.subscribe(TOPIC_SET_VEL);
      client.subscribe(TOPIC_CMD);
    } else {
      Serial.print(" Blad rc="); Serial.print(client.state());
      Serial.println(" (retry 2s)");
      delay(2000);
    }
  }
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long currentTime = millis();

  // --- 1. WATCHDOG BEZPIECZEŃSTWA ---
  if (currentTime - lastMqttCmdTime > SAFETY_TIMEOUT) {
    if (targetVelocity != 0.0f) {
      Serial.println("!!! WATCHDOG: Utrata polaczenia - STOP !!!");
      targetVelocity = 0.0f;
    }
  }

  // --- 2. WYSYŁANIE CAN DO ODRIVE (co 50ms) ---
  static unsigned long lastCanCycle = 0;
  if (currentTime - lastCanCycle > 50) {
    lastCanCycle = currentTime;
    sendVelocity(targetVelocity);
    requestEncoderData();
  }

  // --- 3. ODBIÓR DANYCH CAN Z ODRIVE ---
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    long id = CAN.packetId();
    int cmdId = id & 0x01F; 

    if (cmdId == CMD_ID_GET_ENCODER && packetSize >= 8) {
       uint8_t buffer[8];
       for (int i=0; i<8; i++) buffer[i] = CAN.read();
       memcpy(&measuredPos, &buffer[0], 4);
       memcpy(&measuredVel, &buffer[4], 4);
    }
  }

  // --- 4. WYSYŁANIE FEEDBACKU DO PYTHON (co 100ms) ---
  static unsigned long lastMqttPub = 0;
  if (currentTime - lastMqttPub > 100) {
    lastMqttPub = currentTime;
    char msg[64];
    snprintf(msg, sizeof(msg), "{\"v_meas\":%.2f,\"p_meas\":%.2f}", measuredVel, measuredPos);
    client.publish(TOPIC_FEEDBACK, msg);
  }
}

// ==========================================
// FUNKCJE POMOCNICZE CAN
// ==========================================

void sendVelocity(float velocity) {
  float torqueFF = 0.0f;
  int id = (ODRIVE_NODE_ID << 5) | CMD_ID_SET_INPUT_VEL;
  CAN.beginPacket(id);
  CAN.write((uint8_t*)&velocity, 4);
  CAN.write((uint8_t*)&torqueFF, 4);
  CAN.endPacket();
}

void setAxisState(int32_t state) {
  int id = (ODRIVE_NODE_ID << 5) | CMD_ID_SET_AXIS_STATE;
  CAN.beginPacket(id);
  CAN.write((uint8_t*)&state, 4);
  CAN.endPacket();
}

void setControlMode(int32_t controlMode, int32_t inputMode) {
  int id = (ODRIVE_NODE_ID << 5) | CMD_ID_SET_CONTROLLER_MODE;
  CAN.beginPacket(id);
  CAN.write((uint8_t*)&controlMode, 4);
  CAN.write((uint8_t*)&inputMode, 4);
  CAN.endPacket();
}

void requestEncoderData() {
  int id = (ODRIVE_NODE_ID << 5) | CMD_ID_GET_ENCODER;
  CAN.beginPacket(id, 8, true); // Ramka RTR
  CAN.endPacket();
}

// Funkcja Reboot
void rebootODrive() {
  int id = (ODRIVE_NODE_ID << 5) | CMD_ID_REBOOT_ODRIVE;
  CAN.beginPacket(id);
  // Reboot nie wymaga payloadu
  CAN.endPacket();
}