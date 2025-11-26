#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <CAN.h>

// ==========================================
// === PIN CONFIGURATION ===
// ==========================================
// Ethernet (W5500)
#define ETH_CS   5
#define ETH_RST  4
#define ETH_SCLK 18
#define ETH_MISO 19
#define ETH_MOSI 23

// CAN Bus (Remapped)
#define TX_GPIO_NUM   27
#define RX_GPIO_NUM   26

// ==========================================
// === ODRIVE SETTINGS ===
// ==========================================
#define CAN_BAUDRATE    250000
#define ODRIVE_NODE_ID  0

// --- Command IDs ---
#define CMD_ID_SET_AXIS_STATE        0x007 // Set Idle or Closed Loop
#define CMD_ID_GET_ENCODER_ESTIMATES 0x009 // Get Pos/Vel
#define CMD_ID_SET_CONTROLLER_MODES  0x00B // Set Control Mode (Vel/Pos)
#define CMD_ID_SET_INPUT_POS         0x00C // Go to Position
#define CMD_ID_SET_INPUT_VEL         0x00D // Spin at Velocity

// --- ODrive Constants ---
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define CONTROL_MODE_VELOCITY_CONTROL  2
#define INPUT_MODE_PASSTHROUGH         1

// ==========================================
// === NETWORK & MQTT SETTINGS ===
// ==========================================
byte mac[] = { 0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0x01 };
const char* mqttServer = "192.168.1.1"; // CHANGE TO YOUR BROKER IP
const int mqttPort = 1883;
const char* topicTx = "odrive/cmd";    
const char* topicRx = "odrive/status"; 

// ==========================================
// === GLOBAL OBJECTS ===
// ==========================================
EthernetClient ethClient;
PubSubClient client(ethClient);

// Timing
unsigned long lastRequestTime = 0;
const int requestInterval = 100; // Poll telemetry every 100ms

// ==========================================
// === HELPER: SEND FLOAT VIA CAN ===
// ==========================================
void sendCanFloat(int nodeId, int cmdId, float value) {
  int id = (nodeId << 5) | cmdId;
  CAN.beginPacket(id, 8);
  CAN.write((uint8_t*)&value, 4); // Write float (4 bytes)
  for (int i = 0; i < 4; i++) CAN.write(0); // Padding
  CAN.endPacket();
}

// ==========================================
// === HELPER: SEND INT32 VIA CAN ===
// ==========================================
void sendCanInt(int nodeId, int cmdId, int32_t value1, int32_t value2 = 0) {
  int id = (nodeId << 5) | cmdId;
  CAN.beginPacket(id, 8);
  CAN.write((uint8_t*)&value1, 4); 
  CAN.write((uint8_t*)&value2, 4); 
  CAN.endPacket();
}

// ==========================================
// === MQTT CALLBACK (RECEIVE COMMANDS) ===
// ==========================================
// Format: "NodeID, CmdID, RTR, Value"
// For CONSTANT VELOCITY, use CmdID = 13
// Example: "0,13,0,5.0" -> Spin Node 0 at 5 turns/sec
void callback(char* topic, byte* payload, unsigned int length) {
  char msg[64];
  if (length >= 64) length = 63;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.print("\n📩 [MQTT] Recv: "); Serial.println(msg);

  int nodeId, cmdId, rtr;
  float value = 0.0f;

  // Parse generic 4-value format
  int parsed = sscanf(msg, "%d,%d,%d,%f", &nodeId, &cmdId, &rtr, &value);

  if (parsed >= 3) {
    int id = (nodeId << 5) | cmdId; 
    
    Serial.printf("➡️ [CAN] ID: 0x%X (Cmd: %d) | Val: %.2f\n", id, cmdId, value);
    
    // Remote Request (RTR) - Asking for data
    if (rtr == 1) {
      CAN.beginPacket(id, 8, true);
      CAN.endPacket();
    } 
    // Data Packet - Sending a command (Position or Velocity)
    else {
      sendCanFloat(nodeId, cmdId, value);
    }
  } else {
    Serial.println("❌ Format Error. Use: 'Node,Cmd,RTR,Value'");
  }
}

// ==========================================
// === INITIALIZE ODRIVE ===
// ==========================================
void initODrive() {
  Serial.println("⚙️ Configuring ODrive for Velocity Control...");
  
  // 1. Set Control Mode -> Velocity Control (2), Input Mode -> Passthrough (1)
  // We use sendCanInt because this command expects two int32 values
  sendCanInt(ODRIVE_NODE_ID, CMD_ID_SET_CONTROLLER_MODES, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
  delay(10);

  // 2. Set Axis State -> Closed Loop Control (8)
  sendCanInt(ODRIVE_NODE_ID, CMD_ID_SET_AXIS_STATE, AXIS_STATE_CLOSED_LOOP_CONTROL);
  
  Serial.println("⚙️ ODrive Config Sent (Velocity Mode + Closed Loop).");
}

// ==========================================
// === SETUP ===
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- SYSTEM BOOT ---");

  // 1. Init Ethernet
  Ethernet.init(ETH_CS);
  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, LOW); delay(50);
  digitalWrite(ETH_RST, HIGH); delay(50);

  if (Ethernet.begin(mac) == 0) {
    Serial.println("❌ DHCP Failed.");
  } else {
    Serial.print("✅ IP Address: "); Serial.println(Ethernet.localIP());
  }

  // 2. Init MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  // 3. Init CAN
  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
  if (!CAN.begin(CAN_BAUDRATE)) {
    Serial.println("❌ CAN Init Failed!");
    while (1);
  }
  Serial.println("✅ CAN Ready.");

  // 4. Auto-Configure ODrive
  // (Wait 3s for ODrive to boot up fully before sending config)
  delay(3000); 
  initODrive();
}

// ==========================================
// === RECONNECT ===
// ==========================================
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_ODrive_Bridge")) {
      Serial.println(" connected!");
      client.subscribe(topicTx);
    } else {
      Serial.print(" failed, rc="); Serial.print(client.state());
      Serial.println(" try again in 2s");
      delay(2000);
    }
  }
}

// ==========================================
// === MAIN LOOP ===
// ==========================================
void loop() {
  // 1. Maintain Network
  if (!client.connected()) reconnect();
  client.loop();

  // 2. SEND POLLING REQUEST (RTR)
  // Requesting Encoder Estimates (Position & Velocity)
  if (millis() - lastRequestTime > requestInterval) {
    lastRequestTime = millis();
    int id = (ODRIVE_NODE_ID << 5) | CMD_ID_GET_ENCODER_ESTIMATES;
    CAN.beginPacket(id, 8, true);
    CAN.endPacket();
  }

  // 3. READ CAN RESPONSES
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    long id = CAN.packetId();
    int nodeId = id >> 5;
    int cmdId = id & 0x01F;

    // Filter: Is this the Encoder Response?
    if (nodeId == ODRIVE_NODE_ID && cmdId == CMD_ID_GET_ENCODER_ESTIMATES) {
      if (packetSize >= 8) {
        uint8_t buffer[8];
        for (int i = 0; i < 8; i++) buffer[i] = CAN.read();
        
        float position, velocity;
        memcpy(&position, &buffer[0], 4);
        memcpy(&velocity, &buffer[4], 4);

        // Prepare string for MQTT
        char payload[64];
        sprintf(payload, "Pos: %.2f | Vel: %.2f", position, velocity);
        
        // Publish to MQTT
        client.publish(topicRx, payload);
      }
    }
  }
}