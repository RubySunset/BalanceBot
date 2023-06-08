#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

const char *WIFI_SSID = "";
const char *WIFI_PASS = "";

WebSocketsClient webSocket;

// forward declarations
void handleMovement(const char* message);
void handleScan(const char* message);
void handleReceivedText(char* payload);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      break;
    case WStype_TEXT:
      Serial.printf("[WSc] Received text: %s\n", payload);
      handleReceivedText((char*)payload);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Hello World\n");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Connect to wifi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // server address, port and URL
  webSocket.begin("192.168.50.122", 8000, "/ws/rover");

  // event handler
	webSocket.onEvent(webSocketEvent);

  // try every 5000 again if connection has failed
  // idk why but its duplicating connections
	// webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();

  // test - send every second
  static unsigned long lastSendMillis = 0;
  if (millis() - lastSendMillis > 1000) { 
    lastSendMillis = millis();

    // JSON doc fixed memory allocation on stack
    StaticJsonDocument<96> doc;
    doc["type"] = "LDR";
    doc["time"] = millis();
    doc["content"] = "Some values?";
    
    // arduino String class equivalent to C++ string - not null terminated
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
  }
}

void handleReceivedText(char* payload) {
  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, payload);

  if (error) {
    Serial.println(F("Failed to read JSON"));
  }
  else {

    // c style string more efficient apparently so why not
    const char* type = doc["type"];

    if (strcmp(type, "movement") == 0) {
      handleMovement(doc["message"]);
    } 
    else if (strcmp(type, "scan") == 0) {
      handleScan(doc["message"]);
    }
  }
}

void handleMovement(const char* message) {
  // handle movement command here
  Serial.printf("Handling movement command: %s\n", message);
}

void handleScan(const char* message) {
  // handle scan command here
  Serial.printf("Handling scan command: %s\n", message);
}