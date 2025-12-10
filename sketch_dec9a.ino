#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// --- WiFi ---
#define ssid "AIS_2.4G"
#define password "Kb277683"

// --- Hardware ---
#define RX_PIN 18
#define TX_PIN 17

HardwareSerial mySerial(2); // UART2





// --- Read UART data ---
void Read_Uart() {
  while (mySerial.available()) {
    char c = mySerial.read();
      Serial.print("RAW UART: ");
      Serial.println(c);
      sendToFirestore(c);
  }
}


// --- Setup ---
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

}

void sendToFirestore(char c) {
  if (WiFi.status() == WL_CONNECTED) {

    WiFiClientSecure client;
    client.setInsecure();    // IMPORTANT for Firestore HTTPS

    HTTPClient http;

    String url = "https://firestore.googleapis.com/v1/projects/guitar-tuner-6c93d/databases/(default)/documents/detection/current?key=AIzaSyCOMFMmpdH7CMoK84llbdMQaDSp_YyFyek";

    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");

    // bool temp = (c == 'T');

    // String payload = R"(
    // {
    //   "fields": {
    //     "detected": { "booleanValue": temp }
    //   }
    // }
    // )";
    bool temp = (c == 'T');

    String payload = "{\n";
    payload += "  \"fields\": {\n";
    payload += "    \"detected\": { \"booleanValue\": ";
    payload += (temp ? "true" : "false");
    payload += " }\n";
    payload += "  }\n";
    payload += "}";


    int httpCode = http.PATCH(payload);

    Serial.printf("HTTP Response: %d\n", httpCode);
    if (httpCode > 0) {
      Serial.println(http.getString());
    }

    http.end();
  } else {
    Serial.println("WiFi not connected!");
  }
}


// --- Main loop ---
void loop() {

  Read_Uart();

}