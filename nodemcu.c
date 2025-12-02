#include <WiFi.h>
#include <PubSubClient.h>

#define RXD2 16
#define TXD2 17

#define STM_BAUD 115200

WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial STMSerial(2);

String message;
int counter = 0;

// WiFi
const char *ssid = <wifi SSID>; // Enter your WiFi name
const char *password = <password>;  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = <broker>;
const char *topic = <topic>;
const char *mqtt_username = <username>;
const char *mqtt_password = <password>;
const char *client_id = <client id>;
const char *token = <token>;
const char *secret = <secret>;
const int mqtt_port = <port>;

void setup() {
  // Set software serial baud to 115200;
  Serial.begin(115200);

  // Custom Serial
  STMSerial.begin(STM_BAUD,SERIAL_8N1,RXD2,TXD2);
  Serial.println("Serial 2 initiated");
  // Connecting to a Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }

  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
      Serial.printf("The client %s connects to the public MQTT broker\n", WiFi.macAddress().c_str());
      if (client.connect(client_id, token, secret)) {
          Serial.println("NETPIE MQTT broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }
  client.subscribe((char*)"@msg/#");
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void loop() {
  client.loop();
  // delay(1000);
  if(STMSerial.available()) {
    message = STMSerial.readStringUntil('\n');
    // Serial.println("Serial Received [RAW]: " + getValue(message,',',0));
    // Serial.println("Serial Received [BPM]: " + getValue(message,',',3));
    if(counter % 100 == 0) {
      String msg_sample = "{\"data\" : {\"raw\" : ";
      msg_sample += getValue(message,',',1) + "," +  "\"bpm\" : " + getValue(message,',',3) + " }}";
      Serial.println(msg_sample);
      client.publish((char*)"@shadow/data/update",msg_sample.c_str());
      counter = 0;
    }
    counter++;
  }
}

