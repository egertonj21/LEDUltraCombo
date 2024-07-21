#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>
#include <vector>

#define LED_PIN          8       // Pin where the LED strip is connected
#define NUM_LEDS         31      // Number of LEDs in your strip
#define TRIGGER_PIN      10      // Pin for the ultrasonic sensor trigger
#define ECHO_PIN         9       // Pin for the ultrasonic sensor echo
#define MAX_DISTANCE     200     // Maximum distance to measure with the ultrasonic sensor
#define HEARTBEAT_INTERVAL 60000 // Heartbeat interval in milliseconds
#define ALIVE_INTERVAL   15 * 60 * 1000 // Alive message interval in milliseconds
#define DISTANCE_INTERVAL 400    // Interval between distance measurements in milliseconds
#define MIN_DISTANCE     50      // Minimum distance to consider a valid measurement

const char* mqtt_server = "";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC_TRIGGER = "trigger/ledstrip2";
const char* MQTT_TOPIC_DISTANCE = "ultrasonic/distance_sensor2";
const char* ALIVE_TOPIC_LED = "alive/ledstrip2";
const char* ALIVE_TOPIC_DISTANCE = "alive/distance_sensor2";
const char* CONTROL_TOPIC = "control/distance_sensor";
const char* MQTT_TOPIC_ACK = "ack/ledstrip1";  // New topic for acknowledgment

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned long lastHeartbeat = 0;
unsigned long lastDistanceMeasure = 0;
unsigned long lastAliveTime = 0;
bool isAwake = true;

// Structure to track timing for specific LED ranges
struct LedTimer {
  unsigned long endTime;
  int startLED;
  int endLED;
  uint32_t originalColor;
};

std::vector<LedTimer> ledTimers;

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  WiFiManager wifiManager;
  wifiManager.autoConnect("Device2");

  client.setServer(mqtt_server, MQTT_PORT);
  client.setCallback(mqttCallback);

  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - lastHeartbeat > HEARTBEAT_INTERVAL) {
    publishHeartbeat(ALIVE_TOPIC_LED);
    lastHeartbeat = currentMillis;
  }

  checkTimers();

  if (isAwake && currentMillis - lastDistanceMeasure > DISTANCE_INTERVAL) {
    measureAndPublishDistance();
    lastDistanceMeasure = currentMillis;
  }

  if (currentMillis - lastAliveTime >= ALIVE_INTERVAL) {
    publishHeartbeat(ALIVE_TOPIC_DISTANCE);
    lastAliveTime = currentMillis;
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(MQTT_TOPIC_TRIGGER);
      client.subscribe(CONTROL_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishHeartbeat(const char* topic) {
  if (client.publish(topic, "alive")) {
    Serial.printf("Alive message sent to %s\n", topic);
  } else {
    Serial.printf("Failed to send alive message to %s\n", topic);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';

  String message = String(msg);
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == MQTT_TOPIC_TRIGGER) {
    processTriggerMessage(message);
  } else if (String(topic) == CONTROL_TOPIC) {
    processControlMessage(message);
  }
}

void processTriggerMessage(String message) {
  int firstSepIndex = message.indexOf('&');
  int secondSepIndex = message.indexOf('&', firstSepIndex + 1);

  String range = message.substring(0, firstSepIndex);
  String color = message.substring(firstSepIndex + 1, secondSepIndex);
  String durationStr = message.substring(secondSepIndex + 1);

  int startLED = range.substring(0, range.indexOf('-')).toInt();
  int endLED = range.substring(range.indexOf('-') + 1).toInt();
  uint32_t colorCode = getColorFromRGBString(color);
  int duration = durationStr.toInt() * 1000; // Convert duration from seconds to milliseconds

  // Save the current color of the range
  uint32_t originalColor = strip.getPixelColor(startLED);

  // Set LEDs to the new color
  setLEDs(startLED, endLED + 1, colorCode); // endLED is inclusive

  if (duration > 0) {
    unsigned long newEndTime = millis() + duration; // Duration is now in milliseconds
    LedTimer newTimer = { newEndTime, startLED, endLED + 1, originalColor };
    ledTimers.push_back(newTimer);
  }

  // Publish acknowledgment message
  char ackMsg[50];
  snprintf(ackMsg, sizeof(ackMsg), "Trigger processed: %s", message.c_str());
  if (client.publish(MQTT_TOPIC_ACK, ackMsg)) {
    Serial.println("Acknowledgment message sent!");
  } else {
    Serial.println("Failed to send acknowledgment message!");
  }
}

void processControlMessage(String message) {
  if (message == "sleep") {
    isAwake = false;
    Serial.println("Sensor is going to sleep...");
  } else if (message == "wake") {
    isAwake = true;
    Serial.println("Sensor is waking up...");
  }
}

uint32_t getColorFromRGBString(String rgb) {
  int firstCommaIndex = rgb.indexOf(',');
  int secondCommaIndex = rgb.indexOf(',', firstCommaIndex + 1);

  int red = rgb.substring(0, firstCommaIndex).toInt();
  int green = rgb.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
  int blue = rgb.substring(secondCommaIndex + 1).toInt();

  return strip.Color(red, green, blue);
}

void setLEDs(int start, int end, uint32_t color) {
  for (int i = start; i < end; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void checkTimers() {
  unsigned long currentTime = millis();
  for (auto it = ledTimers.begin(); it != ledTimers.end(); ) {
    if (currentTime > it->endTime) {
      // Revert to original color
      setLEDs(it->startLED, it->endLED, it->originalColor);
      it = ledTimers.erase(it); // Remove expired timer
    } else {
      ++it;
    }
  }
}

void turnOffLEDs(int start, int end) {
  for (int i = start; i < end; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();
}

void measureAndPublishDistance() {
  unsigned int uS = sonar.ping();
  unsigned int distance = uS / US_ROUNDTRIP_CM;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 0 && distance < MIN_DISTANCE) { // Ignore erroneous reading of 0
    char payload[10];
    snprintf(payload, sizeof(payload), "%d", distance);

    if (client.publish(MQTT_TOPIC_DISTANCE, payload)) {
      Serial.println("Distance message sent to MQTT broker!");
    } else {
      Serial.println("Failed to send distance message to MQTT broker!");
    }
  }
}
