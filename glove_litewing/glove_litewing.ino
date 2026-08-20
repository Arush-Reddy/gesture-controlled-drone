#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Connect both the glove and the laptop to the LiteWing Wi-Fi access point.
// Replace these placeholders locally. Do not commit real credentials.
const char *WIFI_SSID = "YOUR_LITEWING_SSID";
const char *WIFI_PASSWORD = "YOUR_LITEWING_PASSWORD";

// Broadcast avoids hard-coding the laptop's DHCP address. LiteWing normally
// uses the 192.168.43.x network and the drone itself is 192.168.43.42.
IPAddress bridgeIP(192, 168, 43, 255);
constexpr uint16_t GLOVE_UDP_PORT = 4210;

constexpr int BUTTON_PIN = D3;
constexpr int I2C_SDA_PIN = D4;
constexpr int I2C_SCL_PIN = D5;
constexpr uint32_t SEND_INTERVAL_MS = 20;  // 50 Hz

WiFiUDP udp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
uint32_t lastSendMs = 0;

static float clampValue(float value, float minimum, float maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Reduce control-link latency.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to LiteWing Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }

  Serial.printf("\nConnected. Glove IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("BNO085 not found at 0x4B. Check power, SDA and SCL.");
    while (true) delay(100);
  }

  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable the BNO085 game rotation vector.");
    while (true) delay(100);
  }

  connectWiFi();
  udp.begin(GLOVE_UDP_PORT);
  Serial.println("Streaming raw roll,pitch,button packets");
}

void loop() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  }

  if (!bno08x.getSensorEvent(&sensorValue) ||
      sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
    delay(1);
    return;
  }

  const uint32_t now = millis();
  if (now - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = now;

  const float r = sensorValue.un.gameRotationVector.real;
  const float i = sensorValue.un.gameRotationVector.i;
  const float j = sensorValue.un.gameRotationVector.j;
  const float k = sensorValue.un.gameRotationVector.k;

  const float rawRoll = atan2f(
      2.0f * (r * i + j * k),
      1.0f - 2.0f * (i * i + j * j)) * 180.0f / PI;

  // Clamp before asin to avoid NaN from floating-point rounding.
  const float sinPitch = clampValue(
      2.0f * (r * j - k * i), -1.0f, 1.0f);
  const float rawPitch = asinf(sinPitch) * 180.0f / PI;

  // INPUT_PULLUP: pressed is LOW. The PC treats this as the dead-man switch.
  const int buttonState = (digitalRead(BUTTON_PIN) == LOW) ? 1 : 0;

  char payload[64];
  snprintf(payload, sizeof(payload), "%.2f,%.2f,%d",
           rawRoll, rawPitch, buttonState);

  udp.beginPacket(bridgeIP, GLOVE_UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(payload), strlen(payload));
  udp.endPacket();

  Serial.println(payload);
}
