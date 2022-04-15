//    _  ___          __   _     _
//   | || \ \        / /  | |   (_)
//   | || |\ \  /\  / /__ | |__  _
//   |__   _\ \/  \/ / _ \| '_ \| |
//      | |  \  /\  / (_) | |_) | |
//      |_|   \/  \/ \___/|_.__/|_|.com
//
//  https://4wobi.com/2022/04/14/esp32-esp-c3-32s-kit/
//
//  Install ESP32 Board Package
//  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json
//
//  ESP32C3 Dev Module
//  Flash Size: 2MB(16Mb) or 4MB(32Mb)
//
//  Source: https://microcontrollerslab.com/esp32-rgb-led-web-server/
//

/*  Ai-Thinker ESP-C3-32S-Kit
      RGB LED:      IO5 RGB blue;
                    IO3 RGB red;
                    IO4 RGB green;
      WARLM LED:    IO19
      COLD LED:     IO18
      RIGHT BUTTON: IO9
*/

#include <WiFi.h>
#include <WebServer.h>

#include "html.h"

// WIFI SETTINGS
const char* ssid = "WIFI";
const char* password = "PASSWORD";

const byte DNS_PORT = 53;

// BUTTON SETTINGS
const int buttonPin = 9;
bool buttonState = 1;
bool lastButtonState = 1;

// LED SETTINGS
const int warmPin = 18;
const int coldPin = 19;
bool lampState = 0;
unsigned int ledTimer = 5000;
unsigned long ledLastTimer = 0;

// RGB LED SETTINGS
const int red_pin = 3;
const int green_pin = 4;
const int blue_pin = 5;

// Setting PWM frequency, channels and bit resolution
const int frequency = 5000;
const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;
const int resolution = 8;

WebServer webServer(80);

void handleRoot() {
  String red_pin = webServer.arg(0);
  String green_pin = webServer.arg(1);
  String blue_pin = webServer.arg(2);

  if ((red_pin != "") && (green_pin != "") && (blue_pin != ""))
  {
    ledcWrite(redChannel, 1023 - red_pin.toInt());
    ledcWrite(greenChannel, 1023 - green_pin.toInt());
    ledcWrite(blueChannel, 1023 - blue_pin.toInt());
  }
  Serial.print("Red: ");
  Serial.println(red_pin.toInt());
  Serial.print("Green: ");
  Serial.println(green_pin.toInt());
  Serial.print("Blue: ");
  Serial.println(blue_pin.toInt());
  Serial.println();

  webServer.send(200, "text/html", webpage);
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(warmPin, OUTPUT);
  pinMode(coldPin, OUTPUT);

  ledcSetup(redChannel, frequency, resolution);
  ledcSetup(greenChannel, frequency, resolution);
  ledcSetup(blueChannel, frequency, resolution);

  ledcAttachPin(red_pin, redChannel);
  ledcAttachPin(green_pin, greenChannel);
  ledcAttachPin(blue_pin, blueChannel);

  delay(1000);
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());

  webServer.on("/", handleRoot);
  webServer.begin();
}

void loop() {
  // RGB LED with WEB SERVER
  webServer.handleClient();

  // BUTTON
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH && lastButtonState == 0) {
    Serial.println("Button State HIGH");
    lastButtonState = 1;
  } else if (buttonState == LOW && lastButtonState == 1) {
    Serial.println("Button State LOW");
    lastButtonState = 0;
  }

  // WARM & COLD LEDS
  unsigned long now = millis();
  if (now - ledLastTimer > ledTimer) {
    ledLastTimer = now;
    if (lampState == 0) {
      lampState = 1;
      Serial.println("Cold LED HIGH");
      digitalWrite(warmPin, LOW);
      digitalWrite(coldPin, HIGH);
    }
    else {
      lampState = 0;
      Serial.println("Warm LED HIGH");
      digitalWrite(coldPin, LOW);
      digitalWrite(warmPin, HIGH);
    }
  }
}
