#include <arduino_secrets.h>
#include "thingProperties.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Pinler
const int rainPin = 23;
const int mq135Pin = 34;
const int trigPin = 5;
const int echoPin = 18;
const int servoPin1 = 13;
const int servoPin2 = 12;

Servo rainServo;
Servo moveServo;
Adafruit_BMP085_Unified bmp;
bool servo1Status = false;
bool servo2Status = false;

void setup() {
  Serial.begin(115200);
  delay(1500);
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  pinMode(rainPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  rainServo.attach(servoPin1);
  moveServo.attach(servoPin2);

  rainServo.write(0);
  moveServo.write(0);

  Wire.begin();
  if (!bmp.begin()) {
    Serial.println("BMP180 sensörü yok!");
    while (1);
  }
}

void loop() {
  ArduinoCloud.update();
onServo1ControlChange();  // doğru
onServo2ControlChange();  // doğru


  // 🟢 Yağmur durumu
  int rainState = digitalRead(rainPin);
 if (!servo1Control) {
  if (rainState == HIGH) {
    rainServo.write(0);
    servo1Control = false;
    servo1Status = false; // OFF
  } else {
    rainServo.write(90);
    servo1Control = true;
    servo1Status = true; // ON
  }
}


  // 🟢 Mesafe ölçümü
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0344) / 2.0;

  // Servo kontrolü
  if (!servo2Control) {
  if (distance < 10) {
    moveServo.write(90);
    servo2Control = true;
    servo2Status = true; // ON
  } else {
    moveServo.write(0);
    servo2Control = false;
    servo2Status = false; // OFF
  }
}


  // 🟢 Gaz verisi (MQ135)
  int mq135Value = analogRead(mq135Pin);

  // 🟢 BMP180 verisi
  sensors_event_t event;
  float temp = 0.0;
  float pressure = 0.0;

  bmp.getEvent(&event);
  if (event.pressure) {
    bmp.getTemperature(&temp);
    pressure = event.pressure;
  }

  // 🌐 Cloud'a veri gönder
  mq135ValueCloud = mq135Value;
  temperatureCloud = temp;
  pressureCloud = pressure;

  // ✅ Seri monitöre yaz
  Serial.println("---- Sensor Verileri ----");
  Serial.print("🌧️ Yağmur: ");
  Serial.println(rainState == HIGH ? "YOK" : "VAR");

  Serial.print("📏 Mesafe: ");
  Serial.print(distance);
  Serial.println(" cm");

  Serial.print("💨 MQ135 Gaz: ");
  Serial.println(mq135Value);

  Serial.print("🌡️ Sıcaklık: ");
  Serial.print(temp);
  Serial.println(" °C");

  Serial.print("🌬️ Basınç: ");
  Serial.print(pressure);
  Serial.println(" hPa");
  

  delay(1000);
}

// Cloud üzerinden manuel servo kontrol callback’leri
//void onServo1ControlChange() {
 // if (servo1Control) rainServo.write(90);
//}

//void onServo2ControlChange() {
 // if (servo2Control) moveServo.write(90);
//}
void onServo1ControlChange() {
  rainServo.write(servo1Control ? 90 : 0);
  servo1Status = servo1Control;
}

void onServo2ControlChange() {
  moveServo.write(servo2Control ? 90 : 0);
  servo2Status = servo2Control;
}  
