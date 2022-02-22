#include <Arduino.h>

int sensorPin = A9;
int sensorValue = 0;
int outputValue = 0;

void setup() {
  Serial.begin(9600);
  pinMode( LED_BUILTIN, OUTPUT );
}

void loop() {
  sensorValue = analogRead(sensorPin);
  // outputValue = map( sensorValue, 0, 255, 0, 255);
  analogWrite(LED_BUILTIN, outputValue);
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);
  delay(2);
}