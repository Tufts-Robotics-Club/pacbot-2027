/*
ESP32 Tutorial with HC-SR04 Ultrasonic Sensor
Last Edited: 10/03/2026 by Charlotte Wong


A basic program that configures pins, then uses a distance sensor to print the
distance to the closest object from the sensor in cm.
*/


#include <Arduino.h>



int analogPin = 15; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and VCC
float val = 0;  // variable to store the value read
int redCathode = 5;
int greenCathode = 4;
int  blueCathode = 2;
int highVoltage = 12700;
int medVoltage = 10400;
int lowVoltage = 9000;
float r_1 = 560; //Resistor #1 Value in Ohms
float r_2 = 560; //Resistor #2 Value in Ohms
float r_3 = 5000; //Resistor #3 Value in Ohms
float scaleFactor = ((r_1 + r_2 + r_3) / (r_1 + r_2));

void setup() {

    analogReadResolution(12);
    Serial.begin(115200);
    pinMode(redCathode,  OUTPUT);              
    pinMode(greenCathode, OUTPUT);
    pinMode(blueCathode, OUTPUT);
}



void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redCathode, redValue);
  analogWrite(greenCathode, greenValue);
  analogWrite(blueCathode, blueValue);
}



void loop() {
  val =  scaleFactor * analogReadMilliVolts(analogPin);  // read the input pin
  Serial.println(val);          // debug value
  delay(200);
  if (val < lowVoltage) {
    Serial.println("WARNING: Low battery voltage detected! Please recharge the battery.");
    setColor(100, 0, 0); // Red
  }
  else if (val < medVoltage) {
    Serial.println("Battery voltage is medium. Please consider recharging soon.");
    setColor(100, 100, 0); // Yellow
  }
  else if (val < highVoltage) {
    Serial.println("Battery voltage is high. All systems are good.");
    setColor(0, 100, 0); // Green
  }
}