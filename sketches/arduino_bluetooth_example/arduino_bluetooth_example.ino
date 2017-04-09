/**
 * JY-MCU Bluetooth module communication example for Arduino.
 *
 * Connect RX_PIN to TX pin of the module,
 * Connect TX_PIN to RX pin of the module.
 *
 * Based on SoftwareSerial example by Tom Igoe and Mikal Hart.
 * Adapted for JY-MCU module by Tautvidas Sipavicius.
 *
 * This example code is in the public domain.
 */

#include <SoftwareSerial.h>
  
const int RX_PIN = 10;
const int TX_PIN = 9;
const int BLUETOOTH_BAUD_RATE = 9600;

SoftwareSerial bluetooth(RX_PIN, TX_PIN);

void setup() {
   Serial.begin(9600);
   bluetooth.begin(BLUETOOTH_BAUD_RATE);
   Serial.println("Setup Completed.");
}

void loop() {
  
  if (bluetooth.available()) {
    Serial.write(bluetooth.read());
  }
  if (Serial.available()) {
    bluetooth.write(Serial.read());
  }
}