// Send data to Server

// Libraries for Bluetooth
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  while(!SerialBT.available())
  {
    Serial.println("waiting for bluetooth connection.....");
    delay(1000);
  }
  Serial.println("BLUETOOTH CONNECTED!");

}

void loop() {
  delay(1000);

}
