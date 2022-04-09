// Send data to Server

// Libraries for Bluetooth
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
static unsigned int counter = 0;

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
  counter++;
  SerialBT.print("My counter is ");
  SerialBT.print(counter);
  SerialBT.write(0);
  Serial.println(counter);
}
