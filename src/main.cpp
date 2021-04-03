#include <Arduino.h>

#include <SPI.h>
#include <LoRa.h>

#include <dht.h>
#include <string.h>
#include <stdio.h>

//dht DHT;

//#define DHT22_PIN A5

const int csPin = 10;
const int resetPin = A0;
const int irqPin = 2;

byte localAddress = 0x00;
byte destinationAddress = 0xFF;
long lastSendTime = 0;
int interval = 1000;
int count = 0;


void sendMessage(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destinationAddress);
  LoRa.write(localAddress);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
}

void receiveMessage(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  //Serial.println(recipient);
  byte sender = LoRa.read();
  //Serial.println(sender);
  byte dump = LoRa.read();
  //Serial.println(dump);
  byte incomingLength = LoRa.read();
  // Serial.println(incomingLength);

  String incoming = "";
  int msg_count = 0;

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
    // incoming += LoRa.read();
    msg_count += 1;
  }

////
    //Serial.println(incomingLength);
    //Serial.println(msg_count);
  if (incomingLength != msg_count) {   // check length for error
    // Serial.println(incomingLength);
    // Serial.println(incoming.length());
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0x00) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  Serial.print("\tReceived data " + incoming);
  Serial.print(" from 0x" + String(sender, HEX));
  Serial.println(" to 0x" + String(recipient, HEX));
}



void setup() {
  Serial.begin(9600);
  Serial.println("Start LoRa duplex");

  //LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true) {}
  }
}

void loop() {
  if (millis() - lastSendTime > interval) {
    String sensorData = String(count++);


    //DHT.read22(DHT22_PIN);
    int sensorValue = analogRead(A1);
    double sensor_dif = 1023 - sensorValue;
    double sensor_peak = sensor_dif/700 * 100;
    double perc;
    if (sensor_peak > 100.0) {
        perc = 100.0;
      }
      else {
        perc = sensor_peak;
        }
    String soil_hum = String(perc);
    //String air_hum = String(DHT.humidity);
    //String air_temp = String(DHT.temperature);

    
    sensorData = soil_hum + " "; // + air_hum + " " + air_temp;

    sendMessage(sensorData);

    Serial.print("Sending data " + sensorData);
    Serial.print(" from 0x" + String(localAddress, HEX));
    Serial.println(" to 0x" + String(destinationAddress, HEX));

    lastSendTime = millis();
    interval = random(1000) + 1000;
  }

  receiveMessage(LoRa.parsePacket());
}

