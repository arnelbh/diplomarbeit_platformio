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

// actuators
unsigned int act2 = 48;
unsigned int act3 = 48;
unsigned int act4 = 48;
unsigned int act5 = 48;



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
  Serial.println(recipient);
  byte sender = LoRa.read();
  Serial.println(sender);
  byte dump = LoRa.read();
  Serial.println(dump);
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

  act2 = (int)incoming[0];
  act3 = (int)incoming[1];
  act4 = (int)incoming[2];
  act5 = (int)incoming[3];

  Serial.print("\tReceived data " + incoming);
  //Serial.print(" " + String((int)incoming[0]));
  Serial.print(" from 0x" + String(sender, HEX));
  Serial.println(" to 0x" + String(recipient, HEX));
}


void setup() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);


  Serial.begin(9600);
  Serial.println("Start LoRa duplex");

  //LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true) {}
  }
}

void loop() {

  if (act2 == 49) {
    digitalWrite(A2, HIGH);
    //delay(500);
  } else if (act2 == 48) {
    digitalWrite(A2, LOW);
    //delay(500);
  }

  if (act3 == 49) {
    digitalWrite(A3, HIGH);
  } else if (act3 == 48) {
    digitalWrite(A3, LOW);
  }

    if (act4 == 49) {
    digitalWrite(A4, HIGH);
  } else if (act4 == 48) {
    digitalWrite(A4, LOW);
  }

  if (act5 == 49) {
    digitalWrite(A5, HIGH);
  } else if (act5 == 48) {
    digitalWrite(A5, LOW);
  }

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

