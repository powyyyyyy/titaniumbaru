#include <PS4Controller.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); 

  if (PS4.begin("6c:c8:40:06:53:76")) { 
    Serial.println("PS4 Ready.");
  } else {
    Serial.println("PS4 Flash Failed!");
  }
}

void loop() {
  if (PS4.isConnected()) {
    uint8_t data[7];
    data[0] = (int8_t)PS4.Square();
    data[1] = (int8_t)PS4.Triangle();
    data[2] = (int8_t)PS4.Circle();
    data[3] = (int8_t)PS4.Cross();
    data[4] = (int8_t)PS4.LStickX();
    data[5] = (int8_t)PS4.LStickY();
    data[6] = (int8_t)PS4.RStickX();

    Serial2.print("BNO");      // Header
    Serial2.write(data, 7);    // Payload

    Serial.print("Sent -> [Sq:");
    Serial.print(data[0]);
    Serial.print(" Tr:");
    Serial.print(data[1]);
    Serial.print(" Ci:");
    Serial.print(data[2]);
    Serial.print(" Cr:");
    Serial.print(data[3]);
    Serial.print("] | LX:");
    Serial.print((int8_t)data[4]);
    Serial.print(" LY:");
    Serial.print((int8_t)data[5]);
    Serial.print(" RX:");
    Serial.println((int8_t)data[6]);

    delay(20); 
  } else {
    static uint32_t lastCheck = 0;
    if (millis() - lastCheck > 2000) {
      Serial.println("Waiting for PS4 Controller...");
      lastCheck = millis();
    }
  }
}



#include <PS4Controller.h>
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  
  // WAJIB: Definisikan pin RX/TX agar sinyal keluar ke pin fisik
  // Format: begin(baud, config, rxPin, txPin)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 

  if (PS4.begin("6c:c8:40:06:53:76")) { 
    Serial.println("PS4 Ready.");
  } else {
    Serial.println("PS4 Flash Failed!");
  }
}

void loop() {
  if (PS4.isConnected()) {
    // Gunakan int8_t agar range -128 sampai 127 terkirim dengan benar
    int8_t data[7];
    data[0] = (int8_t)PS4.Square();
    data[1] = (int8_t)PS4.Triangle();
    data[2] = (int8_t)PS4.Circle();
    data[3] = (int8_t)PS4.Cross();
    data[4] = (int8_t)PS4.LStickX();
    data[5] = (int8_t)PS4.LStickY();
    data[6] = (int8_t)PS4.RStickX();

    // Mengirim Header dan Data mentah
    Serial2.write('B');
    Serial2.write('N');
    Serial2.write('O');
    Serial2.write((uint8_t*)data, 7); 

    // Debugging ke Serial Monitor PC
    Serial.printf("Sent -> LX:%d LY:%d\n", data[4], data[5]);

    delay(20); 
  } else {
    static uint32_t lastCheck = 0;
    if (millis() - lastCheck > 2000) {
      Serial.println("Waiting for PS4 Controller...");
      lastCheck = millis();
    }
  }
}
