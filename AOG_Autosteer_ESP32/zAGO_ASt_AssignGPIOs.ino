#include <Wire.h>
#include <SPI.h>
#include "BNO08x_AOG.h"

// ------------------
// Pinbelegung BNO085
// ------------------
// MOSI = 13, MISO = 34, SCK = 14, SS/CS = 15, INT = 4, RST = 2
static constexpr uint8_t BNO_CS_PIN   = 15;
static constexpr uint8_t BNO_WAK_PIN  = 5; // NC: auf diesem Board nicht verfügbar 
static constexpr uint8_t BNO_INT_PIN  = 4;
static constexpr uint8_t BNO_RST_PIN  = 2;
static constexpr uint32_t SPI_SPEED   = 3000000; // max. 3 MHz für BNO080 :contentReference[oaicite:2]{index=2}

void assignGPIOs_start_extHardware() {
  delay(50);

  // I²C für ADS, MMA, CMPS …
  if (!Wire.begin(Set.SDA, Set.SCL, 400000)) {
    Serial.println("error INIT wire, ADS, CMPS, MMA werden nicht funktionieren");
  }
  delay(20);

  // SPI für BNO085: SCK=14, MISO=34, MOSI=13, SS=15
  SPI.begin(
    14 /*SCK*/, 
    34 /*MISO*/, 
    13 /*MOSI*/, 
    15 /*SS*/     // SS geht automatisch als CS
  ); :contentReference[oaicite:3]{index=3}

  // … sonstige GPIO-Initialisierung (LEDs, Schalter, Encoder, PWM)

  // ----------------
  // IMU-Initialisierung
  // ----------------
  byte error = 0;
  switch (Set.IMUType) {
    case 0:
      // ohne IMU: Default-Werte
      steerToAOG[ 9] = 0xB8; steerToAOG[10] = 0x22; roll    = 0;
      steerToAOG[ 7] = 0x0F; steerToAOG[ 8] = 0x27; heading = 0;
      break;

    case 1:
      // BNO055 (unverändert)
      break;

    case 2:
      // CMPS14 (I²C, unverändert)
      Wire.beginTransmission(Set.CMPS14_ADDRESS);
      error = Wire.endTransmission();
      if (error) {
        Serial.println("CMPS14 nicht gefunden");
        Set.IMUType = 0;
      }
      break;

    case 3: {
      // BNO085 per SPI
      if (bno08x.beginSPI(
            BNO_CS_PIN,
            BNO_WAK_PIN,
            BNO_INT_PIN,
            BNO_RST_PIN,
            SPI_SPEED,
            SPI           // Hardware-SPI-Instanz
          )) {
        bno08x.enableGameRotationVector(REPORT_INTERVAL);
      } else {
        Serial.println("BNO085 SPI init fehlgeschlagen!");
        Set.IMUType = 0;
      }
      break;
    }

    default:
      // andere IMU-Typen …
      break;
  }

  // MMA, ADS, EEPROM … (unverändert)
}
