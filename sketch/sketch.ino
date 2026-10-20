/*
 * NORVI X1 (V2 N16R2) + NORVI X-AI4 (ADS1115)
 * Advanced HMI Pressure Dashboard with Motor Indicators & I2C Scan
*/

#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- SENSOR CONFIGURATION ---
const float MAX_PRESSURE = 10.0; // Maximum pressure of your sensors (e.g., 10.0 Bar)
const float TRIGGER_POINT = 5.0; // Pressure threshold to turn Motor ON

// --- PINS ---
#define SDA   8
#define SCL   9
#define MISO  13
#define MOSI  11
#define SCLK  12

// SPI Isolation Pins 
#define ETH_CS 1
#define SD_chipSelect 42
#define DSP_CS 45
#define PORT_RST 38

// --- ADS1115 SETTINGS ---
#define ADS_ADDR 0x49
Adafruit_ADS1115 ads1;

// ---------------- LOVYANGFX SETUP ----------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 12; 
      cfg.pin_mosi = 11; 
      cfg.pin_miso = 13; 
      cfg.pin_dc   = 46; 
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 45; 
      cfg.pin_rst          = 47; 
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = true;
      cfg.invert           = true; 
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = true; 
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;

// --- STATE TRACKING (To prevent screen flickering) ---
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 200; // Update 5 times a second

float lastPres[4] = {-1.0, -1.0, -1.0, -1.0};
int motorState[4] = {-1, -1, -1, -1}; // -1 means uninitialized

// FUNCTION PROTOTYPES
float calculatePressure(float volts);
void drawStaticUI();
void updateChannelUI(int ch, float pressure);
void I2C_SCAN();

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n--- NORVI PRESSURE MONITOR BOOTING ---");

  // 1. Wake up expansion bus & isolate SPI
  pinMode(PORT_RST, OUTPUT); digitalWrite(PORT_RST, HIGH); 
  delay(100);

  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH); 
  pinMode(SD_chipSelect, OUTPUT); digitalWrite(SD_chipSelect, HIGH); 
  pinMode(DSP_CS, OUTPUT); digitalWrite(DSP_CS, HIGH); 
  
  // 2. Initialize I2C and Scan Bus
  Wire.begin(SDA, SCL);
  I2C_SCAN(); // Scan the I2C bus
  
  if (!ads1.begin(ADS_ADDR)) {
    Serial.println("Failed to initialize ADS1115! Check wiring and DIP switch.");
  } else {
    Serial.printf("ADS1115 OK at Address: 0x%02X\n", ADS_ADDR);
  }
  
  ads1.setGain(GAIN_TWO); // +/- 2.048V range
  ads1.setDataRate(RATE_ADS1115_860SPS);

  // 3. Display Setup
  SPI.begin(SCLK, MISO, MOSI);
  delay(100);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Draw the fixed dashboard elements once
  drawStaticUI();
  
  Serial.println("System Ready - Starting Main Loop\n");
}

void loop() {
  // 1. Read Raw ADC & Convert to Voltage
  float volts[4];
  for(int i=0; i<4; i++) {
    volts[i] = ads1.computeVolts(ads1.readADC_SingleEnded(i));
  }

  // 2. Convert Voltage to Pressure (Bar)
  float pres[4];
  for(int i=0; i<4; i++) {
    pres[i] = calculatePressure(volts[i]);
  }

  // 3. Update TFT Dashboard Smoothly
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();

    for(int i=0; i<4; i++) {
      updateChannelUI(i, pres[i]);
    }
  }
}

// ==========================================
// UTILITY FUNCTIONS
// ==========================================

// Scans the I2C bus and prints all discovered addresses to Serial Monitor
void I2C_SCAN() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning I2C Bus...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");
      deviceCount++;
      delay(1