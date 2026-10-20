/*
 * NORVI X1 (V2 N16R2) + NORVI X-AI4 (ADS1115)
 * Advanced HMI Pressure Dashboard with Motor Indicators
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

void setup() {
  Serial.begin(115200);
  delay(200);

  // 1. Wake up expansion bus & isolate SPI
  pinMode(PORT_RST, OUTPUT); digitalWrite(PORT_RST, HIGH); 
  delay(100);

  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH); 
  pinMode(SD_chipSelect, OUTPUT); digitalWrite(SD_chipSelect, HIGH); 
  pinMode(DSP_CS, OUTPUT); digitalWrite(DSP_CS, HIGH); 
  
  // 2. Initialize I2C & ADC
  Wire.begin(SDA, SCL);
  if (!ads1.begin(ADS_ADDR)) {
    Serial.println("Failed to initialize ADS1115!");
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

float calculatePressure(float volts) {
  float mA = volts * 10.0;
  if (mA <= 4.0) return 0.00;
  if (mA > 20.0) mA = 20.0;
  return ((mA - 4.0) / 16.0) * MAX_PRESSURE;
}

// Draws the layout grids and headers that never change
void drawStaticUI() {
  // Top Header Bar
  tft.fillRect(0, 0, 240, 45, tft.color565(0, 51, 102)); // Deep Navy Blue
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(middle_center);
  tft.setFont(&fonts::FreeSansBold12pt7b); 
  tft.drawString("NORVI PANEL", 120, 22);

  // Draw horizontal dividing lines for the 4 rows
  for(int i=0; i<4; i++) {
    int y = 55 + (i * 65);
    tft.drawFastHLine(10, y + 55, 220, TFT_DARKGREY);
    
    // Draw Channel Labels
    tft.setTextColor(TFT_CYAN);
    tft.setTextDatum(middle_left);
    tft.setFont(&fonts::FreeSansBold9pt7b); 
    tft.drawString("CH" + String(i+1), 10, y + 25);
  }
}

// Updates only the numbers and indicator boxes that change
void updateChannelUI(int ch, float pressure) {
  int y_base = 55 + (ch * 65); // Calculate row Y coordinate based on channel

  // 1. Update Pressure Value (Only if it changed by >= 0.01 to prevent flickering)
  if (abs(pressure - lastPres[ch]) >= 0.01) {
    tft.setTextDatum(middle_left);
    tft.setFont(&fonts::FreeSansBold12pt7b); 
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // (Text, Background) erases old text
    
    // Format float with 2 decimals
    char buf[10];
    sprintf(buf, "%5.2f", pressure);
    tft.drawString(buf, 55, y_base + 25);
    
    lastPres[ch] = pressure;
  }

  // 2. Update Motor Indicator Badge
  int isON = (pressure > TRIGGER_POINT) ? 1 : 0;
  
  if (isON != motorState[ch]) { // Only redraw the box if state actually flips
    motorState[ch] = isON;
    
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::FreeSansBold9pt7b); 

    if (isON) {
      // Draw Bright Green ON Badge
      tft.fillRoundRect(135, y_base + 10, 95, 30, 4, TFT_GREEN);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.drawString("MTR " + String(ch+1) + " ON", 135 + 47, y_base + 25);
    } else {
      // Draw Dark Red OFF Badge
      tft.fillRoundRect(135, y_base + 10, 95, 30, 4, TFT_MAROON);
      tft.setTextColor(TFT_WHITE, TFT_MAROON);
      tft.drawString("MTR " + String(ch+1) + " OFF", 135 + 47, y_base + 25);
    }
  }
}