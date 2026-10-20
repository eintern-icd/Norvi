/*
 * NORVI X1 (V2 N16R2) + NORVI X-AI4 (ADS1115)
 * Professional Single-Channel HMI Pressure Dashboard
 * (Optimized & Cleaned)
*/

#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- SENSOR CONFIGURATION ---
const float MIN_PRESSURE = 1.0;  // Minimum range of sensor
const float MAX_PRESSURE = 10.0; // Maximum range of sensor

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
const unsigned long DISPLAY_INTERVAL = 200; // Fast 200ms updates

float lastDisplayPressure = -999.0;
String lastStatusText = "";

// FUNCTION PROTOTYPES
void drawStaticUI();
void updateUI(float mA);

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n--- NORVI PRESSURE HMI BOOTING ---");

  // 1. Wake up expansion bus & isolate SPI
  pinMode(PORT_RST, OUTPUT); digitalWrite(PORT_RST, HIGH); 
  delay(100);

  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH); 
  pinMode(SD_chipSelect, OUTPUT); digitalWrite(SD_chipSelect, HIGH); 
  pinMode(DSP_CS, OUTPUT); digitalWrite(DSP_CS, HIGH); 
  
  // 2. Initialize I2C and ADC
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
  
  Serial.println("System Ready - Starting Main Loop");
}

void loop() {
  // Read Channel 1 (AIN1) Only
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  float volts = ads1.computeVolts(adc0);
  
  // Hardware amplifier converts Voltage * 10 = mA
  float mA = volts * 10.0;

  // Update TFT Dashboard Smoothly
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    updateUI(mA);
  }
}

// ==========================================
// HMI DASHBOARD FUNCTIONS
// ==========================================

void drawStaticUI() {
  // Top Header Bar
  tft.fillRect(0, 0, 240, 40, tft.color565(0, 51, 102)); // Navy Blue
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(middle_center);
  tft.setFont(&fonts::FreeSansBold9pt7b); 
  tft.drawString("Pressure Monitoring", 120, 20);

  // Range Text
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("Measuring Range: 1 - 10 Bar", 120, 60);

  // Gauge Outline and Empty Track
  tft.drawRect(19, 149, 202, 32, TFT_WHITE); // White Border
  tft.fillRect(20, 150, 200, 30, TFT_DARKGREY); // Empty Gauge Track

  // Visual Zone Indicators under the Gauge (Y = 185)
  // 1-3 Bar zone (Width: 44px)
  tft.fillRect(20, 185, 44, 5, TFT_YELLOW);
  // 3-7 Bar zone (Width: 89px)
  tft.fillRect(64, 185, 89, 5, tft.color565(0, 200, 0)); // Green
  // 7-10 Bar zone (Width: 67px)
  tft.fillRect(153, 185, 67, 5, TFT_RED);

  // Gauge Tick Labels
  tft.setTextColor(TFT_WHITE);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextDatum(top_center);
  tft.drawString("1", 20, 195);
  tft.drawString("3", 64, 195);
  tft.drawString("7", 153, 195);
  tft.drawString("10", 220, 195);
}

void updateUI(float mA) {
  float pressure = 0.0;
  bool isError = false;
  String statusText;
  uint16_t statusColor;
  uint16_t barColor;

  // 1. EVALUATE SENSOR LOGIC & NOTIFICATION ZONES
  // Allow a tiny 0.1mA deadband for sensor fluctuation
  if (mA < 3.9 || mA > 20.1) {
    isError = true;
    statusText = "ERROR";
    statusColor = TFT_RED;
    barColor = TFT_DARKGREY; 
  } else {
    // Math: Convert 4-20mA to 1-10 Bar
    pressure = 1.0 + ((mA - 4.0) / 16.0) * 9.0;
    
    // Clamp visual values to bounds
    if (pressure < 1.0) pressure = 1.0;
    if (pressure > 10.0) pressure = 10.0;

    // Determine Notification Zones
    if (pressure < 3.0) {
      statusText = "Low Pressure";
      statusColor = TFT_ORANGE;
      barColor = TFT_ORANGE;
    } else if (pressure <= 7.0) {
      statusText = "Normal Pressure";
      statusColor = tft.color565(0, 200, 0); // Green
      barColor = tft.color565(0, 255, 0);    // Bright Green Bar
    } else {
      statusText = "High Pressure";
      statusColor = TFT_RED;
      barColor = TFT_RED;
    }
  }

  // 2. UPDATE LIVE NUMERIC VALUE & GAUGE (Only if changed to prevent flicker)
  if (abs(pressure - lastDisplayPressure) >= 0.01 || isError) {
    tft.setTextDatum(middle_center);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); 
    tft.setFont(&fonts::FreeSansBold18pt7b); // Huge Font

    if (isError) {
      tft.drawString(" --.-- Bar ", 120, 105);
      tft.fillRect(20, 150, 200, 30, TFT_DARKGREY); // Clear the gauge
    } else {
      char buf[20];
      sprintf(buf, " %5.2f Bar ", pressure);
      tft.drawString(buf, 120, 105);

      // Calculate how many pixels wide the bar should be (0 to 200 pixels)
      int barWidth = ((pressure - 1.0) / 9.0) * 200.0;
      if (barWidth < 0) barWidth = 0;
      if (barWidth > 200) barWidth = 200;

      // Draw active colored bar
      tft.fillRect(20, 150, barWidth, 30, barColor);
      // Clear the rest of the empty track behind it
      tft.fillRect(20 + barWidth, 150, 200 - barWidth, 30, TFT_DARKGREY); 
    }
    
    // Output debug to serial monitor
    Serial.printf("Current: %.2f mA | Pressure: %.2f Bar | Status: %s\n", mA, (isError ? 0.0 : pressure), statusText.c_str());
    lastDisplayPressure = pressure;
  }

  // 3. UPDATE NOTIFICATION BADGE (Only if state text changes)
  if (statusText != lastStatusText) {
    // Draw the rounded notification box
    tft.fillRoundRect(10, 240, 220, 70, 8, statusColor);
    tft.setTextColor(TFT_WHITE, statusColor);
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::FreeSansBold12pt7b);

    if (isError) {
      tft.drawString("SENSOR ERROR", 120, 260);
      tft.drawString("Out of Range", 120, 290);
    } else {
      tft.drawString(statusText, 120, 275);
    }
    
    lastStatusText = statusText;
  }
}