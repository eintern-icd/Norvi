/*
 * NORVI X1 (V2 N16R2) + NORVI X-AI4 (ADS1115)
 * Professional Single-Channel HMI with ROUND GAUGE
 * (Corrected for 0 - 10 Bar Range)
*/

#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- SENSOR CONFIGURATION ---
const float MIN_PRESSURE = 0.0;  // Corrected to 0.0
const float MAX_PRESSURE = 10.0;

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

// --- STATE TRACKING ---
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 150; 

float lastDisplayPressure = -999.0;
String lastStatusText = "";

// FUNCTION PROTOTYPES
void drawStaticUI();
void updateUI(float mA);

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n--- NORVI ROUND GAUGE BOOTING ---");

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
  // Read Channel 1 (AIN1)
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  float volts = ads1.computeVolts(adc0);
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
  tft.fillRect(0, 0, 240, 35, tft.color565(0, 51, 102)); 
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(middle_center);
  tft.setFont(&fonts::FreeSansBold9pt7b); 
  tft.drawString("Pressure Monitoring", 120, 17);

  // Range Text
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("Measuring Range: 0 - 10 Bar", 120, 48); // Corrected to 0

  // --- DRAW STATIC GAUGE ELEMENTS ---
  
  // Outer color-coded ring recalculated for 0-10 scale
  // Total span = 270 degrees (225 to 495). 1 Bar = 27 degrees.
  
  // 0 to 3 Bar Zone (225 to 306 degrees)
  tft.fillArc(120, 160, 82, 85, 225, 306, TFT_YELLOW);
  
  // 3 to 7 Bar Zone (306 to 414 degrees)
  tft.fillArc(120, 160, 82, 85, 306, 414, tft.color565(0, 200, 0)); // Green
  
  // 7 to 10 Bar Zone (414 to 495 degrees)
  tft.fillArc(120, 160, 82, 85, 414, 495, TFT_RED);
  
  // Empty grey track
  tft.fillArc(120, 160, 60, 79, 225, 495, TFT_DARKGREY);

  // Draw Tick Marks Labels
  tft.setTextColor(TFT_WHITE);
  tft.setFont(&fonts::FreeSans9pt7b);
  
  tft.setTextDatum(middle_right);
  tft.drawString("0", 35, 225); // Corrected tick mark to 0
  
  tft.setTextDatum(middle_left);
  tft.drawString("10", 205, 225); 
}

void updateUI(float mA) {
  float pressure = 0.0;
  bool isError = false;
  String statusText;
  uint16_t statusColor;
  uint16_t barColor;

  // 1. EVALUATE SENSOR LOGIC
  if (mA < 3.8 || mA > 20.2) {
    isError = true;
    statusText = "ERROR";
    statusColor = TFT_RED;
    barColor = TFT_DARKGREY; 
  } else {
    // Math updated for 0-10 Bar (4mA = 0, 20mA = 10)
    pressure = ((mA - 4.0) / 16.0) * 10.0; 
    
    if (pressure < 0.0) pressure = 0.0;
    if (pressure > 10.0) pressure = 10.0;

    // Logic updated for 0-3 / 3-7 / 7-10 thresholds
    if (pressure < 3.0) {
      statusText = "Low Pressure";
      statusColor = TFT_ORANGE;
      barColor = TFT_ORANGE;
    } else if (pressure <= 7.0) {
      statusText = "Normal Pressure";
      statusColor = tft.color565(0, 200, 0); 
      barColor = tft.color565(0, 255, 0);    
    } else {
      statusText = "High Pressure";
      statusColor = TFT_RED;
      barColor = TFT_RED;
    }
  }

  // 2. UPDATE ROUND GAUGE 
  if (abs(pressure - lastDisplayPressure) >= 0.01 || isError) {
    tft.setTextDatum(middle_center);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); 

    if (isError) {
      tft.setFont(&fonts::FreeSansBold18pt7b);
      tft.drawString("--.--", 120, 150);
      tft.setFont(&fonts::FreeSans9pt7b);
      tft.drawString("Bar", 120, 185);
      
      tft.fillArc(120, 160, 60, 79, 225, 495, TFT_DARKGREY); 
    } else {
      char buf[10];
      sprintf(buf, "%4.2f", pressure);
      tft.setFont(&fonts::FreeSansBold18pt7b);
      tft.drawString(buf, 120, 150);
      
      tft.setFont(&fonts::FreeSans9pt7b);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); 
      tft.drawString("Bar", 120, 185);

      // Angle calculation updated for 0 baseline
      float targetAngle = 225.0 + ((pressure / 10.0) * 270.0);

      // Draw active fill and blank out the rest
      tft.fillArc(120, 160, 60, 79, 225, targetAngle, barColor);
      tft.fillArc(120, 160, 60, 79, targetAngle, 495, TFT_DARKGREY);
    }
    
    lastDisplayPressure = pressure;
  }

  // 3. UPDATE NOTIFICATION BADGE
  if (statusText != lastStatusText) {
    tft.fillRoundRect(15, 250, 210, 60, 8, statusColor);
    tft.setTextColor(TFT_WHITE, statusColor);
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::FreeSansBold12pt7b);

    if (isError) {
      tft.drawString("SENSOR ERROR", 120, 265);
      tft.drawString("Out of Range", 120, 290);
    } else {
      tft.drawString(statusText, 120, 280); 
    }
    
    lastStatusText = statusText;
  }
}