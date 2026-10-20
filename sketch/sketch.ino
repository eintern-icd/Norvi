/*
 * NORVI X1 (V2 N16R2) + NORVI X-AI4 (ADS1115)
 * Professional Single-Channel HMI with ROUND GAUGE
 * (Includes Advanced LED State Tracking & Error Blinking)
*/

#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- SENSOR CONFIGURATION ---
const float MIN_PRESSURE = 0.0;  
const float MAX_PRESSURE = 10.0;

// --- LED PINS ---
// IMPORTANT: Change these to the actual GPIO pins your front panel LEDs are connected to!
#define LED_1_PIN 14 // Indicator for Low Pressure / Error
#define LED_2_PIN 15 // Indicator for High Pressure / Error

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

// LED Blinking Variables
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL = 500; // 500ms blink rate
bool blinkState = false;
int systemState = 0; // 0=Normal, 1=Low, 2=High, 3=Error

float lastDisplayPressure = -999.0;
String lastStatusText = "";

// FUNCTION PROTOTYPES
void drawStaticUI();
void updateUI(float mA);

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n--- NORVI ROUND GAUGE BOOTING ---");

  // 1. Initialize physical LEDs
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  digitalWrite(LED_1_PIN, LOW);  
  digitalWrite(LED_2_PIN, LOW); 

  // 2. Wake up expansion bus & isolate SPI
  pinMode(PORT_RST, OUTPUT); digitalWrite(PORT_RST, HIGH); 
  delay(100);

  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH); 
  pinMode(SD_chipSelect, OUTPUT); digitalWrite(SD_chipSelect, HIGH); 
  pinMode(DSP_CS, OUTPUT); digitalWrite(DSP_CS, HIGH); 
  
  // 3. Initialize I2C and ADC
  Wire.begin(SDA, SCL);
  if (!ads1.begin(ADS_ADDR)) {
    Serial.println("Failed to initialize ADS1115!");
  } 
  
  ads1.setGain(GAIN_TWO); 
  ads1.setDataRate(RATE_ADS1115_860SPS);

  // 4. Display Setup
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
  // 1. Read Channel 1 (AIN1)
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  float volts = ads1.computeVolts(adc0);
  float mA = volts * 10.0;

  // 2. Update TFT Dashboard Smoothly
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    updateUI(mA);
  }

  // 3. Handle Physical LEDs based on Current System State
  if (systemState == 3) {
    // ERROR STATE: Blink both LEDs continuously
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = millis();
      blinkState = !blinkState;
      digitalWrite(LED_1_PIN, blinkState);
      digitalWrite(LED_2_PIN, blinkState);
    }
  } else if (systemState == 1) {
    // LOW PRESSURE: LED 1 ON, LED 2 OFF
    digitalWrite(LED_1_PIN, HIGH);
    digitalWrite(LED_2_PIN, LOW);
  } else if (systemState == 2) {
    // HIGH PRESSURE: LED 1 OFF, LED 2 ON
    digitalWrite(LED_1_PIN, LOW);
    digitalWrite(LED_2_PIN, HIGH);
  } else {
    // NORMAL PRESSURE: Both LEDs OFF
    digitalWrite(LED_1_PIN, LOW);
    digitalWrite(LED_2_PIN, LOW);
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
  tft.drawString("Measuring Range: 0 - 10 Bar", 120, 48); 

  // --- DRAW STATIC GAUGE ELEMENTS ---
  
  // 0 to 3 Bar Zone (135 to 216 degrees)
  tft.fillArc(120, 160, 82,