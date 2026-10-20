/*
 * NORVI X1 (V2 N16R2) + NORVI X-AI4 (ADS1115) + PCA9536D
 * Professional Single-Channel 0-10 Bar HMI with Round Gauge & Front Panel LEDs
*/

#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <PCA9536D.h>
#include <CST816S.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- SENSOR CONFIGURATION ---
const float MIN_PRESSURE = 0.0;  
const float MAX_PRESSURE = 10.0;

// --- PINS & I2C ---
#define SDA 8
#define SCL 9
#define MISO 13
#define MOSI 11
#define SCLK 12

// SPI Isolation Pins 
#define ETH_CS 1
#define SD_chipSelect 42
#define DSP_CS 45
#define PORT_RST 38

// Front Panel LEDs (PCA9536D)
#define IO_PB1  0
#define IO_LED1 1 // Error / Low LED
#define IO_LED2 2 // Run / High LED
#define IO_PB3  3

// --- HARDWARE OBJECTS ---
#define ADS_ADDR 0x49
Adafruit_ADS1115 ads1;
PCA9536 io;
CST816S touch(8, 9, 47, 48); 

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

// LED Blinking Variables for Error State
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

  Serial.println("\n--- NORVI PRESSURE HMI BOOTING ---");

  // 1. Wake up expansion bus & isolate SPI
  pinMode(PORT_RST, OUTPUT); 
  digitalWrite(PORT_RST, HIGH); // Enable PCA9536 / PCA9539
  delay(100);

  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH); 
  pinMode(SD_chipSelect, OUTPUT); digitalWrite(SD_chipSelect, HIGH); 
  pinMode(DSP_CS, OUTPUT); digitalWrite(DSP_CS, HIGH); 
  
  // 2. Initialize I2C and Peripherals
  Wire.begin(SDA, SCL);
  delay(100);

  if (io.begin() == false) {
    Serial.println("PCA9536 not detected!");
  } else {
    io.pinMode(IO_PB1, INPUT);
    io.pinMode(IO_LED1, OUTPUT);
    io.pinMode(IO_LED2, OUTPUT);
    io.pinMode(IO_PB3, INPUT);

    // Initial state: Both LEDs OFF (HIGH)
    io.digitalWrite(IO_LED1, HIGH);
    io.digitalWrite(IO_LED2, HIGH);
  }

  // 3. Initialize ADS1115
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
  // 1. Read Channel 1 (AIN1) from ADS1115
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  float volts = ads1.computeVolts(adc0);
  float mA = volts * 10.0;

  // 2. Update TFT Dashboard Smoothly
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    updateUI(mA);
  }

  // 3. Handle Physical Front Panel LEDs (Active LOW: LOW = ON, HIGH = OFF)
  if (systemState == 3) {
    // ERROR STATE: Blink both LEDs continuously
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = millis();
      blinkState = !blinkState;
      io.digitalWrite(IO_LED1, blinkState ? HIGH : LOW);
      io.digitalWrite(IO_LED2, blinkState ? HIGH : LOW);
    }
  } else if (systemState == 1) {
    // LOW PRESSURE: Turn on ONLY LED 1 (LED 1 = LOW, LED 2 = HIGH)
    io.digitalWrite(IO_LED1, LOW);
    io.digitalWrite(IO_LED2, HIGH);
  } else if (systemState == 2) {
    // HIGH PRESSURE: Turn on ONLY LED 2 (LED 1 = HIGH, LED 2 = LOW)
    io.digitalWrite(IO_LED1, HIGH);
    io.digitalWrite(IO_LED2, LOW);
  } else {
    // NORMAL PRESSURE: Both LEDs OFF (Both HIGH)
    io.digitalWrite(IO_LED1, HIGH);
    io.digitalWrite(IO_LED2, HIGH);
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

  // --- DRAW STATIC GAUGE ELEMENTS (Rotated 90 deg counter-clockwise) ---
  
  // 0 to 3 Bar Zone (135 to 216 degrees)
  tft.fillArc(120, 160, 82, 85, 135, 216, TFT_YELLOW);
  
  // 3 to 7 Bar Zone (216 to 324 degrees)
  tft.fillArc(120, 160, 82, 85, 216, 324, tft.color565(0, 200, 0)); // Green
  
  // 7 to 10 Bar Zone (324 to 405 degrees)
  tft.fillArc(120, 160, 82, 85, 324, 405, TFT_RED);
  
  // Empty grey track
  tft.fillArc(120, 160, 60, 79, 135, 405, TFT_DARKGREY);

  // Draw Tick Marks Labels 
  tft.setTextColor(TFT_WHITE);
  tft.setFont(&fonts::FreeSans9pt7b);
  
  tft.setTextDatum(middle_center);
  tft.drawString("0", 55, 235); 
  tft.drawString("10", 185, 235); 
}

void updateUI(float mA) {
  float pressure = 0.0;
  bool isError = false;
  String statusText;
  uint16_t statusColor;
  uint16_t barColor;

  // 1. EVALUATE SENSOR LOGIC & SET SYSTEM STATE
  if (mA < 3.8 || mA > 20.2) {
    isError = true;
    systemState = 3; // ERROR
    statusText = "ERROR";
    statusColor = TFT_RED;
    barColor = TFT_DARKGREY; 
  } else {
    // Math for 0-10 Bar (4mA = 0, 20mA = 10)
    pressure = ((mA - 4.0) / 16.0) * 10.0; 
    
    if (pressure < 0.0) pressure = 0.0;
    if (pressure > 10.0) pressure = 10.0;

    // Logic for 0-3 / 3-7 / 7-10 thresholds
    if (pressure < 3.0) {
      systemState = 1; // LOW
      statusText = "Low Pressure";
      statusColor = TFT_ORANGE;
      barColor = TFT_ORANGE;
    } else if (pressure <= 7.0) {
      systemState = 0; // NORMAL
      statusText = "Normal Pressure";
      statusColor = tft.color565(0, 200, 0); 
      barColor = tft.color565(0, 255, 0);  
    } else {
      systemState = 2; // HIGH
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
      
      tft.fillArc(120, 160, 60, 79, 135, 405, TFT_DARKGREY); 
    } else {
      char buf[10];
      sprintf(buf, "%4.2f", pressure);
      tft.setFont(&fonts::FreeSansBold18pt7b);
      tft.drawString(buf, 120, 150);
      
      tft.setFont(&fonts::FreeSans9pt7b);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); 
      tft.drawString("Bar", 120, 185);

      // Angle calculation for rotated speedometer gauge
      float targetAngle = 135.0 + ((pressure / 10.0) * 270.0);

      tft.fillArc(120, 160, 60, 79, 135, targetAngle, barColor);
      tft.fillArc(120, 160, 60, 79, targetAngle, 405, TFT_DARKGREY);
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