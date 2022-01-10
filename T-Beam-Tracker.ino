/* ATMS Tracker for T-Beam board */
/* AB0TJ, 2021 */

#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <Wire.h>
#include <TinyGPS++.h>
#include <axp20x.h>     // https://github.com/lewisxhe/AXP202X_Library
#include "SSD1306Wire.h"      
#include "atms.h"

TinyGPSPlus gps;
AXP20X_Class axp;
SSD1306Wire display(0x3C, SDA, SCL);  // ADDRESS, SDA, SCL

#define LORA_SCK 5    // SX1276 SCK
#define LORA_MISO 19  // SX1276 MISO
#define LORA_MOSI 27  // SX1276 MOSI
#define LORA_CS 18    // SX1276 CS
#define LORA_RST 23   // SX1276 RST
#define LORA_IRQ 26   // SX1276 IRQ (interrupt request)

const uint8_t I2C_SDA = 21;
const uint8_t I2C_SCL = 22;
const uint8_t AXP_INT = 35;
const uint16_t PWR_DOWN_MV = 3500;

const uint8_t BUTTON_PIN = 38;

const int redLED = 4; // red LED -- blink when packet sent

hw_timer_t * timer = NULL;  // Hardware timer for beacon timing
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // For synchronizing variable access
volatile uint8_t timerTick = 1;  // Used to notify the main loop that a second has passed
void IRAM_ATTR onTimer(); // Function prototype for timer tick

const char comment[] = "ATMS (433MHz LoRa) Test Tracker"; // TODO: these consts should be configurable
const char callsign[] = "AB0TJ";
const uint8_t ssid = 1;
const uint8_t icon = '"';
const uint8_t icon_var = '/';
const uint8_t txPower = 20;         // 20dBm = 100 mW
const uint32_t txFreq = 432700000;  // 432.700 MHz
const uint16_t commentRate = 3600;  // In seconds. 0 to disable

const uint16_t sb_static_rate = 0;  // 0 = use smartbeaconing
const uint16_t sb_low_rate = 900;
const uint16_t sb_high_rate = 60;
const uint16_t sb_low_speed = 15;
const uint16_t sb_high_speed = 80;
const uint16_t sb_turn_min = 30;
const uint16_t sb_turn_time = 15;
const uint16_t sb_turn_slope = 255;

uint8_t seqNum = 0; // ATMS sequence number
uint16_t beaconTimer = 0;
uint16_t commentTimer = 0;
uint16_t last_heading;

void setup()
{
  Serial.begin(115200); // USB serial
  Serial1.begin(9600, SERIAL_8N1, 34, 12);   // GPS

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

    // Very important for SPI pin & LoRa configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  pinMode(redLED, OUTPUT); // For LED feedback
  digitalWrite(redLED, HIGH); // Turn LED off
  pinMode(BUTTON_PIN, INPUT); // Middle button next to LoRa chip. The one on the right is RESET, careful...

  if (!LoRa.begin(txFreq)) {
    Serial.println("Starting LoRa failed!");
    BlinkLoop();
  }
  LoRa.setSpreadingFactor(9); // ranges from 6-12, default 7 see API docs. Changed for ver 0.1 Glacierjay
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();
  LoRa.setTxPower(txPower, PA_OUTPUT_PA_BOOST_PIN);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL)
  {
    Serial.println("Starting AXP192 interface failed!");
    BlinkLoop();
  }
  axp.setPowerDownVoltage(PWR_DOWN_MV);

  Serial.print("ATMS Tracker Ready!\r\n");

  // Send comment at startup
  SendComment();

  // Wait for GPS location to be valid and send a location beacon
  while (!gps.location.isValid())
  {
    display.clear();
    PrintHeader();       
    display.drawString(0, 13, "Waiting for GPS lock");
    display.drawString(0, 23, "Sats: " + String(gps.satellites.value()));
    display.display();
    smartDelay(1000);
  }
  SendLocation();

  timer = timerBegin(0, 80, true);  // Set up timer with a prescaler of 80 (= 1MHz)
  timerAttachInterrupt(timer, &onTimer, true);  // Run the onTimer function when alarm triggered
  timerAlarmWrite(timer, 1000000, true);  // Set the alarm to trigger once per second
  timerAlarmEnable(timer);  // Enable the alarm
}

void loop()
{
  uint16_t rate;
  uint8_t ticksPassed;

  while (Serial1.available())
      gps.encode(Serial1.read()); // Feed the GPS object if any data has come in

  // Check if a second has passed
  portENTER_CRITICAL(&timerMux);
  ticksPassed = timerTick;
  timerTick = 0;
  portEXIT_CRITICAL(&timerMux);

  if (ticksPassed > 0)
  {
    if (commentRate > 0)
    {
      commentTimer += ticksPassed;
      if (commentTimer >= commentRate)  // Time to send a comment packet
      {
        SendComment();
        commentTimer = 0;
      }
    }

    beaconTimer += ticksPassed;

    // Get the display ready for updated info
    display.clear();
    PrintHeader();
    
    if (gps.location.isValid() && gps.location.age() < 1500) {  // Don't do location things if we don't have reliable location data
      // Update beacon rate
      if (sb_static_rate > 0)
      {
        rate = sb_static_rate;  // SmartBeaconing is disabled so use static rate
      }
      else
      {
        // SmartBeaconing is enabled, so do some math.
        uint16_t spd = gps.speed.mph();
        if (spd <= sb_low_speed)
        {
          rate = sb_low_rate;
        }
        else if (spd >= sb_high_speed)
        {
          rate = sb_high_rate;
        }
        else
        {
          rate = sb_high_rate * sb_high_speed / spd;
          float turn_threshold = sb_turn_min + sb_turn_slope / spd;
          int16_t hdg_chg;
          uint16_t hdg = (gps.course.value() / 100);
          uint16_t hdg_diff = abs(hdg - last_heading);
          
          if (hdg_diff <= 180)
          {
            hdg_chg = hdg_diff;
          }
          else if (hdg > last_heading)
          {
            hdg_chg = hdg_diff - 360;
          }
          else
          {
            hdg_chg = 360 - hdg_diff;
          }
  
          if (abs(hdg_chg) > turn_threshold && beaconTimer > sb_turn_time) beaconTimer = rate;
        }
      }

      // Update display with new data
      display.drawString(0, 13, "Lat: " + String(abs(gps.location.lat()), 4) + (gps.location.rawLat().negative ? " S" : " N"));
      display.drawString(0, 23, "Lon: " + String(abs(gps.location.lng()), 4) + (gps.location.rawLng().negative ? " W" : " E"));
      display.drawString(0, 33, "Spd: " + String(gps.speed.mph(), 0) + "   Alt: " + String(gps.altitude.feet(), 0));
      uint16_t bcnIn = rate - beaconTimer;
      display.drawString(0, 43, "Sats: " + String(gps.satellites.value()) + "   Rate: " + String(rate));
      display.drawString(0, 53, "Beacon in " + String(bcnIn / 60) + "m" + String(bcnIn % 60) + "s");

      // Send a beacon if it is time
      if (beaconTimer >= rate)
      {
        SendLocation();
        beaconTimer = 0;
      }
    }
    else {
      // GPS location wasn't valid. Update the display to say so.
      display.drawString(0, 13, "Invalid GPS data.");
      display.drawString(0, 23, "Sats: " + String(gps.satellites.value()));
    }
    display.display();  // Send new data to the OLED
  }
}

void SendLocation()
{
  last_heading = gps.course.value() / 100;
  
  digitalWrite(redLED, LOW);  // Turn red LED on
  LoRa.beginPacket();

  LoRa.print(callsign);
  LoRa.write(ssid | 0x80);
  LoRa.write(seqNum++);
  LoRa.write(PKT_TYPE_POSITION);

  LoRa.write((uint8_t)gps.location.rawLat().deg);
  uint32_t millionths = gps.location.rawLat().billionths / 1000;
  uint8_t temp = (millionths >> 16) & 0x0F;
  if (gps.location.rawLat().negative) temp |= FLAG_LAT_SOUTH;
  LoRa.write(temp);
  LoRa.write((uint8_t)(millionths >> 8));
  LoRa.write((uint8_t)millionths);

  LoRa.write((uint8_t)gps.location.rawLng().deg);
  millionths = gps.location.rawLng().billionths / 1000;
  temp = (millionths >> 16) & 0x0F;
  if (gps.location.rawLng().negative) temp |= FLAG_LON_WEST;
  LoRa.write(temp);
  LoRa.write((uint8_t)(millionths >> 8));
  LoRa.write((uint8_t)millionths);

  LoRa.write(icon);
  LoRa.write(icon_var);

  LoRa.endPacket();
  digitalWrite(redLED, HIGH); // Turn red LED off
}

void SendComment()
{
  if (commentRate == 0) return;
  
  digitalWrite(redLED, LOW);  // Turn red LED on
  LoRa.beginPacket();
  LoRa.print(callsign);
  LoRa.write(ssid | 0x80);
  LoRa.write(seqNum++);
  LoRa.write(PKT_TYPE_COMMENT);
  LoRa.print(comment);
  LoRa.endPacket();
  digitalWrite(redLED, HIGH); // Turn red LED off
}

void IRAM_ATTR onTimer()  // Runs once per second
{
  portENTER_CRITICAL_ISR(&timerMux);
  timerTick++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

void PrintHeader()
{
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "ATMS: " + String(callsign) + "-" + String(ssid));
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(127, 0, String(axp.getBattVoltage() / 1000, 2) + "V");
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void BlinkLoop()
{
    while (1) {   // Blink LED to indicate error
      digitalWrite(redLED, LOW);
      delay(250);
      digitalWrite(redLED, HIGH);
      delay(250);
    }
}
