#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
#include <Sds011.h>
#include "LoraMessage.h"

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"
#include<Wire.h>
#include <SparkFunHTU21D.h>
HTU21D myHumidity;

#include <TinyGPS++.h>
#include <HardwareSerial.h>


#define GPS_TX 12
#define GPS_RX 15

HardwareSerial GPSSerial(1);
TinyGPSPlus tGps;

#define SDS_PIN_RX 13
#define SDS_PIN_TX 14

SoftwareSerial serialSDS (SDS_PIN_RX, SDS_PIN_TX, false, 192);
Sds011Async< SoftwareSerial > sds011(serialSDS);

// T-Beam specific hardware
#undef BUILTIN_LED
#define BUILTIN_LED 21


constexpr int pm_tablesize = 20;
int pm25_table[pm_tablesize];
int pm10_table[pm_tablesize];

bool is_SDS_running = true;

void start_SDS() {
  Serial.println("Start wakeup SDS011");

  if (sds011.set_sleep(false)) { is_SDS_running = true; }

  Serial.println("End wakeup SDS011");
}

void stop_SDS() {
  Serial.println("Start sleep SDS011");

  if (sds011.set_sleep(true)) { is_SDS_running = false; }

  Serial.println("End sleep SDS011");
}


char s[32]; // used to sprintf for Serial output


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN, // was "14,"
  .dio = {26, 33, 32},
};

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {  

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
    int pm25;
    int pm10;
    LoraMessage message;
    
    while (GPSSerial.available() )
          {
              char data = GPSSerial.read();
              tGps.encode(data);
          }
  
    message.addLatLng(tGps.location.lat(),tGps.location.lng());
    sds011.query_data( pm25,pm10, 5);
    Serial.print("PM10: ");
    Serial.println(float(pm10) / 10);
    Serial.print("PM2.5: ");
    Serial.println(float(pm25) / 10);
    message.addUint16(pm10 * 10);
    message.addUint16(pm25 * 10);
//     
    myHumidity.begin(); 
     
    message.addTemperature(myHumidity.readTemperature());
    message.addHumidity(myHumidity.readHumidity());
    Serial.println(myHumidity.readTemperature());
    Serial.println(myHumidity.readHumidity());
     
    LMIC_setTxData2(1, message.getBytes(), message.getLength(), 0);
    Serial.println(F("Packet queued"));
    digitalWrite(BUILTIN_LED, HIGH);
    }
//    else
    {
      //try again in 3 seconds
//      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("TTN Sensor"));
  
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  serialSDS.begin(9600);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  GPSSerial.setTimeout(2);
  myHumidity.begin();
    
  Serial.println("SDS011 start/stop and reporting sample");
  String firmware_version;
  uint16_t device_id;
  if (!sds011.device_info(firmware_version, device_id)) {
    Serial.println("Sds011::firmware_version() failed");
  }
  else
  {
    Serial.print("Sds011 firmware version: ");
    Serial.println(firmware_version);
    Serial.print("Sds011 device id: ");
    Serial.println(device_id);
  }

  Sds011::Report_mode report_mode;
  if (!sds011.get_data_reporting_mode(report_mode)) {
    Serial.println("Sds011::get_data_reporting_mode() failed");
  }
  if (Sds011::REPORT_ACTIVE != report_mode) {
    Serial.println("Turning on Sds011::REPORT_ACTIVE reporting mode");
    if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
      Serial.println("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
    }
  }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14); 

  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  
}

void loop() {
    os_runloop_once();
}
