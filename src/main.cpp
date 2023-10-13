/* The example is for CubeCell_GPS,
 * GPS works only before lorawan uplink, the board current is about 45uA when in lowpower mode.
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include "GPS_Air530Z.h"
#include "HT_SSD1306Wire.h"
#include "chipIdNameLookup.cpp"
#include "version.h"
#include "font.h"
#include <Adafruit_AHTX0.h>
#include "CayenneLPP.h"
#include <Timezone.h> // https://github.com/JChristensen/Timezone

TimeChangeRule myDST = {"BST", Last, Sun, Mar, 1, +60}; // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"GMT", Last, Sun, Oct, 2, 0};   // Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
TimeChangeRule *tcr; // pointer to the time change rule, use to get TZ abbrev

#define ANSI_ESCAPE_SEQUENCE(c) "\33[" c
#define ESC_CURSOR_UP(L) ANSI_ESCAPE_SEQUENCE(#L "A")
#define ESC_ERASE_LINE ANSI_ESCAPE_SEQUENCE("K")

Adafruit_AHTX0 aht;

Air530Class GPS;
extern SSD1306Wire display;

// when gps waked, if in GPS_UPDATE_TIMEOUT, gps not fixed then into low power mode
#define GPS_UPDATE_TIMEOUT 120000

// once fixed, GPS_CONTINUE_TIME later into low power mode
#define GPS_CONTINUE_TIME 10000
/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/* OTAA para*/
uint8_t devEui[] = {0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x02};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x72, 0xFB, 0x24, 0xF9, 0x0B, 0x66, 0x84, 0x70, 0x50, 0x15, 0x77, 0xF4, 0x93, 0x13, 0x12, 0x08};
/* ABP para*/
uint8_t nwkSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr = (uint32_t)0x00000000;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 120000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

typedef struct
{
  time_t time;
  float latitude;
  float longitude;
  float altitude;
  float course;
  float speed;
  float hdop;
  int satellites;
  float temperature = 0;
  float humidity = 0;
  float batteryVoltage;
} record;

int32_t fracPart(double val, int n)
{
  return (int32_t)((val - (int32_t)(val)) * pow(10, n));
}

void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) // Vext default OFF
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}
void displayGPSInof()
{
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  int index;
  // index = sprintf(str, "%02d-%02d-%02d", GPS.date.year(), GPS.date.month(), GPS.date.day());
  // str[index] = 0;
  //
  // display.drawString(0, 0, str);
  time_t utc = now();
  time_t local = myTZ.toLocal(utc, &tcr);
  index = sprintf(str, "%02d:%02d:%02d", hour(local), minute(local), second(local));
  str[index] = 0;
  display.drawString(0, 0, str);

  display.setFont(ArialMT_Plain_10);

  index = sprintf(str, "lat :  %d.%d", (int)GPS.location.lat(), fracPart(GPS.location.lat(), 4));
  str[index] = 0;
  display.drawString(0, 32, str);

  index = sprintf(str, "lon:%d.%d", (int)GPS.location.lng(), fracPart(GPS.location.lng(), 4));
  str[index] = 0;
  display.drawString(0, 43, str);

  index = sprintf(str, "speed: %d.%d km/h", (int)GPS.speed.kmph(), fracPart(GPS.speed.kmph(), 3));
  str[index] = 0;
  display.drawString(0, 54, str);
  display.display();
}

void printGPSInof()
{
  if (GPS.date.isValid())
  {
    Serial.printf("%d/%02d/%02d", year(), month(), day());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (GPS.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d", hour(), minute(), second());
  }
  else
  {
    Serial.print(" INVALID");
  }

  Serial.print(" LAT: ");
  Serial.print(GPS.location.lat(), 6);
  Serial.print(", LON: ");
  Serial.print(GPS.location.lng(), 6);
  Serial.print(", ALT: ");
  Serial.print(GPS.altitude.meters());

  Serial.print(" SATS: ");
  Serial.print(GPS.satellites.value());
  Serial.print(", HDOP: ");
  Serial.print(GPS.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(GPS.location.age());
  Serial.print(", COURSE: ");
  Serial.print(GPS.course.deg());
  Serial.print(", SPEED: ");
  Serial.println(GPS.speed.kmph());
  Serial.print(ESC_CURSOR_UP(1));
}
void prepareTxFrame()
{
  /*appData size is LoRaWan_APP_Custom_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LoRaWan_APP_Custom_DATA_MAX_SIZE.
    if enabled AT, don't modify LoRaWan_APP_Custom_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LoRaWan_APP_Custom_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE); // or 160?

  float lat, lon, alt, course, speed, hdop, sats;
  sensors_event_t humidity, temp;

  lat = GPS.location.lat();
  lon = GPS.location.lng();
  alt = GPS.altitude.meters();
  course = GPS.course.deg();
  speed = GPS.speed.kmph();
  sats = GPS.satellites.value();
  hdop = GPS.hdop.hdop();

  uint16_t batteryVoltage = getBatteryVoltage();

  if (!aht.begin())
  {
    Serial.println("Could not find AHT? Check wiring");
  }
  else
  {
    Serial.println("AHT10 or AHT20 found");
    aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");
    Serial.print("Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println("% rH");
  }

  lpp.reset();
  lpp.addUnixTime(1, now());
  lpp.addGPS(1, lat, lon, alt);
  lpp.addDirection(1, course);
  lpp.addDistance(1, speed);
  lpp.addAnalogInput(1, hdop);
  lpp.addVoltage(1, batteryVoltage / 1000.0f);

  lpp.addTemperature(1, temp.temperature);
  lpp.addRelativeHumidity(1, humidity.relative_humidity);

  Serial.print("lpp data size: ");
  Serial.print(lpp.getSize());
  Serial.println();

  appDataSize = lpp.getSize();
  memcpy(appData, lpp.getBuffer(), appDataSize);
}

void printHex(uint8_t num)
{
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

bool GpsUpdate()
{

  uint32_t start = millis();
  while ((millis() - start) < GPS_UPDATE_TIMEOUT)
  {
    while (GPS.available() > 0)
    {
      GPS.encode(GPS.read());
    }
    // gps fixed in a second
    if (GPS.location.age() < 1000)
    {
      break;
    }
  }

  // if gps fixed,  GPS_CONTINUE_TIME later stop GPS into low power mode, and every 1 second update gps, print and display gps info
  if (GPS.location.age() < 1000)
  {
    start = millis();
    uint32_t printinfo = 0;
    while ((millis() - start) < GPS_CONTINUE_TIME)
    {
      while (GPS.available() > 0)
      {
        GPS.encode(GPS.read());
      }

      if ((millis() - start) > printinfo)
      {
        printinfo += 1000;
        printGPSInof();
        displayGPSInof();
      }
    }
  }
  else
  {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 32 - 16 / 2, "No GPS signal");
    Serial.println("No GPS signal");
    display.display();
    delay(2000);
    return false;
  }
  if (GPS.time.isValid())
  {
    //   setTime(myTZ.toUTC(compileTime()));

    setTime(GPS.time.hour(),
            GPS.time.minute(),
            GPS.time.second(),
            GPS.date.day(),
            GPS.date.month(),
            GPS.date.year());
  }

  return true;
}
void displayVersionAndName()
{
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, chipIdNameLookup());
  display.drawString(0, 18, String("FW ver: ") + VERSION);
  display.setFont(Dialog_plain_8);
  display.drawString(0, 36, BUILD_TIMESTAMP);
  display.display();

  display.clear();
}

void setup()
{
  Serial.begin(115200);
  delay(1000 * 2);
  VextON(); // oled power on;
  delay(10);
  display.init();
  display.clear();

  LoRaWAN.generateDeveuiByChipID();
  displayVersionAndName();
  delay(2000);

  Serial.print("DevEUI : ");
  int i;
  for (i = 0; i < sizeof(devEui); i++)
  {
    printHex(devEui[i]);
  }
  Serial.println();

#if (AT_SUPPORT)
  enableAt();
#endif
  LoRaWAN.displayMcuInit();
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
  display.clear();
  Serial.println("Waiting for GPS FIX ...");

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32 - 16 / 2, "GPS Searching...");
  Serial.println("GPS Searching...");
  display.display();

  GPS.begin();
}
void print_uint64_t(uint64_t num)
{

  char rev[128];
  char *p = rev + 1;

  while (num > 0)
  {
    *p++ = '0' + (num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev)
  {
    Serial.print(*p--);
  }
}

void loop()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
#if (LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
#endif
#if (AT_SUPPORT)
    getDevParam();
#endif
    printDevParam();
    LoRaWAN.init(loraWanClass, loraWanRegion);
    deviceState = DEVICE_STATE_JOIN;
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    display.clear();
    LoRaWAN.displayJoining();
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    prepareTxFrame();
    LoRaWAN.displaySending();
    LoRaWAN.send();
    txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_CYCLE;
    // LoRaWan duty cycle timer will set state to DEVICE_STATE_SEND
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    // Schedule next packet transmission
    txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    GpsUpdate();
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}
