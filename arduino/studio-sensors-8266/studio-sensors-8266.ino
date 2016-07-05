/*
  MQTT Connected Studio-sensors and thermostat control
*/
#include <ESP8266WiFi.h>
#include <RCSwitch.h>
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <time.h>

IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
unsigned int localPort = 2390;      // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

//Time since Unix epoch
unsigned long epoch = 0;

// Connect to the WiFi
const char* ssid[] = {  "wifi-1", "wifi-2", };

const char* password = "-------";

WiFiClient espClient;
PubSubClient client(espClient);

#define RF_TX_PIN D3 /*24*/
#define DHTPIN D4 /*26*/

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define DHT_READ_INTERVAL 10000
#define CONTROL_INTERVAL 10000
#define LCD_INTERVAL 2000
#define HEATER_CHANGE_INTERVAL 60000
#define HEATER_TX_INTERVAL 1000
#define TEMP_ANALYSIS_INTERVAL 60000
#define STATUS_UPDATE_INTERVAL 20000
#define NTP_UPDATE_INTERVAL 10000

/*
 * Bits for publishing status updates
 */
#define PUB_ALL 0xffff
#define PUB_TEMP1     0x0001
#define PUB_HUMIDITY1 0x0002
#define PUB_ROC1      0x0004
#define PUB_THERM_CTL 0x0010
#define PUB_THERM_SP0 0x0020
#define PUB_THERM_SP1 0x0040
#define PUB_DISP0     0x0080
#define PUB_HEATER0   0x0100
#define PUB_HEATER1   0x0200
#define PUB_TIME      0x1000


void sendStatus(unsigned int bits);
void sendStatus();

/*
   Codes to send to the remote sockets (24-bit binary)
*/
#define HEATER_0_ON   "110111111000011010001111"
#define HEATER_0_OFF  "110111111000011010001110"
#define HEATER_1_ON   "110111111000011010000111"
#define HEATER_1_OFF  "110111111000011010000110"

DHT dht(DHTPIN, DHTTYPE);
RCSwitch mySwitch = RCSwitch();

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

byte macaddr[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x66};
byte ip4addr[4] = {192, 168, 1, 174};
IPAddress server(192, 168, 1, 127);

float currentTemp = -100.0;
float currentHumidity = -100.0;
float setPointTemp0 = 25.0;
float setPointTemp1 = 21.0;
int thermostatOn = 0;

double tempChangeRate = 0.0; // In degrees per min

long lastReconnectAttempt = 0; // Mqtt
byte lcdBacklight;
byte lcdScreen = 0; // Which screen to display

// Heater State values
#define HEATER_OFF 0
#define HEATER_ON  1
#define HEATER_UNKNOWN 2

class Heater
{
    String id;
    
    // Config
    char *onCode;
    char *offCode;
    long  minChangeInterval = HEATER_CHANGE_INTERVAL; // How often the heater can go from on->off off->on
    long  minTxInterval = HEATER_TX_INTERVAL; // How often we retranmit state to the heater

    // State variables
    unsigned long lastChangeTick; // Tick when last state change occured;
    int state; // Current State
    int nextState; // Requested State
    unsigned long lastTxTick; // Tick when last transmitted to heater

  public:
    Heater(char *on, char *off, char *heaterid)
    {
      onCode = on;
      offCode = off;
      state = HEATER_UNKNOWN;     // Initial state is off
      nextState = HEATER_UNKNOWN;
      lastChangeTick = 0;
      lastTxTick = 0;
      id = (String)heaterid;
    }

    void TurnOn(bool immediate = false)
    {
      nextState = HEATER_ON;
      if (immediate)
        state = nextState;
       Serial.print(("heater TurnOn:"));
       Serial.println(id);
    }

    void TurnOf(bool immediate = false)
    {
      nextState = HEATER_OFF;
      if (immediate)
        state = nextState;
       Serial.print(("heater TurnOff:"));
       Serial.println(id);        
    }

    int getState()
    {
      return state;
    }

    void Update()
    {
      if (nextState == HEATER_UNKNOWN) {
        return;
      }

      if ((millis() - lastChangeTick >= minChangeInterval) || state == HEATER_UNKNOWN) {
        if (state != nextState) {
          Serial.println(("Heater- changing state"));
          // If we got here, we are changing heater state
          state = nextState;
          lastChangeTick = millis();
        }
      }

      if (millis() - lastTxTick >= minTxInterval) {
        lastTxTick = millis();
        
        switch (state) {
          case HEATER_ON:
            mySwitch.send(onCode);
            break;
	    
          case HEATER_OFF:
            mySwitch.send(offCode);
            break;
	    
          default:
            Serial.println(("Heater- state not expected"));
        }
        delay(1000);
      }
    }
};

Heater heatZero(HEATER_0_ON, HEATER_0_OFF, "heater 0");
Heater heatOne(HEATER_1_ON, HEATER_1_OFF, "heater 1");

/*
  Here are the topics that we publish to or subscribe to:
 
  Pub   studio/sensor/1/temperature
  Pub   studio/sensor/1/humidity
  Subs  studio/thermostat/control
  Subs  studio/thermostat/sp/0
  Subs  studio/thermostat/sp/1
  Subs  studio/display/0
  Subs  studio/heater/0
  Subs  studio/heater/1
*/

/*
 * Reconnect to the MQTT broker
 * On connection, subscribe to our topics
 */
boolean reconnect() {
  if (client.connect("studioSensors")) {
    client.subscribe("studio/thermostat/#");
    client.subscribe("studio/heater/#");
    client.subscribe("studio/display/0");
  }
  return client.connected();
}

/*
 * LCD Display callback
 */
void display_cb(char* topic, String* payload, unsigned int length)
{
  Serial.println(("display_cb"));
  lcdBacklight = payload->toInt();
  if (lcdBacklight == 0) {
    lcd.noBacklight();
  } else {
    lcd.backlight();
  }
  sendStatus(PUB_DISP0);
}

/*
 * Heater control callback - common routine
 */
void heater_cb(Heater &heater, String* payload, unsigned int length)
{
  Serial.println(("heater_cb processing"));

  int onOff = payload->toInt();

  switch (onOff) {
    case 0:
      heater.TurnOf(true);
      break;
    case 1:
      heater.TurnOn(true);
      break;
  }
}

/*
 * Callback for Heater 0
 */
void heaterZero_cb(char* topic, String* payload, unsigned int length)
{
  Serial.println(("heaterZero_cb"));
  heater_cb(heatZero, payload, length);
  sendStatus(PUB_HEATER0);
}

/*
 * Callback for heater 1
 */
void heaterOne_cb(char* topic, String* payload, unsigned int length)
{
  Serial.println(("heaterOne_cb"));
  heater_cb(heatOne, payload, length);
  sendStatus(PUB_HEATER1);
}

/*
 * Call back for thermostat control
 */
void thermostat_ctl_cb(char* topic, String* payload, unsigned int length)
{
  Serial.println(("thermostat_ctl_cb"));
  thermostatOn = payload->toInt();
  // When control is turned off, also turn off heaters
  if (!thermostatOn) {
    heatZero.TurnOf(true);
    heatOne.TurnOf(true);
  }
  sendStatus(PUB_THERM_CTL | PUB_HEATER0 | PUB_HEATER1);
}

/*
 * Call back for thermostat Set Point control - common routine
 */
void thermostat_sp_cb(float &theSP, String* payload, unsigned int length)
{
  Serial.println(("thermostat_sp_cb"));

  float sp = payload->toFloat();

  if (sp > 1.0 && sp < 30.0) {
    theSP = sp;
    Serial.println(("thermostat_sp_cb updated"));
  }
}

/*
 * Call back for thermostat Set Point 0
 */
void thermostat_sp0_cb(char* topic, String* payload, unsigned int length)
{
  Serial.println(("thermostat_sp0_cb"));
  thermostat_sp_cb(setPointTemp0, payload, length);
  sendStatus(PUB_THERM_SP0);
}


/*
 * Call back for thermostat Set Point 1
 */
void thermostat_sp1_cb(char* topic, String* payload, unsigned int length)
{
  Serial.println(("thermostat_sp1_cb"));
  thermostat_sp_cb(setPointTemp1, payload, length);
  sendStatus(PUB_THERM_SP1);
}

/*
 * Here we implement a registration system for callbacks that
 * are needed when we receive an update from the MQTT broker on
 * one of the topics we are subscribing to
 */

struct cb_struct {
  String registeredTopic;
  void (*callback)(char* , String* , unsigned int );
};

struct cb_struct cbacks[8];
byte num_cbacks = 0;

void register_cb(void (*callback)(char *, String *, unsigned int), char *topic)
{
  cbacks[num_cbacks].registeredTopic = topic;
  cbacks[num_cbacks].callback = callback;
  num_cbacks++;
}

void register_studio_cbs()
{
  register_cb(display_cb, "studio/display/0");
  register_cb(heaterZero_cb, "studio/heater/0");
  register_cb(heaterOne_cb, "studio/heater/1");
  register_cb(thermostat_ctl_cb, "studio/thermostat/control");
  register_cb(thermostat_sp0_cb, "studio/thermostat/sp/0");
  register_cb(thermostat_sp1_cb, "studio/thermostat/sp/1");
}

void callback(char* topic, byte* payload, unsigned int length)
{
  // handle message arrived
  Serial.print(("Message arrived ["));
  Serial.print(topic);
  Serial.print(("] "));

  String inStr;

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    inStr += (char)payload[i];
  }

  Serial.println();

  for (int i = 0; i < num_cbacks; i++) {
    if (cbacks[i].registeredTopic == topic) {
      //      Serial.println("Match");
      (cbacks[i].callback)(topic, &inStr, length);
      return;
    }
  }
}

/*
 * Regular Arduino IDE sketch setup() routine
 */
void setup() {

  Serial.begin(9600);

  Wire.begin(D6,D7); // We talk i2c over these Pins

  // Initialise the 433MHz transmitter that we use to communicate
  // with the heaters

  // Transmitter is connected to RF_TX_PIN
  mySwitch.enableTransmit(RF_TX_PIN);

  // Optional set pulse length.
  mySwitch.setPulseLength(215);

  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set number of transmission repetitions.
  mySwitch.setRepeatTransmit(5);

  // Initialise the DHT-22 temperature and humidity sensor
  dht.begin();

  // Initialise the MQTT broker services 
  client.setServer(server, 1883);
  client.setCallback(callback);

  // Register handlers
  register_studio_cbs();

  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcdBacklight = 1;

  lcd.print(("RRP Studio"));

  // UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
}

/*
 * Read the DHT sensor, obtain temperature and humidity
 */
void readDHT()
{
  static unsigned long lastRan = 0;
  static bool initialised = false;

  if ((millis() < lastRan + DHT_READ_INTERVAL) && initialised) {
    return;
  }
  lastRan = millis();
  initialised = true;

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) ) {
    Serial.println(("Failed to read from DHT sensor!"));
    return;
  }

  if (h != currentHumidity)
    sendStatus(PUB_HUMIDITY1);

  if (t != currentTemp)
    sendStatus(PUB_TEMP1);

  currentHumidity = h;
  currentTemp = t;
}

/*
 * This is our temperature control code. It's rather simple.
 *
 * If temperature is less than set point 0, turn on heater 0
 * If temperature is less than set point 1, turn on heater 1
 *
 */
void controlTemp()
{
  static unsigned long lastRan = 0;
  static bool initialised = false;

  if ((millis() < lastRan + CONTROL_INTERVAL) && initialised) {
    return;
  }
  lastRan = millis();
  initialised = true;

  Serial.println(("controlTemp:"));

  Serial.print(millis());
  Serial.print(("\t"));
  Serial.print(("Temperature: "));
  Serial.print(currentTemp);
  Serial.print(("\t"));
  Serial.print(("SP0: "));
  Serial.print(setPointTemp0);
  Serial.print(("\t"));
  Serial.print(("SP1: "));
  Serial.print(setPointTemp1);
  Serial.println((""));

  // Control
  if (currentTemp < -90.0) {
    Serial.println(("controlTemp: Not init"));
    return; // Initial value not been read from sensor
  }

  if (currentTemp >= setPointTemp0) {
    Serial.println(("controlTemp: SP0 - At temp - Off"));
    heatZero.TurnOf();
  } else {
    Serial.println(("controlTemp: SP0 - Below - On"));
    heatZero.TurnOn();
  }

  if (currentTemp >= setPointTemp1) {
    Serial.println(("controlTemp: SP1 - At temp - Off"));
    heatOne.TurnOf();
  } else {
    Serial.println(("controlTemp: SP1 - Below - On"));
    heatOne.TurnOn();
  }
  sendStatus(PUB_HEATER0 | PUB_HEATER1);
}

/*
 * Send our status to the MQTT broker
 * 
  studio/sensor/1/temperature
  studio/sensor/1/humidity
  studio/thermostat/control
  studio/thermostat/sp/0
  studio/thermostat/sp/1
  studio/display/0
  studio/heater/0
  studio/heater/1
 */
void sendStatus(unsigned int bits)
{
  static char buf[20];

  if (bits & PUB_TEMP1) {
    dtostrf(currentTemp, -5, 1, buf);
    client.publish("studio/sensor/1/temperature", buf, true);
  }
  if (bits & PUB_HUMIDITY1) {
    dtostrf(currentHumidity, -5, 1, buf);
    client.publish("studio/sensor/1/humidity", buf, true);
  }
  if (bits & PUB_ROC1) {
  dtostrf(tempChangeRate, -5, 3, buf);
  client.publish("studio/sensor/1/changeRate", buf, true);
  }

  if (bits & PUB_THERM_CTL) {
    client.publish("studio/state/thermostat/control", itoa(thermostatOn, buf, 10), true);

  }
  if (bits & PUB_THERM_SP0) {
    dtostrf(setPointTemp0, -5, 1, buf);    
    client.publish("studio/state/thermostat/sp/0", buf, true);
  }
  if (bits & PUB_THERM_SP1) {
    dtostrf(setPointTemp1, -5, 1, buf);    
    client.publish("studio/state/thermostat/sp/1", buf, true);
  }
  if (bits & PUB_DISP0) {
    client.publish("studio/state/display/0", itoa(lcdBacklight, buf, 10), true);
  }
  if (bits & PUB_HEATER0) {
    client.publish("studio/state/heater/0", itoa(heatZero.getState(), buf, 10), true);
  }
  if (bits & PUB_HEATER1) {
    client.publish("studio/state/heater/1", itoa(heatOne.getState(), buf, 10), true);
  }
  if (bits & PUB_TIME) {
    client.publish("studio/state/time/0", itoa(epoch, buf, 10), true);
  }
}

void sendStatus()
{
  static unsigned long lastRan = 0;
  
  if (millis() < lastRan + STATUS_UPDATE_INTERVAL) {
    return;
  }
  lastRan = millis();

  sendStatus(PUB_ALL);
}

void lcdscreen0()
{
    
  // Update our LCD information
  lcd.clear();
  lcd.setCursor(0, 0);
  if (thermostatOn == 1) {
    lcd.print("T:");
  } else {
    lcd.print("t:");
  }
  lcd.print(currentTemp, 1);

  lcd.setCursor(8, 0);
  lcd.print("R:");
  lcd.print(tempChangeRate, 3);

  lcd.setCursor(0, 1);
  lcd.print("S0:");
  lcd.print(setPointTemp0, 1);
  lcd.setCursor(8, 1);
  lcd.print("S1:");
  lcd.print(setPointTemp1, 1);
}

// Display status - ethernet etc
void lcdscreen1()
{
  // Update our LCD information
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP:");
  //lcd.print(Ethernet.localIP());
  lcd.print(espClient.localIP());

  lcd.setCursor(0,1);
  lcd.print("MQ:");
  lcd.print(client.connected());
}

// Display temp etc
void updateLCD()
{
  static unsigned long lastRan = 0;

  if (millis() < lastRan + LCD_INTERVAL) {
    return;
  }
  lastRan = millis();

  switch(lcdScreen) {
    case 0:
      lcdscreen0();
      break;
    case 1:
      lcdscreen1();
      break;
    default:
      break;
  }
  lcdScreen++;
  if (lcdScreen > 1)
    lcdScreen = 0;
}

/*
 * analyseTemp - this calculates a rolling average of the rate of
 * change of temperature. 
 */
void analyseTemp()
{
  static unsigned long lastRan = 0;
  static bool initialised = false;
  static float lastTemp;

  double thisTempChange;

  if ((millis() < lastRan + TEMP_ANALYSIS_INTERVAL) && initialised) {
    return;
  }
  lastRan = millis();

  if (!initialised && currentTemp > -20.0) {
    lastTemp = currentTemp;
    initialised = true;
    return;
  }

  // Update our Temp Analysis information. We store the rate of change of
  // temperature. degC/minute. Use rolling average over a 10 minute period
  //
  // tempChangeRate contains this value
  //
  thisTempChange = currentTemp - lastTemp;

  tempChangeRate = ((tempChangeRate * 9) + thisTempChange) / 10;

  Serial.print(("tempChangeRate = "));
  Serial.print(tempChangeRate);
  Serial.println((" C"));

  lastTemp = currentTemp;

  sendStatus(PUB_ROC1);
}

/*
 * Here we use NTP to get the current time. This value is sent to the
 * MQTT broker, eventually it will be used to update an on-board RTC, but
 * for now it just shows that we are alive and kicking
 */
// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void ntpUpdate()
{
  static unsigned long lastRan = 0;

  if (millis() < lastRan + NTP_UPDATE_INTERVAL) {
    return;
  }
  lastRan = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ntpUpdate: Not connected");
    return;
  }


  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("ntpUpdate: no packet yet");
    //get a random server from the pool
    WiFi.hostByName(ntpServerName, timeServerIP); 
  
    sendNTPpacket(timeServerIP); // send an NTP packet to a time server   
    Serial.println("ntpUpdate: Request sent");    
  }
  else {
    Serial.print("ntpUpdate: packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;

    sendStatus(PUB_TIME);
    
    // print Unix time:
    Serial.println(epoch);
  }
}

void loop() {
  static long lastBrokerReconnectAttempt = 0;
  static long lastWifiAttempt = 0;
  static long ssid_idx = 0;

  // If we aren't connected to a Wifi network, connect
  if (WiFi.status() != WL_CONNECTED) {
    
    // If we've been going for a long time, restart the attempt
    long now = millis();

    if (now - lastWifiAttempt > 10000) {
      lastWifiAttempt = now;
      Serial.print("Connecting to ");
      ssid_idx++; if (ssid_idx > 1) ssid_idx = 0;
      Serial.println(ssid[ssid_idx]);
      WiFi.begin(ssid[ssid_idx], password);
    }
  }

  // If we are connected to Wifi, get connected to the MQTT broker
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      long now = millis();
      if (now - lastBrokerReconnectAttempt > 5000) {
        lastBrokerReconnectAttempt = now;
        // Attempt to reconnect
        if (reconnect()) {
          lastBrokerReconnectAttempt = 0;
          Serial.print("Local IP:");
          Serial.println(espClient.localIP());  
        }
      }
    } else {
      // Client connected, run our networking code
      
      client.loop();
    } 
  }

  // Update temp & humidity
  readDHT();

  // Temperature control
  if (thermostatOn)
    controlTemp();

  // Heater control
  heatZero.Update();
  heatOne.Update();

  analyseTemp();

  updateLCD();

  ntpUpdate();
}

