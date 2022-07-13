/**
  ESP8266 Parking sensor firmware for the Open eXtensible Rack System
  
  GitHub repository:
    https://github.com/sumnerboy12/OXRS-BJ-ParkingSensor-ESP-FW
    
  Copyright 2022 Ben Jones <ben.jones12@gmail.com>
*/

/*--------------------------- Macros ----------------------------------*/
#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

/*--------------------------- Libraries -------------------------------*/
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <OXRS_MQTT.h>
#include <OXRS_API.h>
#include <MqttLogger.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

/*--------------------------- Constants -------------------------------*/
// Serial
#define     SERIAL_BAUD_RATE          115200

// REST API
#define     REST_API_PORT             80

// Sensor config
#define     MAX_DISTANCE_CM           300 // max sensor distance is rated at 400-500cm
#define     PING_DELAY_MS             40
#define     PING_AVERAGE_MAX          20
#define     PUBLISH_THRESHOLD_CM      5
#define     TIMEOUT_THRESHOLD_CM      10
#define     TIMEOUT_COUNT             1000

// Hardware pins
#define     RGB_PIN                   4   // D2
#define     ECHO_PIN                  14  // D5
#define     TRIGGER_PIN               12  // D6

// RGB LED
#define     FLASH_PERIOD_MS           200

// State variables, for debugging via serial only
#define     TIMED_OUT                 1
#define     NO_CAR                    2
#define     CAR_DETECTED              3
#define     CAR_APPROACHING           4
#define     CAR_CLOSE                 5
#define     CAR_PARKED                6
#define     CAR_TOO_CLOSE             7

/*--------------------------- Global Variables ------------------------*/
// debugging mode
bool debugEnabled     = false;

// ping average
int pingAverage       = 5;

// ignore readings greater than this
int maxDistance       = 250;

// parking thresholds for the various notifications
int parkDistance      = 110;      // initialise to something sensible
int parkRange         = 15;       // initialise to something sensible
int parkUpper         = 0;             
int parkLower         = 0;
int parkApproach      = 0;
int parkState         = 0;

// keep track of distances to identify when car is parked
int pingAverager[ PING_AVERAGE_MAX ] = {};
int previous          = 0;
int timeout           = 0;
int lastPublish       = 0;
int parked            = -1;

// flash timer
uint32_t lastFlashMs = 0;
bool lastFlashState = false;

// stack size counter
char * stackStart;

/*--------------------------- Instantiate Globals ---------------------*/
// WiFi client
WiFiClient _client;

// MQTT client
PubSubClient _mqttClient(_client);
OXRS_MQTT _mqtt(_mqttClient);

// REST API
WiFiServer _server(REST_API_PORT);
OXRS_API _api(_mqtt);

// Logging
MqttLogger _logger(_mqttClient, "log", MqttLoggerMode::MqttAndSerial);

// RGB LED
Adafruit_NeoPixel _pixels = Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Distance sonar
NewPing _sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);

/*--------------------------- Program ---------------------------------*/
uint32_t getStackSize()
{
  char stack;
  return (uint32_t)stackStart - (uint32_t)&stack;  
}

void getFirmwareJson(JsonVariant json)
{
  JsonObject firmware = json.createNestedObject("firmware");

  firmware["name"] = FW_NAME;
  firmware["shortName"] = FW_SHORT_NAME;
  firmware["maker"] = FW_MAKER;
  firmware["version"] = STRINGIFY(FW_VERSION);

#if defined(FW_GITHUB_URL)
  firmware["githubUrl"] = FW_GITHUB_URL;
#endif
}

void getSystemJson(JsonVariant json)
{
  JsonObject system = json.createNestedObject("system");

  system["heapUsedBytes"] = getStackSize();
  system["heapFreeBytes"] = ESP.getFreeHeap();
  system["flashChipSizeBytes"] = ESP.getFlashChipSize();

  system["sketchSpaceUsedBytes"] = ESP.getSketchSize();
  system["sketchSpaceTotalBytes"] = ESP.getFreeSketchSpace();

  FSInfo fsInfo;
  SPIFFS.info(fsInfo);
  
  system["fileSystemUsedBytes"] = fsInfo.usedBytes;
  system["fileSystemTotalBytes"] = fsInfo.totalBytes;
}

void getNetworkJson(JsonVariant json)
{
  byte mac[6];
  WiFi.macAddress(mac);
  
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  JsonObject network = json.createNestedObject("network");

  network["mode"] = "wifi";
  network["ip"] = WiFi.localIP();
  network["mac"] = mac_display;
}

void getConfigSchemaJson(JsonVariant json)
{
  JsonObject configSchema = json.createNestedObject("configSchema");
  
  // Config schema metadata
  configSchema["$schema"] = JSON_SCHEMA_VERSION;
  configSchema["title"] = FW_SHORT_NAME;
  configSchema["type"] = "object";

  JsonObject properties = configSchema.createNestedObject("properties");

  JsonObject debug = properties.createNestedObject("debug");
  debug["type"] = "boolean";

  JsonObject pingAverage = properties.createNestedObject("pingAverage");
  pingAverage["type"] = "integer";
  pingAverage["minimum"] = 1;
  pingAverage["maximum"] = PING_AVERAGE_MAX;

  JsonObject maxDistance = properties.createNestedObject("maxDistance");
  maxDistance["type"] = "integer";

  JsonObject parkDistance = properties.createNestedObject("parkDistance");
  parkDistance["type"] = "integer";

  JsonObject parkRange = properties.createNestedObject("parkRange");
  parkRange["type"] = "integer";
}

// calculate our thresholds etc
void updateThresholds()
{
  // update the various thresholds
  parkUpper     = parkDistance + (1 * parkRange);
  parkLower     = parkDistance - (1 * parkRange);
  parkApproach  = parkDistance + (3 * parkRange);

  // debugging
  _logger.print(F("[park] park distance: "));
  _logger.print(parkDistance);
  _logger.print(F("cm; range +/-"));
  _logger.print(parkRange);
  _logger.print(F("cm; (upper "));
  _logger.print(parkUpper);
  _logger.print(F("cm; lower "));
  _logger.print(parkLower);
  _logger.print(F("cm; approach "));
  _logger.print(parkApproach);
  _logger.println(F("cm)"));
}

bool getFlashState()
{
  if (millis() - lastFlashMs >= FLASH_PERIOD_MS)
  {
    lastFlashState = !lastFlashState;
    lastFlashMs = millis();
  }
  
  return lastFlashState;
}

void setRGB(int r, int g, int b)
{
  _pixels.setPixelColor(0, _pixels.Color(r, g, b));
  _pixels.show();
}

void off()         { setRGB(0,   0,   0  ); }
void red()         { setRGB(255, 0,   0  ); }
void green()       { setRGB(0,   255, 0  ); }
void blue()        { setRGB(0,   0,   255); }

void flashRed()
{
  if (getFlashState()) red(); else off(); 
}

void flashGreen()
{
  if (getFlashState()) green(); else off(); 
}

void flashBlue()
{
  if (getFlashState()) blue(); else off(); 
}

/**
  API callbacks
*/
void _apiAdopt(JsonVariant json)
{
  // Build device adoption info
  getFirmwareJson(json);
  getSystemJson(json);
  getNetworkJson(json);
  getConfigSchemaJson(json);
}

/**
  MQTT callbacks
*/
void _mqttConnected()
{
  // MqttLogger doesn't copy the logging topic to an internal
  // buffer so we have to use a static array here
  static char logTopic[64];
  _logger.setTopic(_mqtt.getLogTopic(logTopic));

  // Publish device adoption info
  DynamicJsonDocument json(JSON_ADOPT_MAX_SIZE);
  _mqtt.publishAdopt(_api.getAdopt(json.as<JsonVariant>()));

  // Log the fact we are now connected
  _logger.println("[park] mqtt connected");
}

void _mqttDisconnected(int state) 
{
  // Log the disconnect reason
  // See https://github.com/knolleary/pubsubclient/blob/2d228f2f862a95846c65a8518c79f48dfc8f188c/src/PubSubClient.h#L44
  switch (state)
  {
    case MQTT_CONNECTION_TIMEOUT:
      _logger.println(F("[park] mqtt connection timeout"));
      break;
    case MQTT_CONNECTION_LOST:
      _logger.println(F("[park] mqtt connection lost"));
      break;
    case MQTT_CONNECT_FAILED:
      _logger.println(F("[park] mqtt connect failed"));
      break;
    case MQTT_DISCONNECTED:
      _logger.println(F("[park] mqtt disconnected"));
      break;
    case MQTT_CONNECT_BAD_PROTOCOL:
      _logger.println(F("[park] mqtt bad protocol"));
      break;
    case MQTT_CONNECT_BAD_CLIENT_ID:
      _logger.println(F("[park] mqtt bad client id"));
      break;
    case MQTT_CONNECT_UNAVAILABLE:
      _logger.println(F("[park] mqtt unavailable"));
      break;
    case MQTT_CONNECT_BAD_CREDENTIALS:
      _logger.println(F("[park] mqtt bad credentials"));
      break;      
    case MQTT_CONNECT_UNAUTHORIZED:
      _logger.println(F("[park] mqtt unauthorised"));
      break;      
  }
}

void _mqttCallback(char * topic, byte * payload, int length)
{
  // Pass down to our MQTT handler
  _mqtt.receive(topic, payload, length);
}

void _mqttConfig(JsonVariant json)
{
  bool update = false;
  
  if (json.containsKey("debug"))
  {
    debugEnabled = json["debug"].as<bool>();
  }

  if (json.containsKey("pingAverage"))
  {
    pingAverage = min(json["pingAverage"].as<int>(), PING_AVERAGE_MAX);
  }

  if (json.containsKey("maxDistance"))
  {
    maxDistance = json["maxDistance"].as<int>();
  }

  if (json.containsKey("parkDistance"))
  {
    parkDistance = json["parkDistance"].as<int>();
    update = true;
  }

  if (json.containsKey("parkRange"))
  {
    parkRange = json["parkRange"].as<int>();
    update = true;
  }

  if (update){ updateThresholds(); }
}

void publishParkState()
{
  // ignore timeout state changes - since we want to keep the last state
  if (parkState == TIMED_OUT)
    return;
    
  StaticJsonDocument<128> json;
  json["state"] = parkState;

  switch (parkState)
  {
    case NO_CAR:
      json["description"] = "No car detected";
      json["parked"] = false;
      break;
    case CAR_DETECTED:
      json["description"] = "Car detected";
      json["parked"] = false;
      break;
    case CAR_APPROACHING:
      json["description"] = "Car approaching";
      json["parked"] = false;
      break;
    case CAR_CLOSE:
      json["description"] = "Getting close, slow down";
      json["parked"] = true;
      break;
    case CAR_PARKED:
      json["description"] = "Car is parked";
      json["parked"] = true;
      break;
    case CAR_TOO_CLOSE:
      json["description"] = "Too close, backup!";
      json["parked"] = true;
      break;
    default:
      json["description"] = "Unknown state";
      json["parked"] = false;
      break;
  }
  
  _mqtt.publishStatus(json.as<JsonVariant>());
}

void updateParkState(int newState)
{
  // always set the colour for the new state (regardless if the same or not)
  switch (newState)
  {
    case NO_CAR:          blue();       break;
    case CAR_DETECTED:    flashGreen(); break;
    case CAR_APPROACHING: flashGreen(); break;
    case CAR_CLOSE:       flashGreen(); break;
    case CAR_PARKED:      red();        break;
    case CAR_TOO_CLOSE:   flashRed();   break;
    case TIMED_OUT:       off();        break;    
  }

  // only publish state changes
  if (parkState != newState)
  {
    parkState = newState;
    publishParkState();
  }
}

long pingDistance()
{
  // wait between pings - 29ms should be the shortest delay between pings
  delay(PING_DELAY_MS);
  
  // send ping, get ping time in cm
  long cm = _sonar.ping_cm();
  // if no distance is read, set at max distance
  if (cm == 0) cm = maxDistance;
  
  // shift all values in our averager down one (and count for averaging)
  int total = 0;
  for (int i = 1; i < pingAverage; i++)
  {
    pingAverager[i - 1] = pingAverager[i];
    total = total + pingAverager[i];
  }

  // add our latest reading to the end of the list and our sub-total
  pingAverager[pingAverage - 1] = cm;
  total = total + cm;

  // calculate the average
  return total / pingAverage;  
} 

void publishDistance(int measured)
{
  // only publish distance if debugging
  if (!debugEnabled)
    return;

  // rate limit the distance pubs
  int publish = ((int)(measured / PUBLISH_THRESHOLD_CM) * PUBLISH_THRESHOLD_CM);

  // don't bother publishing anything above our max
  publish = min(publish, maxDistance);
  
  // if no change then ignore
  if (publish == lastPublish)
    return;

  StaticJsonDocument<32> json;
  json["distance"] = publish;
  _mqtt.publishTelemetry(json.as<JsonVariant>());
  
  lastPublish = publish;
}

void checkDistance(int measured) 
{
  // distance fluctuates resulting in false movement detection
  if (abs(measured - previous) <= TIMEOUT_THRESHOLD_CM) 
  {
    // increase timeout counter if car has not moved
    timeout++;
  } 
  else 
  {
    // reset timeout counter if there is a significant change
    timeout = 0;
  }
    
  if (timeout >= TIMEOUT_COUNT) 
  {
    // if distance hasn't changed for a while turn off LED
    updateParkState(TIMED_OUT);
    timeout = TIMEOUT_COUNT;
  }
  else
  {
    if (measured > parkUpper)
    { 
      // haven't reached parkUpper threshold yet
      if (measured >= maxDistance)
      {
        updateParkState(NO_CAR);
      } 
      else if (measured < maxDistance && measured > (parkDistance + 125))
      {
        updateParkState(NO_CAR);
      }
      else if (measured <= (parkDistance + 125) && measured > parkApproach)
      {
        updateParkState(CAR_APPROACHING);
      }
      else if (measured <= parkApproach && measured > parkUpper)
      {
        updateParkState(CAR_CLOSE);
      } 
    } 
    else if (measured < parkLower) 
    {
      // closer than lower threshold
      updateParkState(CAR_TOO_CLOSE);
    } 
    else
    { 
      // within upper/lower thresholds - in correct parking spot!
      updateParkState(CAR_PARKED);
    }
    
    // update state variable
    previous = measured;
  }
}

/**
  Initialisation
*/
void initialiseSerial()
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  
  _logger.println(F("[park] starting up..."));

  DynamicJsonDocument json(128);
  getFirmwareJson(json.as<JsonVariant>());

  _logger.print(F("[park] "));
  serializeJson(json, _logger);
  _logger.println();
}

void initialseWifi(byte * mac)
{
  // Ensure we are in the correct WiFi mode
  WiFi.mode(WIFI_STA);

  // Connect using saved creds, or start captive portal if none found
  // Blocks until connected or the portal is closed
  WiFiManager wm;  
  if (!wm.autoConnect("OXRS_WiFi", "superhouse"))
  {
    // If we are unable to connect then restart
    ESP.restart();
  }
  
  // Get ESP8266 base MAC address
  WiFi.macAddress(mac);

  // Format the MAC address for display
  char mac_display[18];
  sprintf_P(mac_display, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Display MAC/IP addresses on serial
  _logger.print(F("[park] mac address: "));
  _logger.println(mac_display);  
  _logger.print(F("[park] ip address: "));
  _logger.println(WiFi.localIP());
}

void initialiseMqtt(byte * mac)
{
  // Set the default client id to the last 3 bytes of the MAC address
  char clientId[32];
  sprintf_P(clientId, PSTR("%02x%02x%02x"), mac[3], mac[4], mac[5]);  
  _mqtt.setClientId(clientId);
  
  // Register our callbacks
  _mqtt.onConnected(_mqttConnected);
  _mqtt.onDisconnected(_mqttDisconnected);
  _mqtt.onConfig(_mqttConfig);
  
  // Start listening for MQTT messages
  _mqttClient.setCallback(_mqttCallback);  
}

void initialiseRestApi(void)
{
  // NOTE: this must be called *after* initialising MQTT since that sets
  //       the default client id, which has lower precendence than MQTT
  //       settings stored in file and loaded by the API

  // Set up the REST API
  _api.begin();

  // Register our callbacks
  _api.onAdopt(_apiAdopt);

  _server.begin();
}

void initialiseRgb()
{
  _logger.println("[park] initialising RGB LED...");
  _pixels.begin();

  _logger.println(" - red test");
  red();
  delay(1000);
  _logger.println(" - green test");
  green();
  delay(1000);
  _logger.println(" - blue test");
  blue();
  delay(1000);  
}

void initialiseSonar()
{
  // initialise our ping averager
  for (int i = 0; i < PING_AVERAGE_MAX; i++)
  {
    pingAverager[i] = 0;
  }

  // calculate our initial thresholds
  updateThresholds();
}

/**
  Setup
*/
void setup() 
{
  // Store the address of the stack at startup so we can determine
  // the stack size at runtime (see getStackSize())
  char stack;
  stackStart = &stack;
  
  // Set up serial
  initialiseSerial();  

  // Set up network and obtain an IP address
  byte mac[6];
  initialseWifi(mac);

  // initialise MQTT
  initialiseMqtt(mac);

  // Set up the REST API
  initialiseRestApi();

  // Set up the RGB
  initialiseRgb();

  // Set up the sonar
  initialiseSonar();
}

/**
  Main processing loop
*/
void loop() 
{
  // Check our MQTT broker connection is still ok
  _mqtt.loop();

  // Handle any REST API requests
  WiFiClient client = _server.available();
  _api.loop(&client);

  // measure the distance (in cm) - defaults to maxDistance if no reading
  int measured = pingDistance();

  // publish the measured distance to MQTT
  publishDistance(measured);
  
  // check our thresholds and update LED/MQTT
  checkDistance(measured);
}
