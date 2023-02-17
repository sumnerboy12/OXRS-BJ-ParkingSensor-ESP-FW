/**
  Parking sensor firmware for the Open eXtensible Rack System
  
  GitHub repository:
    https://github.com/sumnerboy12/OXRS-BJ-ParkingSensor-ESP-FW
    
  Copyright 2022 Ben Jones <ben.jones12@gmail.com>
*/

/*--------------------------- Libraries -------------------------------*/
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>

#if defined(OXRS_ESP32)
#include <OXRS_32.h>                  // ESP32 support
OXRS_32 oxrs;

#elif defined(OXRS_ESP8266)
#include <OXRS_8266.h>                // ESP8266 support
OXRS_8266 oxrs;

#endif

/*--------------------------- Constants -------------------------------*/
// Serial
#define     SERIAL_BAUD_RATE          115200

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

/*--------------------------- Instantiate Globals ---------------------*/
// RGB LED
Adafruit_NeoPixel _pixels = Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Distance sonar
NewPing _sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);

/*--------------------------- Program ---------------------------------*/
void setConfigSchema()
{
  // Define our config schema
  StaticJsonDocument<1024> json;
  
  JsonObject debug = json.createNestedObject("debug");
  debug["type"] = "boolean";

  JsonObject pingAverage = json.createNestedObject("pingAverage");
  pingAverage["type"] = "integer";
  pingAverage["minimum"] = 1;
  pingAverage["maximum"] = PING_AVERAGE_MAX;

  JsonObject maxDistance = json.createNestedObject("maxDistance");
  maxDistance["type"] = "integer";

  JsonObject parkDistance = json.createNestedObject("parkDistance");
  parkDistance["type"] = "integer";

  JsonObject parkRange = json.createNestedObject("parkRange");
  parkRange["type"] = "integer";

  // Pass our config schema down to the hardware library
  oxrs.setConfigSchema(json.as<JsonVariant>());
}

// calculate our thresholds etc
void updateThresholds()
{
  // update the various thresholds
  parkUpper     = parkDistance + (1 * parkRange);
  parkLower     = parkDistance - (1 * parkRange);
  parkApproach  = parkDistance + (3 * parkRange);

  // debugging
  oxrs.print(F("[park] park distance: "));
  oxrs.print(parkDistance);
  oxrs.print(F("cm; range +/-"));
  oxrs.print(parkRange);
  oxrs.print(F("cm; (upper "));
  oxrs.print(parkUpper);
  oxrs.print(F("cm; lower "));
  oxrs.print(parkLower);
  oxrs.print(F("cm; approach "));
  oxrs.print(parkApproach);
  oxrs.println(F("cm)"));
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

void jsonConfig(JsonVariant json)
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
  
  oxrs.publishStatus(json.as<JsonVariant>());
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
  oxrs.publishTelemetry(json.as<JsonVariant>());
  
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
void initialiseRgb()
{
  oxrs.println("[park] initialising RGB LED...");
  _pixels.begin();

  oxrs.println(" - red test");
  red();
  delay(1000);
  oxrs.println(" - green test");
  green();
  delay(1000);
  oxrs.println(" - blue test");
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
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);  
  Serial.println(F("[park] starting up..."));

  // Start hardware
  oxrs.begin(jsonConfig, NULL);

  // Set up the RGB
  initialiseRgb();

  // Set up the sonar
  initialiseSonar();

  // Set up the config schema (for self-discovery and adoption)
  setConfigSchema();
}

/**
  Main processing loop
*/
void loop() 
{
  // Let hardware handle any events etc
  oxrs.loop();

  // measure the distance (in cm) - defaults to maxDistance if no reading
  int measured = pingDistance();

  // publish the measured distance to MQTT
  publishDistance(measured);
  
  // check our thresholds and update LED/MQTT
  checkDistance(measured);
}
