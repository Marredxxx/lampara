#include <Adafruit_NeoPixel.h>

// Define el número de LEDs NeoPixel y el pin al que están conectados
#define NUM_LEDS 60
#define NEOPIXEL_PIN D6  // Cambia a tu pin deseado

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif 

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
  #include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProLight.h"
#include "SinricProTemperaturesensor.h"
#include "DHT.h"

#define WIFI_SSID         "TIGO-9EF5"    
#define WIFI_PASS         "4D9687404497"
#define APP_KEY           "8935dc13-914a-4f07-ba20-704d62fd8e64"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "0467bacb-1b15-4add-aa54-3c1e8c5fbbba-e028b2c5-98a2-43aa-88e9-34449aecca87"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define LIGHT_ID          "651de402ddaa8861a119af63"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         9600                // Change baudrate to your need
#define LIGHT_PIN D6  // Define the pin for the light output (change to D6)
#define TEMP_SENSOR_ID    "651c81aeddaa8861a118f656"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define EVENT_WAIT_TIME   60000
// define array of supported color temperatures
int colorTemperatureArray[] = {2200, 2700, 4000, 5500, 7000};  
int max_color_temperatures = sizeof(colorTemperatureArray) / sizeof(colorTemperatureArray[0]); // calculates how many elements are stored in colorTemperature array

// a map used to convert a given color temperature into color temperature index (used for colorTemperatureArray)
std::map<int, int> colorTemperatureIndex;

#if defined(ESP8266)
       #define DHT_PIN    14
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
      #define DHT_PIN    D5
#endif

DHT dht;                                      // DHT sensor

float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME);

void handleTemperaturesensor() {
  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  temperature = dht.getTemperature();          // get actual temperature in °C
//  temperature = dht.getTemperature() * 1.8f + 32;  // get actual temperature in °F
  humidity = dht.getHumidity();                // get actual humidity

  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("DHT reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature || humidity == lastHumidity) return; // if no values changed do nothing...

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event
  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}


// initializes the map above with color temperatures and index values
// so that the map can be used to do a reverse search like
// int index = colorTemperateIndex[4000]; <- will result in index == 2
void setupColorTemperatureIndex() {
  Serial.printf("Setup color temperature lookup table\r\n");
  for (int i=0;i < max_color_temperatures; i++) {
    colorTemperatureIndex[colorTemperatureArray[i]] = i;
    Serial.printf("colorTemperatureIndex[%i] = %i\r\n", colorTemperatureArray[i], colorTemperatureIndex[colorTemperatureArray[i]]);
  }
}

// we use a struct to store all states and values for our light
struct {
  bool powerState = false;
  int brightness = 0;
  struct {
    byte r = 0;
    byte g = 0;
    byte b = 0;
  } color;
  int colorTemperature = colorTemperatureArray[0]; // set colorTemperature to first element in colorTemperatureArray array
} device_state; 

bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("Device %s power turned %s \r\n", deviceId.c_str(), state ? "on" : "off");
  device_state.powerState = state;

  // Controla los LEDs NeoPixel según el estado
  if (state) {
    // Enciende todos los LEDs en un color específico (por ejemplo, blanco)
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(255, 255, 255));
    }
    strip.show();
  } else {
    // Apaga todos los LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    }
    strip.show();
  }

  return true;
}

bool onBrightness(const String &deviceId, int &brightness) {
  device_state.brightness = brightness;
  Serial.printf("Device %s brightness level changed to %d\r\n", deviceId.c_str(), brightness);

  // Ajusta el brillo de los LEDs NeoPixel
  int scaledBrightness = map(brightness, 0, 100, 0, 255);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(
      (device_state.color.r * scaledBrightness) / 255,
      (device_state.color.g * scaledBrightness) / 255,
      (device_state.color.b * scaledBrightness) / 255
    ));
  }
  strip.show();

  return true;
}

bool onAdjustBrightness(const String &deviceId, int brightnessDelta) {
  device_state.brightness += brightnessDelta;
  Serial.printf("Device %s brightness level changed about %i to %d\r\n", deviceId.c_str(), brightnessDelta, device_state.brightness);
  brightnessDelta = device_state.brightness;
  return true;
}

bool onColor(const String &deviceId, byte &r, byte &g, byte &b) {
  device_state.color.r = r;
  device_state.color.g = g;
  device_state.color.b = b;
  Serial.printf("Device %s color changed to %d, %d, %d (RGB)\r\n", deviceId.c_str(), device_state.color.r, device_state.color.g, device_state.color.b);
  
  // Actualiza el color de los LEDs NeoPixel
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();

  return true;
}

bool onColorTemperature(const String &deviceId, int &colorTemperature) {
  device_state.colorTemperature = colorTemperature;
  Serial.printf("Device %s color temperature changed to %d\r\n", deviceId.c_str(), device_state.colorTemperature);
  return true;
}

bool onIncreaseColorTemperature(const String &deviceId, int &colorTemperature) {
  int index = colorTemperatureIndex[device_state.colorTemperature];              // get index of stored colorTemperature
  index++;                                                                // do the increase
  if (index < 0) index = 0;                                               // make sure that index stays within array boundaries
  if (index > max_color_temperatures-1) index = max_color_temperatures-1; // make sure that index stays within array boundaries
  device_state.colorTemperature = colorTemperatureArray[index];                  // get the color temperature value
  Serial.printf("Device %s increased color temperature to %d\r\n", deviceId.c_str(), device_state.colorTemperature);
  colorTemperature = device_state.colorTemperature;                              // return current color temperature value
  return true;
}

bool onDecreaseColorTemperature(const String &deviceId, int &colorTemperature) {
  int index = colorTemperatureIndex[device_state.colorTemperature];              // get index of stored colorTemperature
  index--;                                                                // do the decrease
  if (index < 0) index = 0;                                               // make sure that index stays within array boundaries
  if (index > max_color_temperatures-1) index = max_color_temperatures-1; // make sure that index stays within array boundaries
  device_state.colorTemperature = colorTemperatureArray[index];                  // get the color temperature value
  Serial.printf("Device %s decreased color temperature to %d\r\n", deviceId.c_str(), device_state.colorTemperature);
  colorTemperature = device_state.colorTemperature;                              // return current color temperature value
  return true;
}

void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
  #endif

  WiFi.begin(WIFI_SSID, WIFI_PASS); 

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

void setupSinricPro() {
  // get a new Light device from SinricPro
  SinricProLight &myLight = SinricPro[LIGHT_ID];

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];

  // set callback function to device
  myLight.onPowerState(onPowerState);
  myLight.onBrightness(onBrightness);
  myLight.onAdjustBrightness(onAdjustBrightness);
  myLight.onColor(onColor);
  myLight.onColorTemperature(onColorTemperature);
  myLight.onIncreaseColorTemperature(onIncreaseColorTemperature);
  myLight.onDecreaseColorTemperature(onDecreaseColorTemperature);

  // Setup the light output pin
  pinMode(LIGHT_PIN, OUTPUT);
  
  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  SinricPro.begin(APP_KEY, APP_SECRET);
}

// main setup function
void setup() {
  // Inicializa la tira de NeoPixel
  strip.begin();
  strip.show(); // Inicializa todos los LEDs a 'apagado'

  Serial.begin(BAUD_RATE);
  Serial.printf("\r\n\r\n");
  dht.setup(DHT_PIN);
  setupColorTemperatureIndex();
  setupWiFi();
  setupSinricPro();
}

void loop() {
  SinricPro.handle();
  handleTemperaturesensor();
}
