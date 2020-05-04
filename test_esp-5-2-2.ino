
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiUdp.h>
//#include <DHTesp.h> je passe sur sur la biblio DHT grace à l exemple
#include <DHT.h>
//DHTesp dht; j fais sauter ca pour le remplacer par l exemple voir les defines
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* VARIABLE DEFINITIONS *********************************/

#define DHTTYPE    DHT11 //remplacement

#define MOISTURE 12
#define FLOAT 2
#define GREENLED 4
#define REDLED 5
#define PUMP 0
#define TEMP 3
DHT dht(TEMP , DHTTYPE); //remplacement
/************************* WIFI SSID & PASS *********************************/

#define WLAN_SSID       ".." // je fais attention maintenant
#define WLAN_PASS       ".."

/************************* ADAFRUIT AUTHORISATION *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883 // use 8883 for SSL
#define AIO_USERNAME    "..k" // pareil j fais gaffe .. coquinou !
#define AIO_KEY         ".."

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** ADAFRUIT FEEDS ***************************************/

Adafruit_MQTT_Publish moisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature"); // coquille tempreture sans doute
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish messages = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/messages");

Adafruit_MQTT_Subscribe waterswitch = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterswitch");
Adafruit_MQTT_Subscribe level = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/level");
void MQTT_connect();

/*************************** SETUP ************************************/

Adafruit_MQTT_Subscribe *subscription;
bool waterEmpty;
bool waterSwitchAuto;
bool waterSwitchEnabled;
int moistureLevel;
int moistureLevelTarget;
double sensorTemperature;
double sensorHumidity;

void setup() {
  //  Define inputs & outputs
  Serial.begin(115200);
  Serial.print("Executing code...");
  pinMode(PUMP, OUTPUT);
  pinMode(FLOAT, INPUT_PULLUP);
  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  mqtt.subscribe(&waterswitch);
  mqtt.subscribe(&level);

      digitalWrite(PUMP, LOW);

  waterEmpty = true;
  waterSwitchAuto = false;
  waterSwitchEnabled = true;
  moistureLevel = 100;
  moistureLevelTarget = 0;
  sensorHumidity = 0.0;
  sensorTemperature = 0.0;
 //dht.setup(17, DHTesp::DHT11); //je vire ce dechet qui trainait d une tentative precedante
}

void loop() {
  // connect
  MQTT_connect();
  
  // read
  readMoistureLevel();
  float temp = readTemperatureHumidity(); // j ai repris une coquille
  readWaterLevel();
  readAdafruitSubscriptions();

  // write
  publishStatusAdafruit();
  updateLEDs();
  controlPump();
  delay(5000);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void readMoistureLevel()
{
    // read moisture sensor
  int sensorValue = digitalRead(12);
  int percents = sensorValue / 10.24;
  moistureLevel = 100 - percents;
}

float readTemperatureHumidity()// j ai repris une coquille
{
/*  // read temperature and humidity sensor
  int chk = DHTesp.read11(TEMP); // si on reutilise penser à reecrire DHT.read11 etc
  if (chk == DHTLIB_OK)
  {
    sensorTemperature = DHTesp.temperature;
    sensorHumidity = DHTesp.humidity;
  }*/
      sensorHumidity = dht.readHumidity(); // j ai un peu bidouillé ce bloc .. :)
      sensorTemperature = dht.readTemperature();
    // if humidity read failed, don't change h value 
    if (isnan(sensorHumidity ) && isnan(sensorTemperature)) {
      Serial.println("Failed to read from DHT sensor!");
    return sensorHumidity;
    return sensorTemperature;
    }
}


void readWaterLevel()
{
  //  read water level sensor
  if (digitalRead(FLOAT) == HIGH) {
    waterEmpty = true;
  } else {
    waterEmpty = false;
  }
}

void readAdafruitSubscriptions() 
{
    // Read Adafruit subscriptions and update internal variables
  subscription = mqtt.readSubscription(5000);
  if (subscription == &waterswitch) {
    if (strcmp((char *)waterswitch.lastread, "Auto") == 0) {
      waterSwitchAuto = true;
    } else {
      waterSwitchAuto = false;
    }
  }
  else if (subscription == &level) {
    uint16_t levelval = atoi((char *)level.lastread);  // convert to a number
    moistureLevelTarget = levelval;
  }
}

void publishStatusAdafruit() {
  // Publish sensor values to Adafruit
  if (!waterEmpty && waterSwitchAuto)
  {
    messages.publish("Water level ok");
  }
  else
  {
    messages.publish("Refil water please!");
  }
  moisture.publish(moistureLevel);
  temperature.publish(sensorTemperature, 1);
  humidity.publish(sensorHumidity, 1);
}

void updateLEDs() {
  // Control green led
  if (waterSwitchAuto)
  {
    digitalWrite(GREENLED, HIGH);
  }
  else
  {
    digitalWrite(GREENLED, LOW);
  }
  // Control red led
  if (waterEmpty)
  {
    digitalWrite(REDLED, HIGH);
  }
  else
  {
    digitalWrite(REDLED, LOW);
  }  
}

void controlPump() {
    // Control pump
  if (waterSwitchEnabled && waterSwitchAuto && !waterEmpty && moistureLevel < moistureLevelTarget) {
    digitalWrite(PUMP, HIGH);
    delay(2000);
    digitalWrite(PUMP, LOW);
  }
  else {
    digitalWrite(PUMP, LOW);
  }
}
