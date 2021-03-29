#include <WiFi.h>
#include <PubSubClient.h>
#include "credentials.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RCSwitch.h>
RCSwitch rcSwitch = RCSwitch();

#define DHTPIN 32         // Pin, der mit dem Sensor verbunden ist
#define DHTTYPE DHT11       // DHT 11

const char* ssid = "Blueberry";
const char* password = "";
const char* mqttServer = "";
const char* mqttUsername = "gruppe6";
const char* mqttPassword = "";
const char* mqttDeviceId = "x85";

char temperatureTopic[] = "ES/WS20/gruppe6/temperature";
char humidityTopic[] = "ES/WS20/gruppe6/humidity";
char set_temperatureTopic[] = "ES/WS20/gruppe6/set_temperature";
char set_humidityTopic[] = "ES/WS20/gruppe6/set_humidity";
char temperatureSetPointTopic[] = "ES/WS20/gruppe6/temperature_setpoint";
char humiditySetPointTopic[] = "ES/WS20/gruppe6/humidity_setpoint";

double temperatureSetPoint = -1; // -1: Nicht gesetzt
double humiditySetPoint = -1;

int ledPinRed = 15;
int ledPinYellow = 13;
int ledPinGreen = 12;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
long lastMsg = 0;
char msg[50];

DHT_Unified dht(DHTPIN, DHTTYPE);


void setup_wifi()
{
  delay(10);

  Serial.println();
  Serial.print("Connecting to wifi ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
 // Zähler
 int i = 0;
 // Hilfsvariablen für die Konvertierung der Nachricht in ein String
 char message_buff[100];

 Serial.println("Message arrived: topic: " + String(topic));
 Serial.println("Length: " + String(length,DEC));

 // Kopieren der Nachricht und erstellen eines Bytes mit abschließendem \0
 for(i=0; i<length; i++) {
 message_buff[i] = payload[i];
 }
 message_buff[i] = '\0';

 // Konvertiert payload in einen String
 String msgString = String(message_buff);
 Serial.println("Payload: " + msgString);

  if(String(topic).equals(set_temperatureTopic)){
    temperatureSetPoint = msgString.toFloat();
  }else if(String(topic).equals(set_humidityTopic)){
    humiditySetPoint = msgString.toFloat();
  }

}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("connecting to mqtt...");

    if (client.connect(mqttDeviceId, mqttUsername, mqttPassword))
    {
      client.subscribe(set_temperatureTopic);
      client.subscribe(set_humidityTopic);
      Serial.println("connected.");

    } else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void ledBlink(int times){
  for(int i = 0; i < times; i++){
    digitalWrite (ledPinRed, HIGH);
    digitalWrite (ledPinYellow, HIGH);
    digitalWrite (ledPinGreen, HIGH);
    delay(1000);
    digitalWrite (ledPinRed, LOW);
    digitalWrite (ledPinYellow, LOW);
    digitalWrite (ledPinGreen, LOW);
    delay(1000);
  }

}

void setup()
{
  pinMode (ledPinRed, OUTPUT);
  pinMode (ledPinYellow, OUTPUT);
  pinMode (ledPinGreen, OUTPUT);

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

  // Sendemodul an dem digitalen PIN 10 angeschlossen.
  rcSwitch.setProtocol(3, 103);
  rcSwitch.enableTransmit(33);
  rcSwitch.setRepeatTransmit(10);

  dht.begin();
  // Temperatursensordaten ausgeben
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Luftfeuchtigkeitssensordaten ausgeben
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));

  ledBlink(3);
}

// Steckdosen-Steuerung

bool heatingmat_on = false;
bool humidifier_on = false;

// Heizmatte
void set_heatingmat(bool on) {
  if(heatingmat_on != on){
    heatingmat_on = on;
    if(on) {
      Serial.print("\nturn on heatingmat");
      // Funktioniert nicht bei jedem Signal, weshalb eine Schleife verwendet wird
      for(int i = 0; i < 90; i++) {
        Serial.print(".");
        rcSwitch.send(14143100, 24);
        delay(1000);
        rcSwitch.send(14245948, 24);
        delay(1000);
      }
    } else
    {
      Serial.print("\nturn off heatingmat");
      for(int i = 0; i < 90; i++) {
        Serial.print(".");
        rcSwitch.send(14455212, 24);
        delay(1000);
        rcSwitch.send(14355484, 24);
        delay(1000);
      }
    }
    Serial.println(" ");
  }

}

// Luftbefeuchter
void set_humidifier(bool on) {
  if(humidifier_on != on) {
    humidifier_on = on;
      if(on){
          Serial.print("\nturn on humidifier");
          for(int i = 0; i < 180; i++){
            Serial.print(".");
            rcSwitch.send(14355474, 24);
            delay(1000);
          }
      }else{
          Serial.print("\nturn off humidifier");
          for(int i = 0; i < 180; i++){
            Serial.print(".");
            rcSwitch.send(14245938, 24);
            delay(1000);
          }
      }
    Serial.println(" ");
    }
}

// Differenz zwischen Sollwert und Istwert ermitteln und Ampel basierend darauf setzen
void warnLed(float temperatureC){
  if(temperatureSetPoint == -1){
    digitalWrite (ledPinRed, LOW);
    digitalWrite (ledPinYellow, LOW);
    digitalWrite (ledPinGreen, LOW);
  } else if((temperatureC <= (temperatureSetPoint+2)) && (temperatureC >= (temperatureSetPoint-2))){
    digitalWrite (ledPinRed, LOW);
    digitalWrite (ledPinYellow, LOW);
    digitalWrite (ledPinGreen, HIGH);
  } else if (((temperatureC > (temperatureSetPoint+2)) && (temperatureC <= (temperatureSetPoint+4))) || ((temperatureC < temperatureSetPoint-2) && (temperatureC >= (temperatureSetPoint-4)))) {
    digitalWrite (ledPinRed, LOW);
    digitalWrite (ledPinYellow, HIGH);
    digitalWrite (ledPinGreen, LOW);
  } else {
    digitalWrite (ledPinRed, HIGH);
    digitalWrite (ledPinYellow, LOW);
    digitalWrite (ledPinGreen, LOW);
  }
}


void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000)
  {
    lastMsg = now;
    char payLoad[1];

    // Sollwerte senden
    client.publish(temperatureSetPointTopic, temperatureSetPoint == -1 ? "N/A" : String(temperatureSetPoint).c_str());
    Serial.println("published temperaturSetPoint (" + String(temperatureSetPoint) + ")");
    client.publish(humiditySetPointTopic, humiditySetPoint == -1 ? "N/A" : String(humiditySetPoint).c_str());
    Serial.println("published humiditySetPoint (" + String(humiditySetPoint) + ")");

    // Temperaturevent getten und Wert ausgeben
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    } else {
      Serial.print(F("Temperature: "));
      float temperatureC = event.temperature;
      Serial.print(temperatureC);
      Serial.println(F("°C"));
      client.publish(temperatureTopic, String(temperatureC).c_str());

      // Ampel setzen
      warnLed(temperatureC);

      // Temperatur regeln
      if(temperatureC < temperatureSetPoint){
        set_heatingmat(true);
      } else {
        set_heatingmat(false);
      }
    }

    // Luftfeuchtigkeitsevent getten und Wert ausgeben
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    } else {
      Serial.print(F("Humidity: "));
      float humidity = event.relative_humidity;
      Serial.print(humidity);
      Serial.println(F("%"));
      client.publish(humidityTopic, String(humidity).c_str());

      // Luftfeuchtigkeit regeln
      if(humidity < humiditySetPoint){
        set_humidifier(true);
      } else {
        set_humidifier(false);
      }
    }
  }
}
