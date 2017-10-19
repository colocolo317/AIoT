/*
 Basic MQTT example

 This sketch demonstrates the basic capabilities of the library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include "Wire.h"
#define TCAADDR 0x70
#include "SparkFunHTU21D.h"

HTU21D myHumidity;

char ssid[] = "WIFI_SSID";     // your network SSID (name)
char pass[] = "WIFI_PASSWD";   // your network password
int status  = WL_IDLE_STATUS;    // the Wifi radio's status

char mqttServer[]     = HOST;
char clientId[]       = "amebaClient";
char publishTopic[]   = "TestTopic";
char publishPayload[80] = "1111111";
char subscribeTopic[] = "inTopic";

void tcaselect(uint8_t i){
  if (i>7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(publishTopic, publishPayload);
      // ... and resubscribe
      client.subscribe(subscribeTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(38400);
  Wire.begin();
  
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
    
    
  }
  Serial.println("Initializing I2C devices...");
  myHumidity.begin();
  
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

  // Allow the hardware to sort itself out
  delay(1500);
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  tcaselect(1);
  
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  String timeMeasured = String("Times: " + String(millis()));
  Serial.print("Time:");
  Serial.print(millis());
  String tempMeasured = String("\nTemperature: "+ String(temp));
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  String hmdMeasured = String("\nHumidity: "+ String(humd));
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");
  Serial.println();
  
  String measuredInfo = timeMeasured + tempMeasured + hmdMeasured;
  measuredInfo.toCharArray(publishPayload, 80);
  client.publish(publishTopic, publishPayload);
  Serial.println(publishPayload);
  
  client.loop();

  delay(5000);
}
