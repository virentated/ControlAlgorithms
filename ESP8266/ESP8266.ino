/*
Code Authored by Keegan Kelly
*/
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

const char *ssid = "RobotWifi";
const char *password = "12345678";
// connects to the wifi network
void connectWiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
}

// sends a GET request to the server and sends the response over Serial to the Arduino
void GET(String address)
{
  // connects to server
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(5000);
  http.begin(client, address);
  // makes request
  int httpCode = http.GET();
  if (httpCode > 0)
  {
    // payload from the server is received as a String
    String payload = http.getString();
    // converting the string to a JSON document and then sending it over Serial
    char buff[payload.length()];
    payload.toCharArray(buff, payload.length());
    StaticJsonDocument<1000> doc;
    DeserializationError error = deserializeJson(doc, buff);
    serializeJson(doc, Serial);
    http.end();
  }
  else
  {
    http.end();
    // this will be an error when trying to deserialize and then the arduino will send a request again
    Serial.println("Error on sending GET request");
    return;
  }
}
// sends a PUT request to the server
void PUT(String address, String payload)
{
  // connects to server
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(1000);
  http.begin(client, address);
  // makes request
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.PUT(payload);
  http.end();
  return;
}
void setup()
{
  Serial.begin(115200);
  connectWiFi();
}

void loop()
{
  // waits for serial from arduino
  if (Serial.available())
  {
    // puts the serial communication from the arduino into a json document
    StaticJsonDocument<200> req;
    DeserializationError error = deserializeJson(req, Serial);
    if (error == DeserializationError::Ok)
    {
      // does a get request to the server
      if (req["type"].as<String>() == "GET")
      {
        GET(req["address"].as<String>());
      }
      // does a put request only for currently working with the position (could easily be adapted if needed)
      if (req["type"].as<String>() == "PUT")
      {
        StaticJsonDocument<200> doc;
        doc["id"] = req["id"].as<int>();
        String payload;
        // add position array to doc
        if (req.containsKey("position"))
        {
          JsonArray pos = doc.createNestedArray("position");
          pos.add(req["position"][0].as<float>());
          pos.add(req["position"][1].as<float>());
          pos.add(req["position"][2].as<float>());
        }
        else if (req.containsKey("ready"))
        {
          doc["ready"] = req["ready"].as<int>();
        }
        serializeJson(doc, payload);
        PUT(req["address"].as<String>(), payload);
        doc.clear();
        // makes a payload to be sent to the server
      }
    }
    req.clear();
  }
}
