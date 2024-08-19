#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h> // for not having to have the raw html in the main
#include <ESPmDNS.h>  // Include the mDNS library to maintain a local hostname making it accessible via domain name rather than IP address
#include "servointerface.h"

const char *ssid = "servossid";
const char *password = "servopassword";
const String hostname = "servocontrol";

WebServer server(80);
ServoInterface ms24;
const int servoPin = 38;
const int digitalBluePin = 13;

// Function to handle setting the servo position
void handleSetServo()
{
  Serial.println("update received");
  if (server.hasArg("percent"))
  {
    String percentStr = server.arg("percent");
    int percent = percentStr.toInt();

    // Simulate setting the servo position
    Serial.print("Setting servo to ");
    Serial.print(percent);
    Serial.println("% deployment.");

    int servoPhysicalMax = 190;
    ms24.setAngle(percent * servoPhysicalMax / 100);

    server.send(200, "text/plain", "Servo position set to " + percentStr + "%");
  }
  else
  {
    server.send(400, "text/plain", "Missing 'percent' parameter");
  }
}
// Function to serve the HTML file
void handleRoot()
{
  File file = LittleFS.open("/index.html", "r");
  if (!file)
  {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}


// Function to keep the connection alive
void handleKeepAlive()
{
  server.send(200, "text/plain", "Keep Alive");
  Serial.println("Keep Alive");
}


// Function to list files in the LittleFS recursively down levels
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  Serial.printf("Done listing directory: %s\n", dirname);
}

// Function to setup Wi-Fi
// @param timeout: time in seconds to wait for connection
void setupWifi(const char *ssid, const char *password, int timeout)
{
  digitalWrite(digitalBluePin, LOW);
  WiFi.begin(ssid, password);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.printf("Connecting to WiFi %i/%i\n", i, timeout);
    i++;
    if (i > timeout)
    {
      Serial.println("Connection timed out");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());
    digitalWrite(digitalBluePin, HIGH);
  }
}

// Function to setup mDNS
void setupMDNS(String hostName, int timeout)
{
  // Set up mDNS
  MDNS.begin(hostName);
  int i = 0;
  while (!MDNS.begin(hostName))
  {
    delay(1000);
    Serial.println("Error setting up MDNS responder!");
    if (i > timeout)
    {
      Serial.println("mDNS responder timed out");
      break;
    }
  }
  if (MDNS.begin(hostName))
  {
    Serial.println("mDNS responder started");
    // Serial.println(MDNS.port()); //?
  }
}

void setup()
{
  Serial.begin(115200);

  ms24.setup(servoPin, 270, 500, 2500);
  ms24.setSerialPrint(true);
  
  delay(10000);

  pinMode(digitalBluePin, OUTPUT);
  digitalWrite(digitalBluePin, LOW);


  // Mount LittleFS
  while (!LittleFS.begin())
  {
    Serial.println("LittleFS Mount Failed");

    // Check for possible causes
    if (LittleFS.format())
    {
      Serial.println("LittleFS Formatted and Mounted Successfully");
    }
    else
    {
      Serial.println("LittleFS Formatting Failed");
    }
    delay(1000);
  }
  Serial.println("LittleFS mounted successfully");

  // Connect to wifi
  setupWifi(ssid, password, 10);

  // Setup mDNS
  setupMDNS(hostname, 10);

  // Define routes
  server.on("/", handleRoot); // Serve the HTML page
  server.on("/set_servo", handleSetServo);

  // Start server
  server.begin();
  Serial.println("Server started");

  listDir(LittleFS, "/", 1);
}


void loop()
{
  server.handleClient();

  // reconnect if connection is lost from wifi
  if (WiFi.status() != WL_CONNECTED)
  {
    ms24.setAngle(190);
    setupWifi(ssid, password, 5);
    ms24.setAngle(0);
    setupWifi(ssid, password, 5);
  }
  // if(!MDNS.begin(hostname)) {
  //   setupMDNS(hostname, 10);
  // }
  if (!ms24.isAttached())
  {
    Serial.println("Servo detached");
    ms24.setup(servoPin, 270, 500, 2500);
    delay(1000);
    if (ms24.isAttached())
    {
      Serial.println("Servo reattached");
    }
  }
}
