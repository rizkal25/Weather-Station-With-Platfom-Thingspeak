

// Your GPRS credentials (leave empty, if not needed)
const char apn[]      = "internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password

#define GSM_PIN       ""
const char server[] = "api.thingspeak.comm"; // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "";         // resource path, for example: /post-data.php
const int  port = 80;                             // server port number
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers
int pinInterrupt = 14;
int Count = 0;
double kecepatanangin;
#define sensorcahaya 27
String apiKey = "40CLQ4YYXUFPDGH8";
double value;
// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
// BME280 pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for BME280 sensor
TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme; 

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000000UL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3600        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

//void onChange()
//{
//  if ( digitalRead(pinInterrupt) == LOW )
//    Count++;
//}

void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
//  pinMode( pinInterrupt, INPUT_PULLUP);// set the interrupt pin
 
  //Enable
//  attachInterrupt( digitalPinToInterrupt(pinInterrupt), onChange, FALLING);
 
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

 
  if (modem.simUnlock(GSM_PIN)) {
    Serial.println("SIM unlocked");
  } else {
    Serial.println("Unable to unlock SIM");
  }
  
  Serial.println("Connecting to GPRS...");
  if (!modem.gprsConnect(apn,gprsUser , gprsPass)) {
    Serial.println("GPRS connection failed");
    while (1);
  }
  
  Serial.println("GPRS connected");

  
  // You might need to change the BME280 I2C address, in our case it's 0x76
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
//   if ((millis() - lastDebounceTime) > debounceDelay)
//  {
//    lastDebounceTime = millis();
//    kecepatanangin = ((Count*8.75)/100);
//    Serial.print(kecepatanangin);
//    Serial.println("m/s");
//
//}
  
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // Convert to hPa
//  value = analogRead(sensorcahaya);
//  value = map(value, 0, 1024, 300, 1100);
 
//  Serial.println(value);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Pressure: ");
  Serial.println(pressure);
//  Serial.print (value);

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
  }
  else {
    SerialMon.println(" OK");
    
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
      SerialMon.println(" fail");
    }
    else {
      SerialMon.println(" OK");
    
      // Making an HTTP POST request
      SerialMon.println("Performing HTTP POST request...");
      sendToThingspeak(temperature, humidity, pressure,kecepatanangin, value);
      delay(10000);
  
     
  }
  // Put ESP32 into deep sleep mode (with timer wake up)
//  esp_deep_sleep_start();
}
}
void sendToThingspeak(float temperature, float humidity, float pressure //float kecepatanangin, //float value){
 
  String getData = "/update?api_key=";
  getData += apiKey;
  getData += "&field1=" + String(temperature);
  getData += "&field2=" + String(humidity);
  getData += "&field3=" + String(pressure);
//  getData += "&field4=" + String(kecepatanangin);
//  getData += "&field5=" + String(value);
//  client.get(getData);
//  int statusCode = client.responseStatusCode();
//  String response = client.responseBody();
  
//  Serial.print("HTTP Response code: ");
//  Serial.println(statusCode);
//  Serial.print("Server response: ");
//  Serial.println(response);
 if (client.connect("api.thingspeak.com", 80)) {
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + String(apiKey) + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(getData.length());
    client.print("\n\n");
    client.print(getData);

    Serial.println("Sending data to Thingspeak...");

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        SerialMon.print(c);
      }
    }

    client.stop();
    SerialMon.println("\nData sent!");
  } else {
    SerialMon.println("Failed to connect to Thingspeak");
  }
}
