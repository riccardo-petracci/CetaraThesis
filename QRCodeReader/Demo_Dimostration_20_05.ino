#include <AceRoutine.h>
#include <Arduino.h>
using namespace ace_routine;

#include <ArduinoUniqueID.h>  // Device ID
#include <SPI.h>              // protocol to SD
#include <SD.h>               // Labrary fo SD
#include <stdio.h>            // IO        
#include <ArduinoHttpClient.h>// POST
#include <Arduino_MKRGPS.h>   // GPS
#include <MKRGSM.h>           // GSM
#include <time.h>             // GPS to Time
#include <Wire.h>             // I2C
#include <Adafruit_GFX.h>     // Display
#include <Adafruit_SSD1306.h> // Display
#include <wiring_private.h>   // Second serial
#include <MD5.h>
#include <RTCZero.h>

#define DEBUG                 true // false <<<=============
#define MAX_DELAY_CONNECTION 5000  // connection
#define SSD1306_128_64             // OLED display model
#define SCREEN_WIDTH 128           // OLED display width, in pixels
#define SCREEN_HEIGHT 64           // OLED display height, in pixels
#define DIM_QRCODE 100             // number of byte of QR code
#define PIN_POWER  0               // pin 4 for power button
#define PIN_ACTIVITY 7             // led for activity
#define SIZE_RECORD 256            // size of record in flash
#define PORT 443                   // port 443https 80 http
#define FILE_NAME "record.txt"     // file name record in flash
#define FILE_LOG  "log.txt"        // file with success and errors
#define PINNUMBER "PIN_NUMBER"
#define GPRS_APN "GPRS_APN"
#define GPRS_LOGIN ""
#define GPRS_PASSWORD ""


/* BEGIN COMMON ---------------------- */

typedef struct { // GPS values
  unsigned long time;
  float  latitude;
  float  longitude;
  float  altitude;
  int    satellites;
} position;

File log_file, record_file;
int error = 0;                    // variable for errors
position lastGPS;                 // last GPS value
RTCZero rtc;                      // Real Time Clock
float batteryLevel;               // last voltage of battey
String QRCode;                    // Buffer for QRCode
String QRCode_old;                // Last QRCode
Adafruit_SSD1306 display(4);      // Oled reset
boolean stringComplete = false;   //--
int countstr = 0;                 // For Scanner
unsigned long millisendstr = 0;   //--
String uid;
GSMClient client;
GSM  gsmAccess;
GPRS gprs;
char server[] = "server";
String method = "GET";
String path = "path";
int port = 80; // port 80 is the default for HTTP
String key = "key_string";

int dividerPin = A1;    //pin for the divider
float diValue = 0;        // variable to store the value coming from the divider

static const unsigned long batteryDelay = 30;  //delay coroutine battery in seconds
static const unsigned long gpsDelay = 300;      //delay coroutine gps in seconds
static const unsigned long readDelay = 10;     //delay coroutine reading and sending in seconds

/* END COMMON ---------------------- */

String fscanf(File f) {
  String s = "";
  while (f.available()) {
    char ch = f.read();
    if (ch == '\n') return s;
    else if (ch == '\r') {
      ;
    }
    else s += ch;
  }
}

void log(String info) {
  String out = String(rtc.getYear()) + "/" + String(rtc.getMonth()) + "/" +
               String(rtc.getDay()) + "-" + String(rtc.getHours()) + ":" +
               String(rtc.getMinutes()) + "-" + String(rtc.getSeconds());
  out += " " + info;
  if (DEBUG) Serial.println(out);
  else {
    log_file.println(out);
    log_file.flush();
  }
}

COROUTINE(gps) {  // reading gps value
  COROUTINE_LOOP() {
    log("WakeUp GPS");
    GPS.wakeup();
    while (!GPS.available()) {
      COROUTINE_YIELD();
    }
    log("Init GPS");
    printOLED("Updating GPS...", 0, 0);
    time_t epochTime = GPS.getTime();
    struct tm now = *localtime(&epochTime);
    rtc.setDate(now.tm_mday, now.tm_mon + 1, now.tm_year);
    rtc.setTime(now.tm_hour, now.tm_min, now.tm_sec);
    lastGPS.time       = GPS.getTime();
    lastGPS.latitude   = GPS.latitude();
    lastGPS.longitude  = GPS.longitude();
    lastGPS.altitude   = GPS.altitude();
    lastGPS.satellites = GPS.satellites();
    printOLED("GPS updated", 0, 0);
    log("End GPS");
    //GPS.standby();
    COROUTINE_DELAY_SECONDS(gpsDelay);
  }
}

void printOLED(char *t, int r, int c) { // write display message
  display.clearDisplay();  //Pulisce il buffer da inviare al display
  display.setTextSize(1);  //Imposta la grandezza del testo (1-8)
  display.setTextColor(WHITE); //Imposta il colore del testo (Solo bianco)
  display.setCursor(r, c); //Imposta la posizione del cursore (Larghezza,Altezza)
  display.println(t); //Stringa da visualizzare
  display.display(); //Invia il buffer da visualizzare al display
}

COROUTINE(battery) { // value of battery
  COROUTINE_LOOP() {
    diValue = analogRead(dividerPin);
    float voltage = diValue * (5.0 / 1023.0);
    if (voltage < 3.0) {
      batteryLevel = analogRead(ADC_BATTERY) * 3.3f / 1023.0f / 1.2f * (1.2f + 0.33f);
      char result[20];
      sprintf(result, "Battery %f", batteryLevel);
      log(result);
      printOLED(result, 0, 0);
    } else {
      printOLED("battery charging", 0, 0);
    }
    COROUTINE_DELAY_SECONDS(batteryDelay);
  }
}


void writeRecord() {
  record_file = SD.open(FILE_NAME, FILE_WRITE);
  String str;
  str += "device=" + uid + "&";
  str += "unixtime=" + String(lastGPS.time) + "&";
  str += "latitude=" + String(lastGPS.latitude) + "&";
  str += "longitude=" + String(lastGPS.longitude) + "&";
  str += "altitude=" + String(lastGPS.altitude) + "&";
  str += "satellites=" + String(lastGPS.satellites) + "&";
  String subStr = QRCode.substring(QRCode.indexOf("qrcode=") + 7);
  str += "qrcode=" + String(subStr);
  QRCode = "";
  subStr = "";
  stringComplete = false;
  countstr = 0;
  record_file.println(str);
  record_file.flush();
  record_file.close();
  log(str);
  printOLED("Record Written", 0, 0);
}

COROUTINE(serialScanerEvent) { //Read QRCodes
  COROUTINE_LOOP() {
    diValue = analogRead(dividerPin);
    float voltage = diValue * (5.0 / 1023.0);
    if (voltage > 3.0) {
      printOLED("scanner ready", 0, 0);
      delay(500);
      while (Serial1.available()) {
        char inChar = (char)Serial1.read();
        QRCode += inChar;
        countstr++;
        millisendstr = millis();
      }
      //COROUTINE_YIELD();
      if (millis() - millisendstr > 1000 && countstr > 0) {
        stringComplete = true;
        if (QRCode != QRCode_old) { // anti aliasing
          QRCode_old = QRCode;
          printOLED("QRCode acquired", 0, 0);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
        }
        writeRecord();
      }
    } else {
      printOLED("need 5v to\nuse scanner", 0, 0);
    }
    COROUTINE_DELAY_SECONDS(2);
  }
}

COROUTINE(readAndSendRecord) {
  COROUTINE_LOOP() {
    if (SD.exists(FILE_NAME)) {
      char *buffer;
      boolean connected = false;
      int byteRead = 0, statusCode, numberSend = 0;
      Serial.println("Connecting...");
      printOLED("Connecting GSM,\nscanner not\navailable", 0, 0);
      if ((gsmAccess.begin(PINNUMBER) == GSM_READY) &&
          (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
        log("GPRS attach");
        record_file = SD.open(FILE_NAME, FILE_READ);
        record_file.seek(0);
        boolean finish = false;
        while (!finish) {
          unsigned long timeout = millis() + MAX_DELAY_CONNECTION;
          while (millis() < timeout) {
            if (client.connect(server, port)) {
              log("connected");
              printOLED("Connected", 0, 0);
              connected = true;
              break;
            }
            COROUTINE_YIELD();
          }
          if (connected) {
            String dataToGet = fscanf(record_file);
            if (dataToGet == "") {
              finish = true;
              while (record_file.available()) {
                record_file.close();
                delay(500);
              }
              while (SD.exists(FILE_NAME)) {
                SD.remove(FILE_NAME);
                delay(500);
              }
              log("Remove buffer");
              printOLED("Record sent and buffer removed", 0, 0);
              break;
            }
            delay(500);
            printOLED("Sending data\nto server . . .", 0, 0);
            String secret = dataToGet + key;
            Serial.println(secret);
            char charBuf[250];
            secret.toCharArray(charBuf, 250);
            unsigned char* hash = MD5::make_hash(charBuf);
            char *md5str = MD5::make_digest(hash, 16);
            free(hash);
            // send HTTP request header
            client.println(method + " " + path + dataToGet + "&z=" + md5str + " HTTP/1.1");
            client.println("Host: " + String(server));
            client.println("Connection: close");
            client.println(); // end HTTP request header
            Serial.println(dataToGet);
            Serial.println(md5str);
            free(md5str);

            while (client.connected()) {
              if (client.available()) {
                // read an incoming byte from the server and print it to serial monitor:
                char c = client.read();
                Serial.print(c);
                String one = String(c);
                if (one.indexOf("200") != -1) {
                  printOLED("HTTP request ack 200", 0, 0);
                }
              }
            }
          } else {
            log("Connection timeout");
          }
          client.stop();
          delay(1000);
        }
      }
      //gsmAccess.lowPowerMode();
      //Serial.println("gsm low power mode");
    } else {
      printOLED("No record to send", 0, 0);
      delay(2000);
      //gsmAccess.shutdown();
      //Serial.println("record.txt does not exist, gsm shutdown");
    }
    COROUTINE_DELAY_SECONDS(readDelay);
  }
}


void setup() {
  Serial.begin(9600); log("Serial ready");
  Serial1.begin(9600); log("Scanner ready");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!GPS.begin()) {
    Serial.println("Failed to initialize GPS!");
    while (1);
  }
  //UNIQUEID
  byte buffer[100];
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    buffer[i] = UniqueID[i];
    uid += String(buffer[i], HEX);
  }
  uid.toUpperCase();
  log("UniqueID: " + uid);
  pinMode(PIN_POWER, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);     // attach Led power
  analogReference(AR_DEFAULT);
  rtc.begin(); // real time clock
  QRCode.reserve(DIM_QRCODE);
  QRCode_old.reserve(DIM_QRCODE);
  QRCode_old = "$$$$$$$$$$$$$";
  printOLED("Ready", 0, 0);
  log("End init");
  if (!SD.begin(4)) {
    printOLED("SD Error", 0, 0); log("SD error");
    delay(2000);
  }
}

void loop() {
  gps.runCoroutine();
  battery.runCoroutine();
  serialScanerEvent.runCoroutine();
  readAndSendRecord.runCoroutine();
}
