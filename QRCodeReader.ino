#include <Scheduler.h>        // Cooperative task scheduler
#include <UrlEncode.h>        // Encode for GET request
#include <ArduinoUniqueID.h>  // Device ID
#include <SPI.h>              // protocol to SD
#include <SD.h>               // Labrary fo SD
#include <stdio.h>            // IO        
#include <ArduinoHttpClient.h>// POST
#include <Arduino_MKRGPS.h>   // GPS
#include <MKRGSM.h>           // GSM
#include <time.h>             // GPS to Time
//#include <Arduino_MKRMEM.h>   // Flash
#include <Wire.h>             // I2C
#include <Adafruit_GFX.h>     // Display
#include <Adafruit_SSD1306.h> // Display
#include <wiring_private.h>   // Second serial
#include "ArduinoLowPower.h"  // LowEnergy

#define DEBUG                 true // false <<<=============
#define MAX_DELAY_CONNECTION 5000 // connection
#define SSD1306_128_64        // OLED display model
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
#define DIM_QRCODE 100        // number of byte of QR code
#define PIN_POWER  4          // pin 4 for power button
#define PIN_ACTIVITY 7        // led for activity
#define SIZE_RECORD 256       // size of record in flash
#define PORT 443               // port 443https 80 http
#define FILE_NAME "record.txt"// file name record in flash
#define FILE_LOG  "log.txt"   // file with success and errors
//#define SERVER    "4helix.unicam.it"
//#define PATH "/cetara.php?"
//#define PINNUMBER "1234"
#define GPRS_APN "web.ho-mobile.it"
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
boolean stringComplete = false;  //--
int countstr = 0;                // For Scanner
unsigned long millisendstr = 0;  //--
String uid;
GSMClient client;
GSM  gsmAccess;
GPRS gprs;
char server[] = "vps-c0d4c583.vps.ovh.net";
String path = "/qrcode/index.php";
int port = 80; // port 80 is the default for HTTP

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

void gps() {  // reading gps value
  const long gpsDelay = 300000;
  log("WakeUp GPS");
  GPS.wakeup();
  while (!GPS.available()) {
    //Serial.println("gps not available, yield()");
    delay(1000);
    yield();
  }
  log("Init GPS");
  time_t epochTime = GPS.getTime();
  struct tm now = *localtime(&epochTime);
  rtc.setDate(now.tm_mday, now.tm_mon + 1, now.tm_year);
  rtc.setTime(now.tm_hour, now.tm_min, now.tm_sec);
  lastGPS.time       = GPS.getTime();
  lastGPS.latitude   = GPS.latitude();
  lastGPS.longitude  = GPS.longitude();
  lastGPS.altitude   = GPS.altitude();
  lastGPS.satellites = GPS.satellites();
  log("End GPS");
  GPS.standby();
  delay(gpsDelay);
}

void printOLED(char *t, int r, int c) { // write display message
  display.clearDisplay();  //Pulisce il buffer da inviare al display
  display.setTextSize(1);  //Imposta la grandezza del testo (1-8)
  display.setTextColor(WHITE); //Imposta il colore del testo (Solo bianco)
  display.setCursor(r, c); //Imposta la posizione del cursore (Larghezza,Altezza)
  display.println(t); //Stringa da visualizzare
  display.display(); //Invia il buffer da visualizzare al display
}

void battery() { // value of battery
  const int batteryDelay = 300000;
  batteryLevel = analogRead(ADC_BATTERY) * 3.3f / 1023.0f / 1.2f * (1.2f + 0.33f);
  char result[20];
  sprintf(result, "Battery %f", batteryLevel);
  log(result);
  printOLED(result, 0, 0);
  //yield();
  delay(batteryDelay);
}

void enableSleep() {
  log("DeepSleep");
  printOLED("deep sleep", 0, 0);
  LowPower.deepSleep();
}

void setDelayRTC() {
  const int wait = 30;
  rtc.setAlarmTime(rtc.getHours(), (rtc.getMinutes() + wait) % 60, 0);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  log("Enable Sleep Timeout");
}

void Wakeup_Sleep() {
  //printOLED("Wake up", 0, 0);
  log("WakeUp_Sleep");
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
  str += "qrcode=" + String(QRCode);
  QRCode = "";
  stringComplete = false;
  countstr = 0;
  record_file.println(str);
  record_file.flush();
  record_file.close();
  log(str);
  printOLED("Record Writed", 0, 0);
  setDelayRTC();
}

void serialScanerEvent() {
  while (Serial1.available() > 0) {
    char inChar = (char)Serial1.read();
    QRCode += inChar;
    countstr++;
    millisendstr = millis();
  }
  yield();
  if (millis() - millisendstr > 1000 && countstr > 0) {
    stringComplete = true;
    if (QRCode != QRCode_old) { // anti aliasing
      QRCode_old = QRCode;
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
    }
    writeRecord();
    readAndSendRecord();
  }
}

void readAndSendRecord() {    //check exist file name, if no record.txt i
  if (SD.exists(FILE_NAME)) { //don't run this method
    record_file = SD.open(FILE_NAME, FILE_WRITE);
    char *buffer;
    boolean connected = false;
    int byteRead = 0, statusCode, numberSend = 0;
    Serial.println("Connecting...");
    if ((gsmAccess.begin() == GSM_READY) &&
        (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
      log("GPRS attach");
      unsigned long timeout = millis() + MAX_DELAY_CONNECTION;
      while (millis() < timeout) {
        if (client.connect(server, port)) {
          log("connected");
          connected = true;
          break;
        }
        yield();
      }
      if (connected) {
        record_file.seek(0);
        Serial.println("Seek position 0");
        while (record_file.available()) {
          String dataToGet = fscanf(record_file);
          client.println("GET " + path + "?" + dataToGet);
          int maxDelay = 10; // sec
          Serial.println(dataToGet);
        }
        record_file.close();
        SD.remove(FILE_NAME);
        log("Remove buffer");
        printOLED("Record sent and buffer removed", 0, 0);
      } else {
        log("Connection timeout");
      }
      client.stop();
      record_file.close();
    }
    gsmAccess.lowPowerMode();
    Serial.println("gsm low power mode");
  } else {
    gsmAccess.shutdown();
    Serial.println("record.txt does not exist, gsm shotdown");
  }
  delay(1000*60*10);
}

void setup() {
  Serial.begin(9600); while (!Serial); log("Serial ready");
  Serial1.begin(9600); while (!Serial); log("Scanner ready");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!GPS.begin()) { //param GPS_MODE_SHIELD if the shield is used instead of I2C
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

  pinMode(LED_BUILTIN, OUTPUT);     // attach Led power
  pinMode(PIN_POWER, INPUT_PULLUP); // attach button on pin
  analogReference(AR_DEFAULT);
  rtc.begin(); // real time clock
  rtc.attachInterrupt(enableSleep); // attach timer for sleep
  QRCode.reserve(DIM_QRCODE);
  QRCode_old.reserve(DIM_QRCODE);
  QRCode_old = "$$$$$$$$$$$$$";
  LowPower.attachInterruptWakeup(PIN_POWER, Wakeup_Sleep, FALLING);
  printOLED("Ready", 0, 0);
  if (!SD.begin()) {
    printOLED("SD Error", 0, 0); log("SD error");
  }
  log("End init");
  Scheduler.startLoop(gps);
  delay(1000);
  Scheduler.startLoop(serialScanerEvent);
  Scheduler.startLoop(readAndSendRecord);
  //Scheduler.startLoop(battery);
}

void loop() {
  battery();
}
