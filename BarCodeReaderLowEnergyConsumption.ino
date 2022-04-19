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
//#include <Arduino_MKRMEM.h> // Flash
#include <Wire.h>             // I2C
#include <Adafruit_GFX.h>     // Display
#include <Adafruit_SSD1306.h> // Display
#include <wiring_private.h>   // Second serial
#include "ArduinoLowPower.h"  // LowEnergy

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
boolean stringComplete = false;   //--
int countstr = 0;                 // For Scanner
unsigned long millisendstr = 0;   //--
String uid;
GSMClient client;
GSM  gsmAccess;
GPRS gprs;
char server[] = "vps-a75923cc.vps.ovh.net";
String method = "GET";
String path = "/qrcode/index.php?";
int port = 80; // port 80 is the default for HTTP

static const unsigned long batteryDelay = 60;
static const unsigned long gpsDelay = 1800;
static const unsigned long readDelay = 300;

volatile int p;
volatile boolean rtcSleep = false;
volatile boolean GPSUpdated = false;

volatile int lastActivity =0;

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
    GPSUpdated = false;
    log("WakeUp GPS");
    GPS.wakeup();
    while (!GPS.available()) {
      COROUTINE_YIELD();
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
    GPSUpdated = true;
    printOLED("GPS updated",0,0);
    log("End GPS");
    GPS.standby();
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
    batteryLevel = analogRead(ADC_BATTERY) * 3.3f / 1023.0f / 1.2f * (1.2f + 0.33f);
    char result[20];
    sprintf(result, "Battery %f", batteryLevel);
    log(result);
    printOLED(result, 0, 0);
    COROUTINE_DELAY_SECONDS(batteryDelay);
  }
}

void writeRecord() {
  record_file = SD.open(FILE_NAME, FILE_WRITE);
  const int wait = 5;
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
  lastActivity = rtc.getMinutes()+wait;
}

COROUTINE(serialScanerEvent) { //Read QRCodes
  COROUTINE_LOOP() {
    //if(5volt in == true){
    while (Serial1.available() > 0) {
      char inChar = (char)Serial1.read();
      QRCode += inChar;
      countstr++;
      millisendstr = millis();
    }
    COROUTINE_YIELD();
    if (millis() - millisendstr > 1000 && countstr > 0) {
      stringComplete = true;
      if (QRCode != QRCode_old) { // anti aliasing
        QRCode_old = QRCode;
        printOLED("QRCode acquired",0,0);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
      }
      writeRecord();
    }
  //}
    COROUTINE_DELAY_SECONDS(2);
  }
}

COROUTINE(readAndSendRecord) {  //check exist file name, if no record.txt i
  COROUTINE_LOOP() {            //don't run this method
    if (SD.exists(FILE_NAME)) {
      //record_file = SD.open(FILE_NAME, FILE_WRITE);
      char *buffer;
      boolean connected = false;
      int byteRead = 0, statusCode, numberSend = 0;
      Serial.println("Connecting...");
      printOLED("Connecting GSM,\nscanner not\navailable", 0, 0);
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
          COROUTINE_YIELD();
        }
        if (connected) {
          record_file = SD.open(FILE_NAME, FILE_WRITE);
          delay(200);
          record_file.seek(0);
          Serial.println("Seek position 0");
          while (record_file.available()) {
            printOLED("Sending data\nto server . . .",0,0);
            String dataToGet = fscanf(record_file);
            // send HTTP request header
            client.println(method + " " + path + dataToGet + " HTTP/1.1");
            client.println("Host: " + String(server));
            client.println("Connection: close");
            client.println(); // end HTTP request header
            Serial.println(dataToGet);
          }

          while (client.connected()) {
            if (client.available()) {
              // read an incoming byte from the server and print it to serial monitor:
              char c = client.read();
              Serial.print(c);
              String one = String(c);
              boolean ok = one.indexOf('200');
              if(ok){printOLED("HTTP request ack 200",0,0);}
            }
          }

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
        } else {
          log("Connection timeout");
        }
        client.stop();
        //record_file.close();
      }
      gsmAccess.lowPowerMode();
      Serial.println("gsm low power mode");
    } else {
      gsmAccess.shutdown();
      Serial.println("record.txt does not exist, gsm shutdown");
    }
    COROUTINE_DELAY_SECONDS(readDelay);
  }
}

COROUTINE(LowEnergyManager) {
  COROUTINE_LOOP() {
    //inactivity time passed
    if(rtc.getMinutes() >= lastActivity){
      rtcSleep = true;
      }else{rtcSleep=false;}
    if(rtcSleep == true && GPSUpdated == true){  
      if(SD.exists(FILE_NAME)){
        printOLED("Sleep\nPress button to\nresume",0,0);
        LowPower.sleep(readDelay*1000);
        rtcSleep = false;
        lastActivity = rtc.getMinutes()+2;
        }else{          
          printOLED("DeepSleep\nPress button to\nresume",0,0);
          LowPower.deepSleep(gpsDelay*1000);
          rtcSleep = false;
          lastActivity = rtc.getMinutes()+2;
          }
      }
    COROUTINE_DELAY_SECONDS(5);
  }
}
   
void setup() {
  Serial.begin(9600);  log("Serial ready");
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
  pinMode(PIN_POWER, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);     // attach Led power
  analogReference(AR_DEFAULT);
  rtc.begin(); // real time clock
  QRCode.reserve(DIM_QRCODE);
  QRCode_old.reserve(DIM_QRCODE);
  QRCode_old = "$$$$$$$$$$$$$";
  printOLED("Ready", 0, 0);
  log("End init");
  if(!SD.begin(4)) {
    printOLED("SD Error", 0, 0); log("SD error");
  }
  LowPower.attachInterruptWakeup(PIN_POWER, Wakeup_Sleep, FALLING);
}

void loop() {
  for (int i = 0; i < p; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  gps.runCoroutine();
  battery.runCoroutine();
  serialScanerEvent.runCoroutine();
  readAndSendRecord.runCoroutine();
  LowEnergyManager.runCoroutine();
}

void Wakeup_Sleep(){
  printOLED("Wake up",0,0);
  Serial.println("wake up");
  p=1;
}
