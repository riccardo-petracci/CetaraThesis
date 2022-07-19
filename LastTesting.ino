/*********************/
/*     LIBRARIES    */
/********************/

#include <AceRoutine.h>
#include <Arduino.h>
using namespace ace_routine;

#include <MicroNMEA.h>
#include <string.h>
#include <ArduinoUniqueID.h>  // Device ID
#include <SPI.h>              // protocol to SD
#include <SD.h>               // Labrary fo SD
#include <stdio.h>            // IO        
#include <Adafruit_GFX.h>     // Display
#include <Adafruit_SSD1306.h> // Display
#include "wiring_private.h"   // pinPeripheral() function
#include "ArduinoLowPower.h"  // LowEnergy
#include <MD5.h>
#include <RTCZero.h>
#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module

/*********************/
/*    DEFINITIONS   */
/********************/

#define DEBUG                 true // false <<<=============
#define SSD1306_128_64             // OLED display model
#define SCREEN_WIDTH 128           // OLED display width, in pixels
#define SCREEN_HEIGHT 64           // OLED display height, in pixels
#define FILE_NAME "record.txt"     // file name record in SD card

/*********************/
/*     VARIABLES    */
/********************/

typedef struct { // GPS values
  unsigned long time;
  float  latitude;
  float  longitude;
  float  altitude;
  int    satellites;
} position;

File record_file;
position lastGPS;                 // last GPS value
RTCZero rtc;                      // Real Time Clock
Adafruit_SSD1306 display(4);      // Oled reset
RFID nano; //Create instance
String EPCstr;
Uart mySerial(&sercom2, 2, 3, SERCOM_RX_PAD_1, UART_TX_PAD_2);//D2-TX, D3-RX
String uid;

static const unsigned long batteryDelay = 60;  //delay coroutine battery in seconds
static const unsigned long gpsDelay = 600;      //delay coroutine gps in seconds
static const unsigned long readDelay = 150;     //delay coroutine reading and sending in seconds

volatile int p;                         //variable for the lowpower
volatile boolean rtcSleep = false;      //varible used to manage the sleep
volatile boolean GPSUpdated = false;    //variable used to trace GPS update

volatile int lastActivity = 0;        //inactivity variable
volatile int forceStop = 0;           //Forcing GPS

String server = "";
String path = "";
String key = "";

char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));

float latitude_mdeg ;
float longitude_mdeg ;

int PWR_KEY = 9;
int RST_KEY = 6;
int LOW_PWR_KEY = 5;
bool ModuleState = false;
int LED = 13;

/*********************/
/*      SETUP       */
/********************/

void SERCOM2_Handler()
{
  mySerial.IrqHandler();
}

void setup() {

  pinMode(PWR_KEY, OUTPUT);
  pinMode(RST_KEY, OUTPUT);
  pinMode(LOW_PWR_KEY, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(RST_KEY, LOW);
  digitalWrite(LOW_PWR_KEY, HIGH);
  digitalWrite(PWR_KEY, HIGH);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);

  SerialUSB.begin(115200); while (!SerialUSB); SerialUSB.println("SerialUSB ready");
  Serial1.begin(115200);
  digitalWrite(LED, HIGH);

  digitalWrite(PWR_KEY, LOW);
  delay(3000);
  digitalWrite(PWR_KEY, HIGH);
  delay(5000);
  ModuleState = moduleStateCheck();
  if (ModuleState == false) { //if it's off, turn on it.
    digitalWrite(PWR_KEY, LOW);
    delay(3000);
    digitalWrite(PWR_KEY, HIGH);
    delay(5000);
    SerialUSB.println("Now turnning the A9/A9G on.");
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  printOLED("Initialization", 0, 0);
  rtc.begin();
  sendData("AT+GPS=1", 1000, DEBUG);//1: turn on GPS  0:Turn off GPS
  sendData("AT+CGDCONT=1,\"IP\",\"TM\"", 2000, DEBUG);

  //UNIQUEID
  byte buffer[100];
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    buffer[i] = UniqueID[i];
    uid += String(buffer[i], HEX);
  }
  uid.toUpperCase();
  SerialUSB.print("UniqueID: ");
  SerialUSB.println(uid);

  if (!SD.begin(4)) {
    printOLED("SD Error", 0, 0);
    SerialUSB.println("SD init error");
    while (1);
  }
  //LowPower.attachInterruptWakeup(PIN_POWER, Wakeup_Sleep, FALLING);
  //RFID setup
  //Because we are using a hardware SerialUSB port in this example we can
  //push the SerialUSB speed much faster to 115200bps
  while (true) {
    if (setupNano(38400) == false) //Configure nano to run at 115200bps
    {
      SerialUSB.println(F("Module failed to respond. Please check wiring."));
      printOLED("RFID error", 0, 0);
      //while (1); //Freeze!
    } else {
      SerialUSB.println("RFID ready");
      printOLED("RFID Ready", 0, 0);
      break;
    }
  }
  nano.setRegion(REGION_NORTHAMERICA); //Set to North America

  nano.setReadPower(2700); //5.00 dBm. Higher values may caues USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling
  digitalWrite(LED, LOW);
  printOLED("Maduino Ready", 0, 0);
}

/*********************/
/*    COROUTINES    */
/********************/

COROUTINE(gps) {  // reading gps value
  COROUTINE_LOOP() {
    GPSUpdated = false;
    SerialUSB.println("WakeUp GPS");
    sendData("AT+GPS=1", 1000, DEBUG);
    delay(300);
    Serial1.println("AT+GPSRD=1");
    delay(500);
    while (Serial1.available() > 0)
    {
      char inByte = Serial1.read(); // Get GPS data
      if (DEBUG) {
        SerialUSB.print(inByte);
      }
      nmea.process(inByte);
    }
    SerialUSB.print("\nValid fix: ");
    SerialUSB.println(nmea.isValid() ? "yes" : "no");
    if (nmea.isValid()) {
      SerialUSB.println("GPS updating information");
      printOLED("GPS info updating", 0, 0);
      rtc.setTime(nmea.getHour(), nmea.getMinute(), nmea.getSecond()); //h , m , s
      rtc.setDate(nmea.getDay(), nmea.getMonth(), nmea.getYear() - 16); //d , m , y
      lastGPS.time       = rtc.getEpoch();

      latitude_mdeg = nmea.getLatitude();
      latitude_mdeg = latitude_mdeg / 1000000.;
      lastGPS.latitude   = latitude_mdeg;

      longitude_mdeg = nmea.getLongitude();
      longitude_mdeg = longitude_mdeg / 1000000.;
      lastGPS.longitude  = longitude_mdeg;
      lastGPS.altitude   = 0;
      lastGPS.satellites = nmea.getNumSatellites();

      SerialUSB.println("GPS value updated");
      printOLED("GPS value updated", 0, 0);

      //Printing values
      if (DEBUG) {
        SerialUSB.println("GPS data from lastGPS");
        SerialUSB.print("Latitude (deg): ");
        SerialUSB.println(lastGPS.latitude);
        SerialUSB.print("Longitude (deg): ");
        SerialUSB.println(lastGPS.longitude);
        SerialUSB.print("Satellites: ");
        SerialUSB.println(lastGPS.satellites);

        SerialUSB.println("\n Data/Time value from rtc");
        SerialUSB.print(rtc.getYear());
        SerialUSB.print('-');
        SerialUSB.print(int(rtc.getMonth()));
        SerialUSB.print('-');
        SerialUSB.print(int(rtc.getDay()));
        SerialUSB.print('T');
        SerialUSB.print(int(rtc.getHours()));
        SerialUSB.print(':');
        SerialUSB.print(int(rtc.getMinutes()));
        SerialUSB.print(':');
        SerialUSB.println(int(rtc.getSeconds()));
        SerialUSB.print("unixtime: ");
        SerialUSB.println(lastGPS.time);
      }
      Serial1.println("AT+GPSRD=0");
      Serial1.println("AT+GPS=0");
      GPSUpdated = true;
      COROUTINE_DELAY_SECONDS(gpsDelay);
    } else {
      Serial1.println("AT+GPSRD=0");
      COROUTINE_DELAY_SECONDS(30);
    }
  }
}

/*COROUTINE(RFIDScan) { //Read QRCodes
  COROUTINE_LOOP() {
    nano.startReading(); //Begin scanning for tags
    delay(500);
    if (nano.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SerialUSB.println(F("Scanning"));
      } else if (responseType == RESPONSE_IS_TAGFOUND) {
        //show tag found with led blink 2 times
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        //If we have a full record we can pull out the fun bits
        int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        //byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response
        tagEPCBytes = nano.getTagEPCBytes();

        SerialUSB.print("nano.getTagEPCBytes: ");
        SerialUSB.println(tagEPCBytes);

        writeRecord();

        SerialUSB.print(F(" rssi["));
        SerialUSB.print(rssi);
        SerialUSB.print(F("]"));

        SerialUSB.print(F(" freq["));
        SerialUSB.print(freq);
        SerialUSB.print(F("]"));

        SerialUSB.print(F(" time["));
        SerialUSB.print(timeStamp);
        SerialUSB.print(F("]"));

        //Print EPC bytes, this is a subsection of bytes from the response/msg array
        SerialUSB.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          if (nano.msg[31 + x] < 0x10) SerialUSB.print(F("0")); //Pretty print
          SerialUSB.print(nano.msg[31 + x], HEX);
          SerialUSB.print(F(" "));
        }
        SerialUSB.print(F("]"));

        SerialUSB.println();
      } else if (responseType == ERROR_CORRUPT_RESPONSE) {
        SerialUSB.println("Bad CRC");
      } else {
        //Unknown response
        SerialUSB.print("Unknown error");
      }
    }
    nano.stopReading();

    COROUTINE_DELAY_SECONDS(2);
  }
  }*/

COROUTINE(RFIDScan) { //Read QRCodes
  COROUTINE_LOOP() {

    byte myEPC[12]; //Most EPCs are 12 bytes
    byte myEPClength;
    byte responseType = 0;
    digitalWrite(LED, HIGH);

    //while (responseType != RESPONSE_SUCCESS) { //RESPONSE_IS_TAGFOUND)
    //COROUTINE_YIELD();

    myEPClength = sizeof(myEPC); //Length of EPC is modified each time .readTagEPC is called
    SerialUSB.println(F("Searching for tag"));
    responseType = nano.readTagEPC(myEPC, myEPClength, 2000); //Scan for a new tag up to 2000ms

    digitalWrite(LED, LOW);
    if (responseType == RESPONSE_SUCCESS) {
      //show tag found with led blink 2 times
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      delay(500);
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);

      //Print EPC
      SerialUSB.print(F(" epc["));
      for (byte x = 0 ; x < myEPClength ; x++) {
        if (myEPC[x] < 0x10) {
          SerialUSB.print(F("0"));
          EPCstr += "0";
        }
        SerialUSB.print(myEPC[x], HEX);
        EPCstr += String(myEPC[x], HEX);
        SerialUSB.print(F(" "));
      }
      SerialUSB.println(F("]"));
      SerialUSB.print("EPC in stringa: ");
      SerialUSB.println(EPCstr);
      writeRecord();
    }else{
      SerialUSB.println("No tag found");
      printOLED("No tag\nfound");
    }
    COROUTINE_YIELD();
    COROUTINE_DELAY_SECONDS(1);
  }
}


COROUTINE(readAndSendRecord) {
  COROUTINE_LOOP() {
    if (SD.exists(FILE_NAME)) {
      SerialUSB.println("Connecting...");
      printOLED("Connecting GPRS...", 0, 0);

      sendData("AT+CREG?", 1000, DEBUG);
      sendData("AT+CCID", 1000, DEBUG);
      sendData("AT+CGATT=1", 1000, DEBUG);
      sendData("AT+CGACT=1,1", 1000, DEBUG);
      //sendData("AT+CIPSTART=\"TCP\",\"vps-a75923cc.vps.ovh.net\",80", 1000, DEBUG);
      //COROUTINE_YIELD();
      String response = "";
      Serial1.println("AT+CIPSTART=\"TCP\",\"vps-a75923cc.vps.ovh.net\",80");
      long int time = millis();
      while ((time + 2000) > millis()) {
        while (Serial1.available()) {
          char c = Serial1.read();
          response += c;
        }
      }
      if (DEBUG) {
        SerialUSB.print(response);
      }
      if (response.indexOf("CONNECT") > 0) {
        int i = 1;
        record_file = SD.open(FILE_NAME, FILE_READ);
        record_file.seek(0);
        while (1) {
          printOLED("Sending data\nto server . . .", 0, 0);
          String dataToGet = fscanf(record_file);
          if (dataToGet == "") {
            record_file.close();
            delay(500);
            SD.remove(FILE_NAME);
            delay(500);
            SerialUSB.println("Buffer removed");
            printOLED("Buffer removed", 0, 0);
            i = 0;
            break;
          }
          String secret = dataToGet + key;
          SerialUSB.println(secret);
          char charBuf[250];
          secret.toCharArray(charBuf, 250);
          unsigned char* hash = MD5::make_hash(charBuf);
          char *md5str = MD5::make_digest(hash, 16);
          free(hash);
          String cmdToSend = "AT+HTTPGET=\"http://vps-a75923cc.vps.ovh.net/qrcode/write1.php?" + dataToGet + "&z=" + md5str  + "\"";
          response = "";
          time = millis();
          Serial1.println(cmdToSend);
          while ((time + 4000) > millis()) {
            while (Serial1.available()) {
              char c = Serial1.read();
              response += c;
            }
          }
          if (DEBUG) {
            SerialUSB.println(response);
          }
          free(md5str);
          if (response.indexOf("Dns,fail") > 0) {
            SerialUSB.println("Data sending failed");
            break;
          }
          //COROUTINE_YIELD();
        }
      } else {
        SerialUSB.println("Connection error");
        printOLED("Connection error", 0, 0);
      }
    }
    COROUTINE_DELAY_SECONDS(readDelay);
  }
}


/*********************/
/*      METHODS     */
/********************/

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
  //str += "rfid=" + String(EPCstr);
  str += "qrcode=" + String(EPCstr);
  record_file.println(str);
  record_file.flush();
  record_file.close();
  SerialUSB.println(str);
  printOLED("Record Written", 0, 0);
  lastActivity = rtc.getMinutes() + wait;
}

/*void Wakeup_Sleep() {
  printOLED("Wake up", 0, 0);
  Serial.println("wake up");
  p = 1;
  rtcSleep = false;
  lastActivity = rtc.getMinutes() + 5;
  }*/

String fscanf(File f) { //purificatore
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

String sendData(String command, const int timeout, boolean debug) {
  String response = "";
  Serial1.println(command);
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
  }
  if (debug) {
    SerialUSB.print(response);
  }
  return response;
}

void printOLED(char *t, int r, int c) { // write display message
  display.clearDisplay();  //Pulisce il buffer da inviare al display
  display.setTextSize(1);  //Imposta la grandezza del testo (1-8)
  display.setTextColor(WHITE); //Imposta il colore del testo (Solo bianco)
  display.setCursor(r, c); //Imposta la posizione del cursore (Larghezza,Altezza)
  display.println(t); //Stringa da visualizzare
  display.display(); //Invia il buffer da visualizzare al display
}

bool moduleStateCheck() {
  int i = 0;
  bool state = false;
  for (i = 0; i < 10; i++)
  {
    String msg = String("");
    msg = sendData("AT", 1000, DEBUG);
    if (msg.indexOf("OK") >= 0)
    {
      SerialUSB.println("A9/A9G Module had turned on.");
      state = true;
      return state;
    }
    delay(500);
  }
  return state;
}

//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.enableDebugging(SerialUSB); //Print the debug statements to the SerialUSB port

  //nano.begin(softSerial); //Tell the library to communicate over software serial port
  nano.begin(mySerial); //Tell the library to communicate over Teensy Serial Port # 5 (pins 33/34)
  delay(1000);
  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered

  //Software serial
  //softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  //while (softSerial.isListening() == false); //Wait for port to open

  //Hardware serial
  mySerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  delay(1000);
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM_ALT);
  while (!mySerial);

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  //while (softSerial.available()) softSerial.read();
  while (mySerial.available()) mySerial.read();

  nano.getVersion();
  delay(1000);

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();
    delay(1000);

    SerialUSB.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    //softSerial.begin(115200); //Start software serial at 115200
    mySerial.begin(115200); //Start serial at 115200
    delay(1000);
    pinPeripheral(2, PIO_SERCOM);
    pinPeripheral(3, PIO_SERCOM_ALT);
    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg
    delay(1000);

    //softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
    mySerial.begin(baudRate); //Start the serial port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano.getVersion();
  delay(1000);

  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

void loop() {
  gps.runCoroutine();
  //battery.runCoroutine();
  RFIDScan.runCoroutine();
  readAndSendRecord.runCoroutine();
  //LowEnergyManager.runCoroutine();
}
