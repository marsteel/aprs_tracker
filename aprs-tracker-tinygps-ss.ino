#include <TinyGPS.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include <Arduino.h>
#include <U8x8lib.h>
#include <stdlib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Use OLED 12864

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address or use iic scanner to scan; 0x3D for 128x64, 0x3C for 128x32 // On Arduino Nano SDA -> A4 SCL -> A5

//static const int RXPin = 10, TXPin =11; // Arduino Nano RX=D10, TX=D11
static const uint32_t GPSBaud = 9600; // TODO 9600 is too high, need to lower it to 4800

bool screencleared = false;
bool send_aprs_update = false;

char charCallsign[] = "BH1RJC"; // Your callsign
char charSSID[] = "9"; // Your SSID
char charCallsign_SSID[] = "BH1RJC-9"; // Your callsign-SSID
String stringCallsign = "BH1RJC"; // Your callsign
String stringSSID = "9"; // Your SSID


char Time[]  = "UTC 00:00:00";  // variable definitions
char char_GPS_Fixed[]  = "FIX 00";  // variable definitions

float lastTxdistance, homeDistance, base = 0.0;
unsigned long lastTx = 0;
unsigned long txTimer = 0;

unsigned long txInterval = 80000L;  // Initial 80 secs internal

long lat = 0;
long lon = 0;

int year = 0;
byte month = 0, day = 0, hour = 0, minute = 0, second = 0, hundredths = 0;
unsigned long age = 0;
float falt = 0;
float fkmph = 0;
int ialt = 0;

byte GPS_baud_rate_4800_mssage[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0xC0, 0x12, 0x00, 0x00, 0x07, 0x00, 0x03,
0x00, 0x00, 0x00, 0x00, 0x00, 0xCF, 0xE4}; // Set GPS baud rate to 4800 on NEO-7M GPS module

byte GPS_baud_rate_9600_NMEA_mssage[] = {0xB5, // Set GPS baud rate to 9600 and NMEA only on NEO-7M GPS module
                  0x62,
                  0x06,
                  0x00,
                  0x14,
                  0x00,
                  0x01,
                  0x00,
                  0x00,
                  0x00,
                  0xD0,
                  0x08,
                  0x00,
                  0x00,
                  0x80,
                  0x25,
                  0x00,
                  0x00,
                  0x07,
                  0x00,
                  0x02,
                  0x00,
                  0x00,
                  0x00,
                  0x00,
                  0x00,
                  0xA1,
                  0xAF
                  };
TinyGPS gps;
// the timer object
SimpleTimer APRS_Send_Timer;   //Declaration for a APRS timer for transmission interval

// The serial connection to the GPS device
SoftwareSerial GPSSerial(10, 11);  // PIN8/9 on Leonardo  //PIN 10/11 on Nano

void setup()
{

  delay(1000); //delay 1sec
  Serial.begin(9600); // debug and serial comm RX/TX to MicroModem
  GPSSerial.begin(9600); // Connect to GPS as 9600
  GPSSerial.write(GPS_baud_rate_9600_NMEA_mssage, sizeof(GPS_baud_rate_9600_NMEA_mssage)); // Set GPS baud rate to 4800

  // Set an interval to 30 secs for the APRS send
  APRS_Send_Timer.setInterval(30000);
     
  //Display the About screen
  u8x8.begin();
  u8x8.setPowerSave(0);
  //u8x8.clear();

  //draw splash screen
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
  u8x8.drawString(0, 0, "GPS NOT FIXED");   // Print GPS NOT FIXED
  drawLeft(2, charCallsign); // Print callsign

  u8x8.setFont(u8x8_font_chroma48medium8_r);
  drawLeft(6, "v1.0");   // Print program version

  u8x8.refreshDisplay();    // only required for SSD1606/7

  //Print version
  Serial.println(F("APRS Tracker program"));
  Serial.println(F("by BH1RJC"));
  Serial.println();
  //APRS_init();
  char myCALL[] = "BH1RJC";
  
  //initialMicroAPRS(); // initialize MicroARPS modem
}


void loop()
{

  bool newData = false;


  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPSSerial.available())
    {
      char c = GPSSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData) {

    if (!screencleared) // do once
    {
      int r;
      for ( int r = 0; r < u8x8.getRows(); r++ )
      {
        u8x8.clearLine(r);
        //  u8x8.clear();
        //u8x8.display().setColor(BLACK);
        // u8x8.refreshDisplay();    // only required for SSD1606/7
      }
      screencleared = true;
    }
    
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age); //int year; byte month, day, hour, minute, second, hundredths;
    gps.get_position(&lat, &lon, &age);
    falt = gps.f_altitude(); // +/- altitude in meters
    fkmph = gps.f_speed_kmph(); // speed in km/hr
    ialt = int(falt * 3.281); // integer value of altitude in feet

    if (age == TinyGPS::GPS_INVALID_AGE)
      Serial.println(F("No fix detected"));
    else if (age > 5000)
      Serial.println(F("Warning: possible stale data!"));


     //   Serial.print(static_cast<int>(day)); Serial.print(F("/")); Serial.print(static_cast<int>(month)); Serial.print(F("/")); Serial.print(year);
     //   Serial.print(F(" ")); Serial.print(static_cast<int>(hour)); Serial.print(F(":")); Serial.print(static_cast<int>(minute)); Serial.print(F(":")); Serial.print(static_cast<int>(second));
    
     //   Serial.print(F(" "));
      //  Serial.print(deg_to_nmea(lat, true));
     //   Serial.print(F("/"));
    
    //Serial.print(deg_to_nmea(lon, false));
     //   Serial.print(F(" Altitude m/ft: ")); Serial.print(falt); Serial.print(F("/")); Serial.println(ialt);
    
    //    Serial.print(F("!="));
    //    Serial.print(deg_to_nmea(lat, true));
    //    Serial.print(F("/"));
    //    Serial.print(deg_to_nmea(lon, false));
    //    Serial.println(F("-PHG2410MicroAPRS"));
    

    
    char aprsLatitude[8]; //ddmm.hhN Latitude for APRS fixed 8-character field, in degrees and decimal minutes (to two decimal places)
    char aprsLongitude[9]; //dddmm.hhW Longitude for APRS fixed 9-character field, in degrees and decimal minutes (to two decimal places), followed by the letter E for east or W for west.
    String stringAprsLatitude = ""; //ddmm.hhN Latitude for APRS fixed 8-character field, in degrees and decimal minutes (to two decimal places)
    String stringAprsLongitude = ""; //dddmm.hhW Longitude for APRS fixed 9-character field, in degrees and decimal minutes (to two decimal places), followed by the letter E for east or W for west.

    // St//ring

    //    long lat=(long)gps.location.lat();
    //    long lon=(long)gps.location.lng();

   // Serial.print(deg_to_nmea(lat, true));

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    
    //u8x8.drawString(12, 7, "v1.0");
    u8x8.drawString(10, 0, charCallsign);
    u8x8.drawString(0, 0, "FIXED");
    u8x8.drawString(0, 2, "LAT");
    u8x8.drawString(0, 4, "LNG");
    //u8x8.noInverse();
    enum {BufSize = 3}; // Space for 2 digits
    char satchar2[BufSize] = "";
    //snprintf (satchar2, BufSize, "%d", gps.satellites.value());

    //u8x8.drawString(4, 0, "  ");

    // Print gps.satellites() when GPS fixed
    char satellites_value_buffer[3];
    //// satellites used in last full GPGGA sentence
    //inline unsigned short satellites() { return _numsats; }
    
    dtostrf(gps.satellites(), 2, 0, satellites_value_buffer); //convert INT gps.satellites() to char buffer satellites_value_buffer[] two digitals, no digit after decimal point
    satellites_value_buffer[2]='\0'; //set the closing NULL character to satellites_value_buffer[]
    
   // Serial.println(satellites_value_buffer); // Printer sat in use
    
    if (age != TinyGPS::GPS_INVALID_AGE and satellites_value_buffer != "255"){
     // u8x8.drawString(4, 0, satellites_value_buffer); //print_int(gps.satellites());
      u8x8.drawString(6, 0, satellites_value_buffer); //print_int(gps.satellites()) as NN after FIXED; "FIXED NN"
    }
    
    //u8x8.drawString(4, 0, satchar2);

    //    char latchar[7]=""; // // ddmm.hhN Buffer big enough for 9-character float
    //    dtostrf(convertDegMin(gps.location.lat()), 7, 2, latchar); // ddmm.hhN   ddmers long, with 2 characters after the decimal point. Leave room for large numbers
    //    //dtostrf(fabs(convertDegMin(gps.location.lat())), 2, 2, stringAprsLatitude );    m.hh is 7 charact
    //    // Determime to print N or S to the last char
    //    if ( convertDegMin(gps.location.lat()) >= 0 ) {
    //      latchar[7] = 'N';
    //      latchar[8] = '\0';
    //    } else if (convertDegMin(gps.location.lat()) < 0 ) {
    //      latchar[7] = 'S';
    //      latchar[8] = '\0';
    //    }
    //
    //    //aprsLatitude =
    //    u8x8.drawString(4, 2, latchar);
    u8x8.drawString(4, 2, deg_to_nmea(lat, true));  //Display LAT in ddmm.hhN
    u8x8.drawString(4, 4, deg_to_nmea(lon, false));//Display LON in dddmm.hhN

    //    //u8x8.drawString(4, 2, stringAprsLatitude);
    //
    //    char longchar[10]=""; // dddmm.hhW or -dddmm.hhW
    //    //char tmp_longchar[10]=""; // dddmm.hhW or -dddmm.hhW
    //    dtostrf(convertDegMin(gps.location.lng()), 8, 2, longchar); // dddmm.hhW  dddmm.hh is 8 characters long, with 2 characters after the decimal point.
    //
    //    // Determime to print E or W to the last char
    //
    //    if ( convertDegMin(gps.location.lng()) >= 0 ) {
    //      longchar[8] = 'E';
    //    } else if ( convertDegMin(gps.location.lng()) < 0 ) {
    //      longchar[8] = 'W';
    //    }
    //    u8x8.drawString(4, 4, longchar);
    //
    //    Serial.println(latchar);  //debug
    //    Serial.println(strlen(latchar));
    //    Serial.println(longchar); //debug
    //    Serial.println(strlen(longchar));


    // Serial.print(F(" "));
    //  Serial.print(deg_to_nmea(lat, true));
    //  Serial.print(F("/"));

    // Serial.print(deg_to_nmea(lon, false));

    
    //    // Print hour
    //    char hour_value_buffer[2];
    //    dtostrf(static_cast<int>(hour), 2, 0, hour_value_buffer); //convert Byte hour/minute/second to char buffer satellites_value_buffer[] two digitals, no digit after decimal point
    //    Serial.println(hour_value_buffer);
    //    int int_hour = static_cast<int>(hour);
    //    if (int_hour < 10) {
    //      u8x8.drawString(0, 7, "0");
    //      u8x8.drawString(1, 7, hour_value_buffer);
    //    } else {
    //      u8x8.drawString(0, 7, hour_value_buffer);
    //    };
    //
    //    u8x8.drawString(2, 7, ":");
    //
    //    // Print minute
    //    char minute_value_buffer[2];
    //    dtostrf(static_cast<int>(minute), 2, 0, hour_value_buffer); //convert Byte hour/minute/second to char buffer satellites_value_buffer[] two digitals, no digit after decimal point
    //    Serial.println(minute_value_buffer);
    //    int int_minute = static_cast<int>(minute);
    //
    //    if (int_minute < 10) {
    //      u8x8.drawString(3, 7, "0");
    //      u8x8.drawString(4, 7, minute_value_buffer);
    //    } else {
    //      u8x8.drawString(3, 7, minute_value_buffer);
    //    };
    //    u8x8.drawString(5, 7, ":");
    //
    //    // Print second
    //    char second_value_buffer[2];
    //    int int_second = static_cast<int>(second);
    //    dtostrf(static_cast<int>(second), 2, 0, second_value_buffer); //convert Byte hour/minute/second to char buffer satellites_value_buffer[] two digitals, no digit after decimal point
    //
    //    if (int_second < 10) {
    //      u8x8.drawString(6, 7, "0");
    //      u8x8.drawString(7, 7, second_value_buffer);
    //    } else {
    //      u8x8.drawString(6, 7, second_value_buffer);
    //      //   };
    

    // update time array
        Time[10] = static_cast<int>(second) / 10 + '0';
        Time[11] = static_cast<int>(second) % 10 + '0';
        Time[7]  = static_cast<int>(minute) / 10 + '0';
        Time[8] = static_cast<int>(minute) % 10 + '0';
        Time[4]  = static_cast<int>(hour)   / 10 + '0';
        Time[5]  = static_cast<int>(hour)   % 10 + '0';

        u8x8.clearLine(7); // Clear the bottom line of the LCD
        u8x8.drawString(0, 7, Time);  // Draw Time
        u8x8.refreshDisplay();    // only required for SSD1606/7

        setAprsUpdateFlag(); // We have valid GPS data, It is okay to send APRS packet.
        char comment []= "BH1RJC APRS Tracker test";
  }

  //Send raw packet to MicroModem
  if(APRS_Send_Timer.isReady() and send_aprs_update ){    // If APRS interval timer is ready(>30sec) and it is okay to send aprs(GPS data is valid.)
    Serial.print(F("!="));   // Construct Raw packet  !=ddmm.hhN/dddmm.hhE-PHG2410 An simple APRS tracker based on MicroAPRS
                              // Read http://www.aprs.net/vm/DOS/PROTOCOL.HTM
    Serial.print(deg_to_nmea(lat, true));   // Insert LAT
    Serial.print(F("/"));
    Serial.print(deg_to_nmea(lon, false));  // Insert LONG
    Serial.println(F(">PHG2410 An simple APRS tracker")); //  > = Car Insert PHG Power,ant/height/Gain, Symbol  and comment send_aprs_update
    send_aprs_update = false; // Reset APRS interval timer after send
    APRS_Send_Timer.reset(); // Reset APRS_Send_Timer
    drawSendIcon();  // Draw transmission icon
  }
  


}

/*
**  Sets flag, which will trigger aprs packet sending
*/
void setAprsUpdateFlag() {
  send_aprs_update = true;
}


void drawSendIcon() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  drawLeft(6, "X");   // Print "X"
  delay(500);
  drawLeft(6, " ");   // Print " "
}
 

    //
    //  ///////////////// Triggered by time updates ///////////////////////
    //  // Update LCD every second
    // // if (gps.time.isUpdated()) {
    //    //int hour = fix.dateTime.hours + 8;//中国时区设置需要+8
    //    //  int minute = fix.dateTime.minutes;
    //    enum {BufSizeTime = 3};
    //
    //    char hourchar[BufSizeTime];
    //    char minutechar[BufSizeTime];
    //    char secondchar[BufSizeTime];
    //
    //    snprintf (hourchar, BufSizeTime, "%d", gps.time.hour());
    //    snprintf (minutechar, BufSizeTime, "%d", gps.time.minute());
    //    snprintf (secondchar, BufSizeTime, "%d", gps.time.second());
    //
    //    u8x8.setFont(u8x8_font_chroma48medium8_r);
    //
    //    if (gps.time.hour() < 10) {
    //      u8x8.drawString(0, 7, "0");
    //      u8x8.drawString(1, 7, hourchar);
    //    } else {
    //      u8x8.drawString(0, 7, hourchar);
    //    };
    //
    //    u8x8.drawString(2, 7, ":");
    //
    //    if (gps.time.minute() < 10) {
    //      u8x8.drawString(3, 7, "0");
    //      u8x8.drawString(4, 7, minutechar);
    //    } else {
    //      u8x8.drawString(3, 7, minutechar);
    //    };
    //
    //    u8x8.drawString(5, 7, ":");
    //
    //    if (gps.time.second() < 10) {
    //      u8x8.drawString(6, 7, "0");
    //      u8x8.drawString(7, 7, secondchar);
    //    } else {
    //      u8x8.drawString(6, 7, secondchar);
    // //   };
    //
    //    u8x8.refreshDisplay();    // only required for SSD1606/7
    //
    //  }


     //timer.run();


  //delay(30000); // Pause for 30sec


  /*
  **  Convert degrees in long format to APRS string format
  **  DDMM.hhN for latitude and DDDMM.hhW for longitude
  **  D is degrees, M is minutes and h is hundredths of minutes.
  **  http://www.aprs.net/vm/DOS/PROTOCOL.HTM
  **  https://github.com/billygr/arduino-aprs-tracker/blob/master/arduino-aprs-tracker/arduino-aprs-tracker.ino
  */
char* deg_to_nmea(long deg, boolean is_lat) {
  bool is_negative = 0;
    if (deg < 0) is_negative = 1;

    // Use the absolute number for calculation and update the buffer at the end
    deg = labs(deg);

    unsigned long b = (deg % 1000000UL) * 60UL;
    unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
    b = (b % 1000000UL) / 10000UL;

    conv_buf[0] = '0';
    // in case latitude is a 3 digit number (degrees in long format)
    if ( a > 9999) {
      snprintf(conv_buf , 6, "%04lu", a);
    } else {
      snprintf(conv_buf + 1, 5, "%04lu", a);
    }

    conv_buf[5] = '.';
    snprintf(conv_buf + 6, 3, "%02lu", b);
    conv_buf[9] = '\0';
    if (is_lat) {
      if (is_negative) {
        conv_buf[8] = 'S';
      }
      else conv_buf[8] = 'N';
      return conv_buf + 1;
      // conv_buf +1 because we want to omit the leading zero
    }
    else {
      if (is_negative) {
        conv_buf[8] = 'W';
      }
      else conv_buf[8] = 'E';
      return conv_buf;
    }
  }



void initialMicroAPRS()
{
  delay(1500); //delay for MicroAPRS board boot up
  //String stringCMD_set_callsign = "c" + stringCallsign;
  //Serial.println(stringCMD_set_callsign); //Set your callsign to MicroAPRS modem
  //String stringCMD_set_SSID = "sc" + stringSSID;
  //Serial.println(stringCMD_set_SSID); //Set your SSID to MicroAPRS modem
  //Serial.println("S"); //Save configuration to MicroAPRS modem
    
}

void initializeTransmitter(){  // initialize VHF Transmitter through serial
  
}


//In case you want to use AT command,  read https://github.com/markqvist/MicroAPRS
// A lof of bugs in this function
void updateAPRSLocation(char* updateLat, char* updateLot)
{
  // Configure your callsign and SSID directly to the KISS modem board.
  //String stringCMD_set_callsign = "c" + stringCallsign;
  //Serial.println(stringCMD_set_callsign); //Set your callsign to MicroAPRS modem

  //String stringCMD_set_SSID = "sc" + stringSSID;
  //Serial.println(stringCMD_set_SSID); //Set your SSID to MicroAPRS modem

  //Serial.println("S"); //Save configuration to MicroAPRS modem
  char llaCommand[11]; // Set latitude (NMEA-format, eg 4903.50N)
  char lloCommand[12]; //Set latitude (NMEA-format, eg 07201.75W)
  strncpy(llaCommand, "lla", 3);
  strncpy(lloCommand, "llo", 3);
  
  Serial.println(llaCommand);
  Serial.println(updateLat);
  Serial.println(lloCommand);
  Serial.println(updateLot);
 // Serial.print(strncat(llaCommand,updateLat,8)); // Set latitude (NMEA-format, eg 4903.50N)
  Serial.println();
 // Serial.print(strncat(lloCommand,updateLot,9)); // Set latitude (NMEA-format, eg 07201.75W)
  Serial.println();
  Serial.println(llaCommand);
  Serial.println(updateLat);
  Serial.println(lloCommand);
  Serial.println(updateLot);


}

void drawLeft(int lin, char* str) { /* function drawLeft */
  //// Create a counter
  int col = 15 - String(str).length();
  u8x8.drawString(col, lin, str);
}


float convertDegMin(float decDeg) {

  float DegMin;

  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  DegMin = ( intDeg * 100 ) + decDeg;

  return DegMin;
}

  //boolean TxtoRadio(int type) {
  //
  //  char tmp[10];
  //  float latDegMin, lngDegMin = 0.0;
  //  String latOut, lngOut, cmtOut = "";
  //  //unsigned int Mem = freeRam();
  //  //float Volt = (float) readVcc()/1000;
  //
  //  float lastTxLat = gps.location.lat();
  //  float lastTxLng = gps.location.lng();
  //
  //  if ( lastTx > 6000 ) { // This prevent ANY condition to Tx below 6 secs
  //
  //    latDegMin = convertDegMin(lastTxLat);
  //    lngDegMin = convertDegMin(lastTxLng);
  //
  //    // Convert Lat float to string
  //    dtostrf(fabs(latDegMin), 2, 2, tmp );
  //    latOut.concat("lla");      // set latitute command
  //
  //    // Append 0 if Lat less than 10
  //    if ( fabs(lastTxLat) < 10 ) {
  //      latOut.concat("0");
  //    }
  //
  //    latOut.concat(tmp);      // Actual Lat in DDMM.MM
  //
  //    // Determine E or W
  //    if (latDegMin >= 0) {
  //      latOut.concat("N");
  //    } else if (latDegMin < 0) {
  //      latOut.concat("S");
  //    }
  //
  //    // Convert Lng float to string
  //    dtostrf(fabs(lngDegMin), 2, 2, tmp );
  //    lngOut.concat("llo");       // set longtitute command
  //
  //    // Append 0 if Lng less than 100
  //    if ( ( fabs(lastTxLng) < 100) ) {
  //      lngOut.concat("0");       // set longtitute command
  //    }
  //
  //    // Append 0 if Lng less than 10
  //    if ( fabs(lastTxLng) < 10 ) {
  //      latOut.concat("0");      // Append 0 if Lng less than 10
  //    }
  //
  //    lngOut.concat(tmp);      // Actual Lng in DDDMM.MM
  //
  //    // Determine E or W
  //    if (lngDegMin >= 0) {
  //      lngOut.concat("E");
  //    } else if (latDegMin < 0) {
  //      lngOut.concat("W");
  //    }
  //
  //
  //
  //    Serial.println(cmtOut);
  //
  //
  //    // Reset all tx timer & Tx variables
  //    txInterval = 80000;
  //    txTimer = millis();
  //    lastTx = 0;
  //
  //    // Tx success, return TRUE
  //    return 1;
  //  } else {
  //    return 0;
  //  }// endif lastTX > 6000
  //
  //}// endof TxtoRadio()


  //void displayInfo()
  //{
  //  Serial.print(F("Location: "));
  //  if (gps.location.isValid())
  //  {
  //    Serial.print(gps.location.lat(), 6);
  //    Serial.print(F(","));
  //    Serial.print(gps.location.lng(), 6);
  //  }
  //  else
  //  {
  //    Serial.print(F("INVALID"));
  //  }
  //
  //  Serial.print(F("  Date/Time: "));
  //  if (gps.date.isValid())
  //  {
  //    Serial.print(gps.date.month());
  //    Serial.print(F("/"));
  //    Serial.print(gps.date.day());
  //    Serial.print(F("/"));
  //    Serial.print(gps.date.year());
  //  }
  //  else
  //  {
  //    Serial.print(F("INVALID"));
  //  }
  //
  //  Serial.print(F(" "));
  //  if (gps.time.isValid())
  //  {
  //    if (gps.time.hour() < 10) Serial.print(F("0"));
  //    Serial.print(gps.time.hour());
  //    Serial.print(F(":"));
  //    if (gps.time.minute() < 10) Serial.print(F("0"));
  //    Serial.print(gps.time.minute());
  //    Serial.print(F(":"));
  //    if (gps.time.second() < 10) Serial.print(F("0"));
  //    Serial.print(gps.time.second());
  //    Serial.print(F("."));
  //    if (gps.time.centisecond() < 10) Serial.print(F("0"));
  //    Serial.print(gps.time.centisecond());
  //  }
  //  else
  //  {
  //    Serial.print(F("INVALID"));
  //  }
  //
  //  Serial.println();
  //}
