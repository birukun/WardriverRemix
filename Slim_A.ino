// Streamlined the JHewitt wardriver.uk
// Simplified to power on and scan. Data must be read off of the microSD card separately. 
// Removed Setup, Access Point, BLE scanning and SIM800L code as well
// Base code by Joseph Hewitt 2023
// Slimming down done by MrBill
// Compatible with JHewitt board and MrBill WardriverRemix board
//This code is for the ESP32 "Side A" of the wardriver hardware revision 3.

const String VERSION = "1.1.420";

#include <GParser.h>
#include <MicroNMEA.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h" 
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <ESP32Time.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels 32 for thin, 64 for square
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define gps_allow_stale_time 60000 //ms to allow stale gps data for when lock lost.

//These variables are used for buffering/caching GPS data.
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
unsigned long lastgps = 0;
String last_dt_string = "";
String last_lats = "";
String last_lons = "";

//Automatically set to true if a blocklist was loaded.
boolean use_blocklist = false;
//millis() when the last block happened.
unsigned long wifi_block_at = 0;

//These variables are used to populate the LCD with statistics.
float temperature;
unsigned int wifi_count;
unsigned int disp_wifi_count;

uint32_t chip_id;

File filewriter;

Preferences preferences;
unsigned long bootcount = 0;

/* The recently seen MAC addresses and cell towers are saved into these arrays so that the 
 * wardriver can detect if they have already been written to the Wigle CSV file.
 * These _len definitions define how large those arrays should be. Larger is better but consumes more RAM.
 */
#define mac_history_len 512
#define blocklist_len 20
//Max blocklist entry length. 32 = max SSID len.
#define blocklist_str_len 32

//TIME ADDITIONS - HHB
//int setUnixtime(int32_t unixtime) {
//  timeval epoch = {unixtime, 0};
//  return settimeofday((const timeval*)&epoch, 0);
//}


struct mac_addr {
   unsigned char bytes[6];
};

struct coordinates {
  double lat;
  double lon;
  int acc;
};

struct block_str {
  char characters[blocklist_str_len];
};

struct mac_addr mac_history[mac_history_len];
unsigned int mac_history_cursor = 0;

struct block_str block_list[blocklist_len];

unsigned long lcd_last_updated;

#define YEAR_2020 1577836800 //epoch value for 1st Jan 2020; dates older than this are considered wrong (this code was written after 2020).
unsigned long epoch;
unsigned long epoch_updated_at;

TaskHandle_t primary_scan_loop_handle;

boolean b_working = false; //Set to true when we receive some valid data from side B.

#define DEVICE_UNKNOWN 254
#define DEVICE_CUSTOM  0
#define DEVICE_REV3    1
#define DEVICE_REV3_5  2
#define DEVICE_REV4    3
byte DEVICE_TYPE = DEVICE_UNKNOWN;

void setup_wifi(){
  //Gets the WiFi ready for scanning by disconnecting from networks and changing mode.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

void clear_display(){
  //Clears the LCD and resets the cursor.
  display.clearDisplay();
  display.setCursor(0, 0);
}

void boot_config(){
  //Load configuration variables and perform first time setup if required.
  Serial.println("Setting/loading boot config..");

  preferences.begin("wardriver", false);
  bool firstrun = preferences.getBool("first", true);
  bool doreset = preferences.getBool("reset", false);
  bootcount = preferences.getULong("bootcount", 0);
  
  Serial.println("Loaded variables");

  DEVICE_TYPE = preferences.getShort("model", DEVICE_UNKNOWN);
  DEVICE_TYPE = identify_model();
  preferences.putShort("model", DEVICE_TYPE);

  setup_wifi();

  bootcount++;
  preferences.putULong("bootcount", bootcount);
}

void setup() {
    delay(500);
    
    Serial.begin(115200);
    Serial.print("Starting v");
    Serial.println(VERSION);

    for(int i=0; i<17; i=i+8) {
      chip_id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }

    Serial.print("Chip ID: ");
    Serial.println(chip_id);

    Serial1.begin(115200,SERIAL_8N1,27,14);
    Serial2.begin(9600);
    
    //ADDED TO ADJUST TIME at setup
    update_epoch(); 

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
      Serial.println(F("SSD1306 allocation failed"));
    }
    display.setRotation(2);
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.println("Starting");
    display.print("Version ");
    display.println(VERSION);
    display.display();
    
    int reset_reason = esp_reset_reason();
    if (reset_reason != ESP_RST_POWERON){
      clear_display();
      display.println("Unexpected reset");
      display.print("Code ");
      display.println(reset_reason);
      display.print("Version ");
      display.println(VERSION);
      display.display();
      delay(4000);
    }
  
  //ADDED TO ADJUST TIME at setup
    update_epoch(); 

    delay(1500);
  
    if(!SD.begin()){
        Serial.println("SD Begin failed!");
        clear_display();
        display.println("SD Begin failed!");
        display.display();
        delay(4000);
    }
    uint8_t cardType = SD.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached!");
        clear_display();
        display.println("No SD Card!");
        display.display();
        delay(10000);
    }
  
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
  
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    while (!filewriter){
      filewriter = SD.open("/test.txt", FILE_APPEND);
      if (!filewriter){
        Serial.println("Failed to open file for writing.");
        clear_display();
        display.println("SD File open failed!");
        display.display();
        delay(1000);
      }
    }
    int wrote = filewriter.print("\n_BOOT_");
    filewriter.print(VERSION);
    filewriter.print(", ut=");
    filewriter.print(micros());
    filewriter.print(", rr=");
    filewriter.print(reset_reason);
    filewriter.print(", id=");
    filewriter.print(chip_id);
    filewriter.flush();
    if (wrote < 1){
      while(true){
        Serial.println("Failed to write to SD card!");
        clear_display();
        display.println("SD Card write failed!");
        display.display();
        delay(4000);
      }
    }
    
    boot_config();
    setup_wifi();

    Serial.print("This device: ");
    Serial.println(device_type_string());
    
    filewriter.print(", bc=");
    filewriter.print(bootcount);
    filewriter.print(", ep=");
    filewriter.print(epoch);
    filewriter.flush();
    filewriter.close();

    if (SD.exists("/bl.txt")){
      Serial.println("Opening blocklist");
      File blreader;
      blreader = SD.open("/bl.txt", FILE_READ);
      byte i = 0;
      byte ci = 0;
      while (blreader.available()){
        char c = blreader.read();
        if (c == '\n' || c == '\r'){
          use_blocklist = true;
          if (ci != 0){
            i += 1;
          }
          ci = 0;
        } else {
          block_list[i].characters[ci] = c;
          ci += 1;
          if (ci >= blocklist_str_len){
            Serial.println("Blocklist line too long!");
            ci = 0;
          }
        }
      }
      blreader.close();
    }

    if (!use_blocklist){
      Serial.println("Not using a blocklist");
    }
    
    Serial.println("Opening destination file for writing");

    String filename = "/HHB-";   //Added HHB and updated version to x.x.420
    filename = filename + bootcount;
    filename = filename + ".csv";
    Serial.println(filename);
    filewriter = SD.open(filename, FILE_APPEND);
    filewriter.print("WigleWifi-1.4,appRelease=" + VERSION + ",model=HHB " + device_type_string() + ",release=1.0.420,device=wardriver.uk " + device_type_string() + ",display=i2c LCD,board=wardriver.uk " + device_type_string() + ",brand=JHewitt\nMAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type\n");
    filewriter.flush();
    
    clear_display();
    display.println("Starting main..");
    display.display();

    xTaskCreatePinnedToCore(
      primary_scan_loop, /* Function to implement the task */
      "primary_scan_loop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      3,  /* Priority of the task */
      &primary_scan_loop_handle,  /* Task handle. */
      0); /* Core where the task should run */
}

void primary_scan_loop(void * parameter){
  //This core will be dedicated entirely to WiFi scanning in an infinite loop.
  while (true){
    disp_wifi_count = wifi_count;
    wifi_count = 0;
    setup_wifi();
    for(int scan_channel = 1; scan_channel < 14; scan_channel++){
      yield();
      //scanNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan, uint8_t channel)
      int n = WiFi.scanNetworks(false,true,false,110,scan_channel);
      if (n > 0){
        wifi_count = wifi_count + n;
        for (int i = 0; i < n; i++) {
          if (seen_mac(WiFi.BSSID(i))){
            //Skip any APs which we've already logged.
            continue;
          }
          //Save the AP MAC inside the history buffer so we know it's logged.
          save_mac(WiFi.BSSID(i));

          String ssid = WiFi.SSID(i);
          ssid.replace(",","_");
          if (use_blocklist){
            if (is_blocked(ssid)){
              Serial.print("BLOCK: ");
              Serial.println(ssid);
              wifi_block_at = millis();
              continue;
            }
            String tmp_mac_str = WiFi.BSSIDstr(i).c_str();
            tmp_mac_str.toUpperCase();
            if (is_blocked(tmp_mac_str)){
              Serial.print("BLOCK: ");
              Serial.println(tmp_mac_str);
              wifi_block_at = millis();
              continue;
            }
          }
          
          filewriter.printf("%s,%s,%s,%s,%d,%d,%s,WIFI\n", WiFi.BSSIDstr(i).c_str(), ssid.c_str(), security_int_to_string(WiFi.encryptionType(i)).c_str(), dt_string().c_str(), WiFi.channel(i), WiFi.RSSI(i), gps_string().c_str());
         
        }
      }
      filewriter.flush();
    }
    yield();
  }
}

void lcd_show_stats(){
  //Clear the LCD then populate it with stats about the current session.
  boolean ble_did_block = false;
  boolean wifi_did_block = false;
  if (millis() - wifi_block_at < 30000){
    wifi_did_block = true;
  }
  clear_display();
  display.print("WiFi:");
  display.print(disp_wifi_count);
  if (wifi_did_block){
    display.print("X");
  }
  if (int(temperature) != 0){
    display.print(" Temp:");
    display.print(temperature);
    display.print("c");
  }
  display.println();
  if (nmea.getHDOP() < 250 && nmea.getNumSatellites() > 0){
    display.print("HDOP:");
    display.print(nmea.getHDOP());
    display.print(" Sats:");
    display.print(nmea.getNumSatellites());
    display.println(nmea.getNavSystem());
  } else {
    display.print("No GPS: ");
  }
  if (b_working){
  display.print("BLE:");
    if (ble_did_block){
    display.print("X");
  }
  display.print(" GSM:");
    } else {
    display.println("NO BLE / GSM");
  }
  display.println(dt_string());
  display.display();
  
}

void loop(){
  //The main loop for the second core; handles GPS, "Side B" communication, and LCD refreshes.
  update_epoch();
  while (Serial2.available()){
    char c = Serial2.read();
    if (nmea.process(c)){
      if (nmea.isValid()){
        lastgps = millis();
        update_epoch();
      }
    }
  }

  if (Serial1.available()){
    String bside_buffer = "";
    while (true){
      char c;
      if (Serial1.available()){
        c = Serial1.read();
        if (c != '\n'){
          bside_buffer += c;
        } else {
          break;
        }
      }
    }
    Serial.println(bside_buffer);
    String towrite = "";
    towrite = parse_bside_line(bside_buffer);
    if (towrite.length() > 1){
      Serial.println(towrite);
      filewriter.print(towrite);
      filewriter.print("\n");
      filewriter.flush();
    }
    
  }
  if (lcd_last_updated == 0 || millis() - lcd_last_updated > 1000){
    lcd_show_stats();
    lcd_last_updated = millis();
  }
}

boolean is_blocked(String test_str){
  if (!use_blocklist){
    return false;
  }
  unsigned int test_str_len = test_str.length();
  if (test_str_len == 0){
    return false;
  }
  if (test_str_len > blocklist_str_len){
    Serial.print("Refusing to blocklist check due to length: ");
    Serial.println(test_str);
    return false;
  }
  for (byte i=0; i<blocklist_len; i++){
    boolean matched = true;
    for (byte ci=0; ci<test_str_len; ci++){
      if (test_str.charAt(ci) != block_list[i].characters[ci]){
        matched = false;
        break;
      }
    }
    if (matched){
      Serial.print("Blocklist match: ");
      Serial.println(test_str);
      return true;
    }
  }
  return false;
}

void save_mac(unsigned char* mac){
  //Save a MAC address into the recently seen array.
  if (mac_history_cursor >= mac_history_len){
    mac_history_cursor = 0;
  }
  struct mac_addr tmp;
  for (int x = 0; x < 6 ; x++){
    tmp.bytes[x] = mac[x];
  }

  mac_history[mac_history_cursor] = tmp;
  mac_history_cursor++;
  Serial.print("Mac len ");
  Serial.println(mac_history_cursor);
}

boolean seen_mac(unsigned char* mac){
  //Return true if this MAC address is in the recently seen array.

  struct mac_addr tmp;
  for (int x = 0; x < 6 ; x++){
    tmp.bytes[x] = mac[x];
  }

  for (int x = 0; x < mac_history_len; x++){
    if (mac_cmp(tmp, mac_history[x])){
      return true;
    }
  }
  return false;
}

void print_mac(struct mac_addr mac){
  //Print a mac_addr struct nicely.
  for (int x = 0; x < 6 ; x++){
    Serial.print(mac.bytes[x],HEX);
    Serial.print(":");
  }
}

boolean mac_cmp(struct mac_addr addr1, struct mac_addr addr2){
  //Return true if 2 mac_addr structs are equal.
  for (int y = 0; y < 6 ; y++){
    if (addr1.bytes[y] != addr2.bytes[y]){
      return false;
    }
  }
  return true;
}

String parse_bside_line(String buff){
  //Provide a String which contains a line from ESP32 side B.
  //A String will be returned which should be written to the Wigle CSV.

  /*
  I am aware that this code isn't great..
  I'm not a huge fan of Strings, especially not this many temporary Strings but it's a quick way to get things working.
  Hopefully the large amount of RAM on the ESP32 will prevent heap fragmentation issues but I'll do extended uptime tests to be sure.
  
  This code sucessfully ran for 48 hours straight starting on the 1st September 2021.
  Multiple tests have since completed with no crashes or issues. While it looks scary, it seems to be stable.
  */
  
  String out = "";
  
  if (buff.indexOf("WI0,") > -1) {
    //WI0,SSID,6,-88,5,00:00:00:00:00:00
    int startpos = buff.indexOf("WI0,")+4;
    int endpos = buff.indexOf(",",startpos);
    String ssid = buff.substring(startpos,endpos);

    startpos = endpos+1;
    endpos = buff.indexOf(",",startpos);
    String channel = buff.substring(startpos,endpos);

    startpos = endpos+1;
    endpos = buff.indexOf(",",startpos);
    String rssi = buff.substring(startpos,endpos);

    startpos = endpos+1;
    endpos = buff.indexOf(",",startpos);
    String security_raw = buff.substring(startpos,endpos);

    startpos = endpos+1;
    endpos = startpos+17;
    String mac_str = buff.substring(startpos,endpos);
    mac_str.toUpperCase();

    if (is_blocked(ssid) || is_blocked(mac_str)){
      out = "";
      Serial.print("BLOCK: ");
      Serial.print(ssid);
      Serial.print(" / ");
      Serial.println(mac_str);
      wifi_block_at = millis();
      return out;
    }

    unsigned char mac_bytes[6];
    int values[6];

    if (6 == sscanf(mac_str.c_str(), "%x:%x:%x:%x:%x:%x%*c", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5])){
      for(int i = 0; i < 6; ++i ){
          mac_bytes[i] = (unsigned char) values[i];
      }
    
      if (!seen_mac(mac_bytes)){
        save_mac(mac_bytes);
        //Save to SD?
        Serial.print("NEW WIFI (SIDE B): ");
        Serial.println(buff);

        String authtype = security_int_to_string((int) security_raw.toInt());
        
        out = mac_str + "," + ssid + "," + authtype + "," + dt_string() + "," + channel + "," + rssi + "," + gps_string() + ",WIFI";
      }
    }
    b_working = true;
  }


  if (buff.indexOf("TEMP,") > -1) {
    int startpos = buff.indexOf("TEMP,")+5;
    String temp = buff.substring(startpos);
    temperature = temp.toFloat();
    Serial.print("Temperature = ");
    Serial.println(temperature);
  }
  
  return out;
}

String dt_string(){
  //Return a datetime String using local timekeeping and GPS data.
  time_t now = epoch;
  struct tm ts;
  char buf[80];

  ts = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ts);
  String out = String(buf);
  
  Serial.print("New dt_str: ");
  Serial.println(out);
  
  return out;
}

String dt_string_from_gps(){
  //Return a datetime String using GPS data only.
  String datetime = "";
  if (nmea.isValid() && nmea.getYear() > 0){
    datetime += nmea.getYear();
    datetime += "-";
    datetime += nmea.getMonth();
    datetime += "-";
    datetime += nmea.getDay();
    datetime += " ";
    datetime += nmea.getHour();
    datetime += ":";
    datetime += nmea.getMinute();
    datetime += ":";
    datetime += nmea.getSecond();
    last_dt_string = datetime;
  } else if (lastgps + gps_allow_stale_time > millis()) {
    datetime = last_dt_string;
  }
  return datetime;
}

String gps_string(){
  //Return a String which can be used in a Wigle CSV line to show the current position.
  //This uses data from GPS and GSM tower locations.
  //output: lat,lon,alt,acc
  String out = "";
  long alt = 0;
  if (!nmea.getAltitude(alt)){
    alt = 0;
  }
  float altf = (float)alt / 1000;

  String lats = String((float)nmea.getLatitude()/1000000, 7);
  String lons = String((float)nmea.getLongitude()/1000000, 7);
  if (nmea.isValid() && nmea.getHDOP() <= 250){
    last_lats = lats;
    last_lons = lons;
  }

  if (nmea.getHDOP() > 250){
    lats = "";
    lons = "";
  }

  //HDOP returned here is in tenths and needs dividing by 10 to make it 'true'.
  //We're using this as a very basic estimate of accuracy by multiplying HDOP with the precision of the GPS module (2.5)
  //This isn't precise at all, but is a very rough estimate to your GPS accuracy.
  float accuracy = ((float)nmea.getHDOP()/10);
  accuracy = accuracy * 2.5;

  if (!nmea.isValid()){
    lats = "";
    lons = "";
    accuracy = 1000;
    if (lastgps + gps_allow_stale_time > millis()){
      lats = last_lats;
      lons = last_lons;
      accuracy = 5 + (millis() - lastgps) / 100;
    } else {
      Serial.println("Bad GPS");
      
    }
  }

  //The module we are using has a precision of 2.5m, accuracy can never be better than that.
  if (accuracy <= 2.5){
    accuracy = 2.5;
  }

  out = lats + "," + lons + "," + altf + "," + accuracy;
  return out;
}

String security_int_to_string(int security_type){
  //Provide a security type int from WiFi.encryptionType(i) to convert it to a String which Wigle CSV expects.
  String authtype = "";
  switch (security_type){
    case WIFI_AUTH_OPEN:
      authtype = "[OPEN]";
      break;
  
    case WIFI_AUTH_WEP:
      authtype = "[WEP]";
      break;
  
    case WIFI_AUTH_WPA_PSK:
      authtype = "[WPA_PSK]";
      break;
  
    case WIFI_AUTH_WPA2_PSK:
      authtype = "[WPA2_PSK]";
      break;
  
    case WIFI_AUTH_WPA_WPA2_PSK:
      authtype = "[WPA_WPA2_PSK]";
      break;
  
    case WIFI_AUTH_WPA2_ENTERPRISE:
      authtype = "[WPA2]";
      break;

    //Requires at least v2.0.0 of https://github.com/espressif/arduino-esp32/
    case WIFI_AUTH_WPA3_PSK:
      authtype = "[WPA3_PSK]";
      break;

    case WIFI_AUTH_WPA2_WPA3_PSK:
      authtype = "[WPA2_WPA3_PSK]";
      break;

    case WIFI_AUTH_WAPI_PSK:
      authtype = "[WAPI_PSK]";
      break;
        
    default:
      authtype = "[UNDEFINED]";
  }

  return authtype;
}

String get_latest_datetime(String filename, boolean date_only){
  //Provide a filename to get the highest datetime from that Wigle CSV file on the SD card.
  Serial.print("Getting latest dt from ");
  Serial.println(filename);
  String buff = "";
  
  File reader = SD.open(filename, FILE_READ);
  int seekto = reader.size()-512;
  if (seekto < 1){
    seekto = 0;
  }
  reader.seek(seekto);
  int ccount = 0;
  while (reader.available()){
    char c = reader.read();
    if (c == '\n' || c == '\r'){
      if (ccount == 10){
        int startpos = buff.indexOf("],2");
        int endpos = buff.indexOf(",",startpos+3);
        if (startpos > 0 && endpos > 0){
          String dt = buff.substring(startpos+2,endpos);
          Serial.print("Got: ");
          Serial.println(dt);
          if (date_only){
            int spacepos = dt.indexOf(" ");
            String new_dt = dt.substring(0,spacepos);
            dt = new_dt;
            Serial.print("Stripped to: ");
            Serial.println(dt);
          }
          return dt;
        } 
      }
      ccount = 0;
      buff = "";
    } else {
      buff.concat(c);
      if (c == ','){
        ccount++;
      }
    }
  }
  return "";
}

void update_epoch(){
  //Update the global epoch variable using the GPS time source.
  String gps_dt = dt_string_from_gps();
  if (!nmea.isValid() || lastgps == 0 || gps_dt.length() < 5){
    unsigned int tdiff_sec = (millis()-epoch_updated_at)/1000;
    if (tdiff_sec < 1){
      return;
    }
    epoch += tdiff_sec;
    epoch_updated_at = millis();
    Serial.print("Added ");
    Serial.print(tdiff_sec);
    Serial.println(" seconds to epoch");
    return;
  }
  
  struct tm tm;

  strptime(gps_dt.c_str(), "%Y-%m-%d %H:%M:%S", &tm );
  epoch = mktime(&tm);
  epoch_updated_at = millis();
}

unsigned long getTime() {
  //Use NTP to get the current epoch value.
  
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return(0);
  }
  time(&now);
  Serial.print("Got time from NTP: ");
  Serial.println(now);
  return now;
}

String device_type_string(){
  String ret = "";
  switch (DEVICE_TYPE){
    case DEVICE_REV3:
      ret = "rev3";
      break;
      
    case DEVICE_REV3_5:
      ret = "rev3 5GHz";
      break;

    case DEVICE_REV4:
      ret = "rev4";
      break;

    case DEVICE_CUSTOM:
      ret = "generic";
      break;

    default:
      ret = "generic";
  }
  
  return ret;
}

byte identify_model(){
  //Block until we know for sure what hardware model this is. Can take a while so cache the response.
  //Return a byte indicating the model, such as DEVICE_REV3.
  //Only call *before* the main loops start, otherwise multiple threads could be trying to access the serial.

  if (DEVICE_TYPE != DEVICE_UNKNOWN){
    //We already know.
    Serial.print("Device already identified as ");
    Serial.println(device_type_string());
    return DEVICE_TYPE;
  }
  
  Serial.println("Identifying hardware..");
  
  //TODO: For models without "side B" serial, detect their respective bus here and do an immediate return.

  //For anything which is rev 3-ish, listen to the "side B" serial for a while.
  //Timeout after a while in case "side B" is dead/missing, it's technically optional.
  int i = 0;
  String buff = "";
  int bufflen = 0;
  while (i < 10000){
    if (Serial1.available()){
      char c = Serial1.read();
      if (c == '\n' || c == '\r'){
        //Handle buff.
        if (buff.indexOf("BLC,") > -1){
          Serial.println("Identified Rev3 (cm)");
          return DEVICE_REV3;
        }
        if (buff.indexOf("REV3!") > -1){
          Serial.println("Identified Rev3");
          return DEVICE_REV3;
        }
        if (buff.indexOf("5GHz,") > -1){
          Serial.println("Identified Rev3 5Ghz (cm)");
          return DEVICE_REV3_5;
        }
        if (buff.indexOf("!REV3.5") > -1){
          Serial.println("Identified Rev3 5Ghz");
          return DEVICE_REV3_5;
        }

        buff = "";
      }
      buff.concat(c);
      bufflen++;
      if (bufflen > 70){
        bufflen = 0;
        buff = "";
      }
      
    }
    delay(1);
    i++;
  }
  Serial.println("Failed to identify hardware");
  return DEVICE_UNKNOWN;
}
