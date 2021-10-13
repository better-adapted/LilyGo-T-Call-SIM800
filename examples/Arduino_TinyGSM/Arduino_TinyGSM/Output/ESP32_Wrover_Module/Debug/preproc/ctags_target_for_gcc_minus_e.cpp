# 1 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"



// Please select the corresponding model

#define SIM800L_IP5306_VERSION_20190610 
// #define SIM800L_AXP192_VERSION_20200327
// #define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

// #define TEST_RING_RI_PIN            //Note will cancel the phone call test

// #define ENABLE_SPI_SDCARD   //Uncomment will test external SD card

// Define the serial console for debug prints, if needed
#define DUMP_AT_COMMANDS 
#define TINY_GSM_DEBUG SerialMon

# 20 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino" 2

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800 /* Modem is SIM800*/
#define TINY_GSM_RX_BUFFER 1024 /* Set RX buffer to 1Kb*/

# 31 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino" 2


# 34 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino" 2
StreamDebugger debugger(Serial1, Serial);
TinyGsm modem(debugger);




#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60*60*10 /* Time ESP32 will go to sleep (in seconds) */


// Your GPRS credentials (leave empty, if missing)
const char apn[] = ""; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[] = ""; // SIM card PIN code, if any

TinyGsmClient client(modem);
const int port = 80;

/*

This is just to demonstrate how to use SPI device externally.

Here we use SD card as a demonstration. In order to maintain versatility,

I chose three boards with free pins as SPI pins

*/
# 84 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
#define setupSDCard() 



void setupModem()
{

    // Keep reset high
    pinMode(5, 0x02);
    digitalWrite(5, 0x1);


    pinMode(4, 0x02);
    pinMode(23, 0x02);

    // Turn on the Modem power first
    digitalWrite(23, 0x1);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(4, 0x1);
    delay(100);
    digitalWrite(4, 0x0);
    delay(1000);
    digitalWrite(4, 0x1);

    // Initialize the indicator as an output
    pinMode(13, 0x02);
    digitalWrite(13, 0x0);
}

void turnOffNetlight()
{
    Serial.println("Turning off SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=0");
}

void turnOnNetlight()
{
    Serial.println("Turning on SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=1");
}

void setup()
{
    // Set console baud rate
    Serial.begin(115200);

    delay(10);

    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    ;

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    Serial1.begin(115200, 0x800001c, 26, 27);
}

class jsn_srt04_tank
{
public:
 const uint32_t tank_height_mm = 820;
 const uint32_t tank_cas_mm2 = 7698;
 const uint32_t tank_max_volume_ml = (tank_height_mm*tank_cas_mm2) / 1000;

 uint32_t reading_mm = 0;
 uint32_t duration_us = 0;
 uint32_t timeouts = 0;
 uint32_t tank_level_mm = 0;
 uint32_t tank_volume_ml = 0;
 float tank_volume_per = 0;

 int _io_pin_ultrasonic_trigger = -1;
 int _io_pin_ultrasonic_input = -1;

 jsn_srt04_tank(int pTrigger, int pInput)
 {
  _io_pin_ultrasonic_trigger = pTrigger;
  _io_pin_ultrasonic_input = pInput;
 }

 void take_reading()
 {
  // Trigger the sensor by setting the io_pin_ultrasonic_trigger high for 10 microseconds:
  digitalWrite(_io_pin_ultrasonic_trigger, 0x1);
  delayMicroseconds(50);
  digitalWrite(_io_pin_ultrasonic_trigger, 0x0);

  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
  duration_us = pulseIn(_io_pin_ultrasonic_input, 0x1, 10000);

  if (duration_us == 0)
  {
   timeouts++;

   tank_level_mm = 0;
   tank_volume_ml = 0;
   tank_volume_per = 0;
  }
  else
  {
   // Calculate the distance:
   reading_mm = duration_us * 0.343 / 2;

   if (reading_mm < tank_height_mm)
   {
    tank_level_mm = tank_height_mm - reading_mm;
   }
   else
   {
    // dont allow a negative level!
    tank_level_mm = 0;
   }

   tank_volume_ml = (tank_level_mm * tank_cas_mm2);
   tank_volume_ml /= 1000;

   tank_volume_per = (1000*tank_volume_ml);
   tank_volume_per /= tank_max_volume_ml;
   tank_volume_per /= 10;
  }

 }

 void print_reading()
 {
  char string_msg[200];
  sprintf(string_msg, "Ultrasonic,reading_mm:%d,timeouts:%d,tank_level_mm:%d,tank_volume_ml:%d,tank_volume_percentage:%.01f%c,@:%ld", reading_mm, timeouts, tank_level_mm, tank_volume_ml, tank_volume_per, '%', millis());
  Serial.println(string_msg);
 }
};

const int io_pin_ultrasonic_trigger = 17;
const int io_pin_ultrasonic_input = 16;
jsn_srt04_tank ultrasonic(io_pin_ultrasonic_trigger, io_pin_ultrasonic_input);

float batt_voltage_nominal=0.0;

String DateString = String("Oct 13 2021") + " " + String("11:10:59");
String VersionString = "1V00a";
String ProductString = "ESP32_SVP_1V00";

String Get_HTTP_Readings_String()
{
 //Check the current connection status		
 int httpCode;
 String str_mac = "";//String(WiFi.macAddress());
 String str_hash_temp = "HASH_TEMP";

 String str_tank_volume_per = String(ultrasonic.tank_volume_per);
 String str_tank_volume_ml = String(ultrasonic.tank_volume_ml);
 String str_tank_level_mm = String(ultrasonic.tank_level_mm);

 String str_batt_voltage_nominal = String(batt_voltage_nominal);

 String str_seconds_since_reboot = String(millis() / 1000);
 String str_sensor_raw_reading_mm = String(ultrasonic.reading_mm);
 String str_sensor_deadband_mm = String(batt_voltage_nominal);
 String str_error_message_string = String("none");

 String str_tank_height_mm = String(ultrasonic.tank_height_mm);
 String str_tank_cas_mm2 = String(ultrasonic.tank_cas_mm2);
 String str_tank_max_volume_ml = String(ultrasonic.tank_max_volume_ml);

 String VersionTemp = ProductString + " " + VersionString + " " + DateString;
 VersionTemp.replace(" ", "_");

 String str_software_version = VersionTemp;

 String values_string = "?";

 values_string += "MAC=" + str_mac;
 values_string += "&";
 values_string += "HASH=" + str_hash_temp;
 values_string += "&";
 values_string += "tank_volume_per=" + str_tank_volume_per;
 values_string += "&";
 values_string += "tank_volume_ml=" + str_tank_volume_ml;
 values_string += "&";
 values_string += "tank_level_mm=" + str_tank_level_mm;
 values_string += "&";
 values_string += "batt_voltage_nominal=" + str_batt_voltage_nominal;
 values_string += "&";
 values_string += "seconds_since_reboot=" + str_seconds_since_reboot;
 values_string += "&";
 values_string += "sensor_raw_reading_mm=" + str_sensor_raw_reading_mm;
 values_string += "&";
 values_string += "sensor_deadband_mm=" + str_sensor_deadband_mm;
 values_string += "&";
 values_string += "error_message_string=" + str_error_message_string;
 values_string += "&";
 values_string += "tank_height_mm=" + str_tank_height_mm;
 values_string += "&";
 values_string += "tank_cas_mm2=" + str_tank_cas_mm2;
 values_string += "&";
 values_string += "software_version=" + str_software_version;

 return values_string;
}


void loop()
{
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    modem.restart();

    // Turn off network status lights to reduce current consumption
    turnOffNetlight();

    // The status light cannot be turned off, only physically removed
    //turnOffStatuslight();

    // Or, use modem.init() if you don't need the complete restart
    String modemInfo = modem.getModemInfo();
    Serial.print("Modem: ");
    Serial.println(modemInfo);

    // Unlock your SIM card with a PIN if needed
    if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
        modem.simUnlock(simPIN);
    }

    Serial.print("Waiting for network...");
    if (!modem.waitForNetwork(240000L)) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

    // When the network connection is successful, turn on the indicator
    digitalWrite(13, 0x1);

    if (modem.isNetworkConnected()) {
        Serial.println("Network connected");
    }

    Serial.print(((reinterpret_cast<const __FlashStringHelper *>(("Connecting to APN: ")))));
    Serial.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

 char server[] = "www.indispensable.systems";
 String resource = "/esp32_readings_process.php" + Get_HTTP_Readings_String();
 int port = 80;

 ///esp32_readings_process.php
    Serial.print("Connecting to ");
    Serial.print(server);
    if (!client.connect(server, port)) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

    // Make a HTTP GET request:
    Serial.println("Performing HTTP GET request...");
    client.print(String("GET ") + resource + " HTTP/1.1\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");
    client.println();

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000L) {
        // Print available data
        while (client.available()) {
            char c = client.read();
            Serial.print(c);
            timeout = millis();
        }
    }
    Serial.println();

    // Shutdown
    client.stop();
    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Server disconnected")))));

    modem.gprsDisconnect();
    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("GPRS disconnected")))));


    // DTR is used to wake up the sleeping Modem
    // DTR is used to wake up the sleeping Modem
    // DTR is used to wake up the sleeping Modem
# 451 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
    // Make the LED blink three times before going to sleep
    int i = 3;
    while (i--) {
        digitalWrite(13, 0x1);
        modem.sendAT("+SPWM=0,1000,80");
        delay(500);
        digitalWrite(13, 0x0);
        modem.sendAT("+SPWM=0,1000,0");
        delay(500);
    }

    //After all off
    modem.poweroff();

 Serial.println();

    Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("Poweroff")))));

    esp_sleep_enable_timer_wakeup(60*60*10 /* Time ESP32 will go to sleep (in seconds) */ * 1000000ULL /* Conversion factor for micro seconds to seconds */);

    esp_deep_sleep_start();

    /*

    The sleep current using AXP192 power management is about 500uA,

    and the IP5306 consumes about 1mA

    */
# 477 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
}
