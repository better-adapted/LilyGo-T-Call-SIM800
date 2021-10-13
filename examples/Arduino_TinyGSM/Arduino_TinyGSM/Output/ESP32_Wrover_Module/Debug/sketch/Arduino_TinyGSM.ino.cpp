#include <Arduino.h>
#line 1 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"


//http://indispensable.systems/esp32_readings_process.php?MAC=C4:4F:33:53:02:A1&HASH=HASH_TEMP&tank_volume_per=0.00&tank_volume_ml=0&tank_level_mm=0&vend_on_time_ms=300&vend_total_count=0&batt_voltage_nominal=0.00&batt_voltage_vend=0.00&seconds_since_reboot=0&sensor_raw_reading_mm=0&sensor_deadband_mm=0.00&error_message_string=none&tank_height_mm=820&tank_cas_mm2=7698&software_version=ESP32_Ultrasonic_1V00_1V00a_Oct_13_2021_10:28:57

// Please select the corresponding model

#define SIM800L_IP5306_VERSION_20190610
// #define SIM800L_AXP192_VERSION_20200327
// #define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

// #define TEST_RING_RI_PIN            //Note will cancel the phone call test

// #define ENABLE_SPI_SDCARD   //Uncomment will test external SD card

// Define the serial console for debug prints, if needed
#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG          SerialMon

#include "utilities.h"

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60*60*10        /* Time ESP32 will go to sleep (in seconds) */


// Your GPRS credentials (leave empty, if missing)
const char apn[]      = ""; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = ""; // SIM card PIN code, if any

TinyGsmClient client(modem);
const int  port = 80;

/*
This is just to demonstrate how to use SPI device externally.
Here we use SD card as a demonstration. In order to maintain versatility,
I chose three boards with free pins as SPI pins
*/
#ifdef ENABLE_SPI_SDCARD

#include "FS.h"
#include "SD.h"
#include <SPI.h>

SPIClass SPI1(HSPI);

#define MY_CS       33
#define MY_SCLK     25
#define MY_MISO     27
#define MY_MOSI     26

void setupSDCard()
{
    SPI1.begin(MY_SCLK, MY_MISO, MY_MOSI, MY_CS);
    //Assuming use of SPI SD card
    if (!SD.begin(MY_CS, SPI1)) {
        Serial.println("Card Mount Failed");
    } else {
        Serial.println("SDCard Mount PASS");
        String size = String((uint32_t)(SD.cardSize() / 1024 / 1024)) + "MB";
        Serial.println(size);
    }
}
#else
#define setupSDCard()
#endif


#line 89 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
void setupModem();
#line 115 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
void turnOffNetlight();
#line 121 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
void turnOnNetlight();
#line 127 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
void setup();
#line 232 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
String Get_HTTP_Readings_String();
#line 291 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
void loop();
#line 89 "D:\\SourceTree\\LilyGo-T-Call-SIM800-BA\\examples\\Arduino_TinyGSM\\Arduino_TinyGSM\\sketches\\Arduino_TinyGSM.ino"
void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);

    // Pull down PWRKEY for more than 1 second according to manual requirements
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LED_OFF);
}

void turnOffNetlight()
{
    SerialMon.println("Turning off SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=0");
}

void turnOnNetlight()
{
    SerialMon.println("Turning on SIM800 Red LED...");
    modem.sendAT("+CNETLIGHT=1");
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);

    delay(10);

    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    setupSDCard();

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
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
		digitalWrite(_io_pin_ultrasonic_trigger, HIGH);
		delayMicroseconds(50);
		digitalWrite(_io_pin_ultrasonic_trigger, LOW);
  
		// Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
		duration_us = pulseIn(_io_pin_ultrasonic_input, HIGH, 10000);

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
  
			tank_volume_ml =   (tank_level_mm * tank_cas_mm2);
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

const int io_pin_ultrasonic_trigger = 32;
const int io_pin_ultrasonic_input = 33;
jsn_srt04_tank ultrasonic(io_pin_ultrasonic_trigger, io_pin_ultrasonic_input);

float batt_voltage_nominal=0.0;

String DateString = String(__DATE__) + " " + String(__TIME__);
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
	String str_sensor_raw_reading_mm =  String(ultrasonic.reading_mm);
	String str_sensor_deadband_mm = String(batt_voltage_nominal);
	String str_error_message_string = String("none");
		
	String str_tank_height_mm = String(ultrasonic.tank_height_mm);
	String str_tank_cas_mm2 = String(ultrasonic.tank_cas_mm2);
	String str_tank_max_volume_ml = String(ultrasonic.tank_max_volume_ml);
		
	String VersionTemp = ProductString + " " + VersionString + " " + DateString;		
	VersionTemp.replace(" ", "_");
		
	String str_software_version = VersionTemp;		

	String values_string = "?";

	values_string += "MAC="  + str_mac;
	values_string += "&";
	values_string += "HASH=" + str_hash_temp;
	values_string += "&";
	values_string += "tank_volume_per="  + str_tank_volume_per;
	values_string += "&";
	values_string += "tank_volume_ml="  + str_tank_volume_ml;
	values_string += "&";
	values_string += "tank_level_mm="  + str_tank_level_mm;		
	values_string += "&";
	values_string += "batt_voltage_nominal="  + str_batt_voltage_nominal;
	values_string += "&";
	values_string += "seconds_since_reboot="  + str_seconds_since_reboot;
	values_string += "&";
	values_string += "sensor_raw_reading_mm="  + str_sensor_raw_reading_mm;
	values_string += "&";
	values_string += "sensor_deadband_mm="  + str_sensor_deadband_mm;
	values_string += "&";
	values_string += "error_message_string="  + str_error_message_string;
	values_string += "&";
	values_string += "tank_height_mm="  + str_tank_height_mm;		
	values_string += "&";
	values_string += "tank_cas_mm2="  + str_tank_cas_mm2;		
	values_string += "&";
	values_string += "software_version="  + str_software_version;
			
	return values_string;			
}


void loop()
{
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
	
    digitalWrite(io_pin_ultrasonic_trigger, HIGH);	
	pinMode(io_pin_ultrasonic_trigger, OUTPUT);
	pinMode(io_pin_ultrasonic_input, INPUT);
	delay(1000);
	
	for (int x = 0; x < 3; x++)
	{		
		ultrasonic.take_reading();
		ultrasonic.print_reading();
		delay(1000);
	}
	

	
    SerialMon.println("Initializing modem...");
    modem.restart();

    // Turn off network status lights to reduce current consumption
    turnOffNetlight();

    // The status light cannot be turned off, only physically removed
    //turnOffStatuslight();

    // Or, use modem.init() if you don't need the complete restart
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem: ");
    SerialMon.println(modemInfo);

    // Unlock your SIM card with a PIN if needed
    if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
        modem.simUnlock(simPIN);
    }
	
    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork(240000L)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");

    // When the network connection is successful, turn on the indicator
    digitalWrite(LED_GPIO, LED_ON);

    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

    SerialMon.print(F("Connecting to APN: "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");
	
	char server[] = "www.indispensable.systems";
	String resource = "/esp32_readings_process.php" + Get_HTTP_Readings_String();
	int port = 80;

	///esp32_readings_process.php
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" OK");

    // Make a HTTP GET request:
    SerialMon.println("Performing HTTP GET request...");
    client.print(String("GET ") + resource + " HTTP/1.1\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");
    client.println();

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000L) {
        // Print available data
        while (client.available()) {
            char c = client.read();
            SerialMon.print(c);
            timeout = millis();
        }
    }
    SerialMon.println();

    // Shutdown
    client.stop();
    SerialMon.println(F("Server disconnected"));

    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));


    // DTR is used to wake up the sleeping Modem
    // DTR is used to wake up the sleeping Modem
    // DTR is used to wake up the sleeping Modem
#ifdef MODEM_DTR
    bool res;

    modem.sleepEnable();

    delay(100);

    // test modem response , res == 0 , modem is sleep
    res = modem.testAT();
    Serial.print("SIM800 Test AT result -> ");
    Serial.println(res);

    delay(1000);

    Serial.println("Use DTR Pin Wakeup");
    pinMode(MODEM_DTR, OUTPUT);
    //Set DTR Pin low , wakeup modem .
    digitalWrite(MODEM_DTR, LOW);


    // test modem response , res == 1 , modem is wakeup
    res = modem.testAT();
    Serial.print("SIM800 Test AT result -> ");
    Serial.println(res);

#endif


#ifdef TEST_RING_RI_PIN
#ifdef MODEM_RI
    // Swap the audio channels
    SerialAT.print("AT+CHFA=1\r\n");
    delay(2);

    //Set ringer sound level
    SerialAT.print("AT+CRSL=100\r\n");
    delay(2);

    //Set loud speaker volume level
    SerialAT.print("AT+CLVL=100\r\n");
    delay(2);

    // Calling line identification presentation
    SerialAT.print("AT+CLIP=1\r\n");
    delay(2);

    //Set RI Pin input
    pinMode(MODEM_RI, INPUT);

    Serial.println("Wait for call in");
    //When is no calling ,RI pin is high level
    while (digitalRead(MODEM_RI)) {
        Serial.print('.');
        delay(500);
    }
    Serial.println("call in ");

    //Wait 10 seconds for the bell to ring
    delay(10000);

    //Accept call
    SerialAT.println("ATA");


    delay(10000);

    // Wait ten seconds, then hang up the call
    SerialAT.println("ATH");
#endif  //MODEM_RI
#endif  //TEST_RING_RI_PIN

    // Make the LED blink three times before going to sleep
    int i = 3;
    while (i--) {
        digitalWrite(LED_GPIO, LED_ON);
        modem.sendAT("+SPWM=0,1000,80");
        delay(500);
        digitalWrite(LED_GPIO, LED_OFF);
        modem.sendAT("+SPWM=0,1000,0");
        delay(500);
    }

    //After all off
    modem.poweroff();
	
	SerialMon.println();

    SerialMon.println(F("Poweroff"));

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    esp_deep_sleep_start();

    /*
    The sleep current using AXP192 power management is about 500uA,
    and the IP5306 consumes about 1mA
    */
}

