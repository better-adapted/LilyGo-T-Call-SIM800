// Please select the corresponding model

 #define SIM800L_IP5306_VERSION_20190610
// #define SIM800L_AXP192_VERSION_20200327
// #define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

#include <Arduino.h>
#include "utilities.h"

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

void setupModem()
{
#ifdef MODEM_RST
    // Keep reset high
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, HIGH);
#endif

    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    digitalWrite(MODEM_PWRKEY, LOW);
    digitalWrite(MODEM_POWER_ON, LOW);

    // Initialize the indicator as an output
    pinMode(LED_GPIO, OUTPUT);

    for(int i=0;i<3;i++)
    {
      digitalWrite(LED_GPIO, LED_ON);
      delay(100);
      digitalWrite(LED_GPIO, LED_OFF);
      delay(100);      
    }
        
    // Turn on the Modem power first
    digitalWrite(MODEM_POWER_ON, HIGH);
}

void test_bootloader_start()
{
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    int i=1000;

    while(i>0)
    {
      SerialAT.write(0xB5);

      delay(5);
      
      if(SerialAT.available())
      {
        byte temp = SerialAT.read();

        if(temp==0x5B)
        {
          digitalWrite(LED_GPIO, LED_ON);
          //SerialMon.printf("Found response i=%d\r\n",i);
          SerialAT.end();
          SerialMon.write(temp);
          break;
        }
      }

      delay(5);
      i--;     
    }   
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);

    // Some start operations
    setupModem();

    //test_bootloader_start();
}

void loop()
{
}
