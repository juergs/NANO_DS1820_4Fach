/*
*                                Nano_328p_Version!!!
*
*******************************************************************************************
*   Bei Programmer-Fehler: Dallas Sensor entfernen! Wenn: Mosi = Data & SCK = Sensor !
*   Auf Pin-Belegung der Platine achten!!!!! -> Board_Typ setzen!
*******************************************************************************************
*      Auszug aus der Beschreibung des Herstellers:
– 120 Powerful Instructions – Most Single Clock Cycle Execution
– 32 x 8-bit General Purpose Working Registers
– Up to 20 MIPS Througput at 20 MHz
– 8K Bytes of In-System Programmable Program Memory Flash
– 512 Bytes In-System Programmable EEPROM
– 512 Bytes Internal SRAM
– Write/Erase Cycles: 10,000 Flash/100,000 EEPROM
– Data retention: 20 Years at 85°C/100 Years at 25°C
– 8-bit Timer/Counter with Prescaler and Two PWM Channels
– 8-bit High Speed Timer/Counter with Separate Prescaler
– USI – Universal Serial Interface with Start Condition Detector
– 10-bit ADC: 4 Single Ended Channels, 2 Differential ADC Channel Pairs
– Temperature Measurement
– On-chip Analog Comparator
– Low Power Idle, ADC Noise Reduction, and Power-down Modes
– Internal Calibrated Oscillator
– Six Programmable I/O Lines
– 2.7 - 5.5V Operating Voltage

- Gehäuseform: DIP, 8-polig, Herstellerbezeichnung "ATtiny85-20PU"

*** Fuses auf 8Mhz internal clock gestellt (Standard) und Clock-Divider nicht gesetzt!
avrdude: safemode: Fuses OK (H:FF, E:DF, L:E2)
Uploading to I/O board using 'USBasp'
Uploader started for board ATtiny w/ ATtiny85
Uploader will use programmer name: usbasp
C:\Users\js\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.0.1-arduino5\bin\avrdude "-CC:\Users\js\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.0.1-arduino5/etc/avrdude.conf" -v -pattiny85 -cusbasp -Pusb "-Uflash:w:C:\Users\js\AppData\Local\Temp\VMicroBuilds\ATtiny85_LaCrosse_TempSensor\attiny_attiny_attiny85/ATtiny85_LaCrosse_TempSensor.ino.hex:i"
avrdude: Version 6.0.1, compiled on Apr 15 2015 at 19:59:58
Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/
Copyright (c) 2007-2009 Joerg Wunsch
System wide configuration file is "C:\Users\js\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.0.1-arduino5/etc/avrdude.conf"
Using Port                    : usb
Using Programmer              : usbasp
AVR Part                      : ATtiny85
Chip Erase delay              : 400000 us
PAGEL                         : P00
BS2                           : P00
RESET disposition             : possible i/o
RETRY pulse                   : SCK
serial program mode           : yes
parallel program mode         : yes
Timeout                       : 200
StabDelay                     : 100
CmdexeDelay                   : 25
SyncLoops                     : 32
ByteDelay                     : 0
PollIndex                     : 3
PollValue                     : 0x53
Memory Detail                 :
Block Poll               Page                       Polled
Memory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack
----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------
eeprom        65    12     4    0 no        512    4      0  4000  4500 0xff 0xff
flash         65     6    32    0 yes      8192   64    128 30000 30000 0xff 0xff
signature      0     0     0    0 no          3    0      0     0     0 0x00 0x00
lock           0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
lfuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
hfuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
efuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
calibration    0     0     0    0 no          2    0      0     0     0 0x00 0x00
Programmer Type : usbasp
Description     : USBasp, http://www.fischl.de/usbasp/


ATMEL ATTINY 25/45/85 / ARDUINO

+-\/-+
Ain0 (D 5) PB5  1|    |8  Vcc
Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1
Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
GND  4|    |5  PB0 (D 0) pwm0
+----+

Install ATtiny - Models in Arduino IDE:
http://highlowtech.org/?p=1695

ATtiny:
https://cpldcpu.wordpress.com/2014/04/25/the-nanite-85/

Powersave modes:
================
http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
http://www.gammon.com.au/forum/?id=11488&reply=9#reply9
http://gammon.com.au/forum/?id=11497&reply=6#reply6
http://www.gammon.com.au/power
https://forum.arduino.cc/index.php?topic=326237.0
https://www.insidegadgets.com/2011/02/05/reduce-attiny-power-consumption-by-sleeping-with-the-watchdog-timer/
... und weitere Powersave-Gedanken:
https://harizanov.com/2013/08/every-%CE%BCa-counts/
http://electronics.stackexchange.com/questions/49182/how-can-i-get-my-atmega328-to-run-for-a-year-on-batteries

Hier noch ein weiteres Projekt mit RFM69:
http://johan.kanflo.com/the-aaduino/
und der Sketch:
https://github.com/kanflo/aaduino

*
* The LaCrosse-protocol seems to be:

Bits 0-3: header
Bits 4-11: device ID, changes when replacing the batteries. Unlike in the post linked above, bit 11 does not appear to be a checksum.
Bits 12-15: either 1111 for automatic transmission (once every 60 seconds) or 1011 for manual transmission (using the button in the battery compartment). Manual transmission does not update the weather station.
Bits 16-27: encode the temperature. The system of encoding decimal digits seems to be ditched in favor of a more elegant one: apply a NOT (change 1 to 0 and 0 to 1), convert to base 10, divide by 10 (into a float), subtract 50, and the result is the temperature in C.
Bits 28-35: encode the relative humidity. Apply a NOT, convert to base 10, and the result is the relative humidity in %.
Bits 36-43: appear to encode a checksum (though I plan to double-check if this is not the dew point, also reported by the weather station).

Example:
HHHH 1000 0010 1111 1101 0010 1111 1101 0011 1010 0100
encoding T=22.0C and RH=44%

*/
//**************************************************************
// Thanks to: http://www.f6fbb.org/domo/sensors/tx3_th.php
// Thanks to: http://forum.arduino.cc/index.php?topic=155483.0
// Thanks to: https://forum.fhem.de/index.php/topic,50333.0.html
//**************************************************************
//  OneWire DS18S20, DS18B20, DS1822 Temperature Version 
//  using this Arduino Library: http://www.pjrc.com/teensy/td_libs_OneWire.html
//  The DallasTemperature library can do all this work for you!
//  http://milesburton.com/Dallas_Temperature_Control_Library
//  http://images.google.de/imgres?imgurl=http://www.tweaking4all.com/wp-content/uploads/2014/03/ds18b20-waterproof.jpg&imgrefurl=http://www.tweaking4all.com/hardware/arduino/arduino-ds18b20-temperature-sensor/&h=988&w=800&tbnid=mowdJDteDQmw_M:&tbnh=104&tbnw=84&docid=7g-v-bKlWHiqKM&usg=__9sTNcsYyWEgAZF-aP5rpUuvCyio=&sa=X&ved=0ahUKEwiRvJfp44HMAhVDDCwKHc1OBgcQ9QEIKjAB
//  http://www.tweaking4all.com/hardware/arduino/arduino-ds18b20-temperature-sensor/
//  https://github.com/PaulStoffregen/OneWire
//  https://arduino-info.wikispaces.com/Brick-Temperature-DS18B20
//  https://arduino-info.wikispaces.com/MultipleTemperatureSensorsToLCD
//  http://www.pjrc.com/teensy/td_libs_OneWire.html
//
//  Hardware Overview and HowTo:
//  http://www.tweaking4all.com/hardware/arduino/arduino-ds18b20-temperature-sensor/
//
//  Additionals:
//  https://gcc.gnu.org/onlinedocs/gcc-3.1/cpp/Standard-Predefined-Macros.html
//  https://gcc.gnu.org/onlinedocs/gcc-3.1/cpp/Invocation.html#Invocation
//  https://gcc.gnu.org/onlinedocs/gcc/Preprocessor-Options.html
//

/* How to connect multiple Sensors:
The DS18B20 Digital Temperature sensor allows you to set multiple in parallel. When doing this, the OneWire library will read all sensors.
For larger networks of sensors (more than 10), using smaller resistors should be considered, for example 1.6 KΩ or even less.
It has been observed that large amounts of sensors (more than 10) in the same network can cause issues (colliding data),
and for that purpose an additional resistor of say 100 … 120 Ω should be added between the data line to the Arduino and the data pin of the sensor, for each sensor !
***/
/*
 * Nano Version:  variable delatime between single send to avoid interference-conflicts with steady time sending sensors.
 *                RCSwitch-Lib added locally.
 */
/******************************************************************************************************************************************************/
//#include "Narcoleptic.h"
#include <Arduino.h>
#include "LaCrosse.h"   //liegen lokal im Verzeichnis!
#include "OneWire.h"    //liegen lokal im Verzeichnis!
#include "RCSwitch.h"   //liegen lokal im Verzeichnis!


//#include <DallasTemperature\DallasTemperature.h>

#define SENSOR_BASIS_ID       105
#define SENSOR_ID_0           SENSOR_BASIS_ID 
#define SENSOR_ID_1           SENSOR_BASIS_ID + 1
#define SENSOR_ID_2           SENSOR_BASIS_ID + 2
#define SENSOR_ID_3           SENSOR_BASIS_ID + 3

// Nano-Belegung
#define DALLAS_SENSOR_PIN_0   2       // Nano Belegung ab D2 = PD2
#define DALLAS_SENSOR_PIN_1   3       // 
#define DALLAS_SENSOR_PIN_2   4       // 
#define DALLAS_SENSOR_PIN_3   5       // 
#define TX_433_PIN            12      // PB4 = D12
#define RX_433_PIN            11      // optional 433-RX for RCSwitch

#define PIN_SEND              TX_433_PIN    //--- where digal pin 433 Sender is connected PB0 on ATtiny-85 pin 5 = PB0 
#define PIN_LED               13            // Nano Led Pin

//--- prepared for multiple sensor-readings
#define MAXSENSORS            4
#define OW_ROMCODE_SIZE       8

// static 18B20-Instances
OneWire  ds_00(DALLAS_SENSOR_PIN_0);      // on arduino port pin 2 (a 4.7K resistor is necessary, between Vcc and DQ-Pin   1=VCC 2=DQ 3=GND, top view + straight side left)
OneWire  ds_01(DALLAS_SENSOR_PIN_1);
OneWire  ds_02(DALLAS_SENSOR_PIN_2);
OneWire  ds_03(DALLAS_SENSOR_PIN_3);

//--- t,h - floats as Temperature and Humidity part of LaCrosseClass 
//--- Message String for // Serial Debug + sprintf buffers
/*
char msg[80];
char tmsg[10];
char hmsg[10];
*/

//--- hold sensor id's
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
unsigned long previousMillis;
unsigned long interval = 5;
byte ledState;
byte onlyOnce = 0; 
long randomNumber; 

//--- prototypes
void ReadSingleOneWireSensor(OneWire ds);

//------------------------------------------------
void setup()
{
  Serial.begin(57600);

  //--- Serial.begin(115200);  
  pinMode(PIN_SEND, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(RX_433_PIN, INPUT);

  //--- preset first SensorId
  LaCrosse.bSensorId = SENSOR_BASIS_ID;

  //--- get a real seed
  randomSeed(analogRead(A0)); 

  delay(1);
}
//-----------------------------------
void loop()
{ 
  if (onlyOnce == 0)
  {
    Serial.println("Hello from NANO ...");
    onlyOnce = 1;
  }

  //--- Sensor_1
  
  LaCrosse.t = 0.0;
  LaCrosse.bSensorId = SENSOR_ID_0;
  ReadSingleOneWireSensor(ds_00); 
  digitalWrite(PIN_LED, HIGH);
  LaCrosse.sendTemperature();
  digitalWrite(PIN_LED, LOW);

  Serial.print("SensorID: ");
  Serial.print((byte)LaCrosse.bSensorId);
  Serial.print(" First: ");
  Serial.println((double) LaCrosse.t);

  randomNumber = random(2000);
  delayMicroseconds(1000 + randomNumber );

  //--- Sensor_2
  digitalWrite(PIN_LED, HIGH);
  LaCrosse.t = 0.0;
  ReadSingleOneWireSensor(ds_01);
  LaCrosse.bSensorId = SENSOR_ID_1; 
  digitalWrite(PIN_LED, HIGH);
  LaCrosse.sendTemperature();
  digitalWrite(PIN_LED, LOW);

  randomNumber = random(2000);
  delayMicroseconds(1000 + randomNumber);

  Serial.print("SensorID: ");
  Serial.print((byte)LaCrosse.bSensorId);
  Serial.print(" Second: ");
  Serial.println((double)LaCrosse.t);

  //--- Sensor_3
  
  LaCrosse.t = 0.0;
  LaCrosse.bSensorId = SENSOR_ID_2;
  ReadSingleOneWireSensor(ds_02);
  digitalWrite(PIN_LED, HIGH);
  LaCrosse.sendTemperature();
  digitalWrite(PIN_LED, LOW);

  randomNumber = random(2000);
  delayMicroseconds(1000 + randomNumber);

  Serial.print("SensorID: ");
  Serial.print((byte)LaCrosse.bSensorId);
  Serial.print(" Third: ");
  Serial.println((double)LaCrosse.t);

  //--- Sensor_4  
  LaCrosse.t = 0.0;
  LaCrosse.bSensorId = SENSOR_ID_3;
  ReadSingleOneWireSensor(ds_03); 
  digitalWrite(PIN_LED, HIGH);
  LaCrosse.sendTemperature();
  digitalWrite(PIN_LED, LOW);
  
  Serial.print("SensorID: ");
  Serial.print((byte)LaCrosse.bSensorId);
  Serial.print(" Fourth: ");
  Serial.println((double)LaCrosse.t);
  
  //blinking();
  randomNumber = random(15000); 
  randomNumber += 30000; 
  
  Serial.print("Pause-Delay (Rand): ");
  Serial.println((long)randomNumber);
  Serial.println("");
  Serial.println("");
  
  delay(randomNumber);

  
  ////--- wait a second between radio-transfers
  //// LaCrosse.sleep(1);
  ////LaCrosse.sendHumidity();
  ////LaCrosse.sleep(50); /* has no power-reduction! */ 

  ////--- preserve more power during sleep phase 
  //pinMode(DALLAS_SENSOR_PIN, INPUT);
  //LaCrosse.setTxPinMode(INPUT);

  ////--- we will wait 5 minutes in reduced power consumption until next data are send. 
  //Narcoleptic.delay_minutes(3);

  ////--- set back to normal operation mode
  //pinMode(DALLAS_SENSOR_PIN, OUTPUT);
  //LaCrosse.setTxPinMode(OUTPUT);
}

void ReadSingleOneWireSensor(OneWire ds)
{
  //--- h + t variables holding measurement results 
  //--- we do not have a humidity sensor, preset a fictive value
  LaCrosse.h = 12.3;

  //--- read temperature as Celsius (the default), reading now from 18x20-Sensors
  //LaCrosse.t = 27.1;

  //--- 18B20 stuff
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  if (!ds.search(addr))
  {
    ds.reset_search();
    delay(250);
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    //Serial.println("CRC is not valid!");
    //--- blink Led instead? 
    return;
  }

  //--- the first ROM byte indicates which chip
  switch (addr[0])
  {
  case 0x10:
    //Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s = 1;
    break;
  case 0x28:
    // Serial.println("  Chip = DS18B20");
    type_s = 0;
    break;
  case 0x22:
    // Serial.println("  Chip = DS1822");
    type_s = 0;
    break;
  default:
    // Serial.println("Device is not a DS18x20 family device.");
    return;
  }

  ds.reset();

  ds.select(addr);

  ds.write(0x44);        // start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
           // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();

  ds.select(addr);

  ds.write(0xBE);         //--- read scratchpad

  for (i = 0; i < 9; i++)
  {
    //--- we need 9 bytes
    data[i] = ds.read();
  }

  //--- Convert the data to actual temperature
  //--- because the result is a 16 bit signed integer, it should
  //--- be stored to an "int16_t" type, which is always 16 bits
  //--- even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s)
  {
    raw = raw << 3;     //--- 9 bit resolution default
    if (data[7] == 0x10)
    {
      //--- "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    };
  }
  else
  {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
                        //// default is 12 bit resolution, 750 ms conversion time
  };

  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;

  //--- t (float) is sended value  by LaCrosse,
  // use alternatively LaCrosse.setSensorId(id) 
  LaCrosse.t = celsius;

  //---- Check if any reads failed and exit early (to try again).  
  if (isnan(LaCrosse.h) || isnan(LaCrosse.t))
  {
    // led blink?
    //--- signalize error condition 
    LaCrosse.t = -99.0;
    LaCrosse.h = 99.9;
    return;
  };
}
//-------------------------------------------------------------------------
void blinking() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState);
  }
}
//-------------------------------------------------------------------------

