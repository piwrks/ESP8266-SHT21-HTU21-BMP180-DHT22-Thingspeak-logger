/*
 *  This sketch sends data via HTTP GET requests to thingspeak service every 10 minutes
 *  You have to set your wifi credentials and your thingspeak key.
 *  Arduino 1.65
 *  thanks to Dani Eichhorn -> http://blog.squix.ch
 
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

#include <Adafruit_BMP085.h>  //https://github.com/adafruit/Adafruit-BMP085-Library V1.0
#include <Wire.h>
#include <math.h>
#include <ESP8266WiFi.h>
extern "C" {
  #include "user_interface.h"
}
#include "DHT.h"  //https://github.com/adafruit/DHT-sensor-library V1.2.3

#define DHTPIN 12     // what pin we're connected to -> 12 = D6 on ESP8266

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE, 15);

// for HTDU21D
#define HTDU21D_ADDRESS 0x40  //Unshifted 7-bit I2C address for the sensor
#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

Adafruit_BMP085 bmp; // Create the bmp object
int altitude = 385; // Höhe des Sensorstandortes ueber dem Meeresspiegel

const char* ssid     = "YOURSSID";
const char* password = "YOURPASSWORD";


const char* host = "api.thingspeak.com";
const char* thingspeak_key = "YOURWRITEAPI";

void turnOff(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);            //  SDA = D2, SCL = D1
  Wire.setClock(400000);
 if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}  
}
  
  // disable all unused outputs to save power
  turnOff(0);
  turnOff(2);
  turnOff(12);
  turnOff(13);
  turnOff(14);
  turnOff(15);

  dht.begin();
  delay(10);
 

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

int value = 0;


void loop() {
  delay(5000);
  ++value;

  Serial.print("connecting to ");
  Serial.println(host);
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  unsigned int rawHumidity = htdu21d_readHumidity();
  unsigned int rawTemperature = htdu21d_readTemp();
  String temp = String(dht.readTemperature());
  String humidity = String(dht.readHumidity());
  String pressure = String(bmp.readSealevelPressure(altitude)/100.0); // pressure in hPA/mBar
  String lux = String(getLux());
  String shttemp = String (calc_temp(rawTemperature));
  String shthum = String (calc_humidity(rawHumidity));
  
  String url = "/update?key=";
  url += thingspeak_key;
  url += "&field1=";
  url += temp;
  url += "&field2=";
  url += humidity;
  url += "&field3=";
  url += pressure;
  url += "&field4=";
  url += lux;
  url += "&field6=";
  url += shttemp;
  url += "&field7=";
  url += shthum;
  
  Serial.print("Requesting URL: ");
  Serial.println(url);
  
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
  delay(10);
  
  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  
  Serial.println();
  Serial.println("closing connection. going to sleep...");
  delay(1000);
  // go to deepsleep for 10 minutes
  system_deep_sleep_set_option(0);
  system_deep_sleep(10 * 60 * 1000000);
}


//--------------------------------- Light intensity
 float getLux(){
  byte dmsb,dlsb;
  float lx;
  // BH1750 address is 0x5c or 0x23 
  
  Wire.beginTransmission(0x23);
  Wire.write(0x11); // high res 2 continuous (.5 lx res)
  Wire.endTransmission();
  delay(200);       // wait 180ms max to complete conversion
  Wire.beginTransmission(0x23);
  Wire.requestFrom(0x23,2);
  dmsb=Wire.read();
  dlsb=Wire.read();
  Wire.endTransmission();  
  // we need to futz a bit because bit 0 is a .5 indicator
  lx=dmsb<<8;
  if (dlsb & 1){
    lx=lx+0.5;
  }
  dlsb=(dlsb>>1); // dump the decimal
  lx=lx+dlsb;
  lx=lx/1.2;
  return(lx);
 }

 //--------------------------------- Temperature and Humidity
 /* 
 HTU21D Humidity Sensor Example Code
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 15th, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 */
//Read the uncompensated temperature value
unsigned int htdu21d_readTemp()
{
  //Request the temperature
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_TEMP_MEASURE_NOHOLD);
  Wire.endTransmission();

  //Wait for sensor to complete measurement
  //delay(60); //44-50 ms max - we could also poll the sensor
  delay(95);

  //Comes back in three bytes, data(MSB) / data(LSB) / CRC
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  int counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100) return 998; //Error out
  }

  unsigned char msb, lsb, crc;
  msb = Wire.read();
  lsb = Wire.read();
  crc = Wire.read(); //We don't do anything with CRC for now

  unsigned int temperature = ((unsigned int)msb << 8) | lsb;
  temperature &= 0xFFFC; //Zero out the status bits but keep them in place

  return temperature;
}

//Read the humidity
unsigned int htdu21d_readHumidity()
{
  byte msb, lsb, checksum;

  //Request a humidity reading
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_HUMD_MEASURE_NOHOLD); //Measure humidity with no bus holding
  Wire.endTransmission();

  //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
  //delay(55);
  delay(95);
  //Read result
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  int counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100) return 0; //Error out
  }

  msb = Wire.read();
  lsb = Wire.read();
  checksum = Wire.read();

  unsigned int rawHumidity = ((unsigned int) msb << 8) | (unsigned int) lsb;
  rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place

  return(rawHumidity);
}

//Given the raw temperature data, calculate the actual temperature
float calc_temp(int SigTemp)
{
  float tempSigTemp = SigTemp / (float)65536; //2^16 = 65536
  float realTemperature = -46.85 + (175.72 * tempSigTemp); //From page 14

  return(realTemperature);  
}

//Given the raw humidity data, calculate the actual relative humidity
float calc_humidity(int SigRH)
{
  float tempSigRH = SigRH / (float)65536; //2^16 = 65536
  float rh = -6 + (125 * tempSigRH); //From page 14

  return(rh);  
}

//Read the user register
byte read_user_register(void)
{
  byte userRegister;

  //Request the user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(READ_USER_REG); //Read the user register
  Wire.endTransmission();

  //Read result
  Wire.requestFrom(HTDU21D_ADDRESS, 1);

  userRegister = Wire.read();

  return(userRegister);  
}

//Write to the user register
//NOTE: We disable all bits except for measurement resolution
//Bit 7 & 0 = Measurement resolution
//Bit 6 = Status of battery
//Bit 5/4/3 = Reserved
//Bit 2 = Enable on-board heater
//Bit 1 = Disable OTP reload
void write_user_register(byte thing_to_write)
{
  byte userRegister = read_user_register(); //Go get the current register state
  userRegister &= 0b01111110; //Turn off the resolution bits
  thing_to_write &= 0b10000001; //Turn off all other bits but resolution bits
  userRegister |= thing_to_write; //Mask in the requested resolution bits

  //Request a write to user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(WRITE_USER_REG); //Write to the user register
  Wire.write(userRegister); //Write to the data
  Wire.endTransmission();
}

//Give this function the 2 byte message (measurement) and the check_value byte from the HTU21D
//If it returns 0, then the transmission was good
//If it returns something other than 0, then the communication was corrupted
//From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
//POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

unsigned int check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  //Test cases from datasheet:
  //message = 0xDC, result is 0x79
  //message = 0x683A, result is 0x7C
  //message = 0x4E85, result is 0x6B

  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the result/check value
  remainder |= check_value_from_sensor; //Add on the check value

  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {
    //Serial.print("remainder:  ");
    //Serial.println(remainder, BIN);
    //Serial.print("divsor:     ");
    //Serial.println(divsor, BIN);
    //Serial.println();

    if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return remainder;
}
