/**The MIT License (MIT)
Copyright (c) 2015 by Daniel Eichhorn
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
thanks to Dani Eichhorn -> http://blog.squix.ch

 *  This sketch sends data via HTTP GET requests to thingspeak service every 10 minutes
 *  You have to set your wifi credentials and your thingspeak key.
 *  Arduino 1.65
 *  added humidex, heatindex, dewpoint, battery voltage
 */

#include <Adafruit_BMP085.h>  //https://github.com/adafruit/Adafruit-BMP085-Library V1.0
#include <Wire.h>
#include <math.h>
#include <ESP8266WiFi.h>
extern "C" {
  #include "user_interface.h"
}
#include "DHT.h"  //https://github.com/adafruit/DHT-sensor-library V1.2.3

#define DHTPIN 12     // what pin we're connected to

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

const char* ssid     = "XXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXX";


const char* host = "api.thingspeak.com";
const char* thingspeak_key = "XXXXXXXXXXXX"; //write key

void turnOff(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);            //  i2C SDA D2, SCL D1
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

int r1 = 3300; //Voltage Divider (220k on board added 110k from Battery + to A0 pin)
int r2 = 1000; //Voltage Divider (100k on board)
int vref = 980; //Internal vref of your Nodemcu/WEMOS D1 mini alternativ 985


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
  //String temp = String(dht.readTemperature());
  //String humidity = String(dht.readHumidity());
  String pressure = String(bmp.readSealevelPressure(altitude)/100.0F); // pressure in hPA/mBar , ohne nachkommastellen = (bmp.readSealevelPressure(altitude)/100.0F, 0);
  String lux = String(getLux());
  String shttemp = String (calc_temp(rawTemperature));
  String shthum = String (calc_humidity(rawHumidity));
  String dewpoint = String (dewPoint(calc_temp(rawTemperature), (calc_humidity(rawHumidity))));  
  String shttempfahrenheit = String (Fahrenheit(calc_temp(rawTemperature)));
  float h = (calc_humidity(rawHumidity));
  float t = (calc_temp(rawTemperature));
  float f = (Fahrenheit(calc_temp(rawTemperature)));
  float hi = dht.computeHeatIndex(f, h);
  float heatindex = dht.convertFtoC(hi);
  float humidex = calculate_humidex (t, h);
  float adc_value = analogRead(A0); //adc.read(0)
  float battery = vref * (adc_value) * (r1 + r2) / r2 / 1024 / 1000;
   
  
  String url = "/update?key=";
  url += thingspeak_key;
  url += "&field1=";
  url += shttemp;
  url += "&field2=";
  url += shthum;
  url += "&field3=";
  url += pressure;
  url += "&field4=";
  url += lux;
  url += "&field5=";
  url += dewpoint;
  url += "&field6=";
  url += heatindex;
  url += "&field7=";
  url += humidex;
  url += "&field8=";
  url += battery;

  
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

  //WAKE_RF_DEFAULT = 0, // RF_CAL or not after deep-sleep wake up, depends on init data byte 108.
  //WAKE_RFCAL = 1,      // RF_CAL after deep-sleep wake up, there will be large current.
  //WAKE_NO_RFCAL = 2,   // no RF_CAL after deep-sleep wake up, there will only be small current.
  //WAKE_RF_DISABLED = 4 // disable RF after deep-sleep wake up, just like modem sleep, there will be the smallest current.
  //ESP.deepSleep(10 * 60 * 1000000, WAKE_RF_DEFAULT); // GPIO16 needs to be tied to RST to wake from deepSleep.
  system_deep_sleep_set_option(0); // GPIO16/D0 needs to be tied to RST to wake from deepSleep.
  system_deep_sleep(10 * 60 * 1000000);
 }

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
  return 1.8 * celsius + 32;
}

// fast integer version with rounding
//int Celcius2Fahrenheit(int celcius)
//{
//  return (celsius * 18 + 5)/10 + 32;
//}

//Fahrenheit to Celsius conversion
double Celsius(double Fahrenheit)
{
  return (Fahrenheit - 32) * 0.55555;
}


//Celsius to Kelvin conversion
double Kelvin(double celsius)
{
  return celsius + 273.15;
}

// dewPoint function NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
//
double dewPoint(double celsius, double humidity)
{
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
  double Td = (b * temp) / (a - temp);
  return Td;
}

//function to calculete Humidex

float calculate_humidex(float temperature,float humidity) {
  float e;

  e = (6.112 * pow(10,(7.5 * temperature/(237.7 + temperature))) * humidity/100); //vapor pressure

  float humidex = temperature + 0.55555555 * (e - 10.0); //humidex
  return humidex;

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

