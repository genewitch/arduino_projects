#include <Wire.h>
#include <Adafruit_BMP085.h>


// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//GYRO
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
int Addr = 105;                 // I2C address of gyro
int x, y, z;

//BORSCHT
Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);

  //borsh
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  //gyro
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x00);    // Set scale (0x00=250 deg/sec0x80=500)
  delay(100);                   // Wait to synchronize

  //rtc
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  //print date
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print(" since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print("s = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println("d");

  // calculate a date which is 7 days and 30 seconds into the future
  DateTime future (now + TimeSpan(7, 12, 30, 6));

  Serial.print(" now + 7d + 30s: ");
  Serial.print(future.year(), DEC);
  Serial.print('/');
  Serial.print(future.month(), DEC);
  Serial.print('/');
  Serial.print(future.day(), DEC);
  Serial.print(' ');
  Serial.print(future.hour(), DEC);
  Serial.print(':');
  Serial.print(future.minute(), DEC);
  Serial.print(':');
  Serial.print(future.second(), DEC);
  Serial.println();

  Serial.println();
  delay(3000);
}

void loop() {
  //Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  //Serial.println(" *C");
  Serial.print("\t");

  //Serial.print("Pressure = ");
  //Serial.print(bmp.readPressure()); //pascal
  //Serial.print(bmp.readPressure()*0.00029529983071445);   //inHg
  Serial.print(bmp.readPressure() * 0.01); //mBar

  //Serial.println(" Pa");
  Serial.print("\t");
  /*
      // Calculate altitude assuming 'standard' barometric
      // pressure of 1013.25 millibar = 101325 Pascal
      Serial.print("Altitude = ");
      Serial.print(bmp.readAltitude());
      Serial.println(" meters");

      Serial.print("Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");

    // you can get a more precise measurement of altitude
    // if you know the current sea level pressure which will
    // vary with weather and such. If it is 1015 millibars
    // that is equal to 101500 Pascals.
      Serial.print("Real altitude = ");
      Serial.print(bmp.readAltitude(101500));
      Serial.println(" meters");

      Serial.println();
  */
  delay(500);

  getGyroValues();              // Get new values
  // In following Dividing by 114 reduces noise
  //Serial.print("/114 X:");
  Serial.print(x / 114);
  Serial.print("\t");

  //Serial.print(" /114 Y:");
  Serial.print(y / 114);
  Serial.print("\t");

  //Serial.print(" /114 Z:");
  Serial.print(z / 114);
  Serial.print("\t");

  //Serial.print("Raw X:");
  Serial.print(x);
  //Serial.print(" Raw Y:");
  Serial.print("\t");

  Serial.print(y);
  Serial.print("\t");
  //Serial.print(" Raw Z:");
  Serial.println(z);
  Serial.print("\t");
}

void getGyroValues () {
  byte MSB, LSB;

  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  x = ((MSB << 8) | LSB);

  MSB = readI2C(0x2B);
  LSB = readI2C(0x2A);
  y = ((MSB << 8) | LSB);

  MSB = readI2C(0x2D);
  LSB = readI2C(0x2C);
  z = ((MSB << 8) | LSB);
}

int readI2C (byte regAddr) {
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);                // Register address to read
  Wire.endTransmission();             // Terminate request
  Wire.requestFrom(Addr, 1);          // Read a byte
  while (!Wire.available()) { };      // Wait for receipt
  return (Wire.read());               // Get result
}

void writeI2C (byte regAddr, byte val) {
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
