//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

// Time between measurements (millisecond)
// To set to maximum rate, set it to zero
#define SAMPLE_RATE 0


#define SD_PIN 4
#define DHT_PIN 7
#define DHT_TYPE DHT22

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
// Libraries
//-------------------------------------------------------------------------------

#include <DS3232RTC.h>
#include <DHT.h>

#include <SPI.h>
#include <SD.h>

#include <Adafruit_BMP085.h>
#include <BH1750.h>

#include <Wire.h>
#include <MPU6050.h>

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
// Variables
//-------------------------------------------------------------------------------
File dataFile;
String filePath;

DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP085 bmp;
BH1750 bh;
MPU6050 mpu;

// To adjust clock
unsigned long subsecondMillis;
unsigned long sampleRateMillis = 0;
unsigned long time2waitMillis;

// To be saved in File
time_t rtcTime;
short int subsecond;
short int extraTime;
short int prevSecond;

Vector mpuNormGyros;
Vector mpuNormAccel;
float mpuTemperature;

int32_t lightLevel;
int32_t bmpRawPressure;
int32_t bmpRawTemperature;

float dhtHumidity;
float dhtTemperature;

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
// Main code
//-------------------------------------------------------------------------------
void setup()
{
  /*
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */

  initializeRTC();

  boolean isDHTok = initializeDHT();
  boolean isBHok = initializeBH();
  boolean isMPUok = initializeMPU();
  boolean isBMPok = initializeBMP();
  boolean isSDok = initializeSDCard();

  initializeLEDs(isSDok, isMPUok && isBHok && isBMPok && isDHTok);
}

void loop()
{
  //sampleRateMillis = millis();

  updateMilliseconds();

  //if(millis()-sampleRateMillis > SAMPLE_RATE)
  {
    // sampleRateMillis = millis();
    
    // It should take 10 ms
    time2waitMillis = millis();
    updateMPUBHData();
    if (millis() - time2waitMillis < 10)
      delay(10 - (millis() - time2waitMillis));

    updateBMPDHTData(); // It takes at least 15 ms
    
    // It should take 40 ms
    time2waitMillis = millis();
    saveDataIntoFile();
    // BH1750 measurement (on high resolution) mode takes about 65 ms
    // There is the need to wait only 40 ms here because 10 ms + 15 ms were waited before
    if (millis() - time2waitMillis < 40)
      delay(40 - (millis() - time2waitMillis));
  }
}

//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
// Functions
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------

void updateMilliseconds()
{
  // To correct 'millisecond'
  extraTime = millis() - subsecondMillis - 1000;
  if (extraTime < 0)
    extraTime = 0;

  rtcTime = RTC.get();

  if (prevSecond != second(rtcTime))
  {
    subsecondMillis = millis() - extraTime;
    prevSecond = second(rtcTime);
  }
  subsecond = millis() - subsecondMillis;
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void updateMPUBHData()
{
  mpuNormGyros = mpu.readNormalizeGyro();
  mpuNormAccel = mpu.readNormalizeAccel();
  mpuTemperature = mpu.readTemperature();

  lightLevel = bh.readLightLevel();
}

//-------------------------------------------------------------------------------
// Description here!
// It takes 15 ms
//-------------------------------------------------------------------------------
void updateBMPDHTData()
{
  time2waitMillis = millis();
  
  // It should end before the temperature measurement starts
  bmpRawPressure = bmp.endPressureMeasurement();
  bmp.startTemperatureMeasurement();

  dhtHumidity = dht.readHumidity();
  dhtTemperature = dht.readTemperature();

  // If DHT has no new reading the time of measurement will be less than 1 ms
  //  The problema is that the bmp temperature measurement takes 5 ms
  if (millis() - time2waitMillis < 15)
    delay(15 - (millis() - time2waitMillis));

  bmpRawTemperature = bmp.endTemperatureMeasurement();
  
  // It should start after the temperature measurement ends
  // The pressure measurement takes about 26 ms
  bmp.startPressureMeasurement(); 
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void saveDataIntoFile()
{
  dataFile = SD.open(filePath, FILE_WRITE);
  if (dataFile)
  {
    save2Digits(hour(rtcTime));
    dataFile.print(":");
    save2Digits(minute(rtcTime));
    dataFile.print(":");
    save2Digits(second(rtcTime));
    dataFile.print(":");
    save3Digits(subsecond);
    dataFile.print("\t");

    dataFile.print(dhtHumidity);
    dataFile.print("\t");
    dataFile.print(dhtTemperature);
    dataFile.print("\t");
    dataFile.print(dht.computeHeatIndex(dhtTemperature, dhtHumidity, false));
    dataFile.print("\t");

    dataFile.print(bmp.computePressure(bmpRawPressure, bmpRawTemperature));
    dataFile.print("\t");
    dataFile.print(bmp.computeTemperature(bmpRawTemperature));
    dataFile.print("\t");
    dataFile.print(bmp.computeAltitude(bmp.computePressure(bmpRawPressure, bmpRawTemperature)));
    dataFile.print("\t");

    saveBHsensor(lightLevel);
    dataFile.print("\t");

    dataFile.print(mpuTemperature);
    dataFile.print("\t");

    // Calculate Pitch and Roll from accelerometer

    dataFile.print(-(atan2(mpuNormAccel.XAxis, sqrt(mpuNormAccel.YAxis * mpuNormAccel.YAxis + mpuNormAccel.ZAxis * mpuNormAccel.ZAxis)) * 180.0) / M_PI);
    dataFile.print("\t");
    dataFile.print((atan2(mpuNormAccel.YAxis, mpuNormAccel.ZAxis) * 180.0) / M_PI);
    dataFile.print("\t");

    // Gyroscope normilized values
    dataFile.print(mpuNormGyros.YAxis); // Pitch
    dataFile.print("\t");
    dataFile.print(mpuNormGyros.XAxis); // Roll
    dataFile.print("\t");
    dataFile.print(mpuNormGyros.ZAxis); // Yaw
    dataFile.print("\t");
    //Accelerometer normilized values
    dataFile.print(mpuNormAccel.YAxis); // Pitch
    dataFile.print("\t");
    dataFile.print(mpuNormAccel.XAxis); // Roll
    dataFile.print("\t");
    dataFile.print(mpuNormAccel.ZAxis); // Yaw
    dataFile.println();

    /*
      dataFile.print(0);
      dataFile.print("\t");
      dataFile.print(0);
      dataFile.print("\t");

      // Gyroscope normilized values
      dataFile.print(0); // Pitch
      dataFile.print("\t");
      dataFile.print(0); // Roll
      dataFile.print("\t");
      dataFile.print(0); // Yaw
      dataFile.print("\t");
      //Accelerometer normilized values
      dataFile.print(0); // Pitch
      dataFile.print("\t");
      dataFile.print(0); // Roll
      dataFile.print("\t");
      dataFile.print(0); // Yaw
      dataFile.println();
    */

    dataFile.close();
  }
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void initializeLEDs(boolean SDCard, boolean sensors)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(30, OUTPUT);

  digitalWrite(17, HIGH); // High turn RXLED off
  digitalWrite(30, HIGH); // High turn TXLED off

  // Problem with SDCard
  // Red LED blinks slow
  if (SDCard == false)
  {
    SPI.end();
    for (;;)
    {
      digitalWrite(LED_BUILTIN, HIGH); // High turn LED on
      delay(500);
      digitalWrite(LED_BUILTIN, LOW); // High turn LED on
      delay(500);
    }
    // Power off device


    return;
  }
  // If there is any sensor with problem
  // Red LED blinks fast
  if (sensors == false)
  {
    SPI.end();
    for (;;)
    {
      digitalWrite(LED_BUILTIN, HIGH); // LED on
      delay(100);
      digitalWrite(LED_BUILTIN, LOW); // LED off
      delay(100);
    }
    // Power off device

    return;
  }
  // Everything is fine
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void initializeRTC()
{
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);

  // set Alarm 1 to occur once per second
  RTC.setAlarm(ALM1_EVERY_SECOND, 0, 0, 0, 0);
  // clear the alarm flag
  RTC.alarm(ALARM_1);

  // In order to synchronize millis with RTC time
  //
  // The if/else statement is important because the function
  // RTC.alarm() changes RTC alarm state when called
  if (RTC.alarm(ALARM_1))
  {
    while (!RTC.alarm(ALARM_1)) {}
    subsecondMillis = millis();
  }
  else
  {
    while (!RTC.alarm(ALARM_1)) {}
    subsecondMillis = millis();
  }

  rtcTime = RTC.get();
  prevSecond = second(rtcTime);
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
boolean initializeBMP()
{
  // By default, BMP mode is ULTRA HIGH
  if (!bmp.begin())
    return false;
  
  bmp.startPressureMeasurement();
  return true;
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
boolean initializeBH()
{
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  // On esp8266 devices you can select SCL and SDA pins using Wire.begin(D4, D3);
  Wire.begin();

  // Define HIGH resolution mode and time for measurement minimum (MTreg = 32)
  if (!bh.begin(BH1750::CONTINUOUS_HIGH_RES_MODE) || !bh.setMTreg(32))
    return false;
  else
    return true;
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
boolean initializeMPU()
{
  if (!mpu.begin(MPU6050_SCALE_1000DPS, MPU6050_RANGE_2G))
    return false;

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  // mpu.setThreshold(3);

  return true;
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
boolean initializeDHT()
{
  dht.begin();

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f))
    return false;
  else
    return true;
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
boolean initializeSDCard()
{
  if (!SD.begin(SD_PIN))
    return false;

  time_t rtcTime = RTC.get();

  String dir;
  dir = String(day(rtcTime)) + "_";
  dir += String(month(rtcTime));

  if (!SD.exists(dir))
    SD.mkdir(dir);

  filePath = dir + "/";
  filePath += String(hour(rtcTime)) + "_";
  filePath += String(minute(rtcTime)) + "_";
  filePath += String(second(rtcTime)) + ".txt";

  // Saving data - it takes at least 30 ms
  dataFile = SD.open(filePath, FILE_WRITE);
  if (dataFile)
  {
    // Heading of the File
    dataFile.println(F("Data File (v3)"));
    dataFile.println(F("dd/mm/yyyy"));
    save2Digits(day(rtcTime));
    dataFile.print("/");
    save2Digits(month(rtcTime));
    dataFile.print("/");
    dataFile.print(year(rtcTime));

    dataFile.println();
    dataFile.println();

    dataFile.print(F("RTC(t) \t\t\tDHT(h)\tDHT(t)\tDHT(i) \t BMP(p)\tBMP(t)\tBMP(a) \tBH(l)"));
    dataFile.print(F("\tMPU(t)\tAcce(p)\tAcce(r)\tGyro(x)\tGyro(y)\tGyro(z)\tAcce(x)\tAcce(y)\tAcce(z)"));

    dataFile.println();
    dataFile.println();

    dataFile.close();

    return true;
  }
  else
    return false;
}







//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void save2Digits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if (digits < 10)
    dataFile.print('0');
  dataFile.print(digits);
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void save3Digits(int digits)
{
  if (digits < 100)
    dataFile.print('0');
  if (digits < 10)
    dataFile.print('0');
  dataFile.print(digits);
}

//-------------------------------------------------------------------------------
// Description here!
//-------------------------------------------------------------------------------
void saveBHsensor(int32_t digits)
{
  if (digits < 10000)
    dataFile.print('0');
  if (digits < 1000)
    dataFile.print('0');
  if (digits < 100)
    dataFile.print('0');
  if (digits < 10)
    dataFile.print('0');
  dataFile.print(digits);
}
