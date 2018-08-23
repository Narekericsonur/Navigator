/*
  This is the code for the Navigator module on a GPS/IMU guided airplane
  Narek Boghozian

  Notes:
    Data Format for Waypoint Data on SD card:
      32bit Lat, 32bit Lng, 16bit Alt
      4byte Lat, 4byte Lng, 2byte Alt



*/

// ------------------------------------------ SD card
#include <SD.h>
#include <SPI.h>
int CS_PIN = 10;
File file;

// ------------------------------------------ IMU
#include <Wire.h>
const int MPU = 0x68;
int16_t imuData[7]; //int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// ------------------------------------------ GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
struct Coord
{
  int32_t lng, lat;
  int16_t alt;
};
struct Orientation // Spherical coordinate system
{
  int16_t theta, phi;
};
class Waypoint
{
public:
  uint8_t numofwaypoints_, currentwaypointnumber_;
  Coord nextWaypoint;
  Waypoint();
  ~Waypoint();
  void extractWaypoints();
  void goToNext();
};
double latD, lngD;
float alt;
Coord currentLocation, previousLocation;
int16_t currentCourse = 0;

// ------------------------------------------ General stuff
int8_t pos[4];
const byte Pilot = 4;
#include <EEPROM.h>

void setup()
{

  // ----------------------------------- GPS
  ss.begin(GPSBaud);

  // ----------------------------------- General
  Serial.begin(115200);
  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0;
  pos[3] = 0; // from -100 to 100 for easy division

  // ----------------------------------- SD
  pinMode(CS_PIN, OUTPUT);
  SD.begin();

  // ----------------------------------- IMU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop()
{
  while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }
  // while(!gps.encode(ss.read())){}
  if (gps.location.isUpdated())
  {
    latD = gps.location.lat();
    lngD = gps.location.lng();
    alt = gps.altitude.meters();
    previousLocation.lat = currentLocation.lat;
    previousLocation.lng = currentLocation.lng;
    previousLocation.alt = currentLocation.alt;
    currentLocation.lat = latD * 1000000;
    currentLocation.lng = lngD * 1000000;
    currentLocation.alt = alt;
    currentCourse = gps.course.deg();
    //Serial.println("");
    Serial.print(gps.location.lat(), 8);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 8);
  }
  calculateTrajectory();
  readIMU(imuData);
  orientationChange();

  //Calculate desired trajectory
  //  - how far until next waypoint
  //  - if less that 6 metres, consider it completed
  //  - what direction do we need to go/ how far we gotta turn

  //Calculate required actions
  //  - rotate/elevate plane depending on wishes of PID

  //Send order for action
}

void readIMU(int16_t *imuData)
{
  for (int i = 0; i < 5; i++)
  {
    imuData[i] = 0;
  }

  for (int i = 0; i < 5; i++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);
    imuData[0] += Wire.read() << 8 | Wire.read();
    imuData[1] += Wire.read() << 8 | Wire.read();
    imuData[2] += Wire.read() << 8 | Wire.read();
    imuData[3] += Wire.read() << 8 | Wire.read();
    imuData[4] += Wire.read() << 8 | Wire.read();
    imuData[5] += Wire.read() << 8 | Wire.read();
    imuData[6] += Wire.read() << 8 | Wire.read();
  }
  for (int i = 0; i < 5; i++)
  {
    imuData[i] /= 5;
  }
}

void logIMU(int16_t *imuData)
{
  file = SD.open("Data.txt", FILE_WRITE);
  for (int i = 0; i < 6; i++)
  {
    file.print(String(imuData[i]));
    file.print(",");
  }
  file.println(String(imuData[6]));
  file.close();
}

void logALL()
{ // logs everything to SD card
}

void transmit(int8_t *pos)
{
  //pos should have 4 values. [throttle, elevator, rudder, aileron]
  //Do you need to give Pilot a heads up before you start transmitting?

  Wire.beginTransmission(Pilot);
  Wire.write(pos[0]);
  Wire.write(pos[1]);
  Wire.write(pos[2]);
  Wire.write(pos[3]);
  Wire.endTransmission();
}

void printLocation()
{
  Serial.print("Lat: ");
  Serial.print(currentLocation.lat);
  Serial.print(", Lng: ");
  Serial.print(currentLocation.lng);
  Serial.print(", Alt: ");
  Serial.println(currentLocation.alt);
}

void Waypoint::extractWaypoints()
{
  file = SD.open("wp.hex");
  file.seek(3);
  int j = 0;
  Coord wp;
  int32_t x;
  int16_t y;
  while (file.available())
  {
    x = 0x0;
    for (int i = 0; i < 4; i++)
    {
      x = x << 8 | file.read();
    }
    wp.lat = x;
    x = 0x0;
    for (int i = 0; i < 4; i++)
    {
      x = x << 8 | file.read();
    }
    wp.lng = x;
    y = 0x0;
    for (int i = 0; i < 2; i++)
    {
      y = y << 8 | file.read();
    }
    wp.alt = y;
    EEPROM.put(j * sizeof(wp), wp);
    j++;
  }
  numofwaypoints_ = j;
  file.close();
}

void Waypoint::goToNext()
{
  EEPROM.get(currentwaypointnumber_ * sizeof(nextWaypoint), nextWaypoint);
}

// from a to b where a = currentLocation and b = nextWaypoint and prevLoc = previousLocation
void calculateTrajectory(Coord a, Coord b, Coord prevLoc, Orientation *currentOrientation, Orientation *desiredOrientation, uint16_t *distanceToWp)
{
  // phi calculations
  double_t aLat, aLng, bLat, bLng;
  int16_t deg;
  deg = 0;
  aLat = a.lat / 1000000;
  aLng = a.lng / 1000000;
  bLat = b.lat / 1000000;
  bLng = b.lng / 1000000;
  deg = gps.courseTo(aLat, aLng, bLat, bLng);
  &desiredOrientation->phi = deg;
  &currentOrientation->phi = gps.course.value();

  // Theta calculations

  //
}

void orientationChange(Orientation currentOrientation, Orientation desiredOrientation, int16_t *imuDataArray, int8_t *posArray)
{
  // PID
}

// Very innacurate but will do the job.
uint16_t arctan(int16_t latA, int16_t lonA, int16_t latB, int16_t lonB)
{

  int16_t x, deg;

  if (latB - latA) // if den is non-zero
  {
    x = (lonB - lonA) / (latB - latA);
  }
  else if (lonB - lonA > 0) // towards right
  {
    x = 50;
  }
  else // towards left
  {
    x = -50;
  }

  // Angle Approximation, piecewise defined function
  if (x < -5)
  {
    deg = -90;
  }
  else if (x < -1)
  {
    deg = 11 * x - 34.9504255;
  }
  else if (x < 1)
  {
    deg = 45.9504 * x;
  }
  else if (x < 5)
  {
    deg = 11 * x + 34.9504255;
  }
  else
  {
    deg = 90;
  }

  // Adjust for behind
  if (latB - latA < 0)
  {
    if (lonB - lonA < 0)
    { // quadrant 2, theta - 180
      deg -= 180;
    }
    else
    { // quadrant 3, theta +180
      deg += 180;
    }
  }
}