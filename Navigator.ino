/*
  This is the code for the Navigator module on a GPS/Accelerometer guided airplane
  Narek Boghozian

  Notes:
    Data Format for Waypoint Data on SD card:
      32bit Lat, 32bit Lng, 16bit Alt
      4byte Lat, 4byte Lng, 2byte Alt

  To-Do:
    Log everything to SD card. Use last data address on eeprom to keep track of which file we're on
    When testing, adjust all the constants and PID function to perfect flight


*/

// ------------------------------------------ SD card
#include <SD.h>
#include <SPI.h>
int CS_PIN = 10;
File file;

// ------------------------------------------ Accelerometer
#include <Wire.h>
const int MPU = 0x68;
int16_t accData[3]; //

// ------------------------------------------ GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
const int16_t targetSize = 10; // how close to waypoint is close enough
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
Waypoint allWaypoints;
Orientation currentOrientation, desiredOrientation;
uint16_t distanceToWp;

// ------------------------------------------ General stuff
int8_t pos[4];
const byte Pilot = 4;
#include <EEPROM.h>
int16_t kP[4], kI[4], kD[4];
float currentSpeed = 100; // 10 m/s ideal (probably)




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

  kP[0] = 10;
  kP[1] = 10;
  kP[2] = 10;
  kP[3] = 10;

  kI[0] = 10;
  kI[1] = 10;
  kI[2] = 10;
  kI[3] = 10;

  kD[0] = 10;
  kD[1] = 10;
  kD[2] = 10;
  kD[3] = 10;

  // ----------------------------------- SD
  pinMode(CS_PIN, OUTPUT);
  SD.begin();

  // ----------------------------------- Accelerometer
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
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
    currentSpeed = gps.speed.meters();
    //Serial.println("");
    // Serial.print(gps.location.lat(), 8);
    // Serial.print(", ");
    // Serial.println(gps.location.lng(), 8);
  }
  calculateTrajectory(currentLocation, allWaypoints.nextWaypoint, 
  previousLocation, &currentOrientation, &desiredOrientation, &distanceToWp);
  readAcc(accData);
  PID();
  logALL();

  //Calculate desired trajectory
  //  - how far until next waypoint
  //  - if less that 6 metres, consider it completed
  //  - what direction do we need to go/ how far we gotta turn

  //Calculate required actions
  //  - rotate/elevate plane depending on wishes of PID

  //Send order for action
}

void readAcc(int16_t *accData)
{
  accData[0] = analogRead(A0);
  accData[1] = analogRead(A1);
  accData[2] = analogRead(A2);
}

void logAcc(int16_t *accData)
{
  file = SD.open("Data.txt", FILE_WRITE);
  for (int i = 0; i < 2; i++)
  {
    file.print(String(accData[i]));
    file.print(",");
  }
  file.println(String(accData[3]));
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
  currentwaypointnumber_++;
}

// from a to b where a = currentLocation and b = nextWaypoint and prevLoc = previousLocation
void calculateTrajectory(Coord a, Coord b, Coord prevLoc, 
Orientation *cOrientation, Orientation *dOrientation, uint16_t *dToWp)
{
  if (distanceTo(a, b) < targetSize)
  {
    allWaypoints.goToNext();
    b = allWaypoints.nextWaypoint;
  }
  // phi calculations
  double_t aLat, aLng, bLat, bLng;
  int16_t deg = 0, dAlt = 0; // d = delta
  &desiredOrientation->phi = arctan(a.Lat, a.Lng, b.Lat, b.Lng); // Desired Bearing
  &currentOrientation->phi = gps.course.value(); // Current Bearing

  // Theta calculations
  //Current
  dAlt = a.alt - prevloc.alt;
  //Desired

}

// determines the 4 pos values to modify control surfaces.
// must take into consideration - orientation we wanna be at (sometimes straight, sometimes angled)
//                                vs what we're at
// 
void PID(Orientation currentOrientation, Orientation desiredOrientation, float currentSpeed,
int16_t *accDataArray, int8_t *posArray, int16_t kPro, int16_t kInt, int16_t kDer)
{
  // PID
  // posArray = [throttle, elevator, rudder, aileron]
  // for now, ignore aileron
  // for now just use pid for altitude and bearing. Experiment with throttle and make it proportional
  
  // throttle (ideal is ~50% power, but 10m/s will be attempted)
  posArray[0] = -5*currentSpeed + 100;
  if (posArray[0] > 100)
  posArray[0] = 100;
  else if(posArray[0] < 100)
  posArray[0] = 0;

  // elevator
  posArray[1] = 

  // rudder



  // aileron (at some point, probably to do with stability of aircraft)


}

// Very innacurate but will do the job.
int16_t arctan(int16_t latA, int16_t lonA, int16_t latB, int16_t lonB)
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

  return deg;
}

int16_t distanceTo(Coord a, Coord b) // Horizontal distance to other coordinate
{
  return sqrt((a.lat - b.lat) ^ 2 + (a.lng - b.lng) ^ 2);
}