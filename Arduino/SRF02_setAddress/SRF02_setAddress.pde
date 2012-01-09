#include <Wire.h>

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
 // Serial.begin(9600);        // start serial communication at 9600bps
  delay(2000);
  changeAddress(0x77,0xF2);    //  (currentaddress(7bits), newAddress(8 bits)); 
}

int state = 0;

void loop()
{

}



// The following code changes the address of a Devantech Ultrasonic Range Finder (SRF10 or SRF08)
// usage: changeAddress(0x70, 0xE6);

void changeAddress(byte oldAddress, byte newAddress)
{
  Wire.beginTransmission(oldAddress);
  Wire.send(0x00);
  Wire.send(0xA0);
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.send(0x00);
  Wire.send(0xAA);
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.send(0x00);
  Wire.send(0xA5);
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.send(0x00);
  Wire.send(newAddress);
  Wire.endTransmission();
}





void readSoftwareVersion()
{
  Serial.println("Check Software version");
  Wire.beginTransmission(113);
  Wire.send(0x01);
  Wire.endTransmission();
  
  Wire.requestFrom(113, 1);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if(1 <= Wire.available())    // if two bytes were received
  {
    int reading = Wire.receive();  // receive high byte (overwrites previous reading)
   
    Serial.print(reading);
    Serial.print(" version\n");   // print the reading
  }
}
