/*
  Read NMEA sentences over I2C using Ublox module SAM-M8Q, NEO-M8P, ZED-F9P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA setences from the Ublox module over I2c and outputs
  them to the serial port
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  ==============================================================================================
  Voir  ~/Arduino/libraries/SparkFun_u-blox_Arduino_Library/src/SparkFun_Ublox_Arduino_Library.h
  ==============================================================================================
  
*/

#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void setup(){
  Serial.begin(115200);
  Serial.println("SparkFun Ublox Example");
  do {
    Serial.println("GPS: trying 115200 baud");
    Serial2.begin(115200, SERIAL_8N1, 34, 12);
    if (myGPS.begin(Serial2) == true) break;
    delay(100);
        
    Serial.println("GPS: trying 38400 baud");
    Serial2.begin(38400, SERIAL_8N1, 34, 12);
    if (myGPS.begin(Serial2) == true) break;    
    
    Serial.println("GPS: trying 57600 baud");
    Serial2.begin(57600, SERIAL_8N1, 34, 12);
    if (myGPS.begin(Serial2) == true) break;
    
    Serial.println("GPS: trying 9600 baud");
    Serial2.begin(9600, SERIAL_8N1, 34, 12);
    if (myGPS.begin(Serial2) == true) break;
    
    else {
        //myGPS.factoryReset();
        delay(1000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);

  //Only leaving GGA/VTG enabled at current navigation rate    
  
  myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA/UBX only
  //Disable or enable various NMEA sentences over the UART1 interface
  //Several of these are on by default on virgin ublox board so let's disable them
  
  //  GxGLL (latitude and long, whith time of position fix and status)
  myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1); 
  //myGPS.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1); 
  
  //  GxGSA (GNSS DOP and Active satellites)
  myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1); 
  //myGPS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  
  //  GxGSV (GNSS satellites in view)
  myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1); 
  //myGPS.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  
  //  GxRMC (Recommended minimum data)
  //myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1); 
  myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
  
  //  GxVTG (course over ground and Ground speed)
  //myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1); 
  myGPS.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);

  //  GxGGA (Global positioning system fix data)
  //myGPS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); 
  myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); 
 
  myGPS.setNavigationFrequency(1);           //Set output to 1 times a second  => 10 si necessaire
  //Serial.println(
    
    
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate:");
  Serial.println(rate);

  //This will pipe all NMEA sentences to the serial port so we can see them
  //myGPS.setNMEAOutputPort(Serial);
}

void loop(){
while(Serial2.available()) Serial.write(Serial2.read());
  /*
 // myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude/ 10000000., 6);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude/ 10000000., 6);
    //Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude/ 1000., 2);
    Serial.print(F(" (mm)"));

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }*/

  //delay(250); //Don't pound too hard on the I2C bus
}
