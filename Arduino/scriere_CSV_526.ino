#include <dht.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C Interface

SoftwareSerial MyBlue(2, 3); // RX | TX 

dht DHT;
 
#define DHT11_PIN 13
#define SensorPin A0 

float sensorValue = 0, moisture_percentage;

int rainPin = A3;

int thresholdValue = 600;

int n, dataArray[6];

void sendRawData(int val){
  
  dataArray[n] = val;
  n++;
}

void sendToTerminal(){
  char sItoa[10];
  
  for(int i = 0; i < n; i++){
    itoa(dataArray[i], sItoa, 10);
  
    Serial.write(sItoa);
    Serial.write(",");
  
    MyBlue.write(sItoa);
    MyBlue.write(",");
    
    }
    
    Serial.write("\n");
    MyBlue.write("\n");
    
    n=0;
  
    delay(1000);
  
}

void setup() {
  
 Serial.begin(9600); 
  MyBlue.begin(9600);  //Baud Rate for AT-command Mode. 
  Serial.println("**AT-command Mode**");
  delay(1000);
 pinMode(rainPin, INPUT);
  //Serial.println("DHT TEST PROGRAM ");
  //Serial.print("LIBRARY VERSION: ");
  //Serial.println(DHT_LIB_VERSION);
  //Serial.println();
  //Serial.println("Type,\tstatus,\tHumidity (%),\tTemperature (C)");
  if (!bmp.begin(0x77)) {//!bmp.begin(0x76) !bmp.begin(0x77)
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
}
 
void loop() { 
  //from bluetooth to Terminal. 
 if (MyBlue.available()) 
   Serial.write(MyBlue.read()); 
 //from termial to bluetooth 
 if (Serial.available()) 
   MyBlue.write(Serial.read());
  
   // READ DATA
  //Serial.print("DHT11, \t");
 
  
  int chk = DHT.read11(DHT11_PIN);
  /*
  switch (chk)
  {
    case DHTLIB_OK:  
        Serial.print("OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Time out error,\t");
        break;
    default:
        Serial.print("Unknown error,\t");
        break;
  }
  */
  
  sendRawData(DHT.humidity);
  
  sendRawData(DHT.temperature);
 
 
 moisture_percentage = ( 100.0 - ( (analogRead(SensorPin)/1023.00) * 100.0 ) ); 

 moisture_percentage = moisture_percentage * 100;
 
 
 //Serial.print("Soil moisture percentage: ");
 sendRawData((int)moisture_percentage);
 
 // read the input on analog pin 0:
  int rainValue = analogRead(rainPin);
  char r_status[50];
  
  //Serial.print("Raindrop value = ");
  sendRawData(rainValue);

  int pressure = bmp.readPressure()/100; //displaying the pressure in hPa
  int altitude = bmp.readAltitude();

  sendRawData(pressure); 
  sendRawData(altitude);
 
 
  sendToTerminal();
  
  //functioneaza asa trebuie sa trimit
  /*
  char incerc[50];
  
  strcpy(incerc, "420");
  
  sendRawData(incerc);   
  */ 
  
  //functioneaza cu wrapper
  /*
  int n = 420;
  String sWrapper = String(n);
  char sArray[50];
  
  sWrapper.toCharArray(sArray, 50);
  sendRawData(sArray);
  */
  
  //functioneaza cu itoa
  
  

  
} 
