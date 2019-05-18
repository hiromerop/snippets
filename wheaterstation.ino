#include <math.h> 
#include <Wire.h>
#include "RTClib.h"
#include "cactus_io_DS18B20.h" 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// VARIABLES
int           DS18B20_Pin = 9;                      //DS18S20 Signal pin on digital 9 
bool          bucketPositionA = false;              // one of the two positions of tipping-bucket               
const double  bucketAmount = 0.01610595;            // inches equivalent of ml to trip tipping-bucket
double        dailyRain = 0.0;                      // rain accumulated for the day
double        hourlyRain = 0.0;                     // rain accumulated for one hour
double        dailyRain_till_LastHour = 0.0;        // rain accumulated for the day till the last hour          
bool          first;                                // as we want readings of the (MHz) loops only at the 0th moment 
float         WindSpeed;                            // speed miles per hour
 
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine 

//CONSTANTS
#define SEALEVELPRESSURE_HPA (1013.25)
#define RainPin 4  // The Rain input is connected to digital pin 4 on the arduino
#define WindSensorPin (2) // The pin location of the anemometer sensor 

DS18B20 ds(DS18B20_Pin);  // Create DS18B20 object 
Adafruit_BME280 bme;      // Create BME280 object 
RTC_Millis rtc;           // software RTC time

void setup() { 
ds.readSensor(); 

Serial.begin(9600);

//RTC INIT
rtc.begin(DateTime(__DATE__, __TIME__));       // start the RTC

//RAIN SENSOR INIT
pinMode(RainPin, INPUT);                       // set the Rain Pin as input.

//WIND SENSOR INIT
pinMode(WindSensorPin, INPUT); 
attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); 
  
Serial.println("                           ***** FOUNTAIN OF LIGHT - WEATHER STATION - 9TH GRADE  *****"); 
Serial.println("___________________________________________________________________________________________________________________"); 
Serial.println(" Temperature C\tTemperature F\tHumdity\t\tPressure\tAltitude\tRainfall\t\tWindSpeed");
Serial.println("___________________________________________________________________________________________________________________");  
} 

void loop() { 
  ds.readSensor();
  DateTime now = rtc.now();
  Rotations = 0; // Set Rotations count to 0 ready for calculations 
  
  sei(); // Enables interrupts 
 
  // convert to mp/h using the formula V=P(2.25/T) 
  // V = P(2.25/3) = P * 0.75 

  //BME CODE
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  //RAIN GAUGE CODE
  // ++++++++++++++++++++++++ Count the bucket tips ++++++++++++++++++++++++++++++++
  if ((bucketPositionA==false)&&(digitalRead(RainPin)==HIGH)){
    bucketPositionA=true;
    dailyRain+=bucketAmount;                               // update the daily rain
  }
  
  if ((bucketPositionA==true)&&(digitalRead(RainPin)==LOW)){
    bucketPositionA=false;  
  } 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  if(now.minute() != 0) first = true;                       // after the first minute is over, be ready for next read
  
  if(now.minute() == 0 && first == true){
 
    hourlyRain = dailyRain - dailyRain_till_LastHour;       // calculate the last hour's rain
    dailyRain_till_LastHour = dailyRain;                    // update the rain till last hour for next calculation
    
    first = false;                                          // execute calculations only once per hour
  }

  // PRINT VALUES
  Serial.print(" "); 
  Serial.print(ds.getTemperature_C()); Serial.print(" *C\t"); 
  Serial.print(ds.getTemperature_F()); Serial.print(" *F\t");

  Serial.print(bme.readHumidity());
  Serial.print(" %\t\t"); 

  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(" hPa\t");

  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(" m\t\t");

  Serial.print(dailyRain,8);  // the '8' ensures the required accuracy
  Serial.print(" inches\t");
 
  Serial.print(WindSpeed);
  Serial.println(" Mp/h"); 
  
  delay(1000); 
  cli(); // Disable interrupts 
  WindSpeed = Rotations * 0.75; 
  
}

// This is the function that the interrupt calls to increment the rotation count 
void isr_rotation () { 

  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact. 
    Rotations++; 
    ContactBounceTime = millis(); 
  } 
}
