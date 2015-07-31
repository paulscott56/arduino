#include <Adafruit_MPL115A2.h>
#include <SparkFunTSL2561.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//period between posts, set at 60 seconds
#define DELAY_PERIOD 3600000
// Important!! We use pin 13 for enable esp8266  
#define WIFI_ENABLE_PIN 13
#define SSID "Jarvis"
#define PASS "$(SivRaj)$"

SFE_TSL2561 light;
Adafruit_MPL115A2 mpl115a2;
//Hardware pin definitions
int UVOUT = A1; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
int lightState;
int tempState;
int UVstate;
int temp = 0;
int test;
// Global variables:
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds
char serialbuffer[100];//serial buffer for request command
long nextTime;//next time in millis that the temperature read will fire
int wifiConnected = 0;
SoftwareSerial mySerial(11, 12); // rx, tx

void setup()
{
  // Initialize the Serial port:
  mySerial.begin(9600);//connection to ESP8266
  Serial.begin(9600);
  Serial.println("Initializing measurements...");
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);

  light.begin();
  mpl115a2.begin();
  unsigned char ID;

  gain = 0;

  unsigned char time = 2;

  light.setTiming(gain,time,ms);
  
  Serial.println("Sensor Powerup");
  light.setPowerUp();
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);//delay
  
  //set mode needed for new boards
  mySerial.println("AT+RST");
  delay(3000);//delay after mode change       
  mySerial.println("AT+CWMODE=1");
  delay(300);
  mySerial.println("AT+RST");
  delay(500);
    
  nextTime = millis();//reset the timer
}

boolean connectWifi() {     
 String cmd="AT+CWJAP=\"";
 cmd+=SSID;
 cmd+="\",\"";
 cmd+=PASS;
 cmd+="\"";
 Serial.println(cmd);
 mySerial.println(cmd);
           
 for(int i = 0; i < 20; i++) {
   Serial.print(".");
   if(mySerial.find("OK"))
   {
     wifiConnected = 1;
     break;
   }
   
   delay(50);
 }
 
 Serial.println(
   wifiConnected ? 
   "OK, Connected to WiFi." :
   "Can not connect to the WiFi."
 );
 
 return wifiConnected;
}


void loop()
{
  
  if(!wifiConnected) {
      mySerial.println("AT");
      delay(1000);
      if(mySerial.find("OK")){
        Serial.println("Module Test: OK");
        delay(1000);
        connectWifi();
      } 
    }

    if(!wifiConnected) {
      delay(500);
      return;
    }

    //output everything from ESP8266 to the Arduino Micro Serial output
    while (mySerial.available() > 0) {
      Serial.write(mySerial.read());
    }

    //to send commands to the ESP8266 from serial monitor (ex. check IP addr)
    if (Serial.available() > 0) {
       //read from serial monitor until terminating character
       int len = Serial.readBytesUntil('\n', serialbuffer, sizeof(serialbuffer));

       //trim buffer to length of the actual message
       String message = String(serialbuffer).substring(0,len-1);
       
       Serial.println("message: " + message);

       //check to see if the incoming serial message is an AT command
       if(message.substring(0,2)=="AT"){
         //make command request
         Serial.println("COMMAND REQUEST");
         mySerial.println(message); 
       }//if not AT command, ignore
    }

    //wait for timer to expire
    if(nextTime<millis()) {
      Serial.print("timer reset: ");
      Serial.println(nextTime);

      //reset timer
      nextTime = millis() + DELAY_PERIOD;
      SendData(tempMeasure(), lightMeasure(), pressureMeasure(), UVMeasure());
      delay(1000);  //delay serial output for convenience
    } 
    
}

float tempMeasure() {
  return mpl115a2.getTemperature();
}  

double lightMeasure(){
  unsigned int data0, data1;
  if (light.getData(data0,data1))
  {
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    good = light.getLux(gain,ms,data0,data1,lux); 
    return lux;
  }
  else
  {
    byte error = light.getError();
    printError(error);
    return 0.0;
  } 
}


float pressureMeasure(){
  float pressureKPA = 0;    
  pressureKPA = mpl115a2.getPressure();  
  return pressureKPA;
}



float UVMeasure()
{
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  return uvIntensity;
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error)
  {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}

// web request needs to be sent without the http for now, https still needs some working
void SendData(float temperature, double lux, float pressure, float uv){
 char temp[10];

 Serial.print("temp: ");     
 Serial.println(temperature);

 dtostrf(temperature,1,2,temp);

 //create start command
 String startcommand = "AT+CIPSTART=\"TCP\",\"data.sparkfun.com\", 80";
 
 mySerial.println(startcommand);
 Serial.println(startcommand);
 
 //test for a start error
 if(mySerial.find("Error")){
   Serial.println("error on start");
   return;
 }
 
 //create the request command
 String sendcommand = "GET /input/"; 
 sendcommand.concat("dZ65WApDYmiA6O7aJZM8");
 sendcommand.concat("?private_key=");
 sendcommand.concat("eEKv0lazDqh4jNA9KRom");
 sendcommand.concat("&temperature=");
 sendcommand.concat(String(temp));
 sendcommand.concat("&lux=");
 sendcommand.concat(String(lux));
 sendcommand.concat("&pressure=");
 sendcommand.concat(String(pressure));
 sendcommand.concat("&uv_intensity=");
 sendcommand.concat(String(uv));
 sendcommand.concat("\r\n");
 //sendcommand.concat(" HTTP/1.0\r\n\r\n");
 
 Serial.println(sendcommand);
 
 
 //send to ESP8266
 mySerial.print("AT+CIPSEND=");
 mySerial.println(sendcommand.length());
 
 //display command in serial monitor
 Serial.print("AT+CIPSEND=");
 Serial.println(sendcommand.length());
 
 if(mySerial.find(">"))
 {
   Serial.println(">");
 }else
 {
   mySerial.println("AT+CIPCLOSE");
   Serial.println("connect timeout");
   delay(1000);
   return;
 }
 
 mySerial.print(sendcommand);
 delay(1000);
 mySerial.println("AT+CIPCLOSE");
} 
