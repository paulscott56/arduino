
#include <Adafruit_MPL115A2.h>
#include <SparkFunTSL2561.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//period between posts, set at 60 seconds
#define DELAY_PERIOD 60000

// Important!! We use pin 13 for enable esp8266  
#define WIFI_ENABLE_PIN 13

#define SSID "Jarvis"
#define PASS "$(SivRaj)$"

// Create an SFE_TSL2561 object, here called "light":
//Create an MPL115A2 pressure object

SFE_TSL2561 light;
Adafruit_MPL115A2 mpl115a2;

//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
//const int lightLEDpin=12;
//const int tempLEDpin=11;
//const int UVLEDpin=10;

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
  //pinMode(lightLEDpin,OUTPUT);
  //pinMode(tempLEDpin,OUTPUT);
  //pinMode(UVLEDpin,OUTPUT);

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
    
    lightMeasure();       //call lightMeasure function to measure and display lux value on serial monitor
 
    pressureMeasure();    //call pressureMeasure function to measure and display temperature and pressure on serial monitor

    UVMeasure();              //call UVmeasure function to measure UV intesity and display on serial monitor

    delay(1000);  //delay serial output for convenience
   
    } 
    
}

void lightMeasure(){
  unsigned int data0, data1;
  
  if (light.getData(data0,data1))
  {
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    good = light.getLux(gain,ms,data0,data1,lux);
    
    
    Serial.print("Lux: ");
    Serial.print(lux);
    
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");

    if(lux<=100) lightState=HIGH;
    else lightState=LOW;
  }
  else
  {
    
    byte error = light.getError();
    printError(error);
  } 
}


void pressureMeasure(){
  
  float pressureKPA = 0, temperatureC = 0;    

  pressureKPA = mpl115a2.getPressure();  
  Serial.print("Pressure: "); Serial.print(pressureKPA, 4); Serial.println(" kPa");

  temperatureC = mpl115a2.getTemperature();  
  Serial.print("Temperature: "); Serial.print(temperatureC, 1); Serial.println(" *C");

  if(temperatureC<=35) tempState=HIGH;
  else tempState=LOW;
  
}



void UVMeasure()
{
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  Serial.print("UV Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);

  if(uvIntensity<6) UVstate=HIGH;
  else UVstate=LOW;
  
  Serial.println();
  Serial.println(" ");
  
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

//web request needs to be sent without the http for now, https still needs some working
void SendTempData(float temperature){
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
 //sendcommand.concat(PUBLIC_KEY);
 sendcommand.concat("?private_key=");
 //sendcommand.concat(PRIVATE_KEY);
 sendcommand.concat("&temp=");
 sendcommand.concat(String(temp));
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

