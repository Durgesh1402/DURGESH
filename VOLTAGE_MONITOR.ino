// Include Libraries

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <DallasTemperature.h>
#include <OneWire.h>


#define UINT16_t_MAX  65536
#define INT16_t_MAX   UINT16_t_MAX/2
#define NUM_SAMPLES 10 
#define ONE_WIRE_BUS 1

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int ledPin = 6;                               // the pin for the LED
unsigned long SLEEP_TIME = 30000;             // Sleep time between reads (in milliseconds)
int sum = 0;                                  // sum of samples taken
unsigned char sample_count = 0;               // current sample number
float vout= 0.0;
float voltage = 0.0;                          // calculated voltage 
int buttonPin = 7;                            // the input pin for offset pushbutton
int buttonState;                              // variable for reading the pin status
int debug = false;

typedef struct __attribute__ ((packed)) sigfox_message {
  uint8_t status;
  int16_t Temperature1;
  int16_t voltage;
  } SigfoxMessage;

//convert float values to int16_t formate

  int16_t convertoFloatToInt16(float value, long max, long min) {
  float conversionFactor = (float) (INT16_t_MAX) / (float)(max - min);
  return (int16_t)(value * conversionFactor);
}

// stub for message which will be sent
SigfoxMessage msg;


void setup(){
 pinMode(ledPin, OUTPUT);                       // Set ledPin as output
 digitalWrite(ledPin, LOW);                     // Make sure ledPin is off

  
 if (debug == true) {
  Serial.begin(9600);
  while (!Serial);
  SigFox.begin();
  sensors.begin();                             // Start up the onewire library
 if ( ! SigFox.begin() ) {
   Serial.println("Error ... rebooting");
   NVIC_SystemReset();
   while(1);
   }
 SigFox.reset();
 delay(100);
 SigFox.debug();
 SigFox.end();
 pinMode(buttonPin, INPUT);
 }
}


void loop(){
  // read the state of the pushbutton value
 buttonState = digitalRead(buttonPin);
 // check if the pushbutton is pressed.
 if (buttonState == LOW) {
  digitalWrite(ledPin, HIGH);                             // Turn on LED to indicate offset being calculated
  delay(5000); 
 
//----------------------------- VOLTAGE MEASURMENT-----------------------------------
  
  sample_count = 0;
  sum = 0;
   while (sample_count < NUM_SAMPLES) {                  // take a number of voltage samples  
   sum += analogRead(A6);
   sample_count++;
   delay(10);
   }
  Serial.print("sum count..."); 
  Serial.println((sum / NUM_SAMPLES));                  // print the count result. will be between 0 and 1023
  vout = ((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0;
  voltage = vout *11.123;
  Serial.print("Voltage = "); 
  Serial.println(voltage, 2);                    // Print voltage in two decimal places
  msg.voltage = convertoFloatToInt16(voltage, 18, 0);
//----------------------------- TEMPERATURE MEASURMENT------------------------------
  sensors.requestTemperatures();                 // Send the command to get temperature readings
  float temperature = sensors.getTempCByIndex(0);  
  Serial.println("Temperature: " + String(temperature) +" C"); 
  msg.Temperature1 = convertoFloatToInt16(temperature, 60, -60);
//-------------------------------------
// Clears all pending interrupts
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.write((uint8_t*)&msg, 12);
  int lastMessageStatus = SigFox.endPacket();
  SigFox.end();
  digitalWrite(ledPin, LOW);                       // Turn off LED to indicate offset being calculated
  LowPower.sleep(60 * 60 * 1000);                 //Sleep for 60 minutes               
 }else{
    reboot();
   }
}
  
void reboot() {
   NVIC_SystemReset();
   while (1) ;
   }
