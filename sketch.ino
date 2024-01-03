#define BLYNK_TEMPLATE_ID "TMPL5ipwyAffK"
#define BLYNK_TEMPLATE_NAME "Energy Optimisation"
#define BLYNK_AUTH_TOKEN "-mAd8ty-qjFRpPTIdRCJZmzNQji6_P9H"

#include <NTPClient.h>
#include <WiFiUdp.h>
// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <Wire.h>  // Include the Wire library for I2C communication
#include <LiquidCrystal_I2C.h>  // Include the I2C LCD library

const long utcOffsetInSeconds = 19800;  // Adjust this value according to your time zone
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address and size

const char* ssid = "Wokwi-GUEST";
const char* password = "";
unsigned long lightsOnTime = 0;
bool manualLightControl = false;
const int voltagePin = 36; // Analog input pin for voltage measurement
const int currentPin = 39; // Analog input pin for current measurement

float voltage, current, power;

BlynkTimer timer;


char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.

char pass[] = "";




#define button1_pin 33
#define button2_pin 32
#define button3_pin 35
#define button4_pin 34

#define relay1_pin 14
#define relay2_pin 27
#define relay3_pin 26
#define relay4_pin 25

int relay1_state = 0;
int relay2_state = 0;
int relay3_state = 0;
int relay4_state = 0;


//Change the virtual pins according the rooms
#define button1_vpin    V1
#define button2_vpin    V2
#define button3_vpin    V3 
#define button4_vpin    V4

//------------------------------------------------------------------------------
// This function is called every time the device is connected to the Blynk.Cloud
// Request the latest state from the server
BLYNK_CONNECTED() {
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button2_vpin);
  Blynk.syncVirtual(button3_vpin);
  Blynk.syncVirtual(button4_vpin);
}

//--------------------------------------------------------------------------
// This function is called every time the Virtual Pin state change
//i.e when web push switch from Blynk App or Web Dashboard
BLYNK_WRITE(button1_vpin) {
  relay1_state = param.asInt();
  digitalWrite(relay1_pin, relay1_state);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button2_vpin) {
  relay2_state = param.asInt();
  digitalWrite(relay2_pin, relay2_state);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button3_vpin) {
  relay3_state = param.asInt();
  digitalWrite(relay3_pin, relay3_state);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button4_vpin) {
  relay4_state = param.asInt();
  digitalWrite(relay4_pin, relay4_state);
}
//--------------------------------------------------------------------------


void setup()
{
  // Debug console
  Serial.begin(115200);
  //--------------------------------------------------------------------
  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);
  pinMode(button3_pin, INPUT_PULLUP);
  pinMode(button4_pin, INPUT_PULLUP);
  //--------------------------------------------------------------------
  pinMode(relay1_pin, OUTPUT);
  pinMode(relay2_pin, OUTPUT);
  pinMode(relay3_pin, OUTPUT);
  pinMode(relay4_pin, OUTPUT);
  //--------------------------------------------------------------------
  //During Starting all Relays should TURN OFF
  digitalWrite(relay1_pin, HIGH);
  digitalWrite(relay2_pin, HIGH);
  digitalWrite(relay3_pin, HIGH);
  digitalWrite(relay4_pin, HIGH);
  //--------------------------------------------------------------------
  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  //--------------------------------------------------------------------
  //Blynk.virtualWrite(button1_vpin, relay1_state);
  //Blynk.virtualWrite(button2_vpin, relay2_state);
  //Blynk.virtualWrite(button3_vpin, relay3_state);
  //Blynk.virtualWrite(button4_vpin, relay4_state);
  //--
  Serial.begin(115200);
  lcd.init();  // Initialize the LCD
  lcd.backlight();  // Turn on the backlight

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  pinMode(voltagePin, INPUT);
  pinMode(currentPin, INPUT);

  timer.setInterval(1000L, sendDataToBlynk); // Send data to Blynk every 1 second
  timeClient.begin();
  timeClient.update();
}


void loop()
{
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
  Blynk.run();
  timer.run();

  timeClient.update();

  listen_push_buttons();
  autoTurnOffLights();
  listen_push_buttons();
}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
void listen_push_buttons(){
    //--------------------------------------------------------------------------
    if(digitalRead(button1_pin) == LOW){
      //delay(200);
      control_relay(1);
      Blynk.virtualWrite(button1_vpin, relay1_state); //update button state
    }
    //--------------------------------------------------------------------------
    else if (digitalRead(button2_pin) == LOW){
      //delay(200);
      control_relay(2);
      Blynk.virtualWrite(button2_vpin, relay2_state); //update button state
    }
    //--------------------------------------------------------------------------
    else if (digitalRead(button3_pin) == LOW){
      //delay(200);
      control_relay(3);
      Blynk.virtualWrite(button3_vpin, relay3_state); //update button state
    }
    //--------------------------------------------------------------------------
    else if (digitalRead(button4_pin) == LOW){
      //delay(200);
      control_relay(4);
      Blynk.virtualWrite(button4_vpin, relay4_state); //update button state
    }
    //--------------------------------------------------------------------------
}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM




//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
void control_relay(int relay){
  //------------------------------------------------
  if (relay == 1) {
    relay1_state = !relay1_state;
    digitalWrite(relay1_pin, relay1_state);
    Serial.print("Relay1 State = ");
    Serial.println(relay1_state);

    if (relay1_state == HIGH) {
      manualLightControl = true;
      lightsOnTime = millis();
    }
  }
  //------------------------------------------------
  else if(relay == 2){
    relay2_state = !relay2_state;
    digitalWrite(relay2_pin, relay2_state);
    //delay(50);
  }
  //------------------------------------------------
  else if(relay == 3){
    relay3_state = !relay3_state;
    digitalWrite(relay3_pin, relay3_state);
    //delay(50);
  }
  //------------------------------------------------
  else if(relay == 4){
    relay4_state = !relay4_state;
    digitalWrite(relay4_pin, relay4_state);
    //delay(50);
  }
  //------------------------------------------------
}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
void sendDataToBlynk() {
  int rawVoltage = analogRead(voltagePin);
  int rawCurrent = analogRead(currentPin);
  
  voltage = rawVoltage / 4095.0 * 3.3; // Convert ADC value to voltage (assuming 3.3V reference)
  current = rawCurrent / 4095.0 * 3.3; // Convert ADC value to current (assuming 3.3V reference)
  
  // Calculate power using P = VI
  power = voltage * current;

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Current: ");
  Serial.print(current);
  Serial.print(" A, Power: ");
  Serial.print(power);
  Serial.println(" W");

  Blynk.virtualWrite(V0, voltage); // Send voltage value to Blynk's virtual pin V0
  Blynk.virtualWrite(V1, current); // Send current value to Blynk's virtual pin V1
  Blynk.virtualWrite(V2, power);   // Send power value to Blynk's virtual pin V2

  // Display values on the LCD
  lcd.clear();  // Clear the LCD screen
  lcd.setCursor(0, 0);  // Set the cursor to the first line
  lcd.print("Voltage: ");
  lcd.print(voltage);
  lcd.print(" V");

  lcd.setCursor(0, 1);  // Set the cursor to the second line
  lcd.print("Current: ");
  lcd.print(current);
  lcd.print(" A");
}

void autoTurnOffLights() {
  int currentHour = timeClient.getHours();
  if (currentHour >= 22 || currentHour < 7) {  // Check if it's between 10 pm and 7 am
    if (!manualLightControl || (millis() - lightsOnTime) >= 3600000) {
      digitalWrite(relay1_pin, LOW);  // Turn off the lights
      Blynk.virtualWrite(button1_vpin, LOW);  // Update the Blynk button state
      relay1_state = LOW;
    }
  }
}
