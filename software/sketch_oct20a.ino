#include <Arduino.h>
#include <Wire.h> //needed for I2C stuff

//Assignments 
const uint8_t MUX_S0 = 2;
const uint8_t MUX_S1 = 3;
const uint8_t MUX_S2 = 4;
const uint8_t MUX_S3 = 5;
const uint8_t MUX_SIG = A0;//analog output
const uint8_t FAULT_LED = 6;
//using unsigned 8 bit ints which only uses a byte, which should give better memory efficiency

//set up for I2C
#define I2C_SLAVE_ADDR 0x08 
//picking 0x08 instead of 0x01 or 4 bc for 7bit I2C, 0x00-0x07 are for special bus operations, so 0x08 is the lowest general purpose address we can use

//Diode sensor calibrations
const float DIODE_TC = -0.0020; //V/C negative bc voltage drops as temp rises
const float V_AT_25C = 0.620;  
//baseline foward voltage drop of diode at room temp     
const float T_REF = 25.0;      
const float ALARM_THRESHOLD_C = 60.0;   

//System states
float temperatures[8];
float maxTemp = -100.0;
uint8_t maxTempChannel = 0;
float minTemp = 1000.0;
uint8_t minTempChannel = 0; 
float avgTemp = 0.0; 
//we need to wire pin 15 (mux's low-enable pin) to gnd so the switches on the mux are always enabled. If we leave it floating then unwanted noise
//could toggle the chip on and off

// Struct formatted for I2C data transfer (fixed-point 10x scaled integers)
struct __attribute__((packed)) I2CPacket { //tjis instructs the compiler to pack all struct fields into memory with no alignment padding bytes for members of different sizes
//ex. int16_t and uint8_t, giving the right byte offset when transmitting to I2C.
  int16_t maxTemp_x10;//2bytes      // e.g., 605 = 60.5°C, By multiplying by 10 and
  // casting to a signed 16-bit integer, a temp like 60.5C becomes 605. This should  have 0.1C precision while cutting down some on the packet size (2 bytes per sensor instead of 4) giving us nice memory savings.
  uint8_t maxChannel;//1 byte       // Channel (0 to 7 )
  int16_t minTemp_x10;  //2 bytes
  uint8_t minChannel;  //1 byte
  int16_t avgTemp_x10; //2 bytes
  int16_t allTemps_x10[8]; // Before for 16 channels, now for 8
} i2cData;

//LED Blinking
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL_MS = 250;
//will toggle the LED state every 250ms. 250ms on + 250ms off = 500ms = 2Hz
bool ledState = false;

// Select Channel on mux
//For N channels we need k binary select lines, N=2^k, so for 16 channels it would be 4 select lines, and for 8 it would be 3
//Instead of connecting diodes right to the nano, we need a mux bc the nano only has 8 analog input pins, and 2 are dedicated to the I2C, giving only 6 pins
//to work with
void selectMuxChannel(uint8_t channel) { 
  //doing this translates into better instructions
  //using right bit shifting with AND masking. It will mive bit n into the LSB position and the & will mask off all other bits leaving only 0 or 1
  //It will also always execute in contant time regardless of the channel
  digitalWrite(MUX_S0, (channel >> 0) & 0x01);
  digitalWrite(MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX_S2, (channel >> 2) & 0x01);
}

//Read Filtered Analog Voltage 
float readChannelVoltage(uint8_t channel) {
  selectMuxChannel(channel);
  delayMicroseconds(10); // Allow multiplexer switch and ADC capacitor to stabilize by finishing and charging/discharging to newly selected channel

  //Discard first reading after channel switch so no crosstalk - 
  analogRead(MUX_SIG); //calling this again drains and recharges the capacitor to the new channel's voltage before taking the real measurement.
  //8x Oversampling for noise suppression
  uint32_t adcSum = 0;
  //using uint32 here for overflow safeguarding
  for (int i = 0; i < 8; i++) {
    adcSum += analogRead(MUX_SIG);
  }
  float avgAdc = (float)adcSum / 8.0f;

  // Internal reference is 1.1V across 1024 steps
  return (avgAdc * 1.1f) / 1024.0f;
}

//I2C Request Callback 
void requestEvent() {
  //opulate the output struct with integers
  i2cData.maxTemp_x10 = (int16_t)(maxTemp * 10.0f);
  i2cData.maxChannel  = maxTempChannel;
  i2cData.minTemp_x10 = (int16_t)(minTemp * 10.0f);
  i2cData.minChannel  = minTempChannel;              
  i2cData.avgTemp_x10 = (int16_t)(avgTemp * 10.0f);
  for (int i = 0; i < 8; i++) {
    i2cData.allTemps_x10[i] = (int16_t)(temperatures[i] * 10.0f);
  }

  //Send packet across the I2C
  Wire.write((uint8_t*)&i2cData, sizeof(i2cData));
  //this will take the starting address of th i2Data sruct , uint8_t* casts it to a pointer to unsigned bytes, sizeof(i2cdata) will tell wire.write the number
  //of bytes to copy into the i2c transmit bufffer
}

void setup() {
  Serial.begin(115200);

  //Configure digital select lines
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
  digitalWrite(FAULT_LED, LOW);

  // Configure internal 1.1V analog reference for high resolution
  analogReference(INTERNAL);  //sets the full -scale ceiling to 1.1V which should cut temp granularity down about 0.54C, since diode Vf will stay 
  //below .7V, matching the ADC to 1.1V will maximize range*
  //adc primining
  analogRead(MUX_SIG); //

  //Initialize I2C 
  Wire.begin(I2C_SLAVE_ADDR); //as a slave
  Wire.onRequest(requestEvent);
  //registers request Event as a callback that will execute automatically with a hardware TWI interrupt whenever an I2C master addresses 0x08 with a read bit

}

void loop() {
  maxTemp = -100.0;
  maxTempChannel = 0;
  minTemp = 1000.0; 
  //using -100 for max temp and 1000 for mintemp so that tempC>maTemp wil be satisfied on start, and minTemp to 1000 guarantees ch=0 satisfies tempC < minTemp
  minTempChannel = 0;      
  float tempSum = 0.0;  

  //Scan all channels
  for (uint8_t ch = 0; ch < 8; ch++) {
    float vForward = readChannelVoltage(ch);

    //T = T_ref + (V - V_ref) / TC - diode relationship
    float tempC = T_REF + ((vForward - V_AT_25C) / DIODE_TC);
    temperatures[ch] = tempC;
    tempSum += tempC; 

    //Track peak temp
    if (tempC > maxTemp) {
      maxTemp = tempC;
      maxTempChannel = ch;
    }
    if (tempC < minTemp) { 
      minTemp = tempC;   
      minTempChannel = ch;    
    }                  
  }
  avgTemp = tempSum / 8.0f;
//dividing by the 8 mux channels, using 8.0f explicitly will force single precision 32bit floating pt division.
  

  //Alert
  if (maxTemp >= ALARM_THRESHOLD_C) {
    unsigned long now = millis();
    //using millis bc delay(250) freezes CPU execution for 1/4 second per half cycle, using millis() lets the LED blink right at 2Hz while the loop scans all 8 channels
    if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
      //millis() returns an unsigned long that rolls over to 0 after 2^32-1 ms = 49.7 days, using unsigned subtraction wraps nicely with two's-compleent aithmetic, addition has the chance of overflowing and breaking the timer
      lastBlinkTime = now;
      ledState = !ledState;
      digitalWrite(FAULT_LED, ledState ? HIGH : LOW);
    }
  } else {
    ledState = false;
    digitalWrite(FAULT_LED, LOW);
    //without this else block if maxtemp goes below 60 C while ledState was true, the if block would stop running and the LED would remain permanently ON 
  }

  //Debug Output 
  static unsigned long lastPrint = 0;
  //the static class will preserve the value of lastPrint in SRAM across multiple calls to loop() while keeping its variable range contained inside loop()
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    //F(..) lets the string literal stay in flash memory, which conserves SRAM for variables and buffers just in case
    Serial.print(F("Max Temp: "));
    Serial.print(maxTemp, 1);
    Serial.print(F(" °C Ch "));
    Serial.print(maxTempChannel);
    Serial.print(F("  Min Temp: "));
    Serial.print(minTemp, 1);
    Serial.print(F(" °C Ch "));
    Serial.print(minTempChannel);
    Serial.print(F("  Avg Temp: "));  
    Serial.print(avgTemp, 1);    
    Serial.print(F(" °C"));
    if (maxTemp >= ALARM_THRESHOLD_C) {
      Serial.print(F(" [HOT!!]"));
    }
    Serial.println();
  }

  delay(50); //Loop freq, paces the main loop execution to about 20Hz preventing any multiplexor switching 
}