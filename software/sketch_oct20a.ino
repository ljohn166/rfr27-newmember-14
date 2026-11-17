#include <Arduino.h>
#include <Wire.h>

//Assignments 
const uint8_t MUX_S0 = 2;
const uint8_t MUX_S1 = 3;
const uint8_t MUX_S2 = 4;
const uint8_t MUX_S3 = 5;
const uint8_t MUX_SIG = A0;
const uint8_t FAULT_LED = 6;

//I2C 
#define I2C_SLAVE_ADDR 0x08

//Diode sensor calibrations
//tempco -2.0 mV/°C (-0.0020 V/°C)
const float DIODE_TC = -0.0020; //V/C
const float V_AT_25C = 0.620;       
const float T_REF    = 25.0;      
const float ALARM_THRESHOLD_C = 60.0;   

//System states
float temperatures[16];
float maxTemperature = -100.0;
uint8_t maxTempChannel = 0;

// Struct formatted for I2C data transfer (fixed-point 10x scaled integers)
struct __attribute__((packed)) I2CPacket {
  int16_t maxTemp_x10;      // e.g., 605 = 60.5°C, By multiplying by 10 and
  // casting to a signed 16-bit integer, a temp like 60.5C becomes 605. This will have 0.1C precision while cutting down some on the packet size (2 bytes per sensor instead of 4).
  uint8_t maxChannel;       // Channel (0 to 15)
  int16_t allTemps_x10[16]; // All 16 channels
} i2cData;

//LED Blink State Vars
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL_MS = 250;//2 Hz 
bool ledState = false;

// Select Channel on multiplex
void selectMuxChannel(uint8_t channel) { 
  //doing this translates into better instructions
  digitalWrite(MUX_S0, (channel >> 0) & 0x01);
  digitalWrite(MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX_S2, (channel >> 2) & 0x01);
  digitalWrite(MUX_S3, (channel >> 3) & 0x01);
}

//Read Filtered Analog Voltage 
float readChannelVoltage(uint8_t channel) {
  selectMuxChannel(channel);
  delayMicroseconds(10); // Allow multiplexer switch and ADC capacitor to stabilize

  //Discard first reading after channel switch so no crosstalk - 
  analogRead(MUX_SIG); //drains and recharges the capacitor to the new channel's voltage before taking the real measurement.

  //8x Oversampling for noise suppression
  uint32_t adcSum = 0;
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
  i2cData.maxTemp_x10 = (int16_t)(maxTemperature * 10.0f);
  i2cData.maxChannel  = maxTempChannel;
  for (int i = 0; i < 16; i++) {
    i2cData.allTemps_x10[i] = (int16_t)(temperatures[i] * 10.0f);
  }

  //Send packet across the I2C
  Wire.write((uint8_t*)&i2cData, sizeof(i2cData));
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
  analogReference(INTERNAL);  //sets the full -scale ceiling to 1.1V which should cut temp granularity down about 0.54C 
  //adc primining
  analogRead(MUX_SIG); //

  //Initialize I2C 
  Wire.begin(I2C_SLAVE_ADDR); //as a slave
  Wire.onRequest(requestEvent);
}

void loop() {
  maxTemperature = -100.0;
  maxTempChannel = 0;

  //Scan all channels
  for (uint8_t ch = 0; ch < 16; ch++) {
    float vForward = readChannelVoltage(ch);

    //T = T_ref + (V - V_ref) / TC - diode relationship
    float tempC = T_REF + ((vForward - V_AT_25C) / DIODE_TC);
    temperatures[ch] = tempC;

    //Track peak temp
    if (tempC > maxTemperature) {
      maxTemperature = tempC;
      maxTempChannel = ch;
    }
  }

  //Alert
  if (maxTemperature >= ALARM_THRESHOLD_C) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
      lastBlinkTime = now;
      ledState = !ledState;
      digitalWrite(FAULT_LED, ledState ? HIGH : LOW);
    }
  } else {
    ledState = false;
    digitalWrite(FAULT_LED, LOW);
  }

  //Debug Output 
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print(F("Max Temp: "));
    Serial.print(maxTemperature, 1);
    Serial.print(F(" °C Ch "));
    Serial.print(maxTempChannel);
    if (maxTemperature >= ALARM_THRESHOLD_C) {
      Serial.print(F(" [HOT!!]"));
    }
    Serial.println();
  }

  delay(50); //Loop freq
}