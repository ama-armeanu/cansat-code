#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Wire.h>
#include "DFRobot_LTR390UV.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 18 //can be any pin really 

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
DFRobot_LTR390UV ltr390(LTR390UV_DEVICE_ADDR, &Wire);

uint16_t baselineFull = 0;
uint16_t baselineIR = 0;
uint32_t baselineUV = 0;

//function to calibrate baselin values for IR and full (IR and Visible light)
void calibrateBaseline(uint8_t samples = 5) {
  uint32_t sumFull = 0;
  uint32_t sumIR = 0;

  //calculating ambient (no light) baseline
  for (uint8_t i = 0; i < samples; i++) {
    uint32_t lum = tsl.getFullLuminosity();
    sumIR += (lum >> 16);
    sumFull += (lum & 0xFFFF);
    sumUv += ltr390.readOriginalData();
    delay(120);
  }

  baselineFull = sumFull / samples;
  baselineIR = sumIR / samples;
  baselinUV = sumUv / samples;

  Serial.print("Baseline Recorded -> Full: ");
  Serial.print(baselineFull);
  Serial.print(" | IR: ");
  Serial.println(baselineIR);
  Serial.print(" | UV: ");
  Serial.println(baselineUV);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN); 
  digitalWrite(LED_PIN, LOW);

  if (!tsl.begin()) {
    Serial.println("TSL2591 not found! Check wiring.");
    while (1);
  }
  while (ltr390.begin() != 0) {
    Serial.println("LTR390 not found! Check wiring.");
    delay(1000);
  }

  tsl.setGain(TSL2591_GAIN_LOW); //gain of 1x
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS); //integration time is 100 ms

  ltr390.setALSOrUVSMeasRate(ltr390.e20bit, ltr390.e100ms); //integration time is 100 ms
  ltr390.setALSOrUVSGain(ltr390.eGain18);
  ltr390.setMode(ltr390.eUVSMode);

  delay(500);

  calibrateBaseline();

}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(250);

  calibrateBaseline(); //for IR and Visible readings 

  uint32_t rawUV = ltr390.readOriginalData();

  uint32_t lum = tsl.getFullLuminosity();
  uint16_t rawIR = lum >> 16;          
  uint16_t rawFull = lum & 0xFFFF;  

  //raw readings
  int32_t netUV = max((int32_t)0, (int32_t)rawUV - (int32_t)baselineUV);
  int32_t netFull = max(0, (int32_t)rawFull - (int32_t)baselineFull);
  int32_t netIR = max(0, (int32_t)rawIR - (int32_t)baselineIR);
  int32_t netVisible = max(0, netFull - netIR);

  //normalising into counts/second (0.1 seconds)
  // new / (gain * time)
  float normUV = netUV / (18.0 * 0.1);
  float normVis = netVisible / (1.0 * 0.1);
  float normIR = netIR / (1.0 * 0.1);

  float irradUV  = normUV  / 1050.0; //UVS responsitivity adjusted for 365nm wavelength
  float irradVis = normVis / 2510.0; //Visible responsitivity adjusted for 625nm wavelength
  float irradIR  = normIR  / 1550.0; //IR responsitivity adjusted for 850nm wavelength

  Serial.print("Measurements in irradiance: ");
  Serial.println("Visible: ", irradVis);
  Serial.println("Ultraviolet: ", irradUV);
  Serial.println("Infrared: ", irradIR);

}
