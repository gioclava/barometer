#include <Arduino.h>
#include <Stream.h>
#include <baro-driver/SPL06-007.h>
#include <crc/crc.h>
#include <Wire.h>
#include <EEPROM.h>

#define DEBUG      0
#define MSP2_SENSOR_BAROMETER       0x1F05
#define MSP2_SENSOR_AIRSPEED        0x1F06
#define PRES_CALIBRATION_ADDRESS 0
#define SWITCH_PIN 10
#define LED_PIN_1 5
#define LED_PIN_2 6
#define LED_PIN_3 9

double currentSpeed = 1;
long lastCalibration = 0;
long lastUnitSpeedOn = 0;
int8_t calibrationValue = 0;
struct mspSensorAirspeedDataMessage_t {
    uint8_t  instance;
    uint32_t timeMs;
    float    diffPressurePa;
    int16_t  temp;
};

struct mspSensorBaroDataMessage_t {
    uint8_t  instance;
    uint32_t timeMs;
    float    pressurePa;
    int16_t  temp; // centi-degrees C
} ;

Stream * _stream;
uint32_t _timeout;

void mspReset()
{
  _stream->flush();
  while (_stream->available() > 0)
    _stream->read();
}
// https://github.com/iNavFlight/inav/wiki/MSP-V2
void send(uint16_t messageID, void * payload, uint16_t size)
{
  uint8_t flag = 0;
    uint8_t crc = 0;
    uint8_t tmp_buf[2];

    _stream->write('$');
    _stream->write('X');
    _stream->write('<');

    crc = crc8_dvb_s2(crc, flag);
    _stream->write(flag);

    memcpy(tmp_buf, &messageID, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    memcpy(tmp_buf, &size, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    uint8_t * payloadPtr = (uint8_t*)payload;
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t b = *(payloadPtr++);
        crc = crc8_dvb_s2(crc, b);
        _stream->write(b);
    }
    _stream->write(crc);
}

void sendV2(uint8_t messageID, void * payload, uint16_t size)
{
  uint8_t flag = 0;
  Serial.write('$');
  Serial.write('X');
  Serial.write('<');
  Serial.write(flag);
  Serial.write(messageID);
  Serial.write(size);
  uint8_t checksum = 0;
  uint8_t * payloadPtr = (uint8_t*)payload;
  for (uint8_t i = 0; i < size; ++i) {
    uint8_t b = *(payloadPtr++);
    checksum = crc8_dvb_s2(checksum, b);
    Serial.write(b);
  }
  Serial.write(checksum);
}

void sendDebug(uint16_t messageID, void * payload, uint16_t size)
{
  uint8_t flag = 0;
  Serial.print('$', HEX);
  Serial.print('-');
  Serial.print('X', HEX);
  Serial.print('-');
  Serial.print('<', HEX);
  Serial.print('-');
  Serial.print(flag, HEX);
  Serial.print('-');
  Serial.print(messageID, HEX);
  Serial.print('-');
  Serial.print(size, HEX);
  Serial.print('-');
  uint8_t checksum = 0;
  uint8_t * payloadPtr = (uint8_t*)payload;
  for (uint8_t i = 0; i < size; ++i) {
    uint8_t b = *(payloadPtr++);
    checksum = crc8_dvb_s2(checksum, b);
    Serial.print(b, HEX);
    Serial.print('-');
  }
  Serial.print(checksum, HEX);
  Serial.println("");
}

void setup() {
    Wire.begin();
    Serial.begin(115200);
    _stream = &Serial;
    if(DEBUG) Serial.println("SPL init");
    SPL_init();
    if(DEBUG) Serial.println("SPL init finished");
    while(!pressureAvailable());
    calibrationValue = EEPROM.read(PRES_CALIBRATION_ADDRESS);
    pinMode(SWITCH_PIN,INPUT);
    pinMode(LED_PIN_1,OUTPUT);
    pinMode(LED_PIN_2,OUTPUT);
    pinMode(LED_PIN_3,OUTPUT);
    digitalWrite(LED_PIN_1,HIGH);
    digitalWrite(LED_PIN_2,HIGH);
    digitalWrite(LED_PIN_3,HIGH);
}

void loop() {
  if(pressureAvailable()){
    double frontPressure = getFrontPressure();
    double middlePressure = getMiddlePressure();
    double middleTemperature = getMiddleTemperature();
    double frontTemperature = getFrontTemperature();
    double difference = frontPressure-middlePressure-calibrationValue;
    if((millis()- lastCalibration > 100) && !digitalRead(SWITCH_PIN)){
      lastCalibration = millis();
      calibrationValue = (int8_t) (frontPressure - middlePressure);
      EEPROM.write(PRES_CALIBRATION_ADDRESS, calibrationValue);
    }
    difference = max(1,difference);
    currentSpeed = sqrt(difference * 2 / 1.225)*3.6;
    mspSensorAirspeedDataMessage_t speedSensor = { 0, millis(), difference, (int16_t) middleTemperature*100};
    mspSensorAirspeedDataMessage_t baroSensor = { 0, millis(), middlePressure, (int16_t) middleTemperature*100};
    
    if(DEBUG){
      //sendDebug(MSP2_SENSOR_AIRSPEED, &speedSensor, sizeof(speedSensor));
      Serial.print("dynamicP pascal: ");
      Serial.print(difference);
      Serial.print(" staticP pascal: ");
      Serial.print(middlePressure);
      Serial.print("\n");
      }
    else{
      send(MSP2_SENSOR_AIRSPEED, &speedSensor, sizeof(speedSensor));
      send(MSP2_SENSOR_BAROMETER, &baroSensor, sizeof(baroSensor));
      //sendV2(MSP2_SENSOR_AIRSPEED, &speedSensor, sizeof(speedSensor));}
    }
  }
  if(currentSpeed>20) digitalWrite(LED_PIN_3,LOW); else digitalWrite(LED_PIN_3,HIGH);
  if(currentSpeed>10) digitalWrite(LED_PIN_1,LOW); else digitalWrite(LED_PIN_1,HIGH);
  if(millis() - lastUnitSpeedOn > 50) digitalWrite(LED_PIN_2,HIGH);
  if(millis() - lastUnitSpeedOn > 3000/((int)currentSpeed % 10)){
    lastUnitSpeedOn = millis();
    digitalWrite(LED_PIN_2,LOW);
    Serial.println(currentSpeed);
  }
}