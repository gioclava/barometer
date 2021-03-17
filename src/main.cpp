#include <Arduino.h>
#include <Stream.h>
#include <baro-driver/SPL06-007.h>
#include <crc/crc.h>

#include <Wire.h>

#define DEBUG       1
#define MSP2_SENSOR_BAROMETER       0x1F05
#define MSP2_SENSOR_AIRSPEED        0x1F06

struct mspSensorAirspeedDataMessage_t {
    uint8_t  instance;
    uint32_t timeMs;
    float    diffPressurePa;
    int16_t  temp;
};

Stream * _stream;
uint32_t _timeout;
mspSensorAirspeedDataMessage_t speedSensor = { 1, 2, 0.0, 3};


void mspReset()
{
  _stream->flush();
  while (_stream->available() > 0)
    _stream->read();
}
// https://github.com/iNavFlight/inav/wiki/MSP-V2
void send(uint16_t messageID, void * payload, uint8_t size)
{
  uint8_t flag = 0;
  _stream->write('$');
  _stream->write('X');
  _stream->write('<');
  _stream->write(flag);
  _stream->write(messageID);
  _stream->write(size);
  uint8_t checksum = 0;
  uint8_t * payloadPtr = (uint8_t*)payload;
  for (uint8_t i = 0; i < size; ++i) {
    uint8_t b = *(payloadPtr++);
    checksum = crc8_dvb_s2(checksum, b);
    _stream->write(b);
  }
  _stream->write(checksum);
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
    //SPL_init(SPL_1);
    //SPL_init(SPL_2);
    
    Serial.begin(19200);
  // put your setup code here, to run once:

}

void loop() {
  if(DEBUG){
      sendDebug(MSP2_SENSOR_AIRSPEED, &speedSensor, sizeof(speedSensor));}
  else{sendV2(MSP2_SENSOR_AIRSPEED, &speedSensor, sizeof(speedSensor));}
  delay(5000);
}