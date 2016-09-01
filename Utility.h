//
// Created by Matteo Favaro on 03/08/16.
//

#ifndef ARDUINO_MCWW_UTILITY_H
#define ARDUINO_MCWW_UTILITY_H

#include <Arduino.h>

#include "RF24Network.h"
#include "Message.h"
#include "Leds.h"
/*
 * return a adress compatible with the RF24Network
 * addressing format, a octal number where the maximum is
 * 04444 -->
 * 1 | 2 | 3 | 4 |
 * 5 | 5 | 5 | 5 |
 *
 * 1: 5th child of node with address 555
 * 2: 5th child of node with address 55
 * 3: 5th child of node with address 5
 * 4: 5th child of master
 */
inline uint16_t getRandomAddressForNetwork() {
  randomSeed(analogRead(A0));
  uint16_t temp;
  uint16_t ret = 1;
  // 4th level
  ret = static_cast<uint16_t >(random(1, 6)); //rand return a long between min and max-1
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: 4th level address:");
    Serial.println(ret, OCT);;
  #endif
  // 3th level
  randomSeed(analogRead(A0));
  temp = static_cast<uint16_t >(random(0, 6));
  ret <<= 3;
  ret &= temp;
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: 3rd level address:");
    Serial.println(ret, OCT);;
  #endif
  // 2nd level
  randomSeed(analogRead(A0));
  temp = static_cast<uint16_t >(random((temp==0)?1:0, 6));
  ret <<= 3;
  ret &= temp;
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: 2nd level address:");
    Serial.println(ret, OCT);
  #endif

  // 1st level
  randomSeed(analogRead(A0));

  temp = static_cast<uint16_t >(random((temp==0)?1:0, 6));
  ret <<= 3;
  ret &= temp;
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: address:");
    Serial.println(ret, OCT);
  #endif

  // END
  return ret;

}



//header type for message
/*
 * User types** (1-127) 1-64 will NOT be acknowledged <br>
 * System types** (128-255) 192 through 255 will NOT be acknowledged<br>
 *
 */
/*
 * if the payload is fragmented the HEADER TYPE doesn't matter whatever it is
 */
#define MESSAGE_HEADER_TYPE            63
/*
 * For the moment the MAX number of receiver is hardcoded in future a vector
 * will be used
 */
#define MAX_RECEIVER 2
#define MAX_RETRY 2
#define MIN_WAIT_TIME 4000
#define MAX_SENSOR_PER_ARDUINO 3


bool isHeartBeatMessage(MessageHelper *message);
bool isMessageFromParent(MessageHelper *message);
Payload_type getPayloadTypeFromData(char* data);
Payload_type getPayloadTypeFromData(char data);
Payload_type getPayloadTypeFromData(unsigned char data);
Payload_type getPayloadTypeFromData(int16_t data);
Payload_type getPayloadTypeFromData(uint16_t data);
Payload_type getPayloadTypeFromData(long data);
Payload_type getPayloadTypeFromData(unsigned long data);
Payload_type getPayloadTypeFromData(float data);
Payload_type getPayloadTypeFromData(bool data);
template<class T>
uint8_t getDim(T const& obj) {return sizeof(obj);}
inline uint8_t getDim(char string[]) {return strlen(string);}

class Sensor {
  const uint16_t address_;  // this is the RF24Network addres in octal form
  uint8_t sensor_id_; // the id is a number that could be 1 to 255
                            // each group of sensor that are connected to a receiver
                            // can have a numeration from 0 to 255
                            // 1 - 255 -> 1 receiver the receiver has always the id 0
                            // simultaneusly a receiver has a ID refered to its own super group
  const Sensor_type sensorType_;
  const Sensor_information_type sensorInformationType_;
  char name[31];//30 char for name I.E "motion stairs entrance 1" (24 char)
  Sensor *parent_;
  bool informationAvailable_;
  bool messageSent_;

 public:
  static Sensor *getSensorObj(MessageHelper *heartbeatMessage);
  Sensor(uint16_t const address,
         uint8_t const id,
         Sensor_type const type,
         Sensor_information_type const infotype);

  Sensor_type getSensorType();
  uint8_t getSensorID();
  void messageSent();
  void setMessageHasToSend();
  void setInformationAvaiable();
  void setInformationUnAvaiable();
  bool isInformationAvaiable();
  bool isMessageSent();
  virtual bool update() = 0;
  virtual bool hasToSend() = 0;
  virtual bool prepareDataMessageToSend(MessageHelper *message) = 0;

  void setSensorID(uint8_t id);
  uint16_t getSensorAddress() const;
  Sensor_information_type getSensorInformationType() const;

  Sensor *getParent();
  void setParent(Sensor *parent);

  void evaluateMessage(MessageHelper *message);

  // Communication methods
  //change all with the new style
  MessageHelper* getMessage();

 protected:
  virtual bool sensorChildrenPresentationReceived(MessageHelper *message) = 0;
  virtual bool sensorParentPresentationReceived(MessageHelper *message) = 0;
  virtual bool setSensorInfo(MessageHelper *message) = 0;
  virtual bool getSensorInfo(MessageHelper *message) = 0;
  virtual bool systemFuncion(MessageHelper *message) = 0;
  virtual bool incomingDataIsAStream(MessageHelper *message) = 0;
  //method for returning available command for this sensor;
};

MessageHelper *prepareHeartChildrenBeatMessage(Sensor *fromSensor, MessageHelper *message);
MessageHelper *prepareHeartParentBeatMessage(Sensor *fromSensor, MessageHelper *message);
MessageHelper *prepareHeartBeatAckMessage(Sensor *fromSensor, MessageHelper *message);
MessageHelper *prepareAckMessage(Sensor *fromSensor, MessageHelper *message);
template<class T>
MessageHelper *prepareNewDataMessage(Sensor *fromSensor, MessageHelper *message, T data) {
  message->setSensorID(fromSensor->getSensorID());
  message->setSensorAddress(fromSensor->getSensorAddress());
  message->setCommand(C_SET);
  message->setSensorType(fromSensor->getSensorType());
  message->setSystemMessageType(I_NEWVALUE);
  message->setSensorInformationType(fromSensor->getSensorInformationType());
  message->setPayloadType(getPayloadTypeFromData(data));
  memcpy(message->getPayload(), &data, getDim(data));
}


class SensorMessageHelper {
  RF24Network *network_;
  RF24NetworkHeader lastHeader;
 public:
  void begin(RF24Network &network);
  uint8_t receiveMessage(MessageHelper * message);
  uint8_t updateNetwork();
  bool networkAvailable();
  bool sendMessage(Sensor *fromSensor,
                   Sensor *toSensor,
                   MessageHelper const *message);
};

class SensorManager {
  //use vector
  Sensor *sensorOnThisArduino[MAX_SENSOR_PER_ARDUINO];
  MessageHelper message_;

 public:
  bool addSensor(Sensor * sensor);
  bool update();
  bool sensorsHaveToSend();
  MessageHelper *getMessage();

  Sensor *getSensors(uint8_t index);
  uint8_t getSensorsNum();
};



class RelaySensor : public Sensor {
  // In future a vector will be used for the moment 5 sensor per recevier is hardcoded
  Sensor* sensormap[5];
  /*
   * this is a sensor in the end even if it is a actuator the only difference is that
   * it receives data(trigger) directly from the sensor that trigger him,
   * this permit to have a tree of sensor without a root as in other case
   * every sensor is connected to none or more receiver
   * and the receiver is connected to a none or more other receiver.
   * for better clarify the hierarchy the follow example is provided
   *
   * 3 Sensor: 2 motion sensor, 1 light sensor
   * 1 receiver: relays
   *  The 3 sensor are "connected to the receiver" and they send data to the receiver
   *  the receiver doen't need a master for knowing what he have to do.
   *  if a new master some other receiver need the data of this receiver he can register
   *  to the receiver e the new receiver will receive the changes
   *
   */
 public:

  RelaySensor(uint16_t const address,
                   uint8_t const id);
  bool prepareDataMessageToSend(MessageHelper *message);
  bool update();
  bool hasToSend();
 private:
  bool sensorChildrenPresentationReceived(MessageHelper *message);
  bool sensorParentPresentationReceived(MessageHelper *message);
  bool setSensorInfo(MessageHelper *message);
  bool getSensorInfo(MessageHelper *message);
  bool systemFuncion(MessageHelper *message);
  bool incomingDataIsAStream(MessageHelper *message);

};

class MotionSensor : public Sensor {
 public:
  MotionSensor(uint16_t const address,
               uint8_t const id);
  bool prepareDataMessageToSend(MessageHelper *message);
  bool update();
  bool hasToSend();
 private:
  bool sensorChildrenPresentationReceived(MessageHelper *message);
  bool sensorParentPresentationReceived(MessageHelper *message);
  bool setSensorInfo(MessageHelper *message);
  bool getSensorInfo(MessageHelper *message);
  bool systemFuncion(MessageHelper *message);
  bool incomingDataIsAStream(MessageHelper *message);

};

class LightLevelSensor : public Sensor {
 public:
  LightLevelSensor(uint16_t const address,
              uint8_t const id);
  bool prepareDataMessageToSend(MessageHelper *message);
  bool update();
  bool hasToSend();
 private:
  bool sensorChildrenPresentationReceived(MessageHelper *message);
  bool sensorParentPresentationReceived(MessageHelper *message);
  bool setSensorInfo(MessageHelper *message);
  bool getSensorInfo(MessageHelper *message);
  bool systemFuncion(MessageHelper *message);
  bool incomingDataIsAStream(MessageHelper *message);

};




#endif //ARDUINO_MCWW_UTILITY_H
