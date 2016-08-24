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
  randomSeed(analogRead(0));
  uint16_t temp;
  uint16_t ret = 1;
  // 4th level
  ret = static_cast<uint16_t >(random(1, 6)); //rand return a long between min and max-1
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: 4th level address:");
    Serial.println(ret, OCT);;
  #endif
  // 3th level
  randomSeed(analogRead(0));
  temp = static_cast<uint16_t >(random(0, 6));
  ret <<= 3;
  ret &= temp;
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: 3rd level address:");
    Serial.println(ret, OCT);;
  #endif
  // 2nd level
  randomSeed(analogRead(0));
  temp = static_cast<uint16_t >(random((temp==0)?1:0, 6));
  ret <<= 3;
  ret &= temp;
  #ifdef SERIAL_DEBUG_UTILITY
    Serial.print("getRandomAddressForNetwork: 2nd level address:");
    Serial.println(ret, OCT);
  #endif

  // 1st level
  randomSeed(analogRead(0));

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

class Sensor;

class SensorMessageHelper {
  RF24Network network_;
  MessageHelper message_;
  RF24NetworkHeader lastHeader;
 public:
  SensorMessageHelper(RF24Network &network);
  bool receiveMessage();
  bool updateNetwork();
  MessageHelper &getMessage();
  bool sendMessage(const Sensor * const & sensor, uint16_t to_node);
  bool sendAck(const Sensor * const & sensor);
  void prepareHeartBeatAckMessage(const Sensor * const & sensor);
  void prepareHeartBeatMessage(const Sensor * const & sensor);
  void prepareAckMessage(const Sensor * const & sensor);
};



class Sensor {
  const uint16_t address_;  // this is the RF24Network addres in octal form
  const uint8_t sensor_id_; // the id is a number that could be 1 to 255
                            // each group of sensor that are connected to a receiver
                            // can have a numeration from 0 to 255
                            // 1 - 255 -> 1 receiver the receiver has always the id 0
                            // simultaneusly a receiver has a ID refered to its own super group
  const Sensor_type sensorType_;
  SensorMessageHelper *messagehelper_;
  const Sensor_information_type sensorInformationType_;
  Leds *leds_;

  Sensor * myreceiver[MAX_RECEIVER]; //use vector for a dynamic number of receiver

 public:
  Sensor(uint16_t const address,
         uint8_t const id,
         Sensor_type const type,
         Sensor_information_type const infotype,
         SensorMessageHelper *sensorMessageHelper,
         Leds *leds);

  Sensor_type getSensorType() const;
  uint8_t getSensorID() const;
  uint16_t getSensorAddress() const;
  Sensor_information_type getSensorInformationType() const;

  void evaluateMessage();

  // Communication methods
  bool sendHeartBeat();
  bool sendMessage();
  void update();
  MessageHelper& getMessage();
  void addReceiver(uint16_t const address,
                   uint8_t const id,
                   Sensor_type const type,
                   Sensor_information_type const infotype,
                   SensorMessageHelper *sensorMessageHelper,
                   Leds *leds );
 protected:
  virtual bool sensorPresentationReceived() = 0;
  virtual bool setSensorInfo() = 0;
  virtual bool getSensorInfo() = 0;
  virtual bool systemFuncion() = 0;
  virtual bool incomingDataIsAStream() = 0;
};



class LightRelaySensor : public Sensor {
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

  LightRelaySensor(uint16_t const address,
                   uint8_t const id,
                   SensorMessageHelper *sensorMessageHelper,
                   Leds *leds);
 private:
  bool sensorPresentationReceived();
  bool setSensorInfo();
  bool getSensorInfo();
  bool systemFuncion();
  bool incomingDataIsAStream();
};

class MotionSensor : public Sensor {
 public:
  MotionSensor(uint16_t const address,
               uint8_t const id,
               SensorMessageHelper *sensorMessageHelper,
               Leds *leds);
 private:
    bool sensorPresentationReceived();
    bool setSensorInfo();
    bool getSensorInfo();
    bool systemFuncion();
    bool incomingDataIsAStream();

};





class LightSensor : public Sensor {
 public:
  LightSensor(uint16_t const address,
              uint8_t const id,
              SensorMessageHelper *sensorMessageHelper,
              Leds *leds);

 private:
  bool sensorPresentationReceived();
  bool setSensorInfo();
  bool getSensorInfo();
  bool systemFuncion();
  bool incomingDataIsAStream();

};

class ReceiverListSensor : public Sensor {
 public:
  ReceiverListSensor(uint16_t const address,
              uint8_t const id,
              SensorMessageHelper *sensorMessageHelper,
              Leds *leds);

 private:
  bool sensorPresentationReceived();
  bool setSensorInfo();
  bool getSensorInfo();
  bool systemFuncion();
  bool incomingDataIsAStream();

};


#endif //ARDUINO_MCWW_UTILITY_H
