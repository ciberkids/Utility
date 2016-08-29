//
// Created by Matteo Favaro on 03/08/16.
//

#include <Message/Message.h>
#include "Utility.h"

#include "Message.h"


SensorMessageHelper::SensorMessageHelper(RF24Network &network) : network_(network)
    {

    }


bool SensorMessageHelper::receiveMessage() {
  bool ret = false;
  if(network_.available()) {
    ret =
        network_.read(lastHeader, &message_.internalMessage_, sizeof(Message)) != 0 ;
    #if defined(SERIAL_DEBUG_MESSAGE_HELPER)
    Serial.println(lastHeader.toString());
    Serial.println(message_.toString());
    #endif
  }
  return ret;
}
bool SensorMessageHelper::updateNetwork() {
  network_.update();
}

MessageHelper &SensorMessageHelper::getMessage() {
  return message_;
}

void SensorMessageHelper::prepareHeartBeatMessage(const Sensor * const & sensor) {
  message_.setSensorID(sensor->getSensorID());
  message_.setSensorAddress(sensor->getSensorAddress());
  message_.setCommand(C_PRESENTATION);
  message_.setSensorType(sensor->getSensorType());
  message_.setSystemMessageType(I_HEARTBEAT);
  message_.setSensorInformationType(sensor->getSensorInformationType());
  message_.setPayloadType(P_HEARTBEAT);
  message_.setPayloadSize(0);
}

void SensorMessageHelper::prepareHeartBeatAckMessage(const Sensor * const & sensor) {
  //use my id for the answer so the receiver will know who is the sender
  message_.setSensorID(sensor->getSensorID());
  //obviusly we are presenting us to the other node
  message_.setCommand(C_PRESENTATION);
  message_.setSensorAddress(sensor->getSensorAddress());
  message_.setSensorType(message_.getSensorType());
  message_.setSystemMessageType(I_HEARTBEAT_RESPONSE);
  message_.setSensorInformationType(sensor->getSensorInformationType());
  message_.setPayloadType(P_HEARTBEAT);
  message_.setPayloadSize(0);
}

void SensorMessageHelper::prepareAckMessage(const Sensor * const & sensor) {
  //use my id for the answer so the receiver will know who is the sender
  message_.setSensorID(sensor->getSensorID());
  message_.setSensorAddress(sensor->getSensorAddress());
  //we are acknowledging last message
  message_.setCommand(C_ACK);
  // my type
  message_.setSensorType(sensor->getSensorType());
  message_.setSystemMessageType(I_ACK);
  // we are acknowledging the information of the incoming message
  message_.setSensorInformationType(message_.getSensorInformationType());
  message_.setPayloadType(P_ACK);
  message_.setPayloadSize(0);
}



bool SensorMessageHelper::sendMessage(const Sensor * const & sensor, uint16_t to_node) {
  RF24NetworkHeader header(to_node, MESSAGE_HEADER_TYPE);
  header.from_node = sensor->getSensorAddress();
  lastHeader = header;
  #if defined(SERIAL_DEBUG)
    Serial.println(F("Sending message with header:"));
    Serial.print(lastHeader.toString());
    delay(1000);
  #endif
  return network_.write(lastHeader, &message_.internalMessage_, sizeof(Message));
}

bool SensorMessageHelper::sendAck(const Sensor * const & sensor) {
  return sendMessage(sensor, lastHeader.from_node);
}
/////--------------------------------------

Sensor::Sensor(uint16_t const address,
               uint8_t const sensor_id,
               Sensor_type const sensorType,
               Sensor_information_type const sensorInformationType,
               SensorMessageHelper *sensorMessageHelper,
               Leds *leds)
    : address_(address),
      sensor_id_(sensor_id),
      sensorType_(sensorType),
      sensorInformationType_(sensorInformationType),
      messagehelper_(sensorMessageHelper),
      leds_(leds)
      { }


Sensor_type Sensor::getSensorType() const {
  return sensorType_;
}
uint8_t Sensor::getSensorID() const {
  return sensor_id_;
}
uint16_t Sensor::getSensorAddress() const {
  return address_;
}
Sensor_information_type Sensor::getSensorInformationType() const {
  return sensorInformationType_;
}

bool Sensor::sendMessage() {
  unsigned long sendAgain = millis() + MIN_WAIT_TIME;
  bool ans = false;
  uint8_t retry = 0;
  for (int i = 0; i < MAX_RECEIVER; i++) {
    leds_->ledGreenLongBlink();

    if(myreceiver[i] == NULL) continue;
    while (!ans && retry < MAX_RETRY) {
      #if defined(SERIAL_DEBUG)
        Serial.print(F("trying to send hearbeat to "));
        Serial.print(myreceiver[i]->getSensorAddress(), OCT);
        Serial.print(F(" i have this address: "));
        Serial.print(address_, OCT);
      Serial.print(F(" retry: "));
      Serial.println(retry);
      delay(2000);
      #endif
      if (messagehelper_->sendMessage(this, myreceiver[i]->getSensorAddress())) {
        leds_->ledWhiteLongBlink();
        //wait ACK
        while (millis() < sendAgain && !ans) {
          messagehelper_->updateNetwork();
          ans = messagehelper_->receiveMessage();
          #if defined(SERIAL_DEBUG)
          if(ans) {
            Serial.println(F("HEART BEAT ANSWERED"));
            Serial.println(messagehelper_->getMessage().toString());
          }
          else {
            Serial.println(F("NO ANSWER"));

          }
          #endif
        }
        sendAgain = millis() + MIN_WAIT_TIME;
        if (!ans) retry++;

        #if defined(SERIAL_DEBUG)
        Serial.println(F("NEW RETRY!!"));
        delay(2000);
        #endif
      }
      else {
        #if defined(SERIAL_DEBUG)
        Serial.println(F("SENDFAIL!!!"));
        delay(2000);
        #endif
        leds_->ledRedThreeLongBlink();
        leds_->ledRedOn();
        retry++;
      }
    }
    if(ans) {
      leds_->ledWhiteThreeLongBlink();
    }
    if(retry > MAX_RETRY) {
      leds_->ledRedThreeLongBlink();
    }
  }
  return ans;
}


bool Sensor::sendHeartBeat() {
  messagehelper_->prepareHeartBeatMessage(this);
  return sendMessage();
}


void Sensor::update() {
  messagehelper_->updateNetwork();
}

MessageHelper& Sensor::getMessage() {
  return messagehelper_->getMessage();
}

void Sensor::evaluateMessage() {

  switch (getMessage().getCommand()) {
    case C_PRESENTATION:
      sensorPresentationReceived(); //register the sensor if necessary
      messagehelper_->prepareHeartBeatAckMessage(this);
      sendMessage();
      break;
    case C_SET:
      setSensorInfo(); //based on sensor information type and ID set the data
      break;
    case C_REQ:
      getSensorInfo(); //based on sensor information type and ID get the data
      break;
    case C_SYSTEM:
      systemFuncion(); // set the sensor type, delay time, or other
      break;
    case C_STREAM:
      incomingDataIsAStream();// some kind or streaming data
      break;
    default:
      return;
  }
}

void Sensor::addReceiver(uint16_t const address,
                         uint8_t const id,
                         Sensor_type const type,
                         Sensor_information_type const infotype,
                         SensorMessageHelper *sensorMessageHelper,
                         Leds  *leds ) {
  //TODO better memory handling when the autoconfiguration will bee implemented
  myreceiver[id] = new ReceiverListSensor(address,id,sensorMessageHelper, leds);
}

Sensor *Sensor::getSensorObj(Sensor_type type, MessageHelper &message) {

  switch(type) {
    case S_LIGHT: return new LightSensor(message.getSensorAddress(),
                                         message.getSensorID(),
                                         NULL,
                                         NULL);
    default:
      return NULL;
  }
}

//---------------------------------------------

LightRelaySensor::LightRelaySensor(uint16_t const address,
                                   uint8_t const id,
                                   SensorMessageHelper *sensorMessageHelper,
                                   Leds *leds) :
    Sensor(address, id, S_LIGHT, V_LIGHT, sensorMessageHelper, leds)
{ }

bool LightRelaySensor::sensorPresentationReceived() {
  if(sensormap[getMessage().getSensorID()] == NULL) {
    sensormap[getMessage().getSensorID()] = Sensor::getSensorObj(getMessage().getSensorType(), getMessage());
  }
  return true;
}
bool LightRelaySensor::setSensorInfo() {

  return false;
}
bool LightRelaySensor::getSensorInfo() {
  return false;
}
bool LightRelaySensor::systemFuncion() {
  return false;
}
bool LightRelaySensor::incomingDataIsAStream() {
  return false;
}



//---------------------------------------------

MotionSensor::MotionSensor(uint16_t const address,
                           uint8_t const id,
                           SensorMessageHelper *sensorMessageHelper,
                           Leds *leds) :
                            Sensor(address,
                                   id,
                                   S_MOTION,
                                   V_MOTION,
                                   sensorMessageHelper,
                                   leds) {
}

bool MotionSensor::sensorPresentationReceived() {
  return false;
}
bool MotionSensor::setSensorInfo() {
  return false;
}
bool MotionSensor::getSensorInfo() {
  return false;
}
bool MotionSensor::systemFuncion() {
  return false;
}
bool MotionSensor::incomingDataIsAStream() {
  return false;
}

//---------------------------------------------

LightSensor::LightSensor(uint16_t const address,
                         uint8_t const id,
                         SensorMessageHelper *sensorMessageHelper,
                         Leds *leds) :
    Sensor(address,
           id,
           S_LIGHT_LEVEL,
           V_LIGHT_LEVEL,
           sensorMessageHelper,
           leds) { }

bool LightSensor::sensorPresentationReceived() {
  return false;
}
bool LightSensor::setSensorInfo() {
  return false;
}
bool LightSensor::getSensorInfo() {
  return false;
}
bool LightSensor::systemFuncion() {
  return false;
}
bool LightSensor::incomingDataIsAStream() {
  return false;
}

//---------------------------------------------

ReceiverListSensor::ReceiverListSensor(uint16_t const address,
                         uint8_t const id,
                         SensorMessageHelper *sensorMessageHelper,
                         Leds *leds) :
    Sensor(address,
           id,
           S_NONE_TYPE,
           V_CUSTOM,
           sensorMessageHelper,
           leds) { }

bool ReceiverListSensor::sensorPresentationReceived() {
  return false;
}
bool ReceiverListSensor::setSensorInfo() {
  return false;
}
bool ReceiverListSensor::getSensorInfo() {
  return false;
}
bool ReceiverListSensor::systemFuncion() {
  return false;
}
bool ReceiverListSensor::incomingDataIsAStream() {
  return false;
}

//---------------------------------------------



