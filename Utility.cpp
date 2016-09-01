//
// Created by Matteo Favaro on 03/08/16.
//

#include "Utility.h"
#include <string.h>
#include "Message.h"


//// ***** FREE FUNC
bool isHeartBeatMessage(MessageHelper * message) {
  return (message->getCommand() == C_PRESENTATION_PARENT ||
      message->getCommand() == C_PRESENTATION_CHILDREN)
      &&
          message->getSystemMessageType() == I_HEARTBEAT;
}
bool isMessageFromParent(MessageHelper * message) {
  return message->getCommand() == C_PRESENTATION_PARENT &&
      message->getSystemMessageType() == I_HEARTBEAT;
}

Payload_type getPayloadTypeFromData(char* data) { return P_STRING; };
Payload_type getPayloadTypeFromData(char data) { return P_CHAR; };
Payload_type getPayloadTypeFromData(unsigned char data) { return P_UCHAR; };
Payload_type getPayloadTypeFromData(unsigned int data) { return P_UINT; };
Payload_type getPayloadTypeFromData(int data) { return P_INT; };
Payload_type getPayloadTypeFromData(long data) { return P_LONG32; };
Payload_type getPayloadTypeFromData(unsigned long data) { return P_ULONG32; };
Payload_type getPayloadTypeFromData(float data) { return P_FLOAT32; };
Payload_type getPayloadTypeFromData(bool data) { return P_BOOL; };

MessageHelper *prepareHeartChildrenBeatMessage(Sensor *fromSensor,
                                               MessageHelper *message) {
  //use my id for the answer so the receiver will know who is the sender
  message->setSensorID(fromSensor->getSensorID());
  //obviusly we are presenting us to the other node
  message->setCommand(C_PRESENTATION_CHILDREN);
  message->setSensorAddress(fromSensor->getSensorAddress());
  message->setSensorType(fromSensor->getSensorType());
  message->setSystemMessageType(I_HEARTBEAT);
  message->setSensorInformationType(fromSensor->getSensorInformationType());
  message->setPayloadType(P_HEARTBEAT);
  return message;
}
MessageHelper *prepareHeartParentBeatMessage(Sensor *fromSensor,
                                             MessageHelper *message) {
  //use my id for the answer so the receiver will know who is the sender
  message->setSensorID(fromSensor->getSensorID());
  //obviusly we are presenting us to the other node
  message->setCommand(C_PRESENTATION_PARENT);
  message->setSensorAddress(fromSensor->getSensorAddress());
  message->setSensorType(fromSensor->getSensorType());
  message->setSystemMessageType(I_HEARTBEAT);
  message->setSensorInformationType(fromSensor->getSensorInformationType());
  message->setPayloadType(P_HEARTBEAT);
  return message;
}

MessageHelper *prepareHeartBeatAckMessage(Sensor *fromSensor,
                                          MessageHelper *message) {
  message->setSensorID(fromSensor->getSensorID());
  message->setSensorAddress(fromSensor->getSensorAddress());
  message->setCommand(C_ACK);
  message->setSensorType(fromSensor->getSensorType());
  message->setSystemMessageType(I_HEARTBEAT);
  message->setSensorInformationType(fromSensor->getSensorInformationType());
  message->setPayloadType(P_HEARTBEAT);
  return message;
}

MessageHelper *prepareAckMessage(Sensor  &fromSensor,
                                 MessageHelper *message) {
  //use my id for the answer so the receiver will know who is the sender
  message->setSensorID(fromSensor.getSensorID());
  message->setSensorAddress(fromSensor.getSensorAddress());
  //we are acknowledging last message
  message->setCommand(C_ACK);
  // my type
  message->setSensorType(fromSensor.getSensorType());
  message->setSystemMessageType(I_ACK);
  // we are acknowledging the information of the incoming message
  message->setSensorInformationType(fromSensor.getSensorInformationType());
  message->setPayloadType(P_ACK);
}




////****** SensorMessageHelper ******

void SensorMessageHelper::begin(RF24Network &network) {
  network_ = &network;
}
uint8_t SensorMessageHelper::receiveMessage(MessageHelper * message) {
  bool ret = false;
    ret =
        network_->read(lastHeader, &message->internalMessage_, sizeof(Message)) != 0 ;
    #if defined(SERIAL_DEBUG_MESSAGE_HELPER)
    Serial.println(lastHeader.toString());
    Serial.println(message_.toString());
    #endif

  return ret;
}
uint8_t SensorMessageHelper::updateNetwork() {
  return network_->update();
}
bool SensorMessageHelper::networkAvailable() {
  return network_->available();
}

bool SensorMessageHelper::sendMessage(Sensor *fromSensor,
                                      Sensor *toSensor,
                                      MessageHelper const *message) {
  RF24NetworkHeader header(toSensor->getSensorAddress(), MESSAGE_HEADER_TYPE);
  header.from_node = fromSensor->getSensorAddress();
  lastHeader = header;
  #if defined(SERIAL_DEBUG)
  Serial.println(F("Sending message with header:"));
  Serial.print(lastHeader.toString());
  delay(1000);
  #endif
  return network_->write(lastHeader, &message->internalMessage_, sizeof(Message));
}


////****** SensorManager ******

bool SensorManager::addSensor(Sensor *newsensor) {
  if(sensorOnThisArduino[newsensor->getSensorID()] == NULL) {
    sensorOnThisArduino[newsensor->getSensorID()] = newsensor;
    return true;
  }
  return false;
}
bool SensorManager::sensorsHaveToSend() {
  bool ret = false;
  for(int i = 0; i < MAX_SENSOR_PER_ARDUINO; i++){
    ret = ret || sensorOnThisArduino[i]->hasToSend();
  }
  return ret;
}
bool SensorManager::update() {
  bool ret = false;
  for(int i = 0; i < MAX_SENSOR_PER_ARDUINO; i++){
    ret = ret || sensorOnThisArduino[i]->update();
  }
  return ret;
}

MessageHelper *SensorManager::getMessage() {
  return &message_;
}



Sensor *SensorManager::getSensors(uint8_t index) {
  return sensorOnThisArduino[index];
}

uint8_t SensorManager::getSensorsNum() {
  return MAX_SENSOR_PER_ARDUINO;
}

////****** Sensor ******

Sensor::Sensor(uint16_t const address,
               uint8_t const sensor_id,
               Sensor_type const sensorType,
               Sensor_information_type const sensorInformationType)
    : address_(address),
      sensor_id_(sensor_id),
      sensorType_(sensorType),
      sensorInformationType_(sensorInformationType)
      {
        informationAvailable_ = false;
        messageSent_ = false;
      }

Sensor_type Sensor::getSensorType() {
  return sensorType_;
}
uint8_t Sensor::getSensorID() {
  return sensor_id_;
}
void Sensor::messageSent() {
  messageSent_ = true;
}
bool Sensor::isMessageSent() {
  return messageSent_;
}
void Sensor::setInformationAvaiable() {
  informationAvailable_ = true;
}
void Sensor::setInformationUnAvaiable() {
  informationAvailable_ = true;
}
bool Sensor::isInformationAvaiable() {
  return informationAvailable_;
}

void Sensor::setSensorID(uint8_t id) {
  sensor_id_ = id;
}

uint16_t Sensor::getSensorAddress() const {
  return address_;
}
Sensor_information_type Sensor::getSensorInformationType() const {
  return sensorInformationType_;
}

void Sensor::setParent(Sensor *parent) {
  parent_ = parent;
}

Sensor *Sensor::getParent() {
  return parent_;
}

void Sensor::setMessageHasToSend() {
  messageSent_ = false;
}

void Sensor::evaluateMessage(MessageHelper *message) {

  switch (getMessage()->getCommand()) {
    case C_PRESENTATION_CHILDREN:
      sensorChildrenPresentationReceived(message); //register the sensor if necessary
      break;
    case C_PRESENTATION_PARENT:
      sensorParentPresentationReceived(message);
      break;
    case C_SET:
      setSensorInfo(message); //based on sensor information type and ID set the data
      break;
    case C_REQ:
      getSensorInfo(message); //based on sensor information type and ID get the data
      break;
    case C_SYSTEM:
      systemFuncion(message); // set the sensor type, delay time, or other
      break;
    case C_STREAM:
      incomingDataIsAStream(message);// some kind or streaming data
      break;
    default:
      return;
  }
}




Sensor *Sensor::getSensorObj( MessageHelper *message) {

  switch(message->getSensorType()) {
    case S_LIGHT: return new RelaySensor(message->getSensorAddress(),
                                         message->getSensorID());
    case S_MOTION: return new MotionSensor(message->getSensorAddress(),
                                           message->getSensorID());
    case S_LIGHT_LEVEL: return new LightLevelSensor(message->getSensorAddress(),
                                                message->getSensorID());
    default:
      return NULL;
  }
}

//---------------------------------------------

RelaySensor::RelaySensor(uint16_t const address,
                                   uint8_t const id) :
    Sensor(address, id, S_LIGHT, V_LIGHT)
{ }


bool RelaySensor::prepareDataMessageToSend(MessageHelper *message) {
}
bool RelaySensor::update() {
}
bool RelaySensor::hasToSend() {
}

bool RelaySensor::sensorParentPresentationReceived(MessageHelper *message) {
}



bool RelaySensor::sensorChildrenPresentationReceived(MessageHelper *message) {
  if(sensormap[getMessage()->getSensorID()] == NULL) {
    sensormap[getMessage()->getSensorID()] = Sensor::getSensorObj(message);
  }
  return true;
}
bool RelaySensor::setSensorInfo(MessageHelper *message) {

  return false;
}
bool RelaySensor::getSensorInfo(MessageHelper *message) {
  return false;
}
bool RelaySensor::systemFuncion(MessageHelper *message) {
  return false;
}
bool RelaySensor::incomingDataIsAStream(MessageHelper *message) {
  return false;
}



//---------------------------------------------

MotionSensor::MotionSensor(uint16_t const address,
                           uint8_t const id) :
                            Sensor(address,
                                   id,
                                   S_MOTION,
                                   V_MOTION) {
}

bool MotionSensor::prepareDataMessageToSend(MessageHelper *message) {
}
bool MotionSensor::update() {
}
bool MotionSensor::hasToSend() {
}
bool MotionSensor::sensorParentPresentationReceived(MessageHelper *message) {
}

bool MotionSensor::sensorChildrenPresentationReceived(MessageHelper *message) {
  return false;
}
bool MotionSensor::setSensorInfo(MessageHelper *message) {
  return false;
}
bool MotionSensor::getSensorInfo(MessageHelper *message) {
  return false;
}
bool MotionSensor::systemFuncion(MessageHelper *message) {
  return false;
}
bool MotionSensor::incomingDataIsAStream(MessageHelper *message) {
  return false;
}

//---------------------------------------------

LightLevelSensor::LightLevelSensor(uint16_t const address,
                         uint8_t const id):
    Sensor(address,
           id,
           S_LIGHT_LEVEL,
           V_LIGHT_LEVEL) { }



bool LightLevelSensor::prepareDataMessageToSend(MessageHelper *message) {
}
bool LightLevelSensor::update() {
}
bool LightLevelSensor::hasToSend() {
}
bool LightLevelSensor::sensorParentPresentationReceived(MessageHelper *message) {
}

bool LightLevelSensor::sensorChildrenPresentationReceived(MessageHelper *message) {
  return false;
}
bool LightLevelSensor::setSensorInfo(MessageHelper *message) {
  return false;
}
bool LightLevelSensor::getSensorInfo(MessageHelper *message) {
  return false;
}
bool LightLevelSensor::systemFuncion(MessageHelper *message) {
  return false;
}
bool LightLevelSensor::incomingDataIsAStream(MessageHelper *message) {
  return false;
}

//---------------------------------------------





