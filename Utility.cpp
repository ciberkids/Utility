//
// Created by Matteo Favaro on 03/08/16.
//

#include "Utility.h"

#include "Message.h"


ReceiverUtility::ReceiverUtility(RF24Network &network, MessageHelper &message) :
    network_(network),
    message_(message){}

void ReceiverUtility::receiveMessage() {
  network_.read(header, &message_.internalMessage_, sizeof(Message));
  #if defined(SERIAL_DEBUG)
    Serial.println(header.toString());
    Serial.println(message_.toString());
  #endif
}
