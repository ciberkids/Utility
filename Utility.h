//
// Created by Matteo Favaro on 03/08/16.
//

#ifndef ARDUINO_MCWW_UTILITY_H
#define ARDUINO_MCWW_UTILITY_H

#include <Arduino.h>

#include "RF24Network.h"
#include "Message.h"
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


class ReceiverUtility {

};


#endif //ARDUINO_MCWW_UTILITY_H
