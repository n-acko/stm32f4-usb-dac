/*
 * logic.h
 *
 *  Created on: Dec 3, 2024
 *      Author: Aleksandar
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

void getInfo(uint32_t* buffer, uint16_t bufferSize, uint8_t* activeHalf);

void recieveData(uint32_t *data, uint16_t size);

void updateBuffer();

void muteOutput(uint8_t val);

void getMute(uint8_t* mute);

void setFreq(uint32_t freq, uint8_t bitDepth);

void getFreqPoint(uint32_t* freqP, uint8_t* updateFreqP, uint8_t* bitDepthP);

void reset();

#endif /* INC_LOGIC_H_ */
