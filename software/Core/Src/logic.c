/*
 * logic.c
 *
 *  Created on: Dec 3, 2024
 *      Author: Aleksandar
 */

#include "logic.h"
#define MULT (8U)

uint32_t* buffPoint;
uint8_t* localActive;
uint16_t localSize;
uint16_t currentSize;
uint32_t *tempBuff;
uint16_t noNewPacket = 0;
uint8_t muteSW = 0;
uint8_t muteHW = 0;
uint8_t *localMute;
uint8_t currentSegment = 0;
uint8_t usbSegment = 0;
int16_t lastDistance = -1, distance = 0;
uint32_t *localFreqP;
uint8_t *localUpdateFreqP;
uint8_t *localBitDepthP;

void getInfo(uint32_t* buffer, uint16_t bufferSize, uint8_t* activeHalf){
	buffPoint = buffer;
	localSize = bufferSize / 2;
	tempBuff = malloc( localSize * sizeof(uint32_t) * MULT);
	memset(tempBuff, 0, (localSize * sizeof(uint32_t) * MULT));
	localActive = activeHalf;
	currentSize = 0;

}

void recieveData(uint32_t *data, uint16_t size){
	noNewPacket = 0;
	if(muteSW){
		muteOutput(2);
	}

	uint32_t *localtemp = data;

	uint16_t inc = 0;
	while(size > 0){
		tempBuff[currentSize++ % (localSize * MULT)] = localtemp[inc++];
		size--;
	}
	if(currentSize >= localSize * MULT){
		currentSize = currentSize % (localSize * MULT);
	}

	if(lastDistance == -1){
		usbSegment = currentSize / localSize;
		if(currentSegment > usbSegment){
			distance = currentSegment - usbSegment;
			lastDistance = distance;
		}
		else{
			distance = currentSegment + MULT - usbSegment;
			lastDistance = distance;
		}
	}
	else{
		usbSegment = currentSize / localSize;
		if(currentSegment > usbSegment){
			lastDistance = distance;
			distance = currentSegment - usbSegment;
		}
		else{
			lastDistance = distance;
			distance = currentSegment + MULT - usbSegment;
		}
		int16_t change = lastDistance - distance;
		if(change < 0 && distance > (MULT / 2)){
			tempBuff[currentSize] = tempBuff[currentSize - 2];
			tempBuff[currentSize + 1] = tempBuff[currentSize - 1];
			currentSize += 2;
		}
		else if(change > 0 && distance < (MULT / 2)){
			currentSize -=2;
		}
	}

}

void updateBuffer(){
	if(*localMute){
		if(*localActive){
			for(int i = 0; i < localSize; i++){
				buffPoint[i] = 0;
			}
		}
		else{
			for(int i = 0; i < localSize; i++){
				buffPoint[i + localSize] = 0;
			}
		}
		return;
	}

	if(*localActive){
		for(int i = 0; i < localSize; i++){
			buffPoint[i] = tempBuff[i + currentSegment * localSize];
		}
	}
	else{
		for(int i = 0; i < localSize; i++){
			buffPoint[i + localSize] = tempBuff[i + currentSegment * localSize];
		}
	}
	currentSegment = (currentSegment + 1) % MULT;

	noNewPacket++;
	if(noNewPacket > 2 && !muteSW){
		muteOutput(3);
		reset();
		noNewPacket = 3;
	}
}

void muteOutput(uint8_t val){
	switch (val) {
		case 0:
			muteHW = 0;
			break;
		case 1:
			muteHW = 1;
			break;
		case 2:
			muteSW = 0;
			break;
		case 3:
			muteSW = 1;
			break;
	}
	*localMute = (muteSW || muteHW);
}

void getMute(uint8_t* mute){
	localMute = mute;
}

void setFreq(uint32_t freq, uint8_t bitDepth){
	if(freq != *localFreqP){
		*localFreqP = freq;
		*localUpdateFreqP = 1;
	}
	if(bitDepth != *localBitDepthP){
		*localBitDepthP = bitDepth;
		*localUpdateFreqP = 1;
	}
}

void getFreqPoint(uint32_t* freqP, uint8_t* updateFreqP, uint8_t* bitDepthP){
	localFreqP = freqP;
	localUpdateFreqP = updateFreqP;
	localBitDepthP = bitDepthP;
}

void reset(){
	currentSegment = 0U;
	currentSize = (localSize * MULT) / 2U;
	memset(tempBuff, 0, localSize * sizeof(uint32_t) * MULT);
}
