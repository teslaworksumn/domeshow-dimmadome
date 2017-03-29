/* 
 * File: domeshow_lib.h   
 * Author: Ian Smith
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DOME_LIB
#define	DOME_LIB

#include <xc.h> // include processor files - each processor file is guarded.  

void writeColor(int r, int g, int b) {
    CCPR4L = 255 - r;
    CCPR5L = 255 - g;
    CCPR6L = 255 - b;
    CCPR7L = 255 - r;
    CCPR8L = 255 - g;
    CCPR9L = 255 - b;
}

uint32_t packColor(int r, int g, int b) {
	return (((long) r << 16) | ((long) g << 8) | ((long) b));
}

int getR(uint32_t x) {
	return (int) (x >> 16);
}

int getG(uint32_t x) {
	return (int) (x >> 8);
}

int getB(uint32_t x) {
	return (int) (x);
}

void writePackedColor(uint32_t x) {
	writeColor(getR(x), getG(x), getB(x));
}

uint32_t Wheel(int WheelPos) {
	WheelPos = 255 - WheelPos;
	if(WheelPos < 85) {
		return packColor(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if(WheelPos < 170) {
		WheelPos -= 85;
		return packColor(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return packColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}



#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

