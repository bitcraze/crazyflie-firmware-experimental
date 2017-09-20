#ifndef __GLOW_H__
#define __GLOW_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * Set new power of glow deck LED
 * @PARAM newPower Value 0-255 to set intensity
 */
void glowOn(uint8_t newPower);

/**
 * Turn off glow deck LED
 */
void glowOff();

/**
 * Set new power of glow deck LED which will be faded to
 * @PARAM newPower Value 0-255 to set end intensity
 */
void glowFade(uint8_t newPower);

/**
 * Set new STROBE frequenzy and turn on functionality
 * @PARAM newPower Value 0-255 to set frequency in Deci-Hz
 */
void glowStrobe(uint8_t newFreqDeciHz);

#endif //__GLOW_H__
