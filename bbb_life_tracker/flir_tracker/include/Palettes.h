#ifndef PALETTES_H
#define PALETTES_H

#include <stdint.h>

extern const uint8_t colormap_rainbow[];
extern const uint8_t colormap_grayscale[];
extern const uint8_t colormap_ironblack[];
extern const uint8_t colormap_blackHot[];
extern const uint8_t colormap_arctic[];
extern const uint8_t colormap_blueRed[];
extern const uint8_t colormap_coldest[];
extern const uint8_t colormap_contrast[];
extern const uint8_t colormap_doubleRainbow[];
extern const uint8_t colormap_grayRed[];
extern const uint8_t colormap_glowBow[];
extern const uint8_t colormap_hottest[];
extern const uint8_t colormap_lava[];
extern const uint8_t colormap_medical[];
extern const uint8_t colormap_wheel2[];
extern const uint8_t colormap_whiteHot[];

#define PALETTES_COUNT 16
extern const uint8_t* palettes[PALETTES_COUNT];
 

#endif
