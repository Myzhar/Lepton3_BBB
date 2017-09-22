#ifndef PALETTES_H
#define PALETTES_H

#include <stdint.h>

#define PALETTES_COUNT 5
extern const uint8_t* palettes[PALETTES_COUNT];

#define LUT_SIZE_8  256   // 8 BIT

void createColorMaps();
extern uint8_t colormap_byr[LUT_SIZE_8*3];
extern uint8_t colormap_bgyr[LUT_SIZE_8*3];
extern uint8_t colormap_whitehot[LUT_SIZE_8*3];
extern uint8_t colormap_blackhot[LUT_SIZE_8*3];
extern uint8_t colormap_whitehotRGB[LUT_SIZE_8*3];

#endif
