#include "PixelFormatConversion.h"

namespace candlewick {

void bgraToRgbaConvert_impl(const Uint32 *bgraPixels, Uint32 *rgbaPixels,
                            Uint32 pixelCount) {
  // define appropriate masks for BGRA format
  Uint32 red_mask = 0x00FF0000;
  Uint32 green_mask = 0x0000FF00;
  Uint32 blue_mask = 0x000000FF;
  Uint32 alpha_mask = 0xFF000000;

  for (Uint32 i = 0; i < pixelCount; ++i) {
    Uint32 pixel = bgraPixels[i];
    rgbaPixels[i] = ((pixel & red_mask) >> 16) |  // Extract Red
                    ((pixel & green_mask)) |      // Keep Green
                    ((pixel & blue_mask) << 16) | // Extract Blue
                    (pixel & alpha_mask);         // Keep Alpha
  }
}

void bgraToRgbaConvert(const Uint32 *bgraPixels, Uint32 *rgbaPixels,
                       Uint32 pixelCount) {
  bgraToRgbaConvert_impl(bgraPixels, rgbaPixels, pixelCount);
}

} // namespace candlewick
