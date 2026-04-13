#pragma once

#include "../core/Texture.h"
#include "../core/Device.h"

#include <SDL3/SDL_gpu.h>

namespace candlewick {

/// \brief Load a texture from an image file (JPEG, PNG, BMP, TGA, HDR, etc.).
///
/// Uses stb_image internally to decode the image data, then uploads it to the
/// GPU as an RGBA8 texture.
[[nodiscard]] Texture loadTextureFromFile(const Device &device,
                                          const char *path);

/// \brief Create a 1x1 white RGBA texture, useful as a fallback when no
/// texture is available.
[[nodiscard]] Texture createWhiteTexture(const Device &device);

/// \brief Create a sampler suitable for material textures (linear filtering,
/// repeat wrap mode).
[[nodiscard]] SDL_GPUSampler *createMaterialSampler(const Device &device);

} // namespace candlewick
