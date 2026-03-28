#include "LoadTexture.h"
#include "../core/CommandBuffer.h"
#include "../core/errors.h"

#include <spdlog/spdlog.h>

#define STB_IMAGE_IMPLEMENTATION
#include "../third-party/stb_image.h"

namespace candlewick {

Texture loadTextureFromFile(const Device &device, const char *path) {
  int width, height, channels;
  // Force 4-channel RGBA output
  stbi_uc *pixels = stbi_load(path, &width, &height, &channels, 4);
  if (!pixels) {
    terminate_with_message("Failed to load texture '{}': {}", path,
                           stbi_failure_reason());
  }

  const Uint32 imageSize = Uint32(width) * Uint32(height) * 4;

  SDL_GPUTextureCreateInfo texInfo{
      .type = SDL_GPU_TEXTURETYPE_2D,
      .format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
      .usage = SDL_GPU_TEXTUREUSAGE_SAMPLER,
      .width = Uint32(width),
      .height = Uint32(height),
      .layer_count_or_depth = 1,
      .num_levels = 1,
      .sample_count = SDL_GPU_SAMPLECOUNT_1,
      .props = 0,
  };
  Texture texture(device, texInfo, path);

  // Upload via transfer buffer
  SDL_GPUTransferBufferCreateInfo transferInfo{
      .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
      .size = imageSize,
      .props = 0,
  };
  SDL_GPUTransferBuffer *transferBuf =
      SDL_CreateGPUTransferBuffer(device, &transferInfo);

  void *map = SDL_MapGPUTransferBuffer(device, transferBuf, false);
  SDL_memcpy(map, pixels, imageSize);
  SDL_UnmapGPUTransferBuffer(device, transferBuf);
  stbi_image_free(pixels);

  CommandBuffer cmdBuf{device};
  SDL_GPUCopyPass *copyPass = SDL_BeginGPUCopyPass(cmdBuf);

  SDL_GPUTextureTransferInfo src{
      .transfer_buffer = transferBuf,
      .offset = 0,
  };
  SDL_GPUTextureRegion dst{
      .texture = texture,
      .w = Uint32(width),
      .h = Uint32(height),
      .d = 1,
  };
  SDL_UploadToGPUTexture(copyPass, &src, &dst, false);
  SDL_EndGPUCopyPass(copyPass);

  if (!cmdBuf.submit()) {
    spdlog::error("Failed to submit texture upload command buffer: {}",
                  SDL_GetError());
  }

  SDL_ReleaseGPUTransferBuffer(device, transferBuf);

  spdlog::info("Loaded texture '{}' ({}x{}, {} channels)", path, width, height,
               channels);
  return texture;
}

Texture createWhiteTexture(const Device &device) {
  SDL_GPUTextureCreateInfo texInfo{
      .type = SDL_GPU_TEXTURETYPE_2D,
      .format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
      .usage = SDL_GPU_TEXTUREUSAGE_SAMPLER,
      .width = 1,
      .height = 1,
      .layer_count_or_depth = 1,
      .num_levels = 1,
      .sample_count = SDL_GPU_SAMPLECOUNT_1,
      .props = 0,
  };
  Texture texture(device, texInfo, "White fallback 1x1");

  const Uint8 white[4] = {255, 255, 255, 255};

  SDL_GPUTransferBufferCreateInfo transferInfo{
      .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
      .size = 4,
      .props = 0,
  };
  SDL_GPUTransferBuffer *transferBuf =
      SDL_CreateGPUTransferBuffer(device, &transferInfo);

  void *map = SDL_MapGPUTransferBuffer(device, transferBuf, false);
  SDL_memcpy(map, white, 4);
  SDL_UnmapGPUTransferBuffer(device, transferBuf);

  CommandBuffer cmdBuf{device};
  SDL_GPUCopyPass *copyPass = SDL_BeginGPUCopyPass(cmdBuf);

  SDL_GPUTextureTransferInfo src{
      .transfer_buffer = transferBuf,
      .offset = 0,
  };
  SDL_GPUTextureRegion dst{
      .texture = texture,
      .w = 1,
      .h = 1,
      .d = 1,
  };
  SDL_UploadToGPUTexture(copyPass, &src, &dst, false);
  SDL_EndGPUCopyPass(copyPass);

  if (!cmdBuf.submit()) {
    spdlog::error("Failed to submit white texture upload: {}", SDL_GetError());
  }

  SDL_ReleaseGPUTransferBuffer(device, transferBuf);
  return texture;
}

SDL_GPUSampler *createMaterialSampler(const Device &device) {
  SDL_GPUSamplerCreateInfo samplerInfo{
      .min_filter = SDL_GPU_FILTER_LINEAR,
      .mag_filter = SDL_GPU_FILTER_LINEAR,
      .mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR,
      .address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
      .address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
      .address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
  };
  SDL_GPUSampler *sampler = SDL_CreateGPUSampler(device, &samplerInfo);
  if (!sampler) {
    terminate_with_message("Failed to create material sampler: {}",
                           SDL_GetError());
  }
  return sampler;
}

} // namespace candlewick
