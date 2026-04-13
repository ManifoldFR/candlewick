#include "candlewick/core/Shader.h"
#include "candlewick/core/Device.h"
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_log.h>

using namespace candlewick;

int main() {

  // Initialize SDL
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Failed to init SDL: %s", SDL_GetError());
    return 1;
  }
  const int numGpus = SDL_GetNumGPUDrivers();
  SDL_Log("Found %d GPU drivers.", numGpus);

  Device device{auto_detect_shader_format_subset(), true};
  SDL_Window *window = SDL_CreateWindow(__FILE_NAME__, 1280, 480, 0);
  if (!window) {
    SDL_Log("Failed to create window! %s", SDL_GetError());
    return 1;
  }

  if (!SDL_ClaimWindowForGPUDevice(device, window)) {
    SDL_Log("Failed to claim window for GPU device");
    return 1;
  }

  auto vertexShader = Shader::fromMetadata(device, "BasicTriangle.vert");
  auto fragmentShader = Shader::fromMetadata(device, "SolidColor.frag");

  SDL_GPUColorTargetDescription coldescs{
      .format = SDL_GetGPUSwapchainTextureFormat(device, window)};
  SDL_GPUGraphicsPipelineCreateInfo pipeInfo;
  SDL_zero(pipeInfo);
  pipeInfo = {
      .vertex_shader = vertexShader,
      .fragment_shader = fragmentShader,
      .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
      .target_info =
          {
              .color_target_descriptions = &coldescs,
              .num_color_targets = 1,
          },
  };
  pipeInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
  SDL_GPUGraphicsPipeline *pipelineFill =
      SDL_CreateGPUGraphicsPipeline(device, &pipeInfo);

  pipeInfo.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_LINE;
  SDL_GPUGraphicsPipeline *pipelineLine =
      SDL_CreateGPUGraphicsPipeline(device, &pipeInfo);
  SDL_Log("Built pipelines");

  bool mustQuit = false;
  Uint32 frame = 0;

  while (!mustQuit) {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_EVENT_QUIT) {
        mustQuit = true;
        break;
      }
    }
    // get command buffer
    SDL_GPUCommandBuffer *cmdbuf = SDL_AcquireGPUCommandBuffer(device);
    if (!cmdbuf) {
      SDL_Log("Failed to acquire command buffer: %s", SDL_GetError());
      return 1;
    }

    SDL_GPUTexture *swapchain;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(cmdbuf, window, &swapchain, NULL,
                                               NULL)) {
      SDL_SubmitGPUCommandBuffer(cmdbuf);
      break;
    } else {
      SDL_Log("Acquired command buffer and swapchain (frame %3d)", frame);
      SDL_GPUColorTargetInfo ctinfo;
      SDL_zero(ctinfo);
      ctinfo.texture = swapchain;
      ctinfo.clear_color = SDL_FColor{0.f, 0.f, 0.f, 1.f};
      ctinfo.load_op = SDL_GPU_LOADOP_CLEAR;
      ctinfo.store_op = SDL_GPU_STOREOP_STORE;
      SDL_GPURenderPass *renderPass =
          SDL_BeginGPURenderPass(cmdbuf, &ctinfo, 1, NULL);
      SDL_BindGPUGraphicsPipeline(renderPass, pipelineFill);

      SDL_DrawGPUPrimitives(renderPass, 3, 1, 0, 0);
      SDL_EndGPURenderPass(renderPass);
      frame++;
    }
    SDL_SubmitGPUCommandBuffer(cmdbuf);
  }
  SDL_Log("Quitting...");
  SDL_ReleaseGPUGraphicsPipeline(device, pipelineFill);
  SDL_ReleaseGPUGraphicsPipeline(device, pipelineLine);
  vertexShader.release();
  fragmentShader.release();

  SDL_ReleaseWindowFromGPUDevice(device, window);
  SDL_DestroyWindow(window);
  device.destroy();
  SDL_Quit();

  return 0;
}
