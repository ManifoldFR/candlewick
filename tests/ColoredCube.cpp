#include "Common.h"

#include "candlewick/core/Shader.h"
#include "candlewick/core/Mesh.h"
#include "candlewick/core/MeshLayout.h"
#include "candlewick/core/Camera.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>

using namespace candlewick;

struct Vertex {
  Float3 pos;
  Float4 col;
};

const Float4 red = {1.f, 0., 0., 1.};
const Float4 green = {0.f, 1., 0., 1.};
const Float4 blue = {0.f, 0., 1., 1.};

// clang-format off
const Vertex vertexData[] = {
  // top
  // f1
  {{0., 0., 1.}, red},
  {{1., 0., 1.}, (blue + red) / 2},
  {{1., 1., 1.}, blue},
  // f2
  {{1., 1., 1.}, blue},
  {{0., 1., 1.}, green},
  {{0., 0., 1.}, red},
  // -y
  // f3
  {{0., 0., 1.}, red},
  {{0., 0., 0.}, blue},
  {{1., 0., 0.}, red},
  // f4
  {{1., 0., 0.}, red},
  {{1., 0., 1.}, (blue + red) / 2},
  {{0., 0., 1.}, red},
  // -x
  // f5
  {{0., 0., 1.}, red},
  {{0., 1., 1.}, red},
  {{0., 1., 0.}, red},
  // f6
  {{0., 1., 0.}, red},
  {{0., 0., 0.}, red},
  {{0., 0., 1.}, red},
  // bottom
  // f7
  {{1., 1., 0.}, blue},
  {{1., 0., 0.}, blue},
  {{0., 0., 0.}, blue},
  // f8
  {{0., 0., 0.}, blue},
  {{0., 1., 0.}, blue},
  {{1., 1., 0.}, blue},
  // +x
  // f9
  {{1., 0., 0.}, green},
  {{1., 1., 0.}, green},
  {{1., 1., 1.}, green},
  // f10
  {{1., 1., 1.}, green},
  {{1., 0., 1.}, green},
  {{1., 0., 0.}, green},
  // +y
  // f11
  {{0., 1., 1.}, blue},
  {{1., 1., 1.}, (red+blue)/2},
  {{1., 1., 0.}, blue},
  // f12
  {{1., 1., 0.}, blue},
  {{0., 1., 0.}, (2 * green+blue)/3},
  {{0., 1., 1.}, blue},
};
// clang-format on

int main() {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    return 1;
  }
  Device device{auto_detect_shader_format_subset()};

  const Uint32 wWidth = 1280;
  const Uint32 wHeight = 720;
  const float aspectRatio = float(wWidth) / wHeight;
  SDL_Window *window = SDL_CreateWindow(__FILE_NAME__, wWidth, wHeight, 0);
  if (!SDL_ClaimWindowForGPUDevice(device, window)) {
    SDL_Log("Error: %s", SDL_GetError());
    return 1;
  }

  // Buffers

  MeshLayout mesh_layout;
  mesh_layout.addBinding(0, sizeof(Vertex))
      .addAttribute(VertexAttrib::Position, 0,
                    SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(Vertex, pos))
      .addAttribute(VertexAttrib::Color0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4,
                    offsetof(Vertex, col));

  SDL_GPUBufferCreateInfo buffer_desc{.usage = SDL_GPU_BUFFERUSAGE_VERTEX,
                                      .size = sizeof(vertexData),
                                      .props = 0};

  Mesh mesh{device, mesh_layout};
  auto buf_vertex = SDL_CreateGPUBuffer(device, &buffer_desc);
  if (!buf_vertex) {
    SDL_Log("Failed to create vertex buffer: %s", SDL_GetError());
    return 1;
  }
  SDL_SetGPUBufferName(device, buf_vertex, "vertex_buf");
  mesh.bindVertexBuffer(0, buf_vertex);

  SDL_GPUTransferBufferCreateInfo transfer_buffer_desc{
      .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
      .size = sizeof(vertexData),
      .props = 0,
  };
  SDL_GPUTransferBuffer *transfer_buffer =
      SDL_CreateGPUTransferBuffer(device, &transfer_buffer_desc);
  void *_map = SDL_MapGPUTransferBuffer(device, transfer_buffer, false);
  SDL_memcpy(_map, vertexData, sizeof(vertexData));
  SDL_UnmapGPUTransferBuffer(device, transfer_buffer);

  SDL_GPUCommandBuffer *cmdbuf;
  // Upload data to buffer
  {
    cmdbuf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass *copy_pass = SDL_BeginGPUCopyPass(cmdbuf);
    SDL_GPUTransferBufferLocation src_location{
        .transfer_buffer = transfer_buffer,
        .offset = 0,
    };

    SDL_GPUBufferRegion dst_location{
        .buffer = mesh.vertexBuffers[0],
        .offset = 0u,
        .size = sizeof(vertexData),
    };

    SDL_UploadToGPUBuffer(copy_pass, &src_location, &dst_location, false);
    SDL_EndGPUCopyPass(copy_pass);
    SDL_SubmitGPUCommandBuffer(cmdbuf);
  }
  SDL_ReleaseGPUTransferBuffer(device, transfer_buffer);

  // Shaders

  auto vertexShader = Shader::fromMetadata(device, "VertexColor.vert");
  auto fragmentShader = Shader::fromMetadata(device, "VertexColor.frag");

  // Pipeline

  SDL_GPUColorTargetDescription color_desc;
  SDL_zero(color_desc);
  color_desc.format = SDL_GetGPUSwapchainTextureFormat(device, window);

  SDL_GPUTextureFormat depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D16_UNORM;
  assert(int(depth_stencil_format) >= 0);

  SDL_GPUGraphicsPipelineCreateInfo pipeline_desc{
      .vertex_shader = vertexShader,
      .fragment_shader = fragmentShader,
      .vertex_input_state = mesh_layout.toVertexInputState(),
      .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
      .rasterizer_state =
          {
              .fill_mode = SDL_GPU_FILLMODE_FILL,
              .cull_mode = SDL_GPU_CULLMODE_NONE,
              .front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE,
          },
      .depth_stencil_state{
          .compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL,
          .enable_depth_test = true,
          .enable_depth_write = true,
      },
      .target_info =
          {
              .color_target_descriptions = &color_desc,
              .num_color_targets = 1,
              .depth_stencil_format = depth_stencil_format,
              .has_depth_stencil_target = true,
          },
      .props = 0};
  SDL_GPUGraphicsPipeline *pipeline =
      SDL_CreateGPUGraphicsPipeline(device, &pipeline_desc);
  if (!pipeline) {
    SDL_Log("Failed to create pipeline: %s", SDL_GetError());
    return 1;
  }

  SDL_Log("Releasing shaders...");
  vertexShader.release();
  fragmentShader.release();

  // Depth texture
  SDL_GPUTexture *depthTexture = createDepthTexture(
      device, window, depth_stencil_format, SDL_GPU_SAMPLECOUNT_1);

  const Radf fov_rad = 45.0_degf;
  Mat4f perp = perspectiveFromFov(fov_rad, aspectRatio, 0.1f, 10.f);
  Mat4f modelMat = Mat4f::Identity();
  modelMat.col(3).head<3>() << -0.5f, -0.5f, -0.5f;
  Mat4f view;
  Mat4f projViewMat;

  Uint32 frame = 0;

  // Main loop
  for (Uint32 i = 0; i < 200; i++) {
    SDL_GPURenderPass *render_pass;
    SDL_GPUBufferBinding vertex_binding = mesh.getVertexBinding(0);
    cmdbuf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUTexture *swapchain;
    const Float3 center{0., 0., 0.};
    Float3 eye{0., 0., 0.};
    // start at phi -> eye.x = 2.5, eye.y = 0.5
    const float phi = 0.05f * static_cast<float>(frame);
    eye.x() += 2.0f * std::cos(phi);
    eye.y() += 2.0f * std::sin(phi);
    view = lookAt(eye, center, {0., 0., 1.});
    projViewMat = perp * view * modelMat;

    if (SDL_AcquireGPUSwapchainTexture(cmdbuf, window, &swapchain, NULL,
                                       NULL)) {
      SDL_GPUColorTargetInfo ctinfo{.texture = swapchain,
                                    .clear_color = SDL_FColor{},
                                    .load_op = SDL_GPU_LOADOP_CLEAR,
                                    .store_op = SDL_GPU_STOREOP_STORE};
      SDL_GPUDepthStencilTargetInfo depth_target;
      SDL_zero(depth_target);
      depth_target.clear_depth = 1.0f;
      depth_target.load_op = SDL_GPU_LOADOP_CLEAR;
      depth_target.store_op = SDL_GPU_STOREOP_DONT_CARE;
      depth_target.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
      depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
      depth_target.texture = depthTexture;
      depth_target.cycle = true;
      render_pass = SDL_BeginGPURenderPass(cmdbuf, &ctinfo, 1, &depth_target);
      SDL_BindGPUGraphicsPipeline(render_pass, pipeline);
      SDL_BindGPUVertexBuffers(render_pass, 0, &vertex_binding, 1);
      constexpr auto vertexCount = std::size(vertexData);
      SDL_PushGPUVertexUniformData(cmdbuf, 0, projViewMat.data(),
                                   sizeof(projViewMat));
      SDL_DrawGPUPrimitives(render_pass, vertexCount, 1, 0, 0);
      SDL_EndGPURenderPass(render_pass);
    } else {
      SDL_Log("Failed to acquire swapchain: %s", SDL_GetError());
      break;
    }
    SDL_SubmitGPUCommandBuffer(cmdbuf);
    SDL_Log("Frame [%3d] phi = %.2f", frame, phi);
    frame++;
  }

  SDL_ReleaseGPUGraphicsPipeline(device, pipeline);

  SDL_ReleaseGPUTexture(device, depthTexture);

  mesh.release();

  SDL_ReleaseWindowFromGPUDevice(device, window);
  SDL_DestroyWindow(window);
  device.destroy();
  SDL_Quit();
}
