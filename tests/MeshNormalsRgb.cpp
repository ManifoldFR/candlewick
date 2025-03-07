#include "Common.h"

#include "candlewick/core/CommandBuffer.h"
#include "candlewick/core/Mesh.h"
#include "candlewick/core/Shader.h"
#include "candlewick/utils/MeshData.h"
#include "candlewick/utils/LoadMesh.h"
#include "candlewick/core/CameraControls.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_filesystem.h>

#include <Eigen/Geometry>

using namespace candlewick;

// const char *meshFilename = "assets/meshes/mammoth.obj";
// const char *meshFilename = "assets/meshes/Stanford_Bunny.stl";
// const char *meshFilename = "assets/meshes/cube.obj";
const char *meshFilename = "assets/meshes/teapot.obj";

const Uint32 wWidth = 1280;
const Uint32 wHeight = 720;
const float aspectRatio = float(wWidth) / wHeight;
static Camera g_camera;

Context ctx;

struct alignas(16) TransformUniformData {
  GpuMat4 mvp;
  GpuMat3 normalMatrix;
};

int main() {
  if (!initExample(ctx, wWidth, wHeight)) {
    return 1;
  }
  Device &device = ctx.device;
  SDL_Window *window = ctx.window;

  const char *basePath = SDL_GetBasePath();
  char meshPath[256];
  SDL_snprintf(meshPath, 256, "%s../../../%s", basePath, meshFilename);

  std::vector<MeshData> meshDatas;
  auto ret = loadSceneMeshes(meshPath, meshDatas);
  if (ret < mesh_load_retc::OK) {
    SDL_Log("Failed to load mesh.");
    return 1;
  }
  SDL_Log("Loaded %zu MeshData objects.", meshDatas.size());
  for (size_t i = 0; i < meshDatas.size(); i++) {
    SDL_Log("Mesh %zu: %u vertices, %u indices", i, meshDatas[i].numVertices(),
            meshDatas[i].numIndices());
  }

  std::vector<Mesh> meshes;
  for (std::size_t j = 0; j < meshDatas.size(); j++) {
    Mesh mesh = createMesh(device, meshDatas[j]);
    uploadMeshToDevice(device, mesh, meshDatas[j]);
    meshes.push_back(std::move(mesh));
  }
  SDL_assert(meshDatas[0].numIndices() == meshes[0].indexCount);

  /** CREATE PIPELINE **/
  Shader vertexShader{device, "VertexNormal.vert", {.uniform_buffers = 1}};
  Shader fragmentShader{device, "VertexNormal.frag", {}};

  SDL_GPUTextureFormat depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D16_UNORM;
  SDL_GPUTexture *depthTexture = createDepthTexture(
      device, window, depth_stencil_format, SDL_GPU_SAMPLECOUNT_1);

  SDL_GPUColorTargetDescription colorTarget;
  SDL_zero(colorTarget);
  colorTarget.format = SDL_GetGPUSwapchainTextureFormat(device, window);
  SDL_GPUDepthStencilTargetInfo depthTarget;
  SDL_zero(depthTarget);
  depthTarget.clear_depth = 1.0;
  depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
  depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;
  depthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
  depthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
  depthTarget.texture = depthTexture;
  depthTarget.cycle = true;

  // create pipeline
  SDL_GPUGraphicsPipelineCreateInfo pipeline_desc{
      .vertex_shader = vertexShader,
      .fragment_shader = fragmentShader,
      .vertex_input_state = meshes[0].layout(),
      .primitive_type = meshDatas[0].primitiveType,
      .rasterizer_state{.fill_mode = SDL_GPU_FILLMODE_FILL,
                        .cull_mode = SDL_GPU_CULLMODE_NONE,
                        .front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE},
      .depth_stencil_state{.compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL,
                           .enable_depth_test = true,
                           .enable_depth_write = true},
      .target_info{.color_target_descriptions = &colorTarget,
                   .num_color_targets = 1,
                   .depth_stencil_format = depth_stencil_format,
                   .has_depth_stencil_target = true},
      .props = 0,
  };
  SDL_GPUGraphicsPipeline *pipeline =
      SDL_CreateGPUGraphicsPipeline(device, &pipeline_desc);
  if (pipeline == NULL) {
    SDL_Log("Failed to create pipeline: %s", SDL_GetError());
    return 1;
  }

  vertexShader.release();
  fragmentShader.release();

  SDL_GPUTexture *swapchain;

  Radf fov = 70.0_degf;
  Mat4f projectionMat = perspectiveFromFov(fov, aspectRatio, 0.1f, 40.0f);

  Uint32 frameNo = 0;
  bool quitRequested = false;
  Eigen::AngleAxisf rot{constants::Pi_2f, Eigen::Vector3f{1., 0., 0.}};
  Eigen::Affine3f modelMat = Eigen::Affine3f{rot}.scale(0.1f);
  const float pixelDensity = SDL_GetWindowPixelDensity(window);

  g_camera.view = lookAt({0.5, 1., 1.}, Float3::Zero(), {0., 0., 1.});
  CylinderCameraControl camControl{g_camera};

  while (frameNo < 500 && !quitRequested) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_EVENT_QUIT) {
        SDL_Log("Application exit requested.");
        quitRequested = true;
        break;
      }

      switch (event.type) {
      case SDL_EVENT_MOUSE_WHEEL: {
        float s = std::exp(kScrollZoom * event.wheel.y);
        fov *= s;
        projectionMat = perspectiveFromFov(fov, aspectRatio, 0.01f, 10.0f);
        break;
      }
      case SDL_EVENT_KEY_DOWN: {
        const float step_size = 0.06f;
        switch (event.key.key) {
        case SDLK_UP:
          camera_util::worldTranslateZ(g_camera, +step_size);
          break;
        case SDLK_DOWN:
          camera_util::worldTranslateZ(g_camera, -step_size);
          break;
        }
        break;
      }
      case SDL_EVENT_MOUSE_MOTION: {
        auto mouseButton = event.motion.state;
        bool controlPressed = SDL_GetModState() & SDL_KMOD_CTRL;
        if (mouseButton & SDL_BUTTON_LMASK) {
          if (controlPressed)
            camControl.moveInOut(0.95f, event.motion.yrel);
          else
            camControl.viewportDrag(
                Float2{event.motion.xrel, event.motion.yrel},
                5e-3f * pixelDensity, 1e-2f * pixelDensity);
        }
        if (mouseButton & SDL_BUTTON_RMASK) {
          float camXLocRotSpeed = 0.01f * pixelDensity;
          camera_util::localRotateXAroundOrigin(
              g_camera, camXLocRotSpeed * event.motion.yrel);
        }
        break;
      }
      }
    }
    // model-view matrix
    const Mat4f modelView = g_camera.view * modelMat.matrix();
    // MVP matrix
    const Mat4f projViewMat = projectionMat * modelView;
    const Mat3f normalMatrix =
        modelView.topLeftCorner<3, 3>().inverse().transpose();

    // render pass

    SDL_GPURenderPass *render_pass;

    CommandBuffer command_buffer(device);
    SDL_Log("Frame [%u]", frameNo);

    if (!SDL_WaitAndAcquireGPUSwapchainTexture(command_buffer, window,
                                               &swapchain, NULL, NULL)) {
      SDL_Log("Failed to acquire swapchain: %s", SDL_GetError());
      break;
    } else {

      SDL_GPUColorTargetInfo ctinfo{
          .texture = swapchain,
          .clear_color = SDL_FColor{0., 0., 0., 0.},
          .load_op = SDL_GPU_LOADOP_CLEAR,
          .store_op = SDL_GPU_STOREOP_STORE,
          .cycle = false,
      };
      SDL_GPUBufferBinding vertex_binding = meshes[0].getVertexBinding(0);
      SDL_GPUBufferBinding index_binding = meshes[0].getIndexBinding();
      render_pass =
          SDL_BeginGPURenderPass(command_buffer, &ctinfo, 1, &depthTarget);
      SDL_BindGPUGraphicsPipeline(render_pass, pipeline);
      SDL_BindGPUVertexBuffers(render_pass, 0, &vertex_binding, 1);
      SDL_BindGPUIndexBuffer(render_pass, &index_binding,
                             SDL_GPU_INDEXELEMENTSIZE_32BIT);

      TransformUniformData cameraUniform{
          projViewMat,
          normalMatrix,
      };

      SDL_PushGPUVertexUniformData(command_buffer, 0, &cameraUniform,
                                   sizeof(cameraUniform));
      SDL_DrawGPUIndexedPrimitives(render_pass, meshes[0].indexCount, 1, 0, 0,
                                   0);

      SDL_EndGPURenderPass(render_pass);
    }

    command_buffer.submit();
    frameNo++;
  }

  for (auto &mesh : meshes) {
    mesh.release();
  }
  SDL_ReleaseGPUGraphicsPipeline(device, pipeline);

  teardownExample(ctx);
  return 0;
}
