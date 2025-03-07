#include "Common.h"

#include "candlewick/core/Mesh.h"
#include "candlewick/core/Shader.h"
#include "candlewick/utils/MeshData.h"
#include "candlewick/utils/LoadMesh.h"
#include "candlewick/core/CameraControls.h"
#include "candlewick/core/LightUniforms.h"
#include "candlewick/core/TransformUniforms.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_filesystem.h>

#include <Eigen/Geometry>

using namespace candlewick;

struct TestMesh {
  const char *filename;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
} meshes[] = {
    {"assets/meshes/teapot.obj",
     Eigen::Affine3f{Eigen::AngleAxisf{constants::Pi_2f, Float3{1., 0., 0.}}}},
    {"assets/meshes/mammoth.obj",
     Eigen::Affine3f{Eigen::UniformScaling(4.0f)}.rotate(
         Eigen::AngleAxisf{constants::Pi_2f, Float3{1., 0., 0.}})},
    {"assets/meshes/stanford-bunny.obj",
     Eigen::Affine3f{Eigen::UniformScaling(12.0f)}.rotate(
         Eigen::AngleAxisf(constants::Pi_2f, Float3{1., 0., 0.}))},
    {"assets/meshes/cube.obj"},
};

const Uint32 wWidth = 1600;
const Uint32 wHeight = 900;
const float aspectRatio = float(wWidth) / wHeight;

Context ctx;

struct light_ubo_t {
  GpuVec3 viewSpaceDir;
  alignas(16) GpuVec3 color;
  float intensity;
};

int main() {
  if (!initExample(ctx, wWidth, wHeight)) {
    return 1;
  }
  Device &device = ctx.device;
  SDL_Window *window = ctx.window;
  const TestMesh test_mesh = meshes[2];

  const char *basePath = SDL_GetBasePath();
  char meshPath[256];
  SDL_snprintf(meshPath, 256, "%s../../../%s", basePath, test_mesh.filename);
  auto modelMat = test_mesh.transform;

  std::vector<MeshData> meshDatas;
  mesh_load_retc ret = loadSceneMeshes(meshPath, meshDatas);
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
  auto vertexShader = Shader::fromMetadata(device, "PbrBasic.vert");
  auto fragmentShader = Shader::fromMetadata(device, "PbrBasic.frag");

  SDL_GPUTextureFormat depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D24_UNORM;
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

  SDL_GPUCommandBuffer *command_buffer;
  SDL_GPUTexture *swapchain;

  Rad<float> fov = 55.0_degf;
  Camera camera{
      .projection = perspectiveFromFov(fov, aspectRatio, 0.01f, 10.0f),
      .view = Eigen::Isometry3f{lookAt({6.0, 0, 3.}, Float3::Zero())},
  };

  Uint32 frameNo = 0;
  bool quitRequested = false;
  const float pixelDensity = SDL_GetWindowPixelDensity(window);

  DirectionalLight myLight{
      .direction = {0., -1., 1.},
      .color = {1.0, 1.0, 1.0},
      .intensity = 4.0,
  };

  while (frameNo < 1000 && !quitRequested) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_EVENT_QUIT) {
        SDL_Log("Application exit requested.");
        quitRequested = true;
        break;
      }
      if (event.type == SDL_EVENT_MOUSE_WHEEL) {
        float wy = event.wheel.y;
        const float scaleFac = std::exp(kScrollZoom * wy);
        // recreate
        fov = std::min(fov * scaleFac, Radf{170.0_degf});
        SDL_Log("Change fov to %f", rad2deg(fov));
        camera.projection = perspectiveFromFov(fov, aspectRatio, 0.01f, 10.0f);
      }
      if (event.type == SDL_EVENT_KEY_DOWN) {
        const float step_size = 0.06f;
        switch (event.key.key) {
        case SDLK_UP:
          camera_util::worldTranslateZ(camera, +step_size);
          break;
        case SDLK_DOWN:
          camera_util::worldTranslateZ(camera, -step_size);
          break;
        }
      }
      if (event.type == SDL_EVENT_MOUSE_MOTION) {
        CylinderCameraControl camControl{camera};
        auto mouseButton = event.motion.state;
        if (mouseButton >= SDL_BUTTON_LMASK) {
          camControl.viewportDrag({event.motion.xrel, event.motion.yrel},
                                  5e-3f * pixelDensity, 1e-2f * pixelDensity);
        }
        if (mouseButton >= SDL_BUTTON_RMASK) {
          float camXLocRotSpeed = 0.01f * pixelDensity;
          camera_util::localRotateXAroundOrigin(camera, camXLocRotSpeed *
                                                            event.motion.yrel);
        }
      }
    }
    // MVP matrix
    const Eigen::Affine3f modelView = camera.view * modelMat;
    const Mat4f mvp = camera.projection * modelView.matrix();
    const Mat3f normalMatrix = math::computeNormalMatrix(modelView);

    // render pass

    SDL_GPURenderPass *render_pass;

    command_buffer = SDL_AcquireGPUCommandBuffer(device);
    SDL_Log("Frame [%u]", frameNo);

    if (!SDL_AcquireGPUSwapchainTexture(command_buffer, window, &swapchain,
                                        NULL, NULL)) {
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
          .modelView = modelView.matrix(),
          .mvp = mvp,
          .normalMatrix = normalMatrix,
      };
      light_ubo_t lightUbo{
          camera.transformVector(myLight.direction),
          myLight.color,
          myLight.intensity,
      };

      auto materialUbo = meshDatas[0].material;

      SDL_PushGPUVertexUniformData(command_buffer, 0, &cameraUniform,
                                   sizeof(cameraUniform));
      SDL_PushGPUFragmentUniformData(command_buffer, 0, &materialUbo,
                                     sizeof(materialUbo));
      SDL_PushGPUFragmentUniformData(command_buffer, 1, &lightUbo,
                                     sizeof(lightUbo));
      SDL_DrawGPUIndexedPrimitives(render_pass, meshes[0].indexCount, 1, 0, 0,
                                   0);

      SDL_EndGPURenderPass(render_pass);
    }

    SDL_SubmitGPUCommandBuffer(command_buffer);
    frameNo++;
  }

  for (auto &mesh : meshes) {
    mesh.release();
  }
  SDL_ReleaseGPUGraphicsPipeline(device, pipeline);

  teardownExample(ctx);
  return 0;
}
