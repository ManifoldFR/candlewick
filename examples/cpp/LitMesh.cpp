#include "Common.h"

#include "candlewick/core/RenderContext.h"
#include "candlewick/core/Mesh.h"
#include "candlewick/core/GraphicsPipeline.h"
#include "candlewick/core/Shader.h"
#include "candlewick/utils/MeshData.h"
#include "candlewick/utils/LoadMesh.h"
#include "candlewick/utils/LoadTexture.h"
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

struct light_ubo_t {
  GpuVec3 viewSpaceDir;
  alignas(16) GpuVec3 color;
  float intensity;
};

int main() {
  if (!SDL_Init(SDL_INIT_VIDEO))
    return 1;
  RenderContext ctx(Device{auto_detect_shader_format_subset(), false},
                    Window{__FILE__, wWidth, wHeight, 0},
                    SDL_GPU_TEXTUREFORMAT_D16_UNORM);
  Device &device = ctx.device;
  SDL_Window *window = ctx.window;
  const TestMesh test_mesh = meshes[0];

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
    Mesh mesh = createMesh(device, meshDatas[j], true);
    meshes.push_back(std::move(mesh));
  }
  SDL_assert(meshDatas[0].numIndices() == meshes[0].indexCount);

  /** CREATE PIPELINE **/
  SDL_GPUDepthStencilTargetInfo depth_target_info;
  GraphicsPipeline pipeline = [&]() {
    auto vertexShader = Shader::fromMetadata(device, "PbrBasic.vert");
    auto fragmentShader = Shader::fromMetadata(device, "PbrBasic.frag");

    assert(ctx.hasDepthTexture());

    SDL_GPUColorTargetDescription color_target_desc;
    SDL_zero(color_target_desc);
    color_target_desc.format = SDL_GetGPUSwapchainTextureFormat(device, window);
    SDL_zero(depth_target_info);
    depth_target_info.clear_depth = 1.0;
    depth_target_info.load_op = SDL_GPU_LOADOP_CLEAR;
    depth_target_info.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth_target_info.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth_target_info.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth_target_info.texture = ctx.depthTarget();
    depth_target_info.cycle = true;

    // create pipeline
    SDL_GPUGraphicsPipelineCreateInfo pipeline_desc{
        .vertex_shader = vertexShader,
        .fragment_shader = fragmentShader,
        .vertex_input_state = meshes[0].layout(),
        .primitive_type = meshDatas[0].primitiveType,
        .rasterizer_state{
            .fill_mode = SDL_GPU_FILLMODE_FILL,
            .cull_mode = SDL_GPU_CULLMODE_NONE,
            .front_face = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE,
        },
        .multisample_state{},
        .depth_stencil_state{
            .compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL,
            .enable_depth_test = true,
            .enable_depth_write = true,
        },
        .target_info{
            .color_target_descriptions = &color_target_desc,
            .num_color_targets = 1,
            .depth_stencil_format = ctx.depthFormat(),
            .has_depth_stencil_target = true,
        },
        .props = 0,
    };
    return GraphicsPipeline(device, pipeline_desc, nullptr);
  }();

  Rad<float> fov = 55.0_degf;
  CylindricalCamera camera{Camera{
      .projection = perspectiveFromFov(fov, aspectRatio, 0.01f, 10.0f),
      .view = Eigen::Isometry3f{lookAt({6.0, 0, 3.}, Float3::Zero())},
  }};

  Uint32 frameNo = 0;
  bool quitRequested = false;
  const float pixelDensity = SDL_GetWindowPixelDensity(window);

  DirectionalLight myLight{
      .direction = {0., -1., 1.},
      .color = {1.0, 1.0, 1.0},
      .intensity = 4.0,
  };

  // Load base color texture (fallback to white for untextured meshes)
  Texture baseColorTex =
      meshDatas[0].baseColorTexturePath.empty()
          ? createWhiteTexture(device)
          : loadTextureFromFile(device,
                                meshDatas[0].baseColorTexturePath.c_str());
  SDL_GPUSampler *materialSampler = createMaterialSampler(device);

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
        camera.camera.projection =
            perspectiveFromFov(fov, aspectRatio, 0.01f, 10.0f);
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
        auto mouseButton = event.motion.state;
        if (mouseButton >= SDL_BUTTON_LMASK) {
          camera.viewportDrag({event.motion.xrel, event.motion.yrel},
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
    const Eigen::Affine3f modelView = camera.camera.view * modelMat;
    const Mat4f mvp = camera.camera.projection * modelView.matrix();
    const Mat3f normalMatrix = math::computeNormalMatrix(modelView);

    // render pass

    SDL_GPURenderPass *render_pass;

    CommandBuffer command_buffer = ctx.acquireCommandBuffer();
    SDL_Log("Frame [%u]", frameNo);

    if (!ctx.waitAndAcquireSwapchain(command_buffer)) {
      SDL_Log("Failed to acquire swapchain: %s", SDL_GetError());
      break;
    } else {

      SDL_GPUColorTargetInfo ctinfo{
          .texture = ctx.colorTarget(),
          .clear_color = SDL_FColor{0., 0., 0., 0.},
          .load_op = SDL_GPU_LOADOP_CLEAR,
          .store_op = SDL_GPU_STOREOP_STORE,
          .cycle = false,
      };
      render_pass = SDL_BeginGPURenderPass(command_buffer, &ctinfo, 1,
                                           &depth_target_info);
      pipeline.bind(render_pass);

      TransformUniformData cameraUniform{
          .modelView = modelView.matrix(),
          .mvp = mvp,
          .normalMatrix = normalMatrix,
      };
      light_ubo_t lightUbo{
          camera.camera.transformVector(myLight.direction),
          myLight.color,
          myLight.intensity,
      };

      rend::bindMesh(render_pass, meshes[0]);

      auto materialUbo = meshDatas[0].material;

      command_buffer.pushVertexUniform(0u, cameraUniform)
          .pushFragmentUniform(0u, materialUbo)
          .pushFragmentUniform(1u, lightUbo);

      // Bind base color texture
      rend::bindFragmentSamplers(render_pass, 2u,
                                 {{
                                     .texture = baseColorTex,
                                     .sampler = materialSampler,
                                 }});

      rend::draw(render_pass, meshes[0]);

      SDL_EndGPURenderPass(render_pass);
    }

    ctx.presentToSwapchain(command_buffer);
    command_buffer.submit();
    frameNo++;
  }

  for (auto &mesh : meshes) {
    mesh.release();
  }
  baseColorTex.destroy();
  SDL_ReleaseGPUSampler(device, materialSampler);
  pipeline.release();
  ctx.destroy();
  SDL_Quit();
  return 0;
}
