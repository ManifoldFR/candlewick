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

const Uint32 wWidth = 1024;
const Uint32 wHeight = 768;
const float aspectRatio = float(wWidth) / wHeight;

struct light_ubo_t {
  GpuVec3 viewSpaceDir;
  alignas(16) GpuVec3 color;
  float intensity;
};

int main(int argc, char **argv) {
  if (!SDL_Init(SDL_INIT_VIDEO))
    return 1;

  const char *meshFile = "assets/meshes/textured_quad.dae";
  if (argc > 1)
    meshFile = argv[1];

  RenderContext ctx(Device{auto_detect_shader_format_subset(), false},
                    Window{"TexturedMesh", wWidth, wHeight, 0},
                    SDL_GPU_TEXTUREFORMAT_D16_UNORM);
  Device &device = ctx.device;
  SDL_Window *window = ctx.window;

  char meshPath[512];
  if (meshFile[0] == '/') {
    // Absolute path — use as-is
    SDL_snprintf(meshPath, sizeof(meshPath), "%s", meshFile);
  } else {
    // Relative path — resolve from project root
    const char *basePath = SDL_GetBasePath();
    SDL_snprintf(meshPath, sizeof(meshPath), "%s../../../%s", basePath,
                 meshFile);
  }
  SDL_Log("Loading mesh: %s", meshPath);

  std::vector<MeshData> meshDatas;
  mesh_load_retc ret = loadSceneMeshes(meshPath, meshDatas);
  if (ret < mesh_load_retc::OK) {
    SDL_Log("Failed to load mesh.");
    return 1;
  }
  SDL_Log("Loaded %zu MeshData objects.", meshDatas.size());
  for (size_t i = 0; i < meshDatas.size(); i++) {
    SDL_Log("  Mesh %zu: %u vertices, %u indices, texture='%s'", i,
            meshDatas[i].numVertices(), meshDatas[i].numIndices(),
            meshDatas[i].baseColorTexturePath.c_str());
  }

  std::vector<Mesh> meshes;
  for (auto &md : meshDatas) {
    meshes.push_back(createMesh(device, md, true));
  }

  // Load textures (or white fallback)
  std::vector<Texture> textures;
  for (auto &md : meshDatas) {
    if (!md.baseColorTexturePath.empty()) {
      textures.push_back(
          loadTextureFromFile(device, md.baseColorTexturePath.c_str()));
    } else {
      textures.push_back(createWhiteTexture(device));
    }
  }
  SDL_GPUSampler *materialSampler = createMaterialSampler(device);

  /** CREATE PIPELINE **/
  SDL_GPUDepthStencilTargetInfo depth_target_info;
  GraphicsPipeline pipeline = [&]() {
    auto vertexShader = Shader::fromMetadata(device, "PbrBasic.vert");
    auto fragmentShader = Shader::fromMetadata(device, "PbrBasic.frag");

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
      .view = Eigen::Isometry3f{lookAt({0., -3., 1.5}, Float3::Zero())},
  }};

  DirectionalLight myLight{
      .direction = {0., -1., 1.},
      .color = {1.0, 1.0, 1.0},
      .intensity = 4.0,
  };

  // Dummy texture for unbound sampler slots (shadow, SSAO)
  Texture dummyTex = createWhiteTexture(device);

  // Dummy uniform data for shadow atlas (fragment uniform slot 2)
  struct alignas(16) DummyShadowAtlas {
    Eigen::Matrix<int, 4, 1, Eigen::DontAlign> regions[4];
  } dummyAtlas{};

  // Dummy vertex uniform for light matrices (vertex uniform slot 1)
  struct alignas(16) DummyLightMatrices {
    GpuMat4 mvps[4];
    Uint32 numLights = 0;
  } dummyLightMats{};

  bool quitRequested = false;
  const float pixelDensity = SDL_GetWindowPixelDensity(window);

  while (!quitRequested) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_EVENT_QUIT) {
        quitRequested = true;
        break;
      }
      if (event.type == SDL_EVENT_MOUSE_WHEEL) {
        float wy = event.wheel.y;
        const float scaleFac = std::exp(kScrollZoom * wy);
        fov = std::min(fov * scaleFac, Radf{170.0_degf});
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
          camera_util::localRotateXAroundOrigin(camera,
                                                camXLocRotSpeed *
                                                    event.motion.yrel);
        }
      }
    }

    auto modelMat = Eigen::Affine3f::Identity();
    const Eigen::Affine3f modelView = camera.camera.view * modelMat;
    const Mat4f mvp = camera.camera.projection * modelView.matrix();
    const Mat3f normalMatrix = math::computeNormalMatrix(modelView);

    CommandBuffer command_buffer = ctx.acquireCommandBuffer();

    if (!ctx.waitAndAcquireSwapchain(command_buffer)) {
      SDL_Log("Failed to acquire swapchain: %s", SDL_GetError());
      break;
    }

    SDL_GPUColorTargetInfo ctinfo{
        .texture = ctx.colorTarget(),
        .clear_color = SDL_FColor{0.15f, 0.15f, 0.15f, 1.0f},
        .load_op = SDL_GPU_LOADOP_CLEAR,
        .store_op = SDL_GPU_STOREOP_STORE,
        .cycle = false,
    };
    SDL_GPURenderPass *render_pass =
        SDL_BeginGPURenderPass(command_buffer, &ctinfo, 1, &depth_target_info);
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

    command_buffer.pushVertexUniform(0u, cameraUniform)
        .pushVertexUniform(1u, dummyLightMats)
        .pushFragmentUniform(0u, meshDatas[0].material)
        .pushFragmentUniform(1u, lightUbo)
        .pushFragmentUniform(2u, dummyAtlas);

    // Bind all 3 fragment sampler slots:
    // slot 0 = shadow map, slot 1 = SSAO, slot 2 = base color texture
    rend::bindFragmentSamplers(render_pass, 0u,
                               {
                                   {.texture = dummyTex,
                                    .sampler = materialSampler},
                                   {.texture = dummyTex,
                                    .sampler = materialSampler},
                                   {.texture = textures[0],
                                    .sampler = materialSampler},
                               });

    rend::bindMesh(render_pass, meshes[0]);
    rend::draw(render_pass, meshes[0]);

    SDL_EndGPURenderPass(render_pass);

    ctx.presentToSwapchain(command_buffer);
    command_buffer.submit();
  }

  for (auto &mesh : meshes)
    mesh.release();
  for (auto &tex : textures)
    tex.destroy();
  dummyTex.destroy();
  SDL_ReleaseGPUSampler(device, materialSampler);
  pipeline.release();
  ctx.destroy();
  SDL_Quit();
  return 0;
}
